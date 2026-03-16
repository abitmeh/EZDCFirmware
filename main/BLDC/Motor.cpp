/*
 * Motor.cpp
 *
 * (c) Tom Davie 28/11/2025
 *
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#include "BLDC/Motor.hpp"
#include "BLDC/MotorConfig.hpp"
#include "Utilities/Tracer.hpp"

#include "ESP32.hpp"
#include "Timer.hpp"

#include <esp_log.h>
#include <esp_task_wdt.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <ranges>

using namespace bldc;

using namespace esp;
using namespace esp::adc;
using namespace esp::mcpwm;

namespace bldc {
    static constexpr std::chrono::microseconds kADCPipelineDelay = 12us;

    InterruptResult IRAM_ATTR _onAdcConversion(const uint8_t* rawData, size_t count, void* userInfo) {
        Motor* motor = reinterpret_cast<Motor*>(userInfo);
        motor->_lastBatchEnd.store(esp::Timer::now(), std::memory_order_relaxed);

        BaseType_t highPriorityTaskWoken = pdFALSE;
        xRingbufferSendFromISR(motor->_rawADCDataRingbuffer, rawData, count, &highPriorityTaskWoken);
        return highPriorityTaskWoken ? InterruptResult::HighPriorityTaskWoken : InterruptResult::NoHighPriorityTaskWoken;
    }

    void _adcTask(void* userInfo) {
        Motor* motor = reinterpret_cast<Motor*>(userInfo);

        while (true) {
            size_t count;
            uint8_t* rawData = reinterpret_cast<uint8_t*>(xRingbufferReceive(motor->_rawADCDataRingbuffer, &count, pdMS_TO_TICKS(1000)));
            if (rawData == nullptr) {
                continue;
            }

            static adc_continuous_data_t parsedData[kADCBufferSize];
            esp_err_t err = ESP_OK;
            motor->_rawAdc->parse(rawData, count, parsedData, err);
            vRingbufferReturnItem(motor->_rawADCDataRingbuffer, rawData);

            if (err != ESP_OK) {
                ESP_DRAM_LOGE(motor->_loggingTag, "ADC conversion failed: %s", esp_err_to_name(err));
                continue;
            }

            const std::chrono::microseconds batchEnd = motor->_lastBatchEnd.load(std::memory_order_relaxed);
            const size_t totalSamples = count / sizeof(adc_digi_output_data_t);
            const size_t totalSlots = totalSamples / (kMotorPhaseCount + 1);
            const size_t sampleFrequency = kAdcFrequency * (kMotorPhaseCount + 1);
            const std::chrono::microseconds batchSpan((static_cast<int64_t>(totalSamples) * 1'000'000LL) / sampleFrequency);
            const std::chrono::microseconds batchStart = batchEnd - batchSpan;

            if (batchSpan == 0us || totalSlots == 0) {
                continue;
            }

            const std::chrono::microseconds valley = motor->_lastValley.load(std::memory_order_relaxed) - kADCPipelineDelay;

            const std::chrono::microseconds offset = valley - batchStart;
            const std::chrono::microseconds offsetInBatch = ((offset % batchSpan) + batchSpan) % batchSpan;
            static constexpr std::chrono::microseconds minTick16 = static_cast<std::chrono::microseconds>(std::numeric_limits<Ticks16>::min());
            static constexpr std::chrono::microseconds maxTick16 = static_cast<std::chrono::microseconds>(std::numeric_limits<Ticks16>::max());

            motor->_offset = std::chrono::duration_cast<Ticks16>(std::clamp(offsetInBatch, minTick16, maxTick16));

            size_t bestSampleSlot = static_cast<size_t>((totalSlots * offsetInBatch) / batchSpan);
            bestSampleSlot = std::min(bestSampleSlot, totalSlots - 1);
            for (size_t j = (kMotorPhaseCount + 1) * bestSampleSlot; j < (kMotorPhaseCount + 1) * (bestSampleSlot + 1); ++j) {
                const adc_continuous_data_t& data = parsedData[j];
                if (!data.valid) {
                    continue;
                }

                const uint8_t phase = motor->_phaseForChannel(data.channel);

                motor->_rawADCValues[phase] = static_cast<uint16_t>(data.raw_data);
            }
            motor->_rawADCValues[motor->_currentMotorState.floatingPhase()] += motor->_integerADCBiases[motor->_currentMotorState.nearestPhaseAngle()];
        }
    }

    InterruptResult IRAM_ATTR _onMcpwmTimerFull(const mcpwm_timer_event_data_t& eventData, void* userData) {
        Motor* motor = reinterpret_cast<Motor*>(userData);

        // Stamp the valley time as early as possible in the ISR so the
        // measurement is as close to the actual counter-full moment as we can get.
        motor->_lastValley.store(esp::Timer::now(), std::memory_order_relaxed);

        if (motor->_inPulseInjectionPhase) {
            return InterruptResult::NoHighPriorityTaskWoken;
        }

        const uint8_t floatingPhase = static_cast<uint8_t>(motor->_currentMotorState.floatingPhase());
        const PhaseAngle motorAngle = motor->_currentMotorState.nearestPhaseAngle();
        assert(floatingPhase < 3);

        static constexpr int16_t inverseAlpha = 5;

        const int16_t floatingPhaseValue = motor->_rawADCValues[floatingPhase] + motor->_integerADCBiases[motorAngle];
        const int16_t vddValue = motor->_rawADCValues[kMotorPhaseCount];
        const int16_t neutralValue = vddValue / 2;

        const int16_t previousFloatingPhaseValue = motor->_floatingPhaseValue;
        const int16_t previousNeutralValue = motor->_neutralValue;

        const bool shouldSmoothWithPreviousValue(motor->_speed.timeInCurrentStep >= 2_tk && previousNeutralValue != 0 &&
                                                 (previousFloatingPhaseValue - previousNeutralValue) * 3 < previousNeutralValue * 10);

        const uint16_t smoothedFloatingPhaseValue =
            shouldSmoothWithPreviousValue ? (previousFloatingPhaseValue * inverseAlpha + (floatingPhaseValue - previousFloatingPhaseValue)) / inverseAlpha
                                          : floatingPhaseValue;

        const uint16_t smoothedNeutralValue = (previousNeutralValue * inverseAlpha + (neutralValue - previousNeutralValue)) / inverseAlpha;

        motor->_setADCValues(smoothedFloatingPhaseValue, smoothedNeutralValue);
        return InterruptResult::NoHighPriorityTaskWoken;
    }
}  // namespace bldc

Motor::Motor(const MotorConfig& config, esp_err_t& err)
    : _inputSwitchContext(McpwmConfig(0, config.inputGPIOs[0], config.inputGPIOs[1], config.inputGPIOs[2]), err),
      _enableSwitchContext(McpwmConfig(1, config.enableGPIOs[0], config.enableGPIOs[1], config.enableGPIOs[2]), err) {
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "Failed to construct McpwmContext: %s", esp_err_to_name(err));
        return;
    }

    _enableSwitchContext.setTimerEventCallback(TimerEvent::Full, _onMcpwmTimerFull, this);

    _rawADCDataRingbuffer = xRingbufferCreateNoSplit(kADCBufferSize * SOC_ADC_DIGI_DATA_BYTES_PER_CONV, 2);
    if (_rawADCDataRingbuffer == nullptr) {
        ESP_LOGE(_loggingTag, "Failed to allocate ADC data ringbuffer");
        err = ESP_ERR_NO_MEM;
        return;
    }

    ADCContinuousConfig adcConfig = {
        .maximumStoredValues = kADCBufferSize,
        .numberOfValuesPerConversionFrame = kADCBufferSize,
        .flushWhenFull = true,
        .samplingFrequencyHz = kAdcFrequency,
    };
    for (const auto& [i, channel] : config.adcConfig.channels | std::ranges::views::enumerate) {
        adcConfig.channels.emplace_back(config.adcConfig.unit, channel, Attenuation::Decibels12, BitWidth::Bits12);
        _channelToPhase[i] = std::pair<adc_channel_t, MotorPhase>(channel, static_cast<MotorPhase>(i));
    }

    _adc = ESP32::sharedESP32()->adcContinuous(adcConfig, err);
    _rawAdc = _adc.get();
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "ESP32::adcContinuous failed: %s", esp_err_to_name(err));
        return;
    }

    ADCContinuousEventCallbacks adcCallbacks{.onConversionComplete = bldc::_onAdcConversion, .onPoolOverflow = nullptr};
    _adc->setEventCallbacks(adcCallbacks, this);

    BaseType_t adcTaskCreationResult = xTaskCreatePinnedToCore(_adcTask, "ADC Task", 4096, this, tskIDLE_PRIORITY + 4, &_adcTaskHandle, 1);
    if (adcTaskCreationResult != pdPASS) {
        ESP_LOGE(_loggingTag, "Failed to create ADC processing task");
        err = ESP_ERR_NO_MEM;
        return;
    }
}

Motor::~Motor() {
    ESP_LOGE(_loggingTag, "WTF");
}

esp_err_t Motor::configureFaultHandling(gpio_num_t gpioNum, bool inverted, esp::mcpwm::GPIOFault::Callback callback) {
    GPIOFaultConfig faultConfig = {.groupId = 0, .interruptPriority = esp::InterruptPriority::Default, .gpioNum = gpioNum, .activeHigh = inverted};
    esp_err_t err = ESP_OK;
    _faultHandler = ESP32::sharedESP32()->mcpwm().gpioFault(faultConfig, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "GPIOFault construction failed: %s", esp_err_to_name(err));
        return err;
    }
    return err;
}

void Motor::start(uint32_t targetRPM) {
    esp_err_t err = ESP_OK;
    err = _inputSwitchContext.start();
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "McpwmContext::start failed: %s", esp_err_to_name(err));
        return;
    }
    err = _enableSwitchContext.start();
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "McpwmContext::start failed: %s", esp_err_to_name(err));
        return;
    }

    _adc->start();

    setAllHighZ();
    _speed.targetRPM = targetRPM;
}

void Motor::stop() {
    setAllHighZ();
    _speed.currentRPM = 0;
    _speed.targetRPM = 0;
}

void Motor::tick() {
    _speed.timeInCurrentStep++;

    calculateSpeed();
}

void Motor::calculateSpeed() {
    if (_inPulseInjectionPhase) {
        return;
    }

    if (_stepDurations.empty()) {
        _speed.currentRPM = 0;
        return;
    }

    Ticks32 totalDuration = 0_tk;
    for (auto iter = _stepDurations.begin(); iter != _stepDurations.end(); ++iter) {
        totalDuration += *iter;
        // Always keep at least one duration, but keep enough to figure out an acurate speed.
        if (totalDuration >= kSpeedAveragingDuration && iter != _stepDurations.begin()) {
            _stepDurations.erase(iter, _stepDurations.end());
            break;
        }
    }

    Ticks32 averageDuration = totalDuration / _stepDurations.size();
    _expectedStepDuration = averageDuration;
    if (averageDuration == 0_tk) {
        return;
    }
    if (_speed.timeInCurrentStep > averageDuration) {
        averageDuration = (totalDuration + _speed.timeInCurrentStep) / (_stepDurations.size() + 1);
    }
    _speed.currentRPM = kTimePerPhaseAt1RPM / averageDuration;
}

void Motor::commutateIfNecessary() {
    if (_nextMotorState != _currentMotorState) {
        _commutate();
    }
}

void Motor::_commutate() {
    _willCommutate();

    _configureMotorState(_nextMotorState);

    //Tracer::sharedTracer()->sendEvent(TraceEvent::PhaseChangeRequested);
    _currentMotorState = _nextMotorState;
    _phaseChangeComplete = true;
    _stepDurations.push_front(_speed.timeInCurrentStep);
    _speed.timeInCurrentStep = 0_tks;
}

void Motor::_configureMotorState(const MotorState& state) {
    for (size_t i = 0; i < 3; ++i) {
        const MotorPhase phase = static_cast<MotorPhase>(i);
        switch (state._phaseStates[i]) {
            case PhaseState::Low:
                _setPhaseLow(phase, state._dutyCycles[i] * kMaxDutyCycle);
                break;
            case PhaseState::High:
                _setPhaseHigh(phase, state._dutyCycles[i] * kMaxDutyCycle);
                break;
            case PhaseState::HighZ:
                _setPhaseHighZ(phase);
                break;
            default:
                std::unreachable();
        }
    }
}

void Motor::_setPhaseHigh(MotorPhase phase, uint32_t dutyCycle) {
    _enableSwitchContext.setDutyCycle(phase, dutyCycle);
    _inputSwitchContext.setGpioValue(phase, 1);
}

void Motor::_setPhaseLow(MotorPhase phase, uint32_t dutyCycle) {
    _inputSwitchContext.setGpioValue(phase, 0);
    _enableSwitchContext.setDutyCycle(phase, dutyCycle);
}

void Motor::_setPhaseHighZ(MotorPhase phase) {
    _enableSwitchContext.setGpioValue(phase, 0);
    _inputSwitchContext.setGpioValue(phase, 0);
}

void Motor::setAllHighZ() {
    _setPhaseHighZ(U);
    _setPhaseHighZ(V);
    _setPhaseHighZ(W);
    _currentMotorState = MotorState({PhaseState::HighZ, PhaseState::HighZ, PhaseState::HighZ}, 0);
}

void Motor::_setWHighVLow(uint32_t dutyCycle) {
    _setPhaseHigh(W, dutyCycle);
    _setPhaseLow(V, dutyCycle);
    _setPhaseHighZ(U);
    _currentMotorState = MotorState({PhaseState::HighZ, PhaseState::Low, PhaseState::High}, dutyCycle);
}

void Motor::_setWHighULow(uint32_t dutyCycle) {
    _setPhaseHigh(W, dutyCycle);
    _setPhaseLow(U, dutyCycle);
    _setPhaseHighZ(V);
    _currentMotorState = MotorState({PhaseState::Low, PhaseState::HighZ, PhaseState::High}, dutyCycle);
}

void Motor::_setVHighULow(uint32_t dutyCycle) {
    _setPhaseHigh(V, dutyCycle);
    _setPhaseLow(U, dutyCycle);
    _setPhaseHighZ(W);
    _currentMotorState = MotorState({PhaseState::Low, PhaseState::High, PhaseState::HighZ}, dutyCycle);
}

void Motor::_setVHighWLow(uint32_t dutyCycle) {
    _setPhaseHigh(V, dutyCycle);
    _setPhaseLow(W, dutyCycle);
    _setPhaseHighZ(U);
    _currentMotorState = MotorState({PhaseState::HighZ, PhaseState::High, PhaseState::Low}, dutyCycle);
}

void Motor::_setUHighWLow(uint32_t dutyCycle) {
    _setPhaseHigh(U, dutyCycle);
    _setPhaseLow(W, dutyCycle);
    _setPhaseHighZ(V);
    _currentMotorState = MotorState({PhaseState::High, PhaseState::HighZ, PhaseState::Low}, dutyCycle);
}

void Motor::_setUHighVLow(uint32_t dutyCycle) {
    _setPhaseHigh(U, dutyCycle);
    _setPhaseLow(V, dutyCycle);
    _setPhaseHighZ(W);
    _currentMotorState = MotorState({PhaseState::High, PhaseState::Low, PhaseState::HighZ}, dutyCycle);
}

void Motor::_setUVHighWLow(uint32_t dutyCycle) {
    _setPhaseHigh(U, dutyCycle * 0.5);
    _setPhaseHigh(V, dutyCycle * 0.5);
    _setPhaseLow(W, dutyCycle);
}

void Motor::_setWHighUVLow(uint32_t dutyCycle) {
    _setPhaseLow(U, dutyCycle * 0.5);
    _setPhaseLow(V, dutyCycle * 0.5);
    _setPhaseHigh(W, dutyCycle);
}

void Motor::_setUWHighVLow(uint32_t dutyCycle) {
    _setPhaseHigh(U, dutyCycle * 0.5);
    _setPhaseHigh(W, dutyCycle * 0.5);
    _setPhaseLow(V, dutyCycle);
}

void Motor::_setVHighUWLow(uint32_t dutyCycle) {
    _setPhaseLow(U, dutyCycle * 0.5);
    _setPhaseLow(W, dutyCycle * 0.5);
    _setPhaseHigh(V, dutyCycle);
}

void Motor::_setVWHighULow(uint32_t dutyCycle) {
    _setPhaseHigh(V, dutyCycle * 0.5);
    _setPhaseHigh(W, dutyCycle * 0.5);
    _setPhaseLow(U, dutyCycle);
}

void Motor::_setUHighVWLow(uint32_t dutyCycle) {
    _setPhaseLow(V, dutyCycle * 0.5);
    _setPhaseLow(W, dutyCycle * 0.5);
    _setPhaseHigh(U, dutyCycle);
}

void Motor::_setADCValues(uint16_t floatingPhaseValue, uint16_t neutralValue) {
    _floatingPhaseValue = floatingPhaseValue;
    _neutralValue = neutralValue;

    if (!_adcBiasLearning) {
        return;
    }

    const Ticks16 relativeZeroCrossBlankingPeriod = std::chrono::duration_cast<Ticks16>(kZeroCrossBlankingPeriod * _expectedStepDuration);
    const Ticks16 absoluteZeroCrossBlankingPeriod = std::chrono::duration_cast<Ticks16>(1000us);
    const Ticks16 zeroCrossBlankingTime = std::max(relativeZeroCrossBlankingPeriod, absoluteZeroCrossBlankingPeriod);

    if (timeInCurrentStep() < zeroCrossBlankingTime) {
        return;
    }

    _observedTotalFloatingPhaseThisCommutation += _floatingPhaseValue;
    _observedTotalNeutralThisCommutation += _neutralValue;
    _numberOfObservedValuesThisCommutation++;
}

void Motor::_willCommutate() {
    const float observationCount = static_cast<float>(_numberOfObservedValuesThisCommutation);
    if (std::abs(observationCount) < std::numeric_limits<float>::epsilon()) {
        return;
    }

    const float observedAverageNeutral = static_cast<float>(_observedTotalNeutralThisCommutation) / observationCount;
    const float observedAverageFloatingPhase = static_cast<float>(_observedTotalFloatingPhaseThisCommutation) / observationCount;
    const float observedBias = observedAverageNeutral - observedAverageFloatingPhase;
    static const float alpha = 0.1f;
    const PhaseAngle phaseAngle = _currentMotorState.nearestPhaseAngle();
    _adcBiases[phaseAngle] += alpha * observedBias;
    _integerADCBiases[phaseAngle] = static_cast<int16_t>(_adcBiases[phaseAngle]);
    _numberOfObservedValuesThisCommutation = 0;
    _observedTotalFloatingPhaseThisCommutation = 0;
    _observedTotalNeutralThisCommutation = 0;

    _ticksInCrossedState = 0_tks;
    _startedBelow = _floatingPhaseValue < _neutralValue;
}

bool Motor::detectZeroCross() {
    const bool isBelow = _floatingPhaseValue < _neutralValue;

    bool isCrossed = _startedBelow != isBelow;
    if (isCrossed) {
        _ticksInCrossedState++;
    } else if (_ticksInCrossedState > 0_tks) {
        _ticksInCrossedState = 0_tks;
    }

    if (_ticksInCrossedState >= kZeroCrossRepeatTime) {
        return true;
    }

    return false;
}

MotorPhase Motor::_phaseForChannel(adc_channel_t channel) {
    for (const auto& [c, p] : _channelToPhase) {
        if (c == channel) {
            return p;
        }
    }

    ESP_DRAM_LOGE(_loggingTag, "No Phase found for channel %u", channel);
    return MotorPhase::U;
}

void Motor::writeSample(TraceSample* sample) {
    sample->phase = _currentMotorState.nearestPhaseAngle();
    sample->phaseValue = _floatingPhaseValue;
    sample->neutralValue = _neutralValue;
    sample->valleyOffsetUs = static_cast<std::chrono::microseconds>(_offset).count();
}
