/*
 * Motor.hpp
 *
 * (c) Tom Davie 28/11/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#pragma once

#include "BLDC/McpwmContext.hpp"
#include "BLDC/MotorConfig.hpp"
#include "BLDC/Types.hpp"
#include "Utilities/Tracer.hpp"

#include "ADC/Continuous.hpp"
#include "MCPWM/GPIOFault.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>
#include <freertos/task.h>

#include <array>
#include <atomic>
#include <deque>
#include <functional>
#include <memory>

#define TRACE_ADC 1

namespace bldc {
    static constexpr uint16_t kADCBufferSize = kAdcSamplesPerPWMPeriod * (kMotorPhaseCount + 1);

    struct MotorADCConfig {
        adc_unit_t unit;
        std::array<adc_channel_t, kMotorPhaseCount + 1> channels;
    };

    struct MotorConfig {
        std::array<gpio_num_t, 3> inputGPIOs;
        std::array<gpio_num_t, 3> enableGPIOs;
        MotorADCConfig adcConfig;
    };

    void _adcTask(void* userInfo);
    esp::InterruptResult _onAdcConversion(const uint8_t* rawData, size_t count, void* userInfo);
    esp::InterruptResult _onMcpwmTimerFull(const mcpwm_timer_event_data_t& eventData, void* userData);

    struct Speed {
    public:
        uint32_t instantaneousRPM() { return timeInCurrentStep == 0_tks ? 0 : static_cast<uint32_t>(kTimePerPhaseAt1RPM / timeInCurrentStep); }

        float targetRPM = 0;
        float currentRPM = 0;
        Ticks32 timeInCurrentStep{0u};
    };

    class Motor;
    using MotorPtr = std::shared_ptr<Motor>;

    class Motor {
    public:
        Motor(const MotorConfig& config, esp_err_t& err);
        ~Motor();

        Motor(const Motor&) = delete;
        Motor& operator=(const Motor&) = delete;
        Motor(Motor&&) = delete;
        Motor& operator=(Motor&&) = delete;

        esp_err_t configureFaultHandling(gpio_num_t gpioNum, bool inverted, esp::mcpwm::GPIOFault::Callback callback);

        void start(uint32_t targetRPM);
        void stop();

        void setAllHighZ();

        uint32_t dutyCycle() const { return _currentMotorState._dutyCycles[_currentMotorState.lowPhase()]; }

        void setDutyCycle(uint32_t dutyCycle) {
            MotorPhase lowPhase = _currentMotorState.lowPhase();
            _enableSwitchContext.setDutyCycle(lowPhase, dutyCycle);
            _currentMotorState._dutyCycles[lowPhase] = dutyCycle;
        }

        uint32_t currentRPM() { return _speed.currentRPM; }

        uint32_t targetRPM() { return _speed.targetRPM; }

        void setTargetRPM(uint32_t targetRPM) { _speed.targetRPM = targetRPM; }

        bool isStalled() const;

        bool isPhaseChangeComplete() const { return _phaseChangeComplete; }

        Ticks16 timeInCurrentStep() const { return _speed.timeInCurrentStep; }

        Ticks16 expectedStepDuration() const { return _expectedStepDuration; }

        void setNextMotorState(MotorState motorState) { _nextMotorState = motorState; }

        void setInPulseInjectionPhase(bool pulseInjection) { _inPulseInjectionPhase = pulseInjection; }

        MotorPhase highImpedencePhase() const { return _currentMotorState.floatingPhase(); }

        void tick();
        void calculateSpeed();
        void commutateIfNecessary();

        bool detectZeroCross();

        void enableADCBiasLearning(bool enable) { _adcBiasLearning = enable; }

        void writeSample(TraceSample* sample);

    private:
        void _commutate();

        void _setPhaseHigh(MotorPhase phase, uint32_t dutyCycle);
        void _setPhaseLow(MotorPhase phase, uint32_t dutyCycle);
        void _setPhaseHighZ(MotorPhase phase);

        void _setWHighULow(uint32_t dutyCycle);
        void _setVHighULow(uint32_t dutyCycle);
        void _setVHighWLow(uint32_t dutyCycle);
        void _setUHighWLow(uint32_t dutyCycle);
        void _setUHighVLow(uint32_t dutyCycle);
        void _setWHighVLow(uint32_t dutyCycle);

        void _setUVHighWLow(uint32_t dutyCycle);
        void _setWHighUVLow(uint32_t dutyCycle);
        void _setUWHighVLow(uint32_t dutyCycle);
        void _setVHighUWLow(uint32_t dutyCycle);
        void _setVWHighULow(uint32_t dutyCycle);
        void _setUHighVWLow(uint32_t dutyCycle);

        void _configureMotorState(const MotorState& state);

        void _setADCValues(uint16_t floatingPhaseValue, uint16_t neutralValue);
        void _willCommutate();

        MotorPhase _phaseForChannel(adc_channel_t channel);

        McpwmContext _inputSwitchContext;
        McpwmContext _enableSwitchContext;
        esp::mcpwm::GPIOFaultPtr _faultHandler;

        MotorState _currentMotorState;
        MotorState _nextMotorState;
        std::deque<Ticks16> _stepDurations;
        Ticks16 _expectedStepDuration{static_cast<uint16_t>(0)};
        bool _phaseChangeComplete = true;
        Speed _speed;
        bool _inPulseInjectionPhase = false;
        std::chrono::microseconds _offset;

        esp::adc::ADCContinuousPtr _adc = nullptr;
        esp::adc::ADCContinuous* _rawAdc = nullptr;
        uint16_t _floatingPhaseValue;
        uint16_t _neutralValue;
        std::atomic<std::chrono::microseconds> _lastValley;
        std::atomic<std::chrono::microseconds> _lastBatchEnd;
        RingbufHandle_t _rawADCDataRingbuffer;
        TaskHandle_t _adcTaskHandle;
        uint16_t _rawADCValues[kMotorPhaseCount + 1];
        int16_t _integerADCBiases[6] = {0, 0, 0, 0, 0, 0};
        float _adcBiases[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        bool _adcBiasLearning = false;
        std::pair<adc_channel_t, MotorPhase> _channelToPhase[kMotorPhaseCount + 1];
        size_t _numberOfObservedValuesThisCommutation = 0;
        uint32_t _observedTotalFloatingPhaseThisCommutation = 0;
        uint32_t _observedTotalNeutralThisCommutation = 0;
        PhaseAngle _learningStep = Degrees0;

        // Zero crossing detection data
        Ticks16 _ticksInCrossedState{0};
        bool _startedBelow = false;

        static constexpr char _loggingTag[] = "bldc::Motor";

        friend void _adcTask(void* userInfo);
        friend esp::InterruptResult _onAdcConversion(const uint8_t* rawData, size_t count, void* userInfo);
        friend esp::InterruptResult _onMcpwmTimerFull(const mcpwm_timer_event_data_t& eventData, void* userData);
    };
}  // namespace bldc

