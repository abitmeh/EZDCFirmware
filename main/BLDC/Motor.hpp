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

#include "ADC/Continuous.hpp"
#include "MCPWM/GPIOFault.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/ringbuf.h>

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
        uint32_t instantaneousRPM() { return timeInCurrentStep == 0 ? 0 : static_cast<uint32_t>(kADCRpmCalculationCoefficient / timeInCurrentStep); }

        float targetRPM = 0;
        float currentRPM = 0;
        uint32_t timeInCurrentStep = 0;
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

        Direction direction() const { return _direction; }

        void setDirection(Direction direction) { _direction = direction; }

        uint32_t dutyCycle() const { return _dutyCycle; }

        void setDutyCycle(uint32_t dutyCycle) { _dutyCycle = dutyCycle; }

        uint32_t currentRPM() { return _speed.currentRPM; }

        uint32_t targetRPM() { return _speed.targetRPM; }

        void setTargetRPM(uint32_t targetRPM) { _speed.targetRPM = targetRPM; }

        bool isStalled() const;

        bool isPhaseChangeComplete() const { return _phaseChangeComplete; }

        MotorStep currentStep() const { return _currentStep; }

        uint32_t timeInCurrentStep() const { return _speed.timeInCurrentStep; }
        uint32_t expectedStepDuration() const { return _expectedStepDuration; }

        void setNextStep(MotorStep step) { _nextStep = step; }

        void setInPulseInjectionPhase(bool pulseInjection) { _inPulseInjectionPhase = pulseInjection; }

        MotorPhase highImpedencePhase() const { return _highImpedencePhase; }

        void tick();
        void calculateSpeed();
        void commutateIfNecessary();

        bool detectZeroCross();

        void setControlPhase(ControlPhase controlPhase) { _controlPhase = controlPhase; }

        void enableADCBiasLearning(bool enable) { _adcBiasLearning = enable; }

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

        void _setADCValues(uint16_t floatingPhaseValue, uint16_t neutralValue);
        void _willCommutate();

        MotorPhase _phaseForChannel(adc_channel_t channel);

        using PhaseSetupFunction = std::function<void(uint32_t dutyCycle)>;
        using PhaseSetupOrder = std::array<PhaseSetupFunction, 6>;
        PhaseSetupOrder kPulseInjectionPhaseSetupOrder{
            std::bind(&Motor::_setUVHighWLow, this, std::placeholders::_1), std::bind(&Motor::_setWHighUVLow, this, std::placeholders::_1),
            std::bind(&Motor::_setUWHighVLow, this, std::placeholders::_1), std::bind(&Motor::_setVHighUWLow, this, std::placeholders::_1),
            std::bind(&Motor::_setVWHighULow, this, std::placeholders::_1), std::bind(&Motor::_setUHighVWLow, this, std::placeholders::_1)};
        PhaseSetupOrder kClockwiseSetupOrder{
            std::bind(&Motor::_setWHighVLow, this, std::placeholders::_1), std::bind(&Motor::_setWHighULow, this, std::placeholders::_1),
            std::bind(&Motor::_setVHighULow, this, std::placeholders::_1), std::bind(&Motor::_setVHighWLow, this, std::placeholders::_1),
            std::bind(&Motor::_setUHighWLow, this, std::placeholders::_1), std::bind(&Motor::_setUHighVLow, this, std::placeholders::_1)};
        PhaseSetupOrder kAnticlockwiseSetupOrder{
            std::bind(&Motor::_setUHighVLow, this, std::placeholders::_1), std::bind(&Motor::_setUHighWLow, this, std::placeholders::_1),
            std::bind(&Motor::_setVHighWLow, this, std::placeholders::_1), std::bind(&Motor::_setVHighULow, this, std::placeholders::_1),
            std::bind(&Motor::_setWHighULow, this, std::placeholders::_1), std::bind(&Motor::_setWHighVLow, this, std::placeholders::_1)};

        McpwmContext _inputSwitchContext;
        McpwmContext _enableSwitchContext;
        esp::mcpwm::GPIOFaultPtr _faultHandler;

        Direction _direction = Clockwise;
        MotorStep _nextStep = Step0;
        MotorStep _currentStep = Step0;
        std::deque<uint32_t> _stepDurations;
        uint32_t _expectedStepDuration = 0;
        MotorPhase _highImpedencePhase = U;
        bool _phaseChangeComplete = true;
        uint16_t _dutyCycle = 0;
        Speed _speed;
        bool _inPulseInjectionPhase = false;
        ControlPhase _controlPhase = PulseInjection;
        uint16_t _debug_ticksToNextStep = 0;
        int16_t _offsetUs;
        
        esp::adc::ADCContinuousPtr _adc = nullptr;
        esp::adc::ADCContinuous* _rawAdc = nullptr;
        uint16_t _floatingPhaseValue;
        uint16_t _neutralValue;
        std::atomic<int64_t> _lastValleyUs;
        std::atomic<int64_t> _lastBatchEndUs;
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
        MotorStep _learningStep = Step0;

        // Zero crossing detection data
        uint16_t _ticksInCrossedState = 0;
        bool _hasBeenUncrossed = false;
        bool _expectingCrossUpwards = false;
        MotorStep _detectionStep = Step0;

        static constexpr uint16_t kStallPeriod = 15'000;
    
        static constexpr char _loggingTag[] = "bldc::Motor";

        friend void _adcTask(void* userInfo);
        friend esp::InterruptResult _onAdcConversion(const uint8_t* rawData, size_t count, void* userInfo);
        friend esp::InterruptResult _onMcpwmTimerFull(const mcpwm_timer_event_data_t& eventData, void* userData);
    };
}  // namespace bldc

