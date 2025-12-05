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

#include <array>
#include <deque>
#include <functional>
#include <memory>

namespace bldc {
    struct MotorConfig {
        McpwmConfig inputSwitchConfig;
        McpwmConfig enableSwitchConfig;
    };

    struct Speed {
    public:
        uint32_t instantaneousRPM() { return timeInCurrentStep == 0 ? 0 : static_cast<uint32_t>(kADCRpmCalculationCoefficient / timeInCurrentStep); }

        uint32_t targetRPM = 0;
        uint32_t currentRPM = 0;
        uint32_t timeInCurrentStep = 0;
    };

    class Motor;
    using MotorPtr = std::shared_ptr<Motor>;

    class Motor {
    public:
        Motor(const MotorConfig& config, esp_err_t& err);

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

        void setNextStep(MotorStep step) { _nextStep = step; }

        void setInPulseInjectionPhase(bool pulseInjection) { _inPulseInjectionPhase = pulseInjection; }

        MotorPhase highImpedencePhase() const { return _highImpedencePhase; }

        void tick();
        void calculateSpeed();
        void turnIfNecessary();

    private:
        void _turn();

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

        Direction _direction = Clockwise;
        MotorStep _nextStep = Step0;
        MotorStep _currentStep = Step0;
        MotorStep _stepInPreviousTick = Step0;
        std::deque<uint32_t> _stepDurations;
        MotorPhase _highImpedencePhase = U;
        bool _phaseChangeComplete = true;
        uint16_t _dutyCycle = 0;
        Speed _speed;
        bool _inPulseInjectionPhase = false;

        static constexpr uint16_t kStallPeriod = 15'000;
        static constexpr char _loggingTag[] = "bldc::Motor";
    };
}  // namespace bldc

