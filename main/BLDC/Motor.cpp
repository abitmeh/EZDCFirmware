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

#include <esp_log.h>

using namespace bldc;

Motor::Motor(const MotorConfig& config, esp_err_t& err)
    : _inputSwitchContext(config.inputSwitchConfig, err), _enableSwitchContext(config.enableSwitchConfig, err) {
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "Motor::Motor failed: %s", esp_err_to_name(err));
        return;
    }
}

void Motor::start(uint32_t targetRPM) {
    setAllHighZ();
    _speed.targetRPM = targetRPM;
    _nextStep = Step0;
    _currentStep = Step0;
    _highImpedencePhase = U;
}

void Motor::stop() {
    setAllHighZ();
    _speed.currentRPM = 0;
    _speed.targetRPM = 0;
    _dutyCycle = 0;
}

bool Motor::isStalled() const {
    return _speed.targetRPM != 0 && _speed.timeInCurrentStep > kStallPeriod;
}

void Motor::tick() {
    _speed.timeInCurrentStep++;

    if (_currentStep == MotorStep::Step0) {
        if (!_speedCalculatedThisRevolution) {
            _speedCalculatedThisRevolution = true;
            if (_speed.timeInCurrentStep >= kInvalidSpeedCalculationSkipLimit) {
                const uint32_t instantaneousRPM = _speed.instantaneousRPM();
                _speed.currentRPM = 0.8 * _speed.currentRPM + 0.2 * instantaneousRPM;
                ESP_LOGD(_loggingTag, "RPM: %lu (%lu), (us per phase: %lu)\n", _speed.currentRPM, instantaneousRPM, _speed.timeInCurrentStep);
            }
        }
    } else {
        uint32_t expectedTicksPerStep = _speed.currentRPM * kADCRpmCalculationCoefficient;
        if (_speed.timeInCurrentStep > expectedTicksPerStep) {
            const uint32_t instantaneousRPM = _speed.instantaneousRPM();
            _speed.currentRPM = 0.8 * _speed.currentRPM + 0.2 * instantaneousRPM;
        }
        _speedCalculatedThisRevolution = true;
    }
}

void Motor::turnIfNecessary() {
    if (_nextStep != _currentStep) {
        _turn();
    }
}

void Motor::_turn() {
    if (_inPulseInjectionPhase) {
        kPulseInjectionPhaseSetupOrder[_nextStep](_dutyCycle);
    } else {
        switch (_direction) {
            case Direction::Clockwise:
                kClockwiseSetupOrder[_nextStep](_dutyCycle);
                break;
            case Direction::Anticlockwise:
                kAnticlockwiseSetupOrder[_nextStep](_dutyCycle);
                break;
        }
    }

    _currentStep = _nextStep;
    _phaseChangeComplete = true;
}

void Motor::_setPhaseHigh(MotorPhase phase, uint32_t dutyCycle) {
    _enableSwitchContext.setDutyCycle(phase, dutyCycle);
    _inputSwitchContext.setGpioValue(phase, 1);
}

void Motor::_setPhaseLow(MotorPhase phase, uint32_t dutyCycle) {
    _enableSwitchContext.setDutyCycle(phase, dutyCycle);
    _inputSwitchContext.setGpioValue(phase, 0);
}

void Motor::_setPhaseHighZ(MotorPhase phase) {
    _enableSwitchContext.setGpioValue(phase, 0);
    _inputSwitchContext.setGpioValue(phase, 0);
}

void Motor::setAllHighZ() {
    _setPhaseHighZ(U);
    _setPhaseHighZ(V);
    _setPhaseHighZ(W);
}

void Motor::_setWHighVLow(uint32_t dutyCycle) {
    _setPhaseHigh(W, dutyCycle);
    _setPhaseLow(V, dutyCycle);
    _setPhaseHighZ(U);
    _highImpedencePhase = U;
}

void Motor::_setWHighULow(uint32_t dutyCycle) {
    _setPhaseHigh(W, dutyCycle);
    _setPhaseLow(U, dutyCycle);
    _setPhaseHighZ(V);
    _highImpedencePhase = V;
}

void Motor::_setVHighULow(uint32_t dutyCycle) {
    _setPhaseHigh(V, dutyCycle);
    _setPhaseLow(U, dutyCycle);
    _setPhaseHighZ(W);
    _highImpedencePhase = W;
}

void Motor::_setVHighWLow(uint32_t dutyCycle) {
    _setPhaseHigh(V, dutyCycle);
    _setPhaseLow(W, dutyCycle);
    _setPhaseHighZ(U);
    _highImpedencePhase = U;
}

void Motor::_setUHighWLow(uint32_t dutyCycle) {
    _setPhaseHigh(U, dutyCycle);
    _setPhaseLow(W, dutyCycle);
    _setPhaseHighZ(V);
    _highImpedencePhase = V;
}

void Motor::_setUHighVLow(uint32_t dutyCycle) {
    _setPhaseHigh(U, dutyCycle);
    _setPhaseLow(V, dutyCycle);
    _setPhaseHighZ(W);
    _highImpedencePhase = W;
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
