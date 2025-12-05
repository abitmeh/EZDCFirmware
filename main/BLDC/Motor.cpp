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

    calculateSpeed();
}

void Motor::calculateSpeed() {
    if (_inPulseInjectionPhase) {
        return;
    }

    if (_currentStep != _stepInPreviousTick) {
        _stepInPreviousTick = _currentStep;
        _stepDurations.push_front(_speed.timeInCurrentStep);
        _speed.timeInCurrentStep = 0;
    }

    if (_stepDurations.empty()) {
        _speed.currentRPM = 0;
        return;
    }

    uint32_t totalDuration = 0;
    for (auto iter = _stepDurations.begin(); iter != _stepDurations.end(); ++iter) {
        totalDuration += *iter;
        if (totalDuration >= kSpeedAveragingDuration) {
            _stepDurations.erase(iter, _stepDurations.end());
            break;
        }
    }

    uint32_t averageDuration = totalDuration / _stepDurations.size();
    if (averageDuration == 0) {
        return;
    }
    if (_speed.timeInCurrentStep > averageDuration) {
        uint32_t totalDuration = _speed.timeInCurrentStep;
        for (auto iter = _stepDurations.begin(); iter != _stepDurations.end(); ++iter) {
            totalDuration += *iter;
            if (totalDuration >= kSpeedAveragingDuration) {
                _stepDurations.erase(iter, _stepDurations.end());
                break;
            }
        }
        averageDuration = totalDuration / (_stepDurations.size() + 1);
    }

    _speed.currentRPM = kADCRpmCalculationCoefficient / averageDuration;
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
