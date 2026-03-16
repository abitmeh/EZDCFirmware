/*
 * PulseInjectionControlStrategy.cpp
 *
 * (c) Tom Davie 2/12/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#include "BLDC/Strategies/PulseInjectionConfig.hpp"
#include "BLDC/Strategies/PulseInjectionControlStrategy.hpp"

#include <esp_log.h>

using namespace bldc;
using namespace esp;

PulseInjectionControlStrategy::PulseInjectionControlStrategy(MotorPtr& motor) : MotorControlStrategy(motor) {}

void PulseInjectionControlStrategy::start(ControlStrategyTransferableState&& state, esp_err_t& err) {
    ESP_LOGD(_loggingTag, "Starting PulseInjectionControlStrategy");
    _motor->setInPulseInjectionPhase(true);
    _state = std::move(state);
}

std::optional<Commutation> PulseInjectionControlStrategy::tick() {
    if (_phase == Cleanup) {
        _motor->setAllHighZ();
        // TO DO: Recover PhaseAngle
        //const PhaseAngle nextStep = static_cast<PhaseAngle>(inject_get_phase(_adcValues.data()));
        const PhaseAngle nextStep = PhaseAngle::Degrees0;  // BUG BUG BUG
        ESP_LOGD(_loggingTag, "inject_adc_value: %lu %lu %lu %lu %lu %lu", _adcValues[0], _adcValues[1], _adcValues[2], _adcValues[3], _adcValues[4],
                 _adcValues[5]);
        _motor->setInPulseInjectionPhase(false);
        _phase = static_cast<PulseInjectionPhase>(_phase + 1);
        const MotorState nextState(nextStep, dutyCycle());
        return Commutation(0_tks, nextState);
    }

    switch (_operation) {
        case Charge: {
            _motor->setAllHighZ();
            _operation = Inject;
            _phase = static_cast<PulseInjectionPhase>(_phase + 1);
            if (_phase == Complete && _delegate != nullptr) {
                _delegate->controlStrategyDidComplete(*this);
            }
            const MotorState nextState(static_cast<PhaseAngle>(_phase), dutyCycle());
            return Commutation(kCapacitorChargeTime, nextState);
        }
        case Inject: {
            _readADC = true;
            _operation = Charge;
            const MotorState nextState(static_cast<PhaseAngle>(_phase), dutyCycle());
            return Commutation(kPulseLength, nextState);
        }
    }

    return std::nullopt;
}

float PulseInjectionControlStrategy::dutyCycle() const {
    return kPulseInjectionDutyCycle;
}

