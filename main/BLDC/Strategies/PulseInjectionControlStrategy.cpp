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

void PulseInjectionControlStrategy::start(esp_err_t& err) {
    ESP_LOGD(_loggingTag, "Starting PulseInjectionControlStrategy");
    _motor->setInPulseInjectionPhase(true);
}

NextChange PulseInjectionControlStrategy::nextStepChange() {
    if (_phase == Cleanup) {
        _motor->setAllHighZ();
        // TO DO: Recover PhaseAngle
        //const PhaseAngle nextStep = static_cast<PhaseAngle>(inject_get_phase(_adcValues.data()));
        const PhaseAngle nextStep = _motor->currentStep();  // BUG BUG BUG
        ESP_LOGD(_loggingTag, "inject_adc_value: %lu %lu %lu %lu %lu %lu", _adcValues[0], _adcValues[1], _adcValues[2], _adcValues[3], _adcValues[4],
                 _adcValues[5]);
        _motor->setInPulseInjectionPhase(false);
        _phase = static_cast<PulseInjectionPhase>(_phase + 1);
        return NextChange(NextStep(0, nextStep));
    }

    switch (_operation) {
        case Charge:
            _motor->setAllHighZ();
            _operation = Inject;
            _phase = static_cast<PulseInjectionPhase>(_phase + 1);
            return NextChange(NextStep(kCapacitorChargeTime, static_cast<PhaseAngle>(_phase)));
        case Inject:
            _readADC = true;
            _operation = Charge;
            return NextChange(NextStep(kPulseLength, static_cast<PhaseAngle>(_phase)));
    }

    return NextChange();
}

float PulseInjectionControlStrategy::dutyCycle() const {
    return kPulseInjectionDutyCycle;
}

std::optional<ControlMode> PulseInjectionControlStrategy::nextControlMode(ControlMode currentControlMode) const {
    return _phase == Complete ? std::optional<ControlMode>(Alignment) : std::nullopt;
}
