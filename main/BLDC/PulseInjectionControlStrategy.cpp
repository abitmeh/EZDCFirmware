/*
 * PulseInjectionControlStrategy.cpp
 *
 * (c) Tom Davie 2/12/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#include "BLDC/PulseInjectionControlStrategy.hpp"
#include "BLDC/MotorConfig.hpp"

#include "ESP32.hpp"

#include <bldc_snls_lib.h>
#include <esp_log.h>

using namespace bldc;
using namespace esp;

PulseInjectionControlStrategy::PulseInjectionControlStrategy(Motor& motor) : MotorControlStrategy(motor) {}

NextChange PulseInjectionControlStrategy::nextStepChange() {
    _motor.setInPulseInjectionPhase(true);

    if (_phase == Cleanup) {
        _motor.setAllHighZ();
        const MotorStep nextStep = static_cast<MotorStep>(inject_get_phase(_adcValues.data()));
        ESP_LOGD(_loggingTag, "Detected phase: %d", _motor.currentStep());
        ESP_LOGD(_loggingTag, "inject_adc_value: %lu %lu %lu %lu %lu %lu", _adcValues[0], _adcValues[1], _adcValues[2], _adcValues[3], _adcValues[4],
                 _adcValues[5]);
        _motor.setInPulseInjectionPhase(false);
        _phase = static_cast<PulseInjectionPhase>(_phase + 1);
        return NextChange({0, nextStep});
    }

    switch (_operation) {
        case Charge:
            _motor.setAllHighZ();
            _operation = Inject;
            _phase = static_cast<PulseInjectionPhase>(_phase + 1);
            return NextChange(NextStep(kCapacitorChargeTime, static_cast<MotorStep>(_phase)));
        case Inject:
            _readADC = true;
            _operation = Charge;
            return NextChange(NextStep(kPulseLength, static_cast<MotorStep>(_phase)));
    }

    return NextChange();
}

uint32_t PulseInjectionControlStrategy::dutyCycle() const {
    return kPulseInjectionDutyCycle;
}

std::optional<ControlPhase> PulseInjectionControlStrategy::nextControlPhase(ControlPhase currentControlPhase) const {
    return _phase == Complete ? std::optional<ControlPhase>(Alignment) : std::optional<ControlPhase>();
}

void IRAM_ATTR PulseInjectionControlStrategy::mcpwmTimerFull() {
    if (_readADC) {
        _adcValues[_phase - 1] = 1700;
        _readADC = false;
    }
}
