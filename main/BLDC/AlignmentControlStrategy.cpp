/*
 * AlignmentControlStrategy.cpp
 *
 * (c) Tom Davie 02/12/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#include "BLDC/AlignmentControlStrategy.hpp"
#include "BLDC/MotorConfig.hpp"

#include "ESP32.hpp"

#include <esp_log.h>

using namespace bldc;
using namespace esp;

AlignmentControlStrategy::AlignmentControlStrategy(Motor& motor) : MotorControlStrategy(motor) {}

NextChange AlignmentControlStrategy::nextStepChange() {
    if (_hasAligned) {
        return NextChange({kAlignmentTimeMs, _motor.currentStep()});
    } else {
        _hasAligned = true;
        return NextChange({0, _motor.currentStep()});
    }
}

uint32_t AlignmentControlStrategy::dutyCycle() const {
    return kAlignmentDutyCycle;
}

std::optional<ControlPhase> AlignmentControlStrategy::nextControlPhase(ControlPhase currentControlPhase) const {
    return _hasAligned ? std::optional<ControlPhase>(Drag) : std::optional<ControlPhase>();
}
