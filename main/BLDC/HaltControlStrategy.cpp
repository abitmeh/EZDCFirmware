/*
 * HallControlStrategy.cpp
 *
 * (c) Tom Davie 02/12/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#include "BLDC/HaltControlStrategy.hpp"
#include "BLDC/MotorConfig.hpp"

#include <esp_log.h>

using namespace bldc;

HaltControlStrategy::HaltControlStrategy(Motor& motor) : MotorControlStrategy(motor) {}

NextChange HaltControlStrategy::nextStepChange() {
    _motor.setTargetRPM(0);
    _motor.setAllHighZ();
    return NextChange();
}

uint32_t HaltControlStrategy::dutyCycle() const {
    return 0;
}

std::optional<ControlPhase> HaltControlStrategy::nextControlPhase(ControlPhase controlPhase) const {
    return controlPhase == Stopped ? std::optional<ControlPhase>() : std::optional<ControlPhase>(Stopped);
}
