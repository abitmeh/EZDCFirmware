/*
 * HallControlStrategy.cpp
 *
 * (c) Tom Davie 02/12/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#include "BLDC/Strategies/HaltControlStrategy.hpp"
#include "BLDC/MotorConfig.hpp"
#include "BLDC/Types.hpp"

#include <esp_log.h>

using namespace bldc;

HaltControlStrategy::HaltControlStrategy(MotorPtr& motor) : MotorControlStrategy(motor) {}

void HaltControlStrategy::start(esp_err_t& err) {
    ESP_LOGD(_loggingTag, "Starting HaltControlStrategy");
}

NextChange HaltControlStrategy::nextStepChange() {
    _motor->setTargetRPM(0);
    _motor->setAllHighZ();
    return NextChange();
}

float HaltControlStrategy::dutyCycle() const {
    return 0.0f;
}

std::optional<ControlMode> HaltControlStrategy::nextControlMode(ControlMode controlMode) const {
    return controlMode == Stopped ? std::nullopt : std::optional<ControlMode>(ControlMode::Stopped);
}
