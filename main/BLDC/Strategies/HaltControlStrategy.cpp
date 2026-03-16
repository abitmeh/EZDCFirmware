/*
 * HallControlStrategy.cpp
 *
 * (c) Tom Davie 02/12/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#include "BLDC/MotorConfig.hpp"
#include "BLDC/Strategies/HaltControlStrategy.hpp"
#include "BLDC/Types.hpp"

#include <esp_log.h>

using namespace bldc;

HaltControlStrategy::HaltControlStrategy(MotorPtr& motor) : MotorControlStrategy(motor) {}

void HaltControlStrategy::start(ControlStrategyTransferableState&& state, esp_err_t& err) {
    ESP_LOGD(_loggingTag, "Starting HaltControlStrategy");
    _state = std::move(state);
}

std::optional<Commutation> HaltControlStrategy::tick() {
    _motor->setTargetRPM(0);
    _motor->setAllHighZ();
    _motor->stop();
    _state._currentStep = Degrees0;
    return std::nullopt;
}

float HaltControlStrategy::dutyCycle() const {
    return 0.0f;
}
