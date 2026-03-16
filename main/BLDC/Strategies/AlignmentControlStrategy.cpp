/*
 * AlignmentControlStrategy.cpp
 *
 * (c) Tom Davie 02/12/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#include "BLDC/Strategies/AlignmentConfig.hpp"
#include "BLDC/Strategies/AlignmentControlStrategy.hpp"

#include "ESP32.hpp"

#include <esp_log.h>

using namespace bldc;
using namespace esp;

static const size_t kStepCount = 10;

AlignmentControlStrategy::AlignmentControlStrategy(MotorPtr& motor) : MotorControlStrategy(motor) {}

void AlignmentControlStrategy::start(ControlStrategyTransferableState&& state, esp_err_t& err) {
    ESP_LOGD(_loggingTag, "Starting AlignmentControlStrategy");
    _state = std::move(state);
}

std::optional<Commutation> AlignmentControlStrategy::tick() {
    const Ticks32 currentTicks = _step * kAlignmentTime / kStepCount;
    _step++;
    const Ticks32 nextTicks = _step * kAlignmentTime / kStepCount;

    const Ticks16 nextPhaseLength = static_cast<Ticks16>(nextTicks - currentTicks);

    const PhaseAngle motorAngle = static_cast<PhaseAngle>(std::min(static_cast<uint8_t>(PhaseAngle::Degrees300), static_cast<uint8_t>(_step)));

    if (_step >= kStepCount && _delegate != nullptr) {
        _delegate->controlStrategyDidComplete(*this);
    }

    MotorState nextState(motorAngle, dutyCycle());

    return Commutation(_step < 10 ? nextPhaseLength : 0_tks, nextState);
}

float AlignmentControlStrategy::dutyCycle() const {
    return kAlignmentStartDutyCycle + static_cast<float>(_step) * (kAlignmentEndDutyCycle - kAlignmentStartDutyCycle) / static_cast<float>(kStepCount);
}
