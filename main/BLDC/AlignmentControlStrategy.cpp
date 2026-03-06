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

static const size_t kStepCount = 10;
static const float kStepFraction = 1.0 / static_cast<float>(kStepCount);

AlignmentControlStrategy::AlignmentControlStrategy(MotorPtr& motor) : MotorControlStrategy(motor) {}

void AlignmentControlStrategy::start(esp_err_t& err) {
    ESP_LOGD(_loggingTag, "Starting AlignmentControlStrategy");
}

NextChange AlignmentControlStrategy::nextStepChange() {
    const uint32_t currentTicks = static_cast<float>(_step) * kStepFraction * kAlignmentTimeMs * kTicksPerSecond * 0.001;
    _step++;
    const float t = static_cast<float>(_step) * kStepFraction;
    const uint32_t nextTicks = t * kAlignmentTimeMs * kTicksPerSecond * 0.001;

    const uint16_t nextPhaseLength = nextTicks - currentTicks;

    const MotorStep motorAngle = static_cast<MotorStep>(std::min(static_cast<uint8_t>(MotorStep::Step5), static_cast<uint8_t>(_step)));
    const NextChange nextChange(NextStep(_step < 10 ? nextPhaseLength : 0, motorAngle));
    return nextChange;
}

float AlignmentControlStrategy::dutyCycle() const {
    return kAlignmentStartDutyCycle + static_cast<float>(_step) * kStepFraction * (kAlignmentEndDutyCycle - kAlignmentStartDutyCycle);
}

std::optional<ControlPhase> AlignmentControlStrategy::nextControlPhase(ControlPhase currentControlPhase) const {
    return _step >= kStepCount ? std::optional<ControlPhase>(Drag) : std::optional<ControlPhase>();
}
