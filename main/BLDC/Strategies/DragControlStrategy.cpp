/*
 * DragControlStrategy.cpp
 *
 * (c) Tom Davie 02/12/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#include "BLDC/Strategies/DragControlStrategy.hpp"
#include "BLDC/MotorConfig.hpp"
#include "BLDC/Strategies/DragConfig.hpp"

#include "ESP32.hpp"

#include <esp_log.h>

#include <cmath>

using namespace bldc;
using namespace esp;

DragControlStrategy::DragControlStrategy(MotorPtr& motor, esp_err_t& err) : MotorControlStrategy(motor) {
    assert(kDragRpmCurve.endX() == kDragDutyCycleCurve.endX());
}

void DragControlStrategy::start(esp_err_t& err) {
    ESP_LOGD(_loggingTag, "Starting DragControlStrategy");
    _timeInDrag = 0_tk;
    _motor->enableADCBiasLearning(true);
}

void DragControlStrategy::stop(esp_err_t& err) {
    _motor->enableADCBiasLearning(false);
}

NextChange DragControlStrategy::nextStepChange() {
    const Ticks32 nextPhaseLength = _durationInNextPhase();
    _timeInDrag += nextPhaseLength;

    return NextChange(NextStep(nextPhaseLength, static_cast<PhaseAngle>((_motor->currentStep() + 1) % 6)));
}

Ticks32 DragControlStrategy::_durationInNextPhase() {
    const uint32_t rpm = kDragRpmCurve(static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(_timeInDrag).count()));
    return kTimePerPhaseAt1RPM / rpm;
}

float DragControlStrategy::dutyCycle() const {
    return kDragDutyCycleCurve(static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(_timeInDrag).count()));
}

std::optional<ControlMode> DragControlStrategy::nextControlMode(ControlMode currentControlMode) const {
    return _timeInDrag >= std::chrono::milliseconds(static_cast<uint64_t>(kDragRpmCurve.endX())) ? std::optional<ControlMode>(ClosedLoop) : std::nullopt;
}
