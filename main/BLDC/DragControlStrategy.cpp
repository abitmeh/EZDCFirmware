/*
 * DragControlStrategy.cpp
 *
 * (c) Tom Davie 02/12/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#include "BLDC/DragControlStrategy.hpp"
#include "BLDC/MotorConfig.hpp"

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
    _timeInDrag = 0;
    _motor->enableADCBiasLearning(true);
}

void DragControlStrategy::stop(esp_err_t& err) {
    _motor->enableADCBiasLearning(false);
}

NextChange DragControlStrategy::nextStepChange() {
    const uint16_t nextPhaseLength = _nextStepLength();
    _timeInDrag += nextPhaseLength;

    return NextChange(NextStep(nextPhaseLength, static_cast<MotorStep>((_motor->currentStep() + 1) % 6)));
}

uint16_t DragControlStrategy::_nextStepLength() {
    const uint32_t rpm = kDragRpmCurve(_timeInDrag / kTicksPerSecond);
    return kADCRpmCalculationCoefficient / rpm;
}

float DragControlStrategy::dutyCycle() const {
    return kDragDutyCycleCurve(_timeInDrag / kTicksPerSecond);
}

std::optional<ControlPhase> DragControlStrategy::nextControlPhase(ControlPhase currentControlPhase) const {
    return _timeInDrag >= kDragRpmCurve.endX() * kTicksPerSecond ? std::optional<ControlPhase>(ClosedLoop) : std::optional<ControlPhase>();
}
