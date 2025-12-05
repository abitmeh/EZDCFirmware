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

using namespace bldc;
using namespace esp;

DragControlStrategy::DragControlStrategy(Motor& motor, esp_err_t& err) : MotorControlStrategy(motor) {}

NextChange DragControlStrategy::nextStepChange() {
    const uint16_t nextPhaseLength = _nextStepLength();
    _motor.setTimeInCurrentStep(0);
    _timeInDrag += nextPhaseLength;
    _proportionThroughDrag = static_cast<float>(_timeInDrag) / kDragDuration;
    return NextChange(NextStep(nextPhaseLength, static_cast<MotorStep>((_motor.currentStep() + 1) % 6)));
}

uint16_t DragControlStrategy::_nextStepLength() {
    const uint32_t rpm = kDragRPMAtRampStart + ((int32_t)kDragRPMAtRampEnd - (int32_t)kDragRPMAtRampStart) * _proportionThroughDrag;
    return kADCRpmCalculationCoefficient / rpm;
}

uint32_t DragControlStrategy::dutyCycle() const {
    return kDragDutyCycleAtRampStart + ((int32_t)kDragDutyCycleAtRampEnd - (int32_t)kDragDutyCycleAtRampStart) * _proportionThroughDrag;
}

std::optional<ControlPhase> DragControlStrategy::nextControlPhase(ControlPhase currentControlPhase) const {
    return _proportionThroughDrag >= 1.0 ? std::optional<ControlPhase>(ClosedLoop) : std::optional<ControlPhase>();
}
