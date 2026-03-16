/*
 * DragControlStrategy.cpp
 *
 * (c) Tom Davie 02/12/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#include "BLDC/MotorConfig.hpp"
#include "BLDC/Strategies/DragConfig.hpp"
#include "BLDC/Strategies/DragControlStrategy.hpp"

#include "ESP32.hpp"

#include <esp_log.h>

#include <cmath>

using namespace bldc;
using namespace esp;

DragControlStrategy::DragControlStrategy(MotorPtr& motor, esp_err_t& err) : MotorControlStrategy(motor) {
    assert(kDragRpmCurve.endX() == kDragDutyCycleCurve.endX());
}

void DragControlStrategy::start(ControlStrategyTransferableState&& state, esp_err_t& err) {
    ESP_LOGI(_loggingTag, "Starting DragControlStrategy");
    _timeInDrag = 0_tk;
    _state = std::move(state);
    _motor->enableADCBiasLearning(true);
}

ControlStrategyTransferableState DragControlStrategy::stop(esp_err_t& err) {
    _motor->enableADCBiasLearning(false);
    return _state;
}

std::optional<Commutation> DragControlStrategy::tick() {
    const Ticks32 nextPhaseLength = _durationInNextPhase();
    static constexpr std::chrono::milliseconds kEndTime(static_cast<uint64_t>(kDragRpmCurve.endX()));

    if (_timeInDrag >= kEndTime && _delegate != nullptr) {
        Tracer::sharedTracer()->sendEvent(TraceEvent::DragEnded);
        _delegate->controlStrategyDidComplete(*this);

        return std::nullopt;
    }

    static constexpr Ticks32 kTenPercentRemainingTime = static_cast<Ticks32>(kEndTime) * rational<uint32_t>(9, 10);
    static constexpr Ticks32 kTenPercentLength = kEndTime - kTenPercentRemainingTime;
    if (_timeInDrag < kTenPercentRemainingTime) {
        _timeInDrag += nextPhaseLength;
        _state._currentStep = static_cast<PhaseAngle>((static_cast<uint8_t>(_state._currentStep) + 1) % 6);
        MotorState nextState(_state._currentStep, dutyCycle());
        return Commutation(nextPhaseLength, nextState);
    }

    const Ticks32 timeThroughLastTenPercent = _timeInDrag - static_cast<Ticks32>(kTenPercentRemainingTime);
    const float p = timeThroughLastTenPercent / kTenPercentLength;

    if (!_motor->isPhaseChangeComplete()) {
        _timeInDrag++;
        return std::nullopt;
    }

    const Ticks16 timeInCurrentStep = _motor->timeInCurrentStep();
    const Ticks16 expectedTimeInCurrentStep = _motor->expectedStepDuration();
    const Ticks16 relativeZeroCrossBlankingPeriod = std::chrono::duration_cast<Ticks16>(kZeroCrossBlankingPeriod * expectedTimeInCurrentStep);
    const Ticks16 absoluteZeroCrossBlankingPeriod = std::chrono::duration_cast<Ticks16>(1000us);
    const Ticks16 zeroCrossBlankingTime = std::max(relativeZeroCrossBlankingPeriod, absoluteZeroCrossBlankingPeriod);
    if (timeInCurrentStep < zeroCrossBlankingTime) {
        _timeInDrag++;
        return std::nullopt;
    }

    const PhaseAngle currentStep = _state._currentStep;
    const bool nextStep = _motor->detectZeroCross();

    if (!nextStep) {
        _timeInDrag++;
        if (timeInCurrentStep >= nextPhaseLength) {
            _state._currentStep = static_cast<PhaseAngle>((static_cast<uint8_t>(_state._currentStep) + 1) % 6);
            MotorState nextState(_state._currentStep, dutyCycle());
            return Commutation(1, nextState);
        }
        return std::nullopt;
    }

    const Ticks16 delay = timeInCurrentStep - 2 * kZeroCrossRepeatTime;
    const Ticks16 timeLeftAccordingToDrag = nextPhaseLength - timeInCurrentStep;
    const Ticks16 mixedTime = delay + std::chrono::duration_cast<Ticks16>((1.0f - p) * (timeLeftAccordingToDrag - static_cast<Ticks32>(delay)));

    _timeInDrag += mixedTime;

    if (nextStep) {
        _state._currentStep = static_cast<PhaseAngle>((static_cast<uint8_t>(currentStep) + 1) % 6);
        MotorState nextState(_state._currentStep, dutyCycle());
        return Commutation(mixedTime, nextState);
    }

    return std::nullopt;
}

Ticks32 DragControlStrategy::_durationInNextPhase() {
    const uint32_t rpm = kDragRpmCurve(static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(_timeInDrag).count()));
    return kTimePerPhaseAt1RPM / rpm;
}

float DragControlStrategy::dutyCycle() const {
    return kDragDutyCycleCurve(static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(_timeInDrag).count()));
}
