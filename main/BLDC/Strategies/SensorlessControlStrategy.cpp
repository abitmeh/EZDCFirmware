/*
 * SensorlessControlStrategy.cpp
 *
 * (c) Tom Davie 2/12/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#include "BLDC/Strategies/SensorlessConfig.hpp"
#include "BLDC/Strategies/SensorlessControlStrategy.hpp"
#include "Utilities/Tracer.hpp"

#include <esp_log.h>

#include <ranges>

using namespace bldc;
using namespace esp;
using namespace esp::adc;

SensorlessControlStrategy::SensorlessControlStrategy(MotorPtr& motor, esp_err_t& err) : MotorControlStrategy(motor) {
    _pidParameters = kPidControlConfig.init_param;
    err = pid_new_control_block(&kPidControlConfig, &_pid);
    if (err != ESP_OK || _pid == nullptr) {
        ESP_LOGE(_loggingTag, "pid_new_control_block failed: %d", err);
        err = (err != ESP_OK) ? err : ESP_FAIL;
        return;
    }
}

void SensorlessControlStrategy::start(ControlStrategyTransferableState&& state, esp_err_t& err) {
    ESP_LOGD(_loggingTag, "Starting SensorlessControlStrategy");

    _state = std::move(state);

    _motor->enableADCBiasLearning(true);

    _calculateSpeed = false;
    _timeSpentAvoidingContinuousCurrent = 0;

    if (_pid == nullptr) {
        ESP_LOGE(_loggingTag, "PID not initialized in closed loop mode");
        err = ESP_ERR_INVALID_STATE;
        return;
    }

    err = pid_reset_ctrl_block(_pid);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "pid_reset_ctrl_block failed: %s", esp_err_to_name(err));
        return;
    }
    const float error = _motor->targetRPM() - _motor->currentRPM();
    const float kp = _pidParameters.kp;
    const float kd = _pidParameters.kd;
    const float transitionIntegral = static_cast<float>(_motor->dutyCycle()) / static_cast<float>(kMaxDutyCycle) - (kp * error + kd * error);
    float result;
    err = pid_compute(_pid, transitionIntegral, &result);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "pid_compute failed: %s", esp_err_to_name(err));
        return;
    }

    Tracer::sharedTracer()->sendEvent(TraceEvent::StartEnded);
}

ControlStrategyTransferableState SensorlessControlStrategy::stop(esp_err_t& err) {
    _motor->enableADCBiasLearning(false);

    return _state;
}

std::optional<Commutation> SensorlessControlStrategy::tick() {
    if (!_motor->isPhaseChangeComplete()) {
        return std::nullopt;
    }

    const Ticks16 timeInCurrentStep = _motor->timeInCurrentStep();
    const Ticks16 expectedTimeInCurrentStep = _motor->expectedStepDuration();
    const Ticks16 relativeZeroCrossBlankingPeriod = std::chrono::duration_cast<Ticks16>(kZeroCrossBlankingPeriod * expectedTimeInCurrentStep);
    const Ticks16 absoluteZeroCrossBlankingPeriod = std::chrono::duration_cast<Ticks16>(1000us);
    const Ticks16 zeroCrossBlankingTime = std::max(relativeZeroCrossBlankingPeriod, absoluteZeroCrossBlankingPeriod);
    if (timeInCurrentStep < zeroCrossBlankingTime) {
        return std::nullopt;
    }

    if (timeInCurrentStep > kStallPeriod && _delegate != nullptr) {
        _delegate->controlStrategyMotorDidStall(*this);
    }

    const PhaseAngle currentStep = _state._currentStep;
    const bool nextStep = _motor->detectZeroCross();

    if (!nextStep) {
        return std::nullopt;
    }

    const Ticks16 delay = timeInCurrentStep - 2 * kZeroCrossRepeatTime;

    MotorState nextState(static_cast<PhaseAngle>((static_cast<uint8_t>(currentStep) + (nextStep ? 1 : 0)) % 6), dutyCycle());
    return Commutation(delay, nextState);
}

float SensorlessControlStrategy::dutyCycle() const {
    float duty = 0;
    float a = static_cast<float>(_motor->targetRPM()) - static_cast<float>(_motor->currentRPM());
    esp_err_t err = pid_compute(_pid, static_cast<float>(a), &duty);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "pid_compute failed: %s", esp_err_to_name(err));
        return 0.0f;
    }

    return duty;
}

void SensorlessControlStrategy::setPIDParameters(const pid_ctrl_parameter_t& parameters, esp_err_t& err) {
    _pidParameters = parameters;

    err = pid_update_parameters(_pid, &parameters);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "pid_update_parameters failed: %s", esp_err_to_name(err));
        return;
    }

    err = pid_reset_ctrl_block(_pid);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "pid_reset_ctrl_block failed: %s", esp_err_to_name(err));
        return;
    }
}
