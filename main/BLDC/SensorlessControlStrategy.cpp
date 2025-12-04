/*
 * SensorlessControlStrategy.cpp
 *
 * (c) Tom Davie 2/12/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#include "BLDC/SensorlessControlStrategy.hpp"
#include "BLDC/MotorConfig.hpp"

#include "ESP32.hpp"

#include <bldc_snls_lib.h>
#include <esp_log.h>

#include <ranges>

using namespace bldc;
using namespace esp;

SensorlessControlStrategy::SensorlessControlStrategy(const SensorlessControlConfig& adcConfig, Motor& motor, esp_err_t& err) : MotorControlStrategy(motor) {
    for (auto [i, channelConfig] : adcConfig | std::views::enumerate) {
        ADCOneshotPtr adcUnit = ESP32::sharedESP32()->adcOneshot(channelConfig.first, err);
        if (adcUnit == nullptr || err != ESP_OK) {
            ESP_LOGE(_loggingTag, "ADCOneshot creation failed: %s", esp_err_to_name(err));
            return;
        }

        ADCOneshotChannelPtr adcChannel = adcUnit->channel(channelConfig.second, err);
        if (adcChannel == nullptr || err != ESP_OK) {
            ESP_LOGE(_loggingTag, "adcChannel creation failed: %s", esp_err_to_name(err));
            return;
        }
        _adcs[i] = adcChannel;
    }

    err = pid_new_control_block(&kPidControlConfig, &_pid);
    if (err != ESP_OK || _pid == nullptr) {
        ESP_LOGE(_loggingTag, "pid_new_control_block failed: %d", err);
        err = (err != ESP_OK) ? err : ESP_FAIL;
        return;
    }
}

void IRAM_ATTR SensorlessControlStrategy::mcpwmTimerFull() {
    _adcValues[1] = 1700;
    _adcValues[0] = _adcs[_motor.highImpedencePhase()]->readIsr();
    if (_adcValues[0] > _maxObservedValue) {
        _maxObservedValue = _adcValues[0];
    }
}

void SensorlessControlStrategy::start(esp_err_t& err) {
    _calculateSpeed = false;
    _timeSpentAvoidingContinuousCurrent = 0;

    if (_pid == nullptr) {
        ESP_LOGE(_loggingTag, "PID not initialized in closed loop mode");
        err = ESP_ERR_INVALID_STATE;
        return;
    }
    pid_reset_ctrl_block(_pid);
}

NextChange SensorlessControlStrategy::nextStepChange() {
    if (!_motor.isPhaseChangeComplete()) {
        return NextChange();
    }

    const uint16_t timeInCurrentStep = _motor.timeInCurrentStep();
    if (timeInCurrentStep < kZeroCrossAvoidContinuousCurrentTime) {
        return NextChange();
    }

    const MotorStep currentStep = _motor.currentStep();
    const MotorStep nextStep = static_cast<MotorStep>(zero_cross_adc_get_phase(currentStep, kZeroCrossRepeatTime, _adcValues.data()));

    if (nextStep == currentStep) {
        return NextChange();
    }

    const uint16_t delay = timeInCurrentStep * 3.0 / kZeroCrossAdvance;

    return NextChange(NextStep(delay, nextStep));
}

uint32_t SensorlessControlStrategy::dutyCycle() const {
    float duty = 0;
    int32_t a = _motor.targetRPM() - _motor.currentRPM();
    pid_compute(_pid, (float)(a), &duty);
    return abs((int)duty);
}

void SensorlessControlStrategy::setPIDParameters(const pid_ctrl_parameter_t* parameters, esp_err_t& err) {
    err = pid_update_parameters(_pid, parameters);
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
