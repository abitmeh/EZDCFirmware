/*
 * McpwmContext.cpp
 *
 * (c) Tom Davie 28/11/2025
 *
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 */

#include "BLDC/McpwmContext.hpp"
#include "BLDC/MotorConfig.hpp"

#include "ESP32.hpp"

#include <esp_log.h>

using namespace bldc;
using namespace esp;
using namespace mcpwm;

void McpwmConfig::configureFaultHandling(gpio_num_t faultPin, bool faultInverted, GPIOFault::Callback faultCallback) {
    _faultGpio = faultPin;
    _faultInverted = faultInverted;
    _faultCallback = faultCallback;
}

McpwmContext::McpwmContext(const McpwmConfig& config, esp_err_t& err) {
    err = ESP_OK;

    const TimerConfig timerConfig = {
        .groupId = config._groupId, .frequency = kMcpwmFrequency, .countMode = MCPWM_TIMER_COUNT_MODE_UP_DOWN, .periodTicks = kMcpwmPeriod};

    _timer = ESP32::sharedESP32()->mcpwm().timer(timerConfig, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "MCPWM::timer failed: %s", esp_err_to_name(err));
        return;
    }

    const OperatorConfig operatorConfig;
    for (uint8_t i = 0; i < kMaxMcpwmComparators; ++i) {
        _operators[i] = _timer->addOperator(operatorConfig, err);
        if (err != ESP_OK) {
            ESP_LOGE(_loggingTag, "Timer::addOperator failed: %s", esp_err_to_name(err));
            return;
        }
    }

    const ComparatorConfig comparatorConfig = {.updateComparatorOnTimerZero = true};
    for (uint8_t i = 0; i < kMaxMcpwmComparators; i++) {
        _comparators[i] = _operators[i]->addComparator(comparatorConfig, err);
        if (err != ESP_OK) {
            ESP_LOGE(_loggingTag, "Operator::addComparator failed: %s", esp_err_to_name(err));
            return;
        }
    }

    GeneratorConfig generatorConfig;
    for (uint8_t i = 0; i < kMaxMcpwmComparators; i++) {
        generatorConfig.gpioNum = config._outputGpios[i];
        _generators[i] = _operators[i]->addGenerator(generatorConfig, err);
        if (err != ESP_OK) {
            ESP_LOGE(_loggingTag, "Operator::addGenerator failed: %s", esp_err_to_name(err));
            return;
        }

        _generators[i]->setActionsOnCompareEvent({Generator::CompareEventAction(MCPWM_TIMER_DIRECTION_UP, _comparators[i], MCPWM_GEN_ACTION_LOW),
                                                  Generator::CompareEventAction(MCPWM_TIMER_DIRECTION_DOWN, _comparators[i], MCPWM_GEN_ACTION_HIGH)},
                                                 err);
        if (err != ESP_OK) {
            ESP_LOGE(_loggingTag, "Generator::setActionsOnCompareEvent failed: %s", esp_err_to_name(err));
            return;
        }
    }

    _timer->setEventCallbacks(config._callbacks, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "Timer::setEventCallbacks failed: %s", esp_err_to_name(err));
        return;
    }

    _timer->enable(err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "Timer::enable failed: %s", esp_err_to_name(err));
        return;
    }

    _timer->start(Timer::StartCommand::NoStop, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "Timer::start failed: %s", esp_err_to_name(err));
        return;
    }

    const GPIOFaultConfig faultConfig = {.groupId = config._groupId,
                                         .gpioNum = config._faultGpio,
                                         .activeHigh = !config._faultInverted,
                                         .pullUp = config._faultInverted,
                                         .pullDown = !config._faultInverted};
    _faultHandle = ESP32::sharedESP32()->mcpwm().gpioFault(faultConfig, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "MCPWM::gpioFault failed: %s", esp_err_to_name(err));
        return;
    }

    GPIOFault::Callbacks faultCallbacks = {.onFaultEnter = config._faultCallback, .onFaultExit = nullptr};
    _faultHandle->setCallbacks(faultCallbacks, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "GPIOFault::setCallbacks failed: %s", esp_err_to_name(err));
        return;
    }
}

McpwmContext::McpwmContext(McpwmContext&& other)
    : _timer(other._timer),
      _operators(std::move(other._operators)),
      _comparators(std::move(other._comparators)),
      _generators(std::move(other._generators)),
      _faultHandle(other._faultHandle),
      _faultCallback(std::move(other._faultCallback)) {}

McpwmContext& McpwmContext::operator=(McpwmContext&& other) {
    _timer = other._timer;
    _operators = std::move(other._operators);
    _comparators = std::move(other._comparators);
    _generators = std::move(other._generators);
    _faultHandle = other._faultHandle;
    _faultCallback = std::move(other._faultCallback);
    return *this;
}

esp_err_t McpwmContext::setDutyCycle(MotorPhase phase, uint32_t dutyCycle) {
    esp_err_t err = ESP_OK;
    _comparators[static_cast<uint8_t>(phase)]->setCompareValue(dutyCycle, err);
    return err;
}

esp_err_t McpwmContext::setGpioValue(MotorPhase phase, bool value) {
    esp_err_t err = ESP_OK;
    _comparators[static_cast<uint8_t>(phase)]->setCompareValue(value ? kMaxDutyCycle : 0, err);
    return err;
}

