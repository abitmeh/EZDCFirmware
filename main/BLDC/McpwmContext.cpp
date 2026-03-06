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
#include "Utilities/Tracer.hpp"

#include "ESP32.hpp"

#include <esp_log.h>

#include <freertos/FreeRTOS.h>

using namespace bldc;
using namespace esp;
using namespace mcpwm;

McpwmContext::McpwmContext(const McpwmConfig& config, esp_err_t& err) {
    err = ESP_OK;

    const TimerConfig timerConfig = {
        .groupId = config._groupId, .frequency = kMcpwmFrequency, .period = kMcpwmPeriod, .countMode = MCPWM_TIMER_COUNT_MODE_UP_DOWN};

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

        _generators[i]->setActionsOnCompareEvent({Generator::CompareEventAction(TimerDirection::Up, _comparators[i], GeneratorAction::Low),
                                                  Generator::CompareEventAction(TimerDirection::Down, _comparators[i], GeneratorAction::High)},
                                                 err);
        if (err != ESP_OK) {
            ESP_LOGE(_loggingTag, "Generator::setActionsOnCompareEvent failed: %s", esp_err_to_name(err));
            return;
        }
    }
}

McpwmContext::McpwmContext(McpwmContext&& other)
    : _timer(other._timer),
      _operators(std::move(other._operators)),
      _comparators(std::move(other._comparators)),
      _generators(std::move(other._generators)) {}

McpwmContext& McpwmContext::operator=(McpwmContext&& other) {
    _timer = other._timer;
    _operators = std::move(other._operators);
    _comparators = std::move(other._comparators);
    _generators = std::move(other._generators);
    return *this;
}

esp_err_t McpwmContext::start() {
    esp_err_t err = ESP_OK;
    _timer->enable(err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "Timer::enable failed: %s", esp_err_to_name(err));
        return err;
    }

    _timer->start(Timer::StartCommand::NoStop, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "Timer::start failed: %s", esp_err_to_name(err));
        return err;
    }

    return err;
}

esp_err_t McpwmContext::setDutyCycle(MotorPhase phase, uint32_t dutyCycle) {
    esp_err_t err = ESP_OK;
    _comparators[static_cast<uint8_t>(phase)]->setCompareValue(dutyCycle, err);
    _generators[static_cast<uint8_t>(phase)]->clearLevel(err);
    return err;
}

esp_err_t McpwmContext::setGpioValue(MotorPhase phase, bool value) {
    esp_err_t err = ESP_OK;
    _generators[static_cast<uint8_t>(phase)]->setLevel(value ? Level::High : Level::Low, true, err);
//    _comparators[static_cast<uint8_t>(phase)]->setCompareValue(value ? kMaxDutyCycle : 0, err);
    return err;
}

esp_err_t McpwmContext::setTimerEventCallback(esp::mcpwm::TimerEvent timerEvent, esp::mcpwm::Timer::EventCallback callback, void* userInfo) {
    esp_err_t err = ESP_OK;
    switch (timerEvent) {
        case TimerEvent::Empty:
            _callbacks.onEmpty = callback;
            _timer->setEventCallbacks(_callbacks, userInfo, err);
            break;
        case TimerEvent::Full:
            _callbacks.onFull = callback;
            _timer->setEventCallbacks(_callbacks, userInfo, err);
            break;
        default:
            break;
    }
    return err;
}

esp_err_t McpwmContext::setComparatorCallback(uint8_t comparatorIndex, esp::mcpwm::ComparatorCallback callback, void* userInfo) {
    if (comparatorIndex >= kMaxMcpwmComparators) {
        ESP_LOGE(_loggingTag, "Comparator index out of range: %u", comparatorIndex);
        return ESP_ERR_INVALID_ARG;
    }

    return _comparators[comparatorIndex]->setCallback(callback, userInfo);
}

