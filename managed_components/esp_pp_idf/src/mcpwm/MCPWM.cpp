/*
 * MCPWM.cpp
 *
 * (c) Tom Davie 03/12/2025 
 * 
 */

#include "mcpwm/MCPWM.hpp"

#include <esp_log.h>

using namespace esp;
using namespace mcpwm;

TimerPtr MCPWM::timer(const TimerConfig& timerConfig, esp_err_t& err) {
    TimerPtr timer = std::shared_ptr<Timer>(new Timer(timerConfig, err));

    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "esp::mcpwm::Timer::Timer failed: %s", esp_err_to_name(err));
        return nullptr;
    }
    if (timer == nullptr) {
        err = ESP_FAIL;
        ESP_LOGE(_loggingTag, "esp::mcpwm::Timer::Timer failed: %s", esp_err_to_name(err));
        return nullptr;
    }

    return timer;
}

GPIOFaultPtr MCPWM::gpioFault(const GPIOFaultConfig& faultConfig, esp_err_t& err) {
    GPIOFaultPtr fault = std::shared_ptr<GPIOFault>(new GPIOFault(faultConfig, err));

    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "esp::mcpwm::GPIOFault::GPIOFault failed: %s", esp_err_to_name(err));
        return nullptr;
    }
    if (fault == nullptr) {
        err = ESP_FAIL;
        ESP_LOGE(_loggingTag, "esp::mcpwm::GPIOFault::GPIOFault failed: %s", esp_err_to_name(err));
        return nullptr;
    }

    return fault;
}