/*
 * GPIOFault.cpp
 *
 * (c) Tom Davie 03/12/2025 
 * 
 */

#include "mcpwm/GPIOFault.hpp"

#include <esp_log.h>

using namespace esp;
using namespace mcpwm;

namespace esp::mcpwm {
    bool IRAM_ATTR _onFaultEnter(mcpwm_fault_handle_t faultHandle, const mcpwm_fault_event_data_t* faultData, void* userInfo) {
        GPIOFault* fault = reinterpret_cast<GPIOFault*>(userInfo);
        return fault->_callbacks.onFaultEnter(*faultData);
    }

    bool IRAM_ATTR _onFaultExit(mcpwm_fault_handle_t faultHandle, const mcpwm_fault_event_data_t* faultData, void* userInfo) {
        GPIOFault* fault = reinterpret_cast<GPIOFault*>(userInfo);
        return fault->_callbacks.onFaultExit(*faultData);
    }
}  // namespace esp::mcpwm

GPIOFault::GPIOFault(const GPIOFaultConfig& config, esp_err_t& err) {
    mcpwm_gpio_fault_config_t faultConfig = {
        .group_id = config.groupId,
        .intr_priority = config.interuptPriority,
        .gpio_num = config.gpioNum,
        .flags = {.active_level = config.activeHigh, .io_loop_back = config.ioLoopBack, .pull_up = config.pullUp, .pull_down = config.pullDown}};
    err = mcpwm_new_gpio_fault(&faultConfig, &_fault);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_new_gpio_fault failed: %s", esp_err_to_name(err));
        return;
    }
}

GPIOFault::~GPIOFault() {
    esp_err_t err = ESP_OK;
    err = mcpwm_del_fault(_fault);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_del_fault failed: %s", esp_err_to_name(err));
        return;
    }
}

void GPIOFault::setCallbacks(const Callbacks& callbacks, esp_err_t& err) {
    _callbacks = callbacks;

    mcpwm_fault_event_callbacks_t faultCallbacks = {.on_fault_enter = callbacks.onFaultEnter ? _onFaultEnter : nullptr,
                                                    .on_fault_exit = callbacks.onFaultExit ? _onFaultExit : nullptr};
    err = mcpwm_fault_register_event_callbacks(_fault, &faultCallbacks, this);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_fault_register_event_callbacks failed: %s", esp_err_to_name(err));
        return;
    }
}

