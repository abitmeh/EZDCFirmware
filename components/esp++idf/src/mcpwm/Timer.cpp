/*
 * Timer.cpp
 *
 * (c) Tom Davie 03/12/2025 
 * 
 */

#include "mcpwm/Timer.hpp"
#include "mcpwm/Operator.hpp"

#include <esp_log.h>

using namespace esp;
using namespace mcpwm;

namespace esp::mcpwm {
    IRAM_ATTR bool _onFull(mcpwm_timer_handle_t timerHandle, const mcpwm_timer_event_data_t* eventData, void* userData) {
        Timer* timer = reinterpret_cast<Timer*>(userData);
        return timer->_callbacks.onFull(*eventData);
    }

    IRAM_ATTR bool _onEmpty(mcpwm_timer_handle_t timerHandle, const mcpwm_timer_event_data_t* eventData, void* userData) {
        Timer* timer = reinterpret_cast<Timer*>(userData);
        return timer->_callbacks.onEmpty(*eventData);
    }

    IRAM_ATTR bool _onStop(mcpwm_timer_handle_t timerHandle, const mcpwm_timer_event_data_t* eventData, void* userData) {
        Timer* timer = reinterpret_cast<Timer*>(userData);
        return timer->_callbacks.onStop(*eventData);
    }
}  // namespace esp::mcpwm

Timer::Timer(const TimerConfig& config, esp_err_t& err) : _config(config) {
    mcpwm_timer_config_t timerConfig = {
        .group_id = config.groupId,
        .clk_src = config.clockSource,
        .resolution_hz = config.frequency,
        .count_mode = config.countMode,
        .period_ticks = config.periodTicks,
        .intr_priority = config.interuptPriority,
        .flags = {.update_period_on_empty = config.updatePeriodOnEmpty, .update_period_on_sync = config.updatePeriodOnSync, .allow_pd = config.allowPowerDown}};

    err = mcpwm_new_timer(&timerConfig, &_timer);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_new_timer failed: %s", esp_err_to_name(err));
        return;
    }
}

Timer::~Timer() {
    esp_err_t err = ESP_OK;

    disable(err);
    if (err) {
        ESP_LOGE(_loggingTag, "mcpwm::Timer::disable failed: %s", esp_err_to_name(err));
    }

    err = mcpwm_del_timer(_timer);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_del_timer failed: %s", esp_err_to_name(err));
        return;
    }
}

OperatorPtr Timer::addOperator(const OperatorConfig& config, esp_err_t& err) {
    // Can't use std::make_shared because we only have access through friendship
    OperatorPtr result = std::shared_ptr<Operator>(new Operator(shared_from_this(), config, err));
    if (err) {
        ESP_LOGE(_loggingTag, "mcpwm::Operator::Operator failed: %s", esp_err_to_name(err));
        return nullptr;
    }
    if (result == nullptr) {
        err = ESP_FAIL;
        ESP_LOGE(_loggingTag, "mcpwm::Operator::Operator failed: %s", esp_err_to_name(err));
        return nullptr;
    }
    return result;
}

void Timer::setEventCallbacks(const EventCallbacks& eventCallbacks, esp_err_t& err) {
    mcpwm_timer_event_callbacks_t callbackConfig = {
        .on_full = eventCallbacks.onFull ? _onFull : nullptr,
        .on_empty = eventCallbacks.onEmpty ? _onEmpty : nullptr,
        .on_stop = eventCallbacks.onStop ? _onStop : nullptr,
    };
    _callbacks = eventCallbacks;
    err = mcpwm_timer_register_event_callbacks(_timer, &callbackConfig, this);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_timer_register_event_callbacks failed: %s", esp_err_to_name(err));
        return;
    }
}

void Timer::disable(esp_err_t& err) {
    if (!_enabled) {
        return;
    }

    err = mcpwm_timer_disable(_timer);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_timer_disable failed: %s", esp_err_to_name(err));
        return;
    }

    _enabled = false;
}

void Timer::enable(esp_err_t& err) {
    if (_enabled) {
        return;
    }

    err = mcpwm_timer_enable(_timer);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_timer_enable failed: %s", esp_err_to_name(err));
        return;
    }

    _enabled = true;
}

mcpwm_timer_start_stop_cmd_t startStopCommand(Timer::StartCommand cmd) {
    switch (cmd) {
        case Timer::StartCommand::NoStop:
            return MCPWM_TIMER_START_NO_STOP;
        case Timer::StartCommand::StopEmpty:
            return MCPWM_TIMER_START_STOP_EMPTY;
        case Timer::StartCommand::StopFull:
            return MCPWM_TIMER_START_STOP_FULL;
    }
    // GCC...
    return MCPWM_TIMER_START_NO_STOP;
}

mcpwm_timer_start_stop_cmd_t startStopCommand(Timer::StopCommand cmd) {
    switch (cmd) {
        case Timer::StopCommand::StopEmpty:
            return MCPWM_TIMER_STOP_EMPTY;
        case Timer::StopCommand::StopFull:
            return MCPWM_TIMER_STOP_FULL;
    }
    // GCC...
    return MCPWM_TIMER_STOP_EMPTY;
}

void Timer::start(StartCommand cmd, esp_err_t& err) {
    err = mcpwm_timer_start_stop(_timer, startStopCommand(cmd));
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_timer_start_stop failed: %s", esp_err_to_name(err));
        return;
    }
}

void Timer::stop(StopCommand cmd, esp_err_t& err) {
    err = mcpwm_timer_start_stop(_timer, startStopCommand(cmd));
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_timer_start_stop failed: %s", esp_err_to_name(err));
        return;
    }
}
