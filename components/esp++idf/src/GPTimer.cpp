/*
 * GPTimer.cpp
 *
 * (c) Tom Davie 29/11/2025 
 * 
 */

#include "GPTimer.hpp"

#include <esp_log.h>

using namespace esp;

namespace esp {
    bool _onAlarm(gptimer_handle_t timerHandle, const gptimer_alarm_event_data_t* eventData, void* userInfo) {
        GPTimer* timer = reinterpret_cast<GPTimer*>(userInfo);
        return timer->onAlarm(*eventData);
    }
}  // namespace esp

GPTimer::GPTimer(const GPTimerConfig& config, esp_err_t& err) {
    _callback = config.callback;

    gptimer_config_t timerConfig = {.clk_src = GPTIMER_CLK_SRC_DEFAULT,
                                    .direction = GPTIMER_COUNT_UP,
                                    .resolution_hz = 1'000'000,
                                    .intr_priority = 0,
                                    .flags = {
                                        .intr_shared = false,
                                        .allow_pd = false,
                                        .backup_before_sleep = false,
                                    }};
    err = gptimer_new_timer(&timerConfig, &_timer);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "gptimer_new_timer failed: %s", esp_err_to_name(err));
        return;
    }

    gptimer_event_callbacks_t callbacks = {.on_alarm = esp::_onAlarm};
    err = gptimer_register_event_callbacks(_timer, &callbacks, this);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "gptimer_register_event_callbacks failed: %s", esp_err_to_name(err));
        return;
    }

    gptimer_alarm_config_t alarmConfig = {.alarm_count = config.durationMicroseconds, .reload_count = 0, .flags = {.auto_reload_on_alarm = true}};
    err = gptimer_set_alarm_action(_timer, &alarmConfig);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "gptimer_set_alarm_action failed: %s", esp_err_to_name(err));
        return;
    }

    err = gptimer_enable(_timer);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "gptimer_enable failed: %s", esp_err_to_name(err));
        return;
    }
}

GPTimer::~GPTimer() {
    esp_err_t err = ESP_OK;
    err = gptimer_stop(_timer);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "gptimer_stop failed: %s", esp_err_to_name(err));
    }

    err = gptimer_disable(_timer);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "gptimer_disable failed: %s", esp_err_to_name(err));
    }

    err = gptimer_del_timer(_timer);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "gptimer_del_timer failed: %s", esp_err_to_name(err));
    }
}

void GPTimer::start(esp_err_t& err) {
    err = gptimer_start(_timer);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "gptimer_start failed: %s", esp_err_to_name(err));
    }
}

void GPTimer::stop(esp_err_t& err) {
    err = gptimer_stop(_timer);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "gptimer_stop failed: %s", esp_err_to_name(err));
    }
}

bool GPTimer::onAlarm(const gptimer_alarm_event_data_t& eventData) {
    if (_callback) {
        return _callback(*this, eventData);
    } else {
        return false;
    }
}
