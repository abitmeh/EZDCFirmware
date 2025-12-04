/*
 * GPTimer.hpp
 *
 * (c) Tom Davie 29/11/2025 
 * 
 */

#pragma once

#include <driver/gptimer.h>

#include <esp_attr.h>

#include <functional>
#include <memory>

namespace esp {
    IRAM_ATTR bool _onAlarm(gptimer_handle_t timerHandle, const gptimer_alarm_event_data_t* eventData, void* userInfo);

    class GPTimer;
    using GPTimerPtr = std::shared_ptr<GPTimer>;

    using GPTimerCallback = std::function<bool(GPTimer&, const gptimer_alarm_event_data_t&)>;

    struct GPTimerConfig {
        uint32_t durationMicroseconds;
        GPTimerCallback callback;
    };

    class GPTimer {
    public:
        GPTimer(const GPTimerConfig& config, esp_err_t& err);
        ~GPTimer();

        void start(esp_err_t& err);
        void stop(esp_err_t& err);

    private:
        gptimer_handle_t _timer;

        IRAM_ATTR bool onAlarm(const gptimer_alarm_event_data_t& eventData);

        GPTimerCallback _callback;

        static constexpr char _loggingTag[] = "esp::GPTimer";

        friend bool _onAlarm(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*);
    };
}  // namespace esp
