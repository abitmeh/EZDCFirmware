/*
 * McpwmContext.hpp
 *
 * (c) Tom Davie 28/11/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#pragma once

#include "BLDC/Types.hpp"

#include "MCPWM/MCPWM.hpp"

#include <array>
#include <functional>
#include <optional>

namespace bldc {
    static constexpr size_t kMaxMcpwmComparators = 3;

    class McpwmContext;

    struct McpwmConfig {
        McpwmConfig() {}

        McpwmConfig(uint8_t groupId) : _groupId(groupId) {}

        McpwmConfig(uint8_t groupId, gpio_num_t uPin, gpio_num_t vPin, gpio_num_t wPin) : _groupId(groupId), _outputGpios({uPin, vPin, wPin}) {}

        uint8_t _groupId = 0;
        std::array<gpio_num_t, 3> _outputGpios = {GPIO_NUM_0, GPIO_NUM_0, GPIO_NUM_0};
    };

    class McpwmContext {
    public:
        McpwmContext(const McpwmConfig& config, esp_err_t& err);
        McpwmContext(const McpwmContext& other) = delete;
        McpwmContext(McpwmContext&& other);

        McpwmContext& operator=(const McpwmContext& other) = delete;
        McpwmContext& operator=(McpwmContext&& other);

        esp_err_t setDutyCycle(MotorPhase phase, uint32_t dutyCycle);
        esp_err_t setGpioValue(MotorPhase phase, bool value);

        esp_err_t setTimerEventCallback(esp::mcpwm::TimerEvent timerEvent, esp::mcpwm::Timer::EventCallback callback, void* userInfo);

        esp_err_t setComparatorCallback(uint8_t comparatorIndex, esp::mcpwm::ComparatorCallback callback, void* userInfo);

        esp_err_t start();

    private:
        esp::mcpwm::TimerPtr _timer;
        std::array<esp::mcpwm::OperatorPtr, kMaxMcpwmComparators> _operators;
        std::array<esp::mcpwm::ComparatorPtr, kMaxMcpwmComparators> _comparators;
        std::array<esp::mcpwm::GeneratorPtr, kMaxMcpwmComparators> _generators;
        esp::mcpwm::Timer::EventCallbacks _callbacks;

        static constexpr char _loggingTag[] = "bldc::McpwmContext";

        friend bool onFull(mcpwm_timer_handle_t timerHandle, const mcpwm_timer_event_data_t* eventData, void* userData);
        friend bool onEmpty(mcpwm_timer_handle_t timerHandle, const mcpwm_timer_event_data_t* eventData, void* userData);
        friend bool onStop(mcpwm_timer_handle_t timerHandle, const mcpwm_timer_event_data_t* eventData, void* userData);
    };
}  // namespace bldc
