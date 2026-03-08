/*
 * MotorController.hpp
 *
 * (c) Tom Davie 30/11/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#pragma once

#include "BLDC/McpwmContext.hpp"
#include "BLDC/Motor.hpp"
#include "BLDC/Strategies/MotorControlStrategy.hpp"
#include "BLDC/Types.hpp"

#include "GPIO.hpp"
#include "GPTimer.hpp"
#include "Interrupt.hpp"

#include <esp_event.h>

#include <optional>

namespace bldc {
    struct MotorControlConfig {
        bldc::MotorConfig motorConfig;

        gpio_num_t sleepGPIONum;
        bool sleepValue;
    };

    void _controlTask(void* userInfo);
    esp::InterruptResult _timerCallback(esp::GPTimer& timer, const gptimer_alarm_event_data_t& eventData, void* userInfo);

    class MotorController {
    public:
        MotorController(const MotorControlConfig& config, esp_err_t& err);
        ~MotorController();

        void configureMotorFaultHandling(gpio_num_t gpio, bool inverted, esp::mcpwm::GPIOFault::Callback callback);

        void start(uint32_t targetRPM, esp_err_t& err);
        void stop(esp_err_t& err);

        Direction direction() const;
        void setDirection(Direction direction);

        float dutyCycle() const;

        uint32_t rpm() const;
        uint32_t targetRPM() const;
        void setTargetRPM(uint32_t targetRPM);

    private:
        void _controlTask();

        void _tick();

        void _setControlMode(ControlMode phase, esp_err_t& err);
        bool _checkForStall(esp_err_t& err);

        esp::GPTimerPtr _timer;
        SemaphoreHandle_t _timerFiredSemaphore;

        MotorPtr _motor;

        ControlMode _ControlMode = Alignment;
        std::array<MotorControlStrategyPtr, kControlModeCount> _controllers;
        std::optional<Ticks16> _timeToNextCommutation;
        bool _running = false;

        esp::mcpwm::Timer::EventCallbacks _mcpwmTimerEventCallbacks;

        esp::GPIOPtr _sleepGPIO;
        bool _sleepValue;

        esp::GPIOPtr _faultGPIO;
        bool _faultValue;
        esp::mcpwm::GPIOFault::Callback _faultCallback;

        static constexpr char _loggingTag[] = "bldc::MotorController";

        friend void bldc::_controlTask(void* userInfo);
        friend esp::InterruptResult _timerCallback(esp::GPTimer& timer, const gptimer_alarm_event_data_t& eventData, void* userInfo);
    };
}  // namespace bldc

