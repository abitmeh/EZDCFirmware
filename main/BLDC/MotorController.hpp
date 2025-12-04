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
#include "BLDC/SensorlessControlStrategy.hpp"
#include "BLDC/Types.hpp"

#include "GPIO.hpp"
#include "GPTimer.hpp"

#include <esp_event.h>

#include <optional>

namespace bldc {
    struct MotorControlConfig {
        bldc::MotorConfig motorConfig;
        bldc::SensorlessControlConfig sensorlessControlConfig;

        gpio_num_t sleepGPIONum;
        bool sleepValue;
    };

    void _controlTask(void* userInfo);

    class MotorController {
    public:
        MotorController(const MotorControlConfig& config, esp_err_t& err);
        ~MotorController();

        void start(uint32_t targetRPM, esp_err_t& err);
        void stop(esp_err_t& err);

        Direction direction() const;
        void setDirection(Direction direction);

        uint32_t dutyCycle() const;
        void setDutyCycle(uint32_t dutyCycle);

        uint32_t rpm() const;
        uint32_t targetRPM() const;
        void setTargetRPM(uint32_t targetRPM);

    private:
        void _controlTask();

        void _tick();

        void _setControlPhase(ControlPhase phase, esp_err_t& err);
        bool _checkForStall(esp_err_t& err);

        esp::GPTimerPtr _timer;
        SemaphoreHandle_t _timerFiredSemaphore;

        MotorPtr _motor;

        ControlPhase _controlPhase = PulseInjection;
        std::array<MotorControlStrategyPtr, kControlPhaseCount> _controllers;
        std::optional<uint16_t> _timeToNextStep;
        bool _running = false;

        esp::mcpwm::Timer::EventCallbacks _mcpwmTimerEventCallbacks;

        esp::GPIOPtr _sleepGPIO;
        bool _sleepValue;

        esp::GPIOPtr _faultGPIO;
        bool _faultValue;
        esp::mcpwm::GPIOFault::Callback _faultCallback;

        static constexpr char _loggingTag[] = "bldc::MotorController";

        friend void bldc::_controlTask(void* userInfo);
    };
}  // namespace bldc
