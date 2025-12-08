/*
 * MotorControlStrategy.hpp
 *
 * (c) Tom Davie 30/11/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#include "Motor.hpp"

#include "ADCOneshot.hpp"

#include <array>
#include <optional>

#pragma once

namespace bldc {
    class MotorControlStrategy;
    using MotorControlStrategyPtr = std::shared_ptr<MotorControlStrategy>;

    using NextStep = std::pair<uint16_t, MotorStep>;

    using NextChange = std::optional<NextStep>;

    class MotorControlStrategy {
    public:
        MotorControlStrategy(Motor& motor) : _motor(motor) {}

        virtual void mcpwmTimerFull() {}

        virtual void start(esp_err_t&) {}

        virtual NextChange nextStepChange() = 0;

        virtual uint32_t dutyCycle() const = 0;

        virtual std::optional<ControlPhase> nextControlPhase(ControlPhase currentControlPhase) const { return std::optional<ControlPhase>(); }

    protected:
        Motor& _motor;

    private:
        static constexpr char _loggingTag[] = "bldc::MotorControlStrategy";
    };
}  // namespace bldc
