/*
 * MotorControlStrategy.hpp
 *
 * (c) Tom Davie 30/11/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#include "BLDC/Motor.hpp"

#include <array>
#include <optional>

#pragma once

namespace bldc {
    class MotorControlStrategy;
    using MotorControlStrategyPtr = std::shared_ptr<MotorControlStrategy>;

    using NextStep = std::pair<Ticks16, PhaseAngle>;

    using NextChange = std::optional<NextStep>;

    class MotorControlStrategy {
    public:
        MotorControlStrategy(MotorPtr& motor) : _motor(motor) {}

        virtual void start(esp_err_t&) {}

        virtual void stop(esp_err_t&) {}

        virtual NextChange nextStepChange() = 0;

        virtual float dutyCycle() const = 0;

        virtual std::optional<ControlMode> nextControlMode(ControlMode currentControlMode) const { return std::optional<ControlMode>(); }

    protected:
        MotorPtr _motor;

    private:
        static constexpr char _loggingTag[] = "bldc::MotorControlStrategy";
    };
}  // namespace bldc
