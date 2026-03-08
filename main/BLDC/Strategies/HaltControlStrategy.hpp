/*
 * HallControlStrategy.hpp
 *
 * (c) Tom Davie 02/12/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#pragma once

#include "BLDC/Motor.hpp"
#include "BLDC/Strategies/MotorControlStrategy.hpp"
#include "BLDC/Types.hpp"

#include <array>
#include <optional>
#include <tuple>

namespace bldc {
    class McpwmContext;

    class HaltControlStrategy;
    using HaltControlStrategyPtr = std::shared_ptr<HaltControlStrategy>;

    class HaltControlStrategy : public MotorControlStrategy {
    public:
        HaltControlStrategy(MotorPtr& motor);

        virtual void start(esp_err_t&) override;

        virtual NextChange nextStepChange() override;

        virtual float dutyCycle() const override;

        virtual std::optional<ControlMode> nextControlMode(ControlMode currentControlMode) const override;

    private:
        static constexpr char _loggingTag[] = "bldc::HaltControlStrategy";
    };
}  // namespace bldc
