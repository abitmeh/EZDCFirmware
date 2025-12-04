/*
 * AlignmentControlStrategy.hpp
 *
 * (c) Tom Davie 02/12/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#pragma once

#include "BLDC/Motor.hpp"
#include "BLDC/MotorControlStrategy.hpp"
#include "BLDC/Types.hpp"

#include <array>
#include <optional>
#include <tuple>

namespace bldc {
    class McpwmContext;

    class AlignmentControlStrategy;
    using AlignmentControlStrategyPtr = std::shared_ptr<AlignmentControlStrategy>;

    class AlignmentControlStrategy : public MotorControlStrategy {
    public:
        AlignmentControlStrategy(Motor& motor);

        virtual NextChange nextStepChange() override;

        virtual uint32_t dutyCycle() const override;

        virtual std::optional<ControlPhase> nextControlPhase(ControlPhase currentControlPhase) const override;

    private:
        bool _hasAligned = false;

        static constexpr char _loggingTag[] = "bldc::AlignmentControlStrategy";
    };
}  // namespace bldc
