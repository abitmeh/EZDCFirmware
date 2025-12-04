/*
 * DragControlStrategy.hpp
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

    class DragControlStrategy;
    using DragControlStrategyPtr = std::shared_ptr<DragControlStrategy>;

    class DragControlStrategy : public MotorControlStrategy {
    public:
        DragControlStrategy(Motor& motor, esp_err_t& err);

        virtual NextChange nextStepChange() override;

        virtual uint32_t dutyCycle() const override;

        virtual std::optional<ControlPhase> nextControlPhase(ControlPhase currentControlPhase) const override;

    private:
        uint16_t _nextStepLength();

        uint32_t _timeInDrag = 0;
        float _proportionThroughDrag = .0f;

        static constexpr char _loggingTag[] = "bldc::DragControlStrategy";
    };
}  // namespace bldc
