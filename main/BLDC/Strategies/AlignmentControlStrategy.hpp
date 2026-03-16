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
#include "BLDC/Strategies/MotorControlStrategy.hpp"
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
        AlignmentControlStrategy(MotorPtr& motor);

        virtual void start(ControlStrategyTransferableState&& state, esp_err_t&) override;

        virtual std::optional<Commutation> tick() override;

        float dutyCycle() const;

    private:
        int8_t _step = 0;

        static constexpr char _loggingTag[] = "bldc::AlignmentControlStrategy";
    };
}  // namespace bldc
