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
#include "BLDC/MotorConfig.hpp"
#include "BLDC/Strategies/MotorControlStrategy.hpp"
#include "BLDC/Types.hpp"
#include "Utilities/LinearPiecewiseLinearFunction.hpp"

#include <array>
#include <optional>
#include <tuple>

namespace bldc {
    class McpwmContext;

    class DragControlStrategy;
    using DragControlStrategyPtr = std::shared_ptr<DragControlStrategy>;

    class DragControlStrategy : public MotorControlStrategy {
    public:
        DragControlStrategy(MotorPtr& motor, esp_err_t& err);

        virtual void start(ControlStrategyTransferableState&&, esp_err_t&) override;
        virtual ControlStrategyTransferableState stop(esp_err_t&) override;

        virtual std::optional<Commutation> tick() override;

        float dutyCycle() const;

    private:
        Ticks32 _durationInNextPhase();

        Ticks32 _timeInDrag = 0_tk;
        float _proportionThroughDrag = .0f;

        static constexpr char _loggingTag[] = "bldc::DragControlStrategy";
    };
}  // namespace bldc
