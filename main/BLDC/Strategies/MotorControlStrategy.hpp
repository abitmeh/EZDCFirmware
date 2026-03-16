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

    class ControlStrategyDelegate {
    public:
        virtual void controlStrategyMotorDidStall(const MotorControlStrategy& controlStrategy) = 0;
        virtual void controlStrategyDidComplete(const MotorControlStrategy& controlStrategy) = 0;
    };

    struct ControlStrategyTransferableState {
        PhaseAngle _currentStep = PhaseAngle::Degrees0;
    };

    class MotorControlStrategy {
    public:
        MotorControlStrategy(MotorPtr& motor) : _motor(motor) {}

        virtual void start(ControlStrategyTransferableState&& state, esp_err_t&) { _state = std::move(state); }

        virtual ControlStrategyTransferableState stop(esp_err_t&) { return _state; }

        virtual std::optional<Commutation> tick() = 0;

        ControlStrategyDelegate* delegate() { return _delegate; }

        void setDelegate(ControlStrategyDelegate* delegate) { _delegate = delegate; }

    protected:
        MotorPtr _motor;

        ControlStrategyDelegate* _delegate;

        ControlStrategyTransferableState _state;

    private:
        static constexpr char _loggingTag[] = "bldc::MotorControlStrategy";
    };
}  // namespace bldc
