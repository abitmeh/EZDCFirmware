#pragma once

#include "Motor.hpp"

namespace bldc {
    class MotorObservation {
        Ticks32 timestamp;
        uint16_t motorAngle;
    }

    class MotorObserver {
    public:
        MotorObserver(const MotorPtr& motor) : _motor(motor) {}

        virtual void start(void) = 0;
        virtual void stop(void) = 0;

    protected:
        MotorPtr _motor;
    };
}  // namespace bldc
