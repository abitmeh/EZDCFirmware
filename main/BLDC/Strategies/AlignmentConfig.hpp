#pragma once

#include <chrono>

using namespace std::chrono_literals;

namespace bldc {
    static constexpr std::chrono::milliseconds kAlignmentTime =
        1000ms; /*!< Duration of alignment, too short may not reach the position, too long may cause the motor to overheat. */
    static constexpr float kAlignmentStartDutyCycle = 0.15f;
    static constexpr float kAlignmentEndDutyCycle = 0.3f;
}  // namespace bldc
