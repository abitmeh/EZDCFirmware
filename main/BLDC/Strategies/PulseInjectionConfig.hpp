#pragma once

#include "BLDC/MotorConfig.hpp"

namespace bldc {
    static constexpr float kPulseInjectionDutyCycle = 0.1f;
    static constexpr Ticks32 kCapacitorChargeTime = 1_tk;
    static constexpr Ticks32 kPulseLength = 1_tk;
}  // namespace bldc
