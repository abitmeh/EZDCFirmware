#pragma once

#include <cstddef>
#include <cstdint>

namespace bldc {
    static constexpr size_t kMcpwmFrequency = 20'000'000;
    static constexpr size_t kMcpwmPeriod = 1000;
    static constexpr size_t kMaxDutyCycle = kMcpwmPeriod / 2;
    static constexpr size_t kMotorDriveFrequency = kMcpwmFrequency / kMcpwmPeriod;
}  // namespace bldc
