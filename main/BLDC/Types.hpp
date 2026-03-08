/*
 * Types.hpp
 *
 * (c) Tom Davie 28/11/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#pragma once

#include "BLDC/Config.hpp"
#include "Utilities/rational.hpp"

#include <chrono>
#include <cstdint>
#include <string>

namespace bldc {
    enum Direction : uint8_t {
        Clockwise = 0,
        Anticlockwise,
    };

    enum ControlMode : uint8_t {
        PulseInjection = 0,
        Alignment,
        Drag,
        ClosedLoop,
        Stalled,
        Stopped,
        Fault,
    };

    static constexpr uint8_t kControlModeCount = static_cast<uint8_t>(ControlMode::Fault) + 1;

    std::string to_string(ControlMode phase);

    inline std::string to_string(ControlMode phase) {
        switch (phase) {
            case PulseInjection:
                return "Pulse Injection";
            case Alignment:
                return "Alignment";
            case Drag:
                return "Drag";
            case ClosedLoop:
                return "Closed Loop";
            case Stalled:
                return "Stalled";
            case Stopped:
                return "Stopped";
            case Fault:
                return "Fault";
        }
        return "GCC Sucks";
    }

    enum MotorPhase : uint8_t {
        U = 0,
        V,
        W,
    };

    enum PhaseAngle : uint8_t {
        Degrees0 = 0,
        Degrees60,
        Degrees120,
        Degrees180,
        Degrees240,
        Degrees300
    };

    static constexpr uint8_t kMotorPhaseCount = static_cast<uint8_t>(MotorPhase::W) + 1;

    using Ticks16 = std::chrono::duration<uint16_t, std::ratio<1, kMotorDriveFrequency>>;
    using Ticks32 = std::chrono::duration<uint32_t, std::ratio<1, kMotorDriveFrequency>>;
    using Ticksf = std::chrono::duration<float, std::ratio<1, kMotorDriveFrequency>>;

    constexpr Ticks32 operator""_tk(unsigned long long tks) {
        return Ticks32(tks);
    }

    constexpr Ticks16 operator""_tks(unsigned long long tks) {
        return Ticks16(tks);
    }

    constexpr Ticksf operator""_tk(long double tks) {
        return Ticksf(tks);
    }

    constexpr Ticks32 operator*(const Ticks32& lhs, const rational<uint32_t>& rhs) {
        return Ticks32((lhs * rhs.numerator()) / rhs.denominator());
    }

    constexpr Ticks32 operator*(const rational<uint32_t>& lhs, const Ticks32& rhs) {
        return Ticks32((rhs * lhs.numerator()) / lhs.denominator());
    }

    constexpr Ticks16 operator*(const Ticks16& lhs, const rational<uint16_t>& rhs) {
        return Ticks16((lhs * rhs.numerator()) / rhs.denominator());
    }

    constexpr Ticks16 operator*(const rational<uint16_t>& lhs, const Ticks16& rhs) {
        return Ticks16((rhs * lhs.numerator()) / lhs.denominator());
    }

    static constexpr rational<uint32_t> percent = rational<uint32_t>(1ul, 100ul);

    constexpr rational<uint32_t> operator""_pc(unsigned long long x) {
        return static_cast<uint32_t>(x) * percent;
    }
}  // namespace bldc
