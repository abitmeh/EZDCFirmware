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

#include <cstdint>
#include <string>

namespace bldc {
    enum Direction : uint8_t {
        Clockwise = 0,
        Anticlockwise,
    };

    enum ControlPhase : uint8_t {
        PulseInjection = 0,
        Alignment,
        Drag,
        ClosedLoop,
        Stalled,
        Stopped,
        Fault,
    };

    static constexpr uint8_t kControlPhaseCount = static_cast<uint8_t>(ControlPhase::Fault) + 1;

    std::string to_string(ControlPhase phase);

    inline std::string to_string(ControlPhase phase) {
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

    enum MotorStep : uint8_t {
        Step0 = 0,
        Step1,
        Step2,
        Step3,
        Step4,
        Step5
    };

    static constexpr uint8_t kMotorPhaseCount = static_cast<uint8_t>(MotorPhase::W) + 1;
}  // namespace bldc
