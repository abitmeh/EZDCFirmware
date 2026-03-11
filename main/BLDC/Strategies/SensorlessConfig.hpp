#pragma once

#include "pid_ctrl.h"

namespace bldc {
    static constexpr float kPidKp = 0.0075f;
    static constexpr float kPidKi = 0.0f;
    static constexpr float kPidKd = 0.0f;
    static constexpr float kPidMinIntegral = 0.0f;
    static constexpr float kPidMaxIntegral = 1500.0f;
    static constexpr float kPidMinOutput = 0.0f;
    static constexpr float kPidMaxOutput = 1.0f;
    static constexpr pid_calculate_type_t kSpeedCalculationType = PID_CAL_TYPE_POSITIONAL;

    static constexpr pid_ctrl_config_t kPidControlConfig = {
        .init_param = {
            .kp = kPidKp,
            .ki = kPidKi,
            .kd = kPidKd,
            .max_output = kPidMaxOutput,
            .min_output = kPidMinOutput,
            .max_integral = kPidMaxIntegral,
            .min_integral = kPidMinIntegral,
            .cal_type = kSpeedCalculationType,
        }};
}  // namespace bldc
