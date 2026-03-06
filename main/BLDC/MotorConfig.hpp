/*
 * MotorConfig.hpp
 *
 * (c) Tom Davie 28/11/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 */

#pragma once

#include "Utilities/LinearPiecewiseLinearFunction.hpp"

#include <pid_ctrl.h>

#include <cstddef>
#include <cstdint>

/**
 * @brief MCPWM Settings
 *
 */
static constexpr size_t kMcpwmFrequency = 20'000'000; /*!< Number of count ticks within a period time_us = 1,000,000 / kMcpwmFrequency */
static constexpr size_t kMcpwmPeriod = 1000;          /*!< pwm_cycle_us = kMcpwmPeriod * 1,000,000 / kMcpwmFrequency */
static constexpr size_t kMaxDutyCycle = kMcpwmPeriod / 2;
static constexpr size_t kMotorDriveFrequency = kMcpwmFrequency / kMcpwmPeriod;
static constexpr size_t kTickUs = 1'000'000 / kMotorDriveFrequency;

/**
 * @brief ADC Settings
 *
 */
 static constexpr size_t kAdcFrequency = 80'000;
 static constexpr size_t kAdcSamplesPerPWMPeriod = kAdcFrequency / kMotorDriveFrequency;

/**
 * @brief Pulse injection-related parameters.
 * @note Used to detect the initial phase of the motor; MCPWM peripheral support is necessary.
 */
static constexpr float kPulseInjectionDutyCycle = 0.1f; /*!< Injected torque. */
static constexpr size_t kCapacitorChargeTime = 1;       /*!< Capacitor charging time. */
static constexpr size_t kPulseLength = 1;               /*!< Injection time */

/**
 * @brief Parameters related to motor alignment.
 *        Used to lock the motor in a specific phase
 *        before strong dragging.
 */
static constexpr size_t kAlignmentTimeMs = 1000; /*!< Duration of alignment, too short may not reach the position, too long may cause the motor to overheat. */
static constexpr float kAlignmentStartDutyCycle = 0.15;
static constexpr float kAlignmentEndDutyCycle = 0.3;

static constexpr bldc::PiecewiseLinearFunction<float, 4> kDragDutyCycleCurve{{
    { 0.0f, 0.55f },
    { 0.75, 0.65f },
    { 1.25f, 0.68f },
    { 11.0f, 0.70f },
}};

static constexpr bldc::PiecewiseLinearFunction<float, 5> kDragRpmCurve{{
    { 0.0f , 800.0f  },
    { 0.5f , 1600.0f },
    { 2.0f , 1900.0f },
    { 3.0f , 2000.0f },
    { 11.0f, 2000.0f },
}};

/**
 * @brief ADC parameters for zero-crossing detection; please do not delete if not in use.
 *
 */
static constexpr size_t kZeroCrossRepeatTime = 3;
static constexpr float kZeroCrossBlankingPeriod = 0.15;

/**
 * @brief Motor parameter settings.
 *
 */
static constexpr size_t kPolePairCount = 1;

/**
 * @brief Closed-loop PID parameters for speed.
 *
 */
static constexpr float kPidKp = 0.0075f;
static constexpr float kPidKi = 0.0f;
static constexpr float kPidKd = 0.0f;
static constexpr float kPidMinIntegral = 0.0f;
static constexpr float kPidMaxIntegral = 1500.0f;
static constexpr float kPidMinOutput = 0.0f;
static constexpr float kPidMaxOutput = 1.0f;
static constexpr pid_calculate_type_t kSpeedCalculationType = PID_CAL_TYPE_POSITIONAL;

/**
 * @brief Speed parameter settings.
 * 
 */
static constexpr float kTicksPerSecond = kMotorDriveFrequency;
static constexpr float kTicksPerMinute = 60.0 * kTicksPerSecond;
static constexpr float kADCRpmCalculationCoefficient = kTicksPerMinute / static_cast<float>(kPolePairCount * 6);

static constexpr size_t kMaxRpm = 35'000;
static constexpr size_t kMinRpm = 0;
static constexpr float kMaxSpeedMeasurementFactor = 1.2;
static constexpr uint32_t kSpeedAveragingTicks = static_cast<uint32_t>(0.02f * kTicksPerSecond);


static constexpr pid_ctrl_config_t kPidControlConfig = {.init_param = {
                                                            .kp = kPidKp,
                                                            .ki = kPidKi,
                                                            .kd = kPidKd,
                                                            .max_output = kPidMaxOutput,
                                                            .min_output = kPidMinOutput,
                                                            .max_integral = kPidMaxIntegral,
                                                            .min_integral = kPidMinIntegral,
                                                            .cal_type = kSpeedCalculationType,
                                                        }};
