/*
 * MotorConfig.hpp
 *
 * (c) Tom Davie 28/11/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 */

#pragma once

#include <pid_ctrl.h>

#include <cstddef>
#include <cstdint>

/**
 * @brief MCPWM Settings
 *
 */
static constexpr size_t kMcpwmFrequency = 20'000'000; /*!< Number of count ticks within a period time_us = 1,000,000 / kMcpwmFrequency */
static constexpr size_t kMcpwmPeriod = 2000;          /*!< pwm_cycle_us = kMcpwmPeriod * 1,000,000 / kMcpwmFrequency */

/**
 * @brief LEDC Settings
 *
 */
static constexpr size_t kMotorDriveFrequency = kMcpwmFrequency / kMcpwmPeriod;

/**
 * @brief No changes should be made here.
 *
 */
static constexpr size_t kAlarmCountMicroseconds = 1'000'000 / kMotorDriveFrequency;
static constexpr size_t kMaxDutyCycle = kMcpwmPeriod / 2;

static constexpr size_t dutyCyclePeriod(float duty) {
    return duty * kMaxDutyCycle;
}

static constexpr size_t k5PercentDutyCycle = dutyCyclePeriod(0.05f);
static constexpr size_t k10PercentDutyCycle = dutyCyclePeriod(0.1f);
static constexpr size_t k15PercentDutyCycle = dutyCyclePeriod(0.15f);
static constexpr size_t k20PercentDutyCycle = dutyCyclePeriod(0.2f);
static constexpr size_t k25PercentDutyCycle = dutyCyclePeriod(0.25f);
static constexpr size_t k30PercentDutyCycle = dutyCyclePeriod(0.3f);
static constexpr size_t k40PercentDutyCycle = dutyCyclePeriod(0.4f);
static constexpr size_t k50PercentDutyCycle = dutyCyclePeriod(0.5f);
static constexpr size_t k60PercentDutyCycle = dutyCyclePeriod(0.6f);
static constexpr size_t k70PercentDutyCycle = dutyCyclePeriod(0.7f);
static constexpr size_t k80PercentDutyCycle = dutyCyclePeriod(0.8f);
static constexpr size_t k90PercentDutyCycle = dutyCyclePeriod(0.9f);
static constexpr size_t k100PercentDutyCycle = dutyCyclePeriod(1.0f);

/**
 * @brief Pulse injection-related parameters.
 * @note Used to detect the initial phase of the motor; MCPWM peripheral support is necessary.
 */
static constexpr size_t kPulseInjectionDutyCycle = k10PercentDutyCycle; /*!< Injected torque. */
static constexpr size_t kCapacitorChargeTime = 1;                       /*!< Capacitor charging time. */
static constexpr size_t kPulseLength = 1;                               /*!< Injection time */

/**
 * @brief Parameters related to motor alignment.
 *        Used to lock the motor in a specific phase
 *        before strong dragging.
 */
static constexpr size_t kAlignmentTimeMs = 300; /*!< Duration of alignment, too short may not reach the position, too long may cause the motor to overheat. */
static constexpr size_t kAlignmentDutyCycle = k20PercentDutyCycle; /*!< alignment torque. */

/**
 * @brief Setting parameters for strong dragging. The principle of
 *        strong dragging is to increase the control frequency and intensity
 * @note  If the control cycle speeds up, corresponding reductions
 *        should be made to the RAMP_TIM_STA, RAMP_TIM_END, RAMP_TIM_STEP
 */
static constexpr size_t kDragPeriodAtRampStart = 600;
static constexpr size_t kDragPeriodAtRampEnd = 20;
static constexpr size_t kDragDutyCycleAtRampStart = k10PercentDutyCycle;
static constexpr size_t kDragDutyCycleAtRampEnd = k25PercentDutyCycle;
static constexpr size_t kDragDuration = 40'000;

/**
 * @brief ADC parameters for zero-crossing detection; please do not delete if not in use.
 *
 */
static constexpr size_t kZeroCrossRepeatTime = 2;
static constexpr size_t kZeroCrossAvoidContinuousCurrentTime = 4;

/**
 * @brief Common parameter for compensated commutation time calculation
 *
 */
static constexpr size_t kZeroCrossAdvance = 8;

/**
 * @brief Motor parameter settings.
 *
 */
static constexpr size_t kPolePairCount = 2;

/**
 * @brief Closed-loop PID parameters for speed.
 *
 */
static constexpr float kPidKp = 0.75f;
static constexpr float kPidKi = 0.00015f;
static constexpr float kPidKd = 0.0f;
static constexpr float kPidMinIntegral = -kMaxDutyCycle / kPidKi;
static constexpr float kPidMaxIntegral = kMaxDutyCycle / kPidKi;
static constexpr size_t kPidMinOutput = k10PercentDutyCycle;
static constexpr size_t kPidMaxOutput = k60PercentDutyCycle;
static constexpr pid_calculate_type_t kSpeedCalculationType = PID_CAL_TYPE_INCREMENTAL;

/**
 * @brief Speed parameter settings.
 *
 */
static constexpr size_t kMaxRpm = 10'000;
static constexpr size_t kMinRpm = 0;
static constexpr float kMaxSpeedMeasurementFactor = 1.2;

static constexpr float kADCRpmCalculationCoefficient = 60.0f * (1'000'000 / kAlarmCountMicroseconds) / (static_cast<float>(kPolePairCount));

static constexpr size_t kInvalidSpeedCalculationSkipLimit = static_cast<size_t>(kADCRpmCalculationCoefficient / (2 * kMaxRpm * kMaxSpeedMeasurementFactor));

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
