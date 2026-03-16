/*
 * MotorConfig.hpp
 *
 * (c) Tom Davie 28/11/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 */

#pragma once

#include "Types.hpp"

#include <pid_ctrl.h>

#include <chrono>
#include <cstddef>
#include <cstdint>

using namespace std::chrono_literals;

namespace bldc {
    static constexpr size_t kAdcFrequency = 80'000;
    static constexpr size_t kAdcSamplesPerPWMPeriod = kAdcFrequency / kMotorDriveFrequency;

    static constexpr Ticks16 kZeroCrossRepeatTime = 3_tk;
    static constexpr bldc::rational<uint32_t> kZeroCrossBlankingPeriod = 25_pc;

    static constexpr size_t kPolePairCount = 6;

    static constexpr Ticks32 kTimePerPhaseAt1RPM = static_cast<Ticks32>(1min) / (kPolePairCount * 6);

    static constexpr size_t kMaxRpm = 2'200;
    static constexpr size_t kMinRpm = 0;
    static constexpr float kMaxSpeedMeasurementFactor = 1.2;
    static constexpr std::chrono::milliseconds kSpeedAveragingDuration = 20ms;
}  // namespace bldc
