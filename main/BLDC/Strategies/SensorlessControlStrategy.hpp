/*
 * SensorlessControlStrategy.hpp
 *
 * (c) Tom Davie 2/12/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#pragma once

#include "BLDC/Motor.hpp"
#include "BLDC/Strategies/MotorControlStrategy.hpp"
#include "BLDC/Types.hpp"

#include <array>
#include <optional>
#include <tuple>
#include <unordered_map>

void adcTask(void* userInfo);

namespace bldc {
    class McpwmContext;

    class SensorlessControlStrategy;
    using SensorlessControlStrategyPtr = std::shared_ptr<SensorlessControlStrategy>;

    class SensorlessControlStrategy : public MotorControlStrategy {
    public:
        SensorlessControlStrategy(MotorPtr& motor, esp_err_t& err);

        virtual void start(ControlStrategyTransferableState&& state, esp_err_t& err) override;
        virtual ControlStrategyTransferableState stop(esp_err_t& err) override;

        virtual std::optional<Commutation> tick() override;

        float dutyCycle() const;

        void setPIDParameters(const pid_ctrl_parameter_t& parameters, esp_err_t& err);

    private:
        bool _detectZeroCross(uint16_t ticksToWait, const std::array<uint32_t, kMotorPhaseCount + 1>& adcValues);
        std::optional<uint16_t> _completePhase();

        pid_ctrl_block_handle_t _pid;
        pid_ctrl_parameter_t _pidParameters;

        bool _calculateSpeed = true;
        uint32_t _timeSpentAvoidingContinuousCurrent = 0;
        uint32_t _maxObservedValue = 0;

        static constexpr char _loggingTag[] = "bldc::SensorlessControlStrategy";
    };
}  // namespace bldc
