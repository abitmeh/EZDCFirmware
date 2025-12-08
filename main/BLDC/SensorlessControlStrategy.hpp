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
#include "BLDC/MotorControlStrategy.hpp"
#include "BLDC/Types.hpp"

#include "ADCOneshot.hpp"

#include <array>
#include <optional>
#include <tuple>

namespace bldc {
    class McpwmContext;

    class SensorlessControlStrategy;
    using SensorlessControlStrategyPtr = std::shared_ptr<SensorlessControlStrategy>;

    using SensorlessControlConfig = std::array<std::pair<adc_unit_t, esp::ADCChannelConfig>, kMotorPhaseCount>;

    class SensorlessControlStrategy : public MotorControlStrategy {
    public:
        SensorlessControlStrategy(const SensorlessControlConfig& adcConfig, Motor& motor, esp_err_t& err);

        virtual void mcpwmTimerFull() override;

        virtual void start(esp_err_t& err) override;

        virtual NextChange nextStepChange() override;

        virtual uint32_t dutyCycle() const override;

        void setPIDParameters(const pid_ctrl_parameter_t* parameters, esp_err_t& err);

    private:
        std::optional<uint16_t> _completePhase();

        std::array<esp::ADCOneshotChannelPtr<esp::Calibrated>, 3> _adcs;

        pid_ctrl_block_handle_t _pid;

        bool _calculateSpeed = true;
        uint32_t _timeSpentAvoidingContinuousCurrent = 0;
        std::array<uint32_t, 3> _adcValues;
        uint32_t _maxObservedValue = 0;

        static constexpr char _loggingTag[] = "bldc::SensorlessControlStrategy";
    };
}  // namespace bldc
