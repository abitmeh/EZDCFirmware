/*
 * PulseInjectionControlStrategy.hpp
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

namespace bldc {
    class McpwmContext;

    class PulseInjectionControlStrategy;
    using PulseInjectionControlStrategyPtr = std::shared_ptr<PulseInjectionControlStrategy>;

    class PulseInjectionControlStrategy : public MotorControlStrategy {
    public:
        PulseInjectionControlStrategy(MotorPtr& motor);

        virtual void start(ControlStrategyTransferableState&& state, esp_err_t& err) override;

        virtual std::optional<Commutation> tick() override;

        float dutyCycle() const;

    private:
        enum PulseInjectionPhase {
            NotStarted = 0,
            Phase1,
            Phase2,
            Phase3,
            Phase4,
            Phase5,
            Phase6,
            Cleanup,
            Complete
        };

        enum PulseInjectionOperation : uint8_t {
            Charge = 0,
            Inject
        };

        PulseInjectionOperation _operation = Charge;
        PulseInjectionPhase _phase = NotStarted;
        bool _readADC = false;
        std::array<uint32_t, 6> _adcValues{0};

        static constexpr char _loggingTag[] = "bldc::PulseInjectionControlStrategy";
    };
}  // namespace bldc
