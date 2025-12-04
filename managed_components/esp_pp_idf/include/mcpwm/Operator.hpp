/*
 * Operator.hpp
 *
 * (c) Tom Davie 03/12/2025 
 * 
 */

#pragma once

#include "Interupt.hpp"

#include <driver/mcpwm_oper.h>

#include <memory>

namespace esp {
    namespace mcpwm {
        struct OperatorConfig {
            InteruptPriority interuptPriority = Default;

            bool updateGeneratorActionOnTimerZero = false;
            bool updateGeneratorActionOnTimerPeak = false;
            bool updateGeneratorActionOnSync = false;
        };

        class Operator;
        using OperatorPtr = std::shared_ptr<Operator>;

        class ComparatorConfig;
        class Comparator;
        using ComparatorPtr = std::shared_ptr<Comparator>;

        class GeneratorConfig;
        class Generator;
        using GeneratorPtr = std::shared_ptr<Generator>;

        class Timer;
        using TimerPtr = std::shared_ptr<Timer>;

        class Operator : public std::enable_shared_from_this<Operator> {
        public:
            ~Operator();

            ComparatorPtr addComparator(const ComparatorConfig& config, esp_err_t& err);
            GeneratorPtr addGenerator(const GeneratorConfig& config, esp_err_t& err);

        private:
            Operator(TimerPtr timer, const OperatorConfig& config, esp_err_t& err);

            mcpwm_oper_handle_t _operator;

            TimerPtr _timer;

            static constexpr char _loggingTag[] = "esp::mcpwm::Operator";

            friend class Timer;
            friend class Comparator;
            friend class Generator;
        };
    }  // namespace mcpwm
}  // namespace esp
