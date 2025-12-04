/*
 * Comparator.hpp
 *
 * (c) Tom Davie 03/12/2025 
 * 
 */

#pragma once

#include "Interupt.hpp"

#include <driver/mcpwm_cmpr.h>

#include <memory>

namespace esp {
    namespace mcpwm {
        struct ComparatorConfig {
            InteruptPriority interuptPriority = Default;

            bool updateComparatorOnTimerZero = false;
            bool updateComparatorOnTimerPeak = false;
            bool updateComparatorOnSync = false;
        };

        class Comparator;
        using ComparatorPtr = std::shared_ptr<Comparator>;

        class Operator;
        using OperatorPtr = std::shared_ptr<Operator>;

        class Comparator {
        public:
            ~Comparator();

            uint32_t compareValue() const;
            void setCompareValue(uint32_t compareValue, esp_err_t& err);

        private:
            Comparator(OperatorPtr oper, const ComparatorConfig& config, esp_err_t& err);

            mcpwm_cmpr_handle_t _comparator;
            uint32_t _compareValue = 0;

            OperatorPtr _operator;

            static constexpr char _loggingTag[] = "esp::mcpwm::Comparator";

            friend class Operator;
            friend class Generator;
        };
    }  // namespace mcpwm
}  // namespace esp
