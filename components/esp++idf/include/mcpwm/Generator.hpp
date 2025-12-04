/*
 * Generator.hpp
 *
 * (c) Tom Davie 03/12/2025 
 * 
 */

#pragma once

#include <driver/gpio.h>
#include <driver/mcpwm_gen.h>

#include <initializer_list>
#include <memory>

namespace esp {
    namespace mcpwm {
        struct GeneratorConfig {
            gpio_num_t gpioNum;

            bool invertPwm = false;
            bool ioLoopBack = false;
            bool ioOpenDrain = false;
            bool pullUp = false;
            bool pullDown = false;
        };

        class Generator;
        using GeneratorPtr = std::shared_ptr<Generator>;

        class Comparator;
        using ComparatorPtr = std::shared_ptr<Comparator>;

        class Operator;
        using OperatorPtr = std::shared_ptr<Operator>;

        class Generator {
        public:
            struct CompareEventAction {
                CompareEventAction(mcpwm_timer_direction_t direction, ComparatorPtr comparator, mcpwm_generator_action_t action);

                mcpwm_timer_direction_t direction;
                ComparatorPtr comparator;
                mcpwm_generator_action_t action;
            };

            ~Generator();

            void setActionOnCompareEvent(const CompareEventAction& action, esp_err_t& err);
            void setActionsOnCompareEvent(std::initializer_list<CompareEventAction> actions, esp_err_t& err);

        private:
            Generator(OperatorPtr oper, const GeneratorConfig& config, esp_err_t& err);

            mcpwm_gen_handle_t _generator;

            OperatorPtr _operator;

            static constexpr char _loggingTag[] = "esp::mcpwm::Generator";

            friend class Operator;
        };
    }  // namespace mcpwm
}  // namespace esp
