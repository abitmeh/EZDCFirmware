/*
 * GPIOFault.hpp
 *
 * (c) Tom Davie 03/12/2025 
 * 
 */

#pragma once

#include "Interupt.hpp"

#include <driver/gpio.h>
#include <driver/mcpwm_fault.h>

#include <functional>
#include <memory>

namespace esp {
    namespace mcpwm {
        struct GPIOFaultConfig {
            uint8_t groupId;
            InteruptPriority interuptPriority = Default;
            gpio_num_t gpioNum;

            bool activeHigh = false;
            bool ioLoopBack = false;
            bool pullUp = false;
            bool pullDown = false;
        };

        class GPIOFault;
        using GPIOFaultPtr = std::shared_ptr<GPIOFault>;

        bool _onFaultEnter(mcpwm_fault_handle_t fault, const mcpwm_fault_event_data_t* faultData, void* userInfo);
        bool _onFaultExit(mcpwm_fault_handle_t fault, const mcpwm_fault_event_data_t* faultData, void* userInfo);

        class GPIOFault {
        public:
            using Callback = std::function<bool(const mcpwm_fault_event_data_t&)>;

            struct Callbacks {
                Callback onFaultEnter;
                Callback onFaultExit;
            };

            ~GPIOFault();

            void setCallbacks(const Callbacks& callbacks, esp_err_t& err);

        private:
            GPIOFault(const GPIOFaultConfig& config, esp_err_t& err);

            Callbacks _callbacks;

            mcpwm_fault_handle_t _fault;

            static constexpr char _loggingTag[] = "esp::mcpwm::GPIOFault";

            friend class MCPWM;

            friend bool _onFaultEnter(mcpwm_fault_handle_t fault, const mcpwm_fault_event_data_t* faultData, void* userInfo);
            friend bool _onFaultExit(mcpwm_fault_handle_t, const mcpwm_fault_event_data_t*, void*);
        };
    }  // namespace mcpwm
}  // namespace esp

