/*
 * ESP32.hpp
 *
 * (c) Tom Davie 29/11/2025 
 * 
 */

#pragma once

#include "ADC.hpp"
#include "GPIO.hpp"
#include "mcpwm/MCPWM.hpp"

#include <memory>

namespace esp {
    class ESP32;
    using ESP32Ptr = std::shared_ptr<ESP32>;

    class ESP32 {
    public:
        static ESP32Ptr sharedESP32();

        virtual size_t numADCs() = 0;
        virtual size_t numGPIOs() = 0;

        ADCOneshotPtr adcOneshot(adc_unit_t unit, esp_err_t& err);
        GPIOPtr gpio(GPIOConfig gpioConfig, esp_err_t& err);

        mcpwm::MCPWM& mcpwm() { return _mcpwm; }

    protected:
        ESP32();

        std::vector<ADCOneshotPtr> _adcs;
        std::vector<GPIOPtr> _gpios;

        mcpwm::MCPWM _mcpwm;

    private:
        static constexpr char _loggingTag[] = "esp::ESP32";
    };
}  // namespace esp
