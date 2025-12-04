/*
 * GPIO.hpp
 *
 * (c) Tom Davie 29/11/2025 
 * 
 */

#pragma once

#include <driver/gpio.h>

#include <memory>

namespace esp {
    struct GPIOConfig {
    public:
        GPIOConfig(gpio_num_t num, gpio_mode_t mode, gpio_pullup_t pullUp = GPIO_PULLUP_DISABLE, gpio_pulldown_t pullDown = GPIO_PULLDOWN_DISABLE);

        GPIOConfig inputGPIO(gpio_num_t num, gpio_pullup_t pullUp = GPIO_PULLUP_DISABLE, gpio_pulldown_t pullDown = GPIO_PULLDOWN_DISABLE);
        GPIOConfig outputGPIO(gpio_num_t num, gpio_pullup_t pullUp = GPIO_PULLUP_DISABLE, gpio_pulldown_t pullDown = GPIO_PULLDOWN_DISABLE);
        GPIOConfig outputGPIOOpenDrain(gpio_num_t num, gpio_pullup_t pullUp = GPIO_PULLUP_DISABLE, gpio_pulldown_t pullDown = GPIO_PULLDOWN_DISABLE);

        gpio_num_t gpioNum;
        gpio_mode_t mode;
        gpio_pullup_t pullUp;
        gpio_pulldown_t pullDown;
    };

    bool operator==(const GPIOConfig& a, const GPIOConfig& b);

    class GPIO {
    public:
        ~GPIO();

        const GPIOConfig& config() const;

        bool level();
        void setLevel(bool level, esp_err_t& err);

    private:
        GPIO(const GPIOConfig& gpioConfig, esp_err_t& err);

        GPIOConfig _config;

        static constexpr char _loggingTag[] = "esp::GPIO";

        friend class ESP32;
    };

    using GPIOPtr = std::shared_ptr<GPIO>;
}  // namespace esp
