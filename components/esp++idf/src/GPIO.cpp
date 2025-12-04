/*
 * GPIO.cpp
 *
 * (c) Tom Davie 29/11/2025 
 * 
 */

#include "GPIO.hpp"

#include <esp_log.h>

using namespace esp;

GPIOConfig::GPIOConfig(gpio_num_t num, gpio_mode_t m, gpio_pullup_t up, gpio_pulldown_t down) : gpioNum(num), mode(m), pullUp(up), pullDown(down) {}

GPIOConfig GPIOConfig::inputGPIO(gpio_num_t num, gpio_pullup_t pullUp, gpio_pulldown_t pullDown) {
    return GPIOConfig(num, GPIO_MODE_INPUT, pullUp, pullDown);
}

GPIOConfig GPIOConfig::outputGPIO(gpio_num_t num, gpio_pullup_t pullUp, gpio_pulldown_t pullDown) {
    return GPIOConfig(num, GPIO_MODE_OUTPUT_OD, pullUp, pullDown);
}

GPIOConfig GPIOConfig::outputGPIOOpenDrain(gpio_num_t num, gpio_pullup_t pullUp, gpio_pulldown_t pullDown) {
    return GPIOConfig(num, GPIO_MODE_OUTPUT_OD, pullUp, pullDown);
}

namespace esp {
    bool operator==(const GPIOConfig& a, const GPIOConfig& b) {
        return a.gpioNum == b.gpioNum && a.mode == b.mode && a.pullUp == b.pullUp && a.pullDown == b.pullDown;
    }
}  // namespace esp

GPIO::GPIO(const GPIOConfig& gpioConfig, esp_err_t& err) : _config(gpioConfig) {
    if (!GPIO_IS_VALID_GPIO(gpioConfig.gpioNum)) {
        ESP_LOGE(_loggingTag, "Invalid GPIO: %u", gpioConfig.gpioNum);
        err = ESP_ERR_INVALID_ARG;
        return;
    }

    gpio_config_t config = {
        .pin_bit_mask = (1ull << gpioConfig.gpioNum),
        .mode = gpioConfig.mode,
        .pull_up_en = gpioConfig.pullUp,
        .pull_down_en = gpioConfig.pullDown,
        .intr_type = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&config);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "gpio_config failed: %s", esp_err_to_name(err));
        return;
    }
}

GPIO::~GPIO() {
    esp_err_t err = ESP_OK;
    err = gpio_reset_pin(_config.gpioNum);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "gpio_reset_pin failed: %s", esp_err_to_name(err));
        return;
    }
}

const GPIOConfig& GPIO::config() const {
    return _config;
}

bool GPIO::level() {
    uint32_t level = gpio_get_level(_config.gpioNum);
    return level != 0;
}

void GPIO::setLevel(bool level, esp_err_t& err) {
    err = gpio_set_level(_config.gpioNum, static_cast<uint32_t>(level));
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "gpio_set_level failed: %s", esp_err_to_name(err));
        return;
    }
}