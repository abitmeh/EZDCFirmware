/*
 * ESP32.cpp
 *
 * (c) Tom Davie 29/11/2025 
 * 
 */

#include "ESP32.hpp"

#if defined(CONFIG_IDF_TARGET_ESP32S3) && CONFIG_IDF_TARGET_ESP32S3
#include "ESP32S3.hpp"
#endif

#include "mcpwm/MCPWM.hpp"

#include <esp_log.h>

using namespace esp;

namespace esp {
    static ESP32Ptr _sharedESP32;
}

ESP32Ptr ESP32::sharedESP32() {
    if (_sharedESP32 == nullptr) {
#if defined(CONFIG_IDF_TARGET_ESP32S3) && CONFIG_IDF_TARGET_ESP32S3
        _sharedESP32 = std::make_shared<ESP32S3>();
#endif
    }

    return _sharedESP32;
}

ESP32::ESP32() : _mcpwm() {}

ADCOneshotPtr ESP32::adcOneshot(adc_unit_t unit, esp_err_t& err) {
    size_t unitNum = static_cast<size_t>(unit);
    if (numADCs() < unitNum) {
        err = ESP_ERR_INVALID_ARG;
        return nullptr;
    }

    ADCOneshotPtr adc = _adcs[unitNum];
    if (adc == nullptr) {
        adc = std::shared_ptr<ADCOneshot>(new ADCOneshot(unit, err));
        if (err != ESP_OK) {
            ESP_LOGE(_loggingTag, "ESP32::adcOneshot failed: %s", esp_err_to_name(err));
            return nullptr;
        }
        _adcs[unitNum] = adc;
    }

    return _adcs[unitNum];
}

GPIOPtr ESP32::gpio(GPIOConfig gpioConfig, esp_err_t& err) {
    if (numGPIOs() < gpioConfig.gpioNum) {
        err = ESP_ERR_INVALID_ARG;
        return nullptr;
    }

    GPIOPtr gpio = _gpios[gpioConfig.gpioNum];
    if (gpio == nullptr) {
        gpio = std::shared_ptr<GPIO>(new GPIO(gpioConfig, err));
        if (err != ESP_OK) {
            ESP_LOGE(_loggingTag, "ESP32::gpio failed: %s", esp_err_to_name(err));
            return nullptr;
        }
        _gpios[gpioConfig.gpioNum] = gpio;
    } else if (gpio->config() != gpioConfig) {
        err = ESP_ERR_INVALID_STATE;
        return nullptr;
    }

    return _gpios[gpioConfig.gpioNum];
}

