/*
 * main.cpp
 *
 * (c) Tom Davie 2/11/2025
 *
 */

#include "BLDC/MotorController.hpp"

#include <driver/gpio.h>
#include <driver/mcpwm_cmpr.h>
#include <driver/mcpwm_fault.h>
#include <driver/mcpwm_gen.h>
#include <driver/mcpwm_oper.h>
#include <driver/mcpwm_timer.h>
#include <driver/spi_slave.h>
#include <esp_adc/adc_continuous.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

#include <array>
#include <cmath>
#include <ranges>
#include <string>

DRAM_ATTR static const char* tag = "EZDC";

using namespace bldc;

static const gpio_num_t kUARTTx = GPIO_NUM_43;
static const gpio_num_t kUARTRx = GPIO_NUM_44;
static const gpio_num_t kMISOPin = GPIO_NUM_4;
static const gpio_num_t kMOSIPin = GPIO_NUM_7;
static const gpio_num_t kClkPin = GPIO_NUM_6;
static const gpio_num_t kDirPin = GPIO_NUM_1;
static const gpio_num_t kStepPin = GPIO_NUM_2;
static const gpio_num_t kDiagPin = GPIO_NUM_3;
static const gpio_num_t kCSPin = GPIO_NUM_5;
static const gpio_num_t kEnPin = GPIO_NUM_8;
static const gpio_num_t kBEMFUPin = GPIO_NUM_12;
static const gpio_num_t kBEMFVPin = GPIO_NUM_11;
static const gpio_num_t kBEMFWPin = GPIO_NUM_10;
static const gpio_num_t kMotorInUPin = GPIO_NUM_13;
static const gpio_num_t kMotorEnUPin = GPIO_NUM_14;
static const gpio_num_t kMotorInVPin = GPIO_NUM_15;
static const gpio_num_t kMotorEnVPin = GPIO_NUM_16;
static const gpio_num_t kMotorInWPin = GPIO_NUM_17;
static const gpio_num_t kMotorEnWPin = GPIO_NUM_18;
static const gpio_num_t kNotSleepPin = GPIO_NUM_33;
static const gpio_num_t kNotFaultPin = GPIO_NUM_21;
static const gpio_num_t kLEDRedPin = GPIO_NUM_34;
static const gpio_num_t kLEDGreenPin = GPIO_NUM_35;
static const gpio_num_t kLEDBluePin = GPIO_NUM_36;

extern "C" {
void app_main(void) {
    esp_err_t err = ESP_OK;

    McpwmConfig inputSwitchConfig(0, kMotorInUPin, kMotorInVPin, kMotorInWPin);
    inputSwitchConfig.configureFaultHandling(kNotFaultPin, true, [] IRAM_ATTR(const mcpwm_fault_event_data_t& faultData) -> bool {
        ESP_DRAM_LOGI(tag, "Doom");
        return false;
    });
    McpwmConfig enableSwitchConfig(1, kMotorEnUPin, kMotorEnVPin, kMotorEnWPin);
    enableSwitchConfig.configureFaultHandling(kNotFaultPin, true, [] IRAM_ATTR(const mcpwm_fault_event_data_t& faultData) -> bool {
        ESP_DRAM_LOGI(tag, "Doom");
        return false;
    });

    SensorlessControlConfig sensorlessConfig;

    static const std::vector<gpio_num_t> channelPins{kBEMFUPin, kBEMFVPin, kBEMFWPin};
    for (auto [i, gpio] : channelPins | std::views::enumerate) {
        adc_unit_t unit;
        adc_channel_t channel;
        err = adc_oneshot_io_to_channel(gpio, &unit, &channel);
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        esp::ADCChannelConfig channelConfig(channel, {
                                                         .atten = ADC_ATTEN_DB_12,
                                                         .bitwidth = ADC_BITWIDTH_12,
                                                     });
        sensorlessConfig[i] = {unit, channelConfig};
    }

    MotorControlConfig config = {
        .motorConfig =
            {
                .inputSwitchConfig = inputSwitchConfig,
                .enableSwitchConfig = enableSwitchConfig,
            },
        .sensorlessControlConfig = sensorlessConfig,
        .sleepGPIONum = kNotSleepPin,
        .sleepValue = false,
    };

    err = esp_event_loop_create_default();
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);

    MotorController controller(config, err);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);

    controller.setDirection(Clockwise);
    controller.start(2000, err);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);

    while (1) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
}