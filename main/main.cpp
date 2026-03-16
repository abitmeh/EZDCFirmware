/*
 * main.cpp
 *
 * (c) Tom Davie 2/11/2025
 *
 */

#include "BLDC/MotorController.hpp"

#include <ADC/Continuous.hpp>
#include <ESP32.hpp>

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/idf_additions.h>

#include <array>
#include <cmath>
#include <ranges>
#include <string>

DRAM_ATTR static const char* loggingTag = "EZDC";

using namespace esp;
using namespace bldc;

static const gpio_num_t kMISOPin = GPIO_NUM_3;
static const gpio_num_t kMOSIPin = GPIO_NUM_11;
static const gpio_num_t kClkPin = GPIO_NUM_10;
static const gpio_num_t kDirPin = GPIO_NUM_37;
static const gpio_num_t kStepPin = GPIO_NUM_1;
static const gpio_num_t kDiagPin = GPIO_NUM_2;
static const gpio_num_t kCSPin = GPIO_NUM_9;
static const gpio_num_t kEnPin = GPIO_NUM_12;
static const gpio_num_t kBEMFUPin = GPIO_NUM_7;
static const gpio_num_t kBEMFVPin = GPIO_NUM_6;
static const gpio_num_t kBEMFWPin = GPIO_NUM_5;
static const gpio_num_t kBEMFVDDPin = GPIO_NUM_8;
static const gpio_num_t kMotorInUPin = GPIO_NUM_13;
static const gpio_num_t kMotorEnUPin = GPIO_NUM_14;
static const gpio_num_t kMotorInVPin = GPIO_NUM_15;
static const gpio_num_t kMotorEnVPin = GPIO_NUM_16;
static const gpio_num_t kMotorInWPin = GPIO_NUM_17;
static const gpio_num_t kMotorEnWPin = GPIO_NUM_18;
static const gpio_num_t kNotSleepPin = GPIO_NUM_33;
static const gpio_num_t kNotFaultPin = GPIO_NUM_21;
static const gpio_num_t kLEDRedPin = GPIO_NUM_36;
static const gpio_num_t kLEDGreenPin = GPIO_NUM_34;
static const gpio_num_t kLEDBluePin = GPIO_NUM_35;

using MotorControllerType = MotorController<ControlMode, kControlModeCount, ControlMode::Alignment>;
using MotorControllerTypePtr = MotorControllerPtr<ControlMode, kControlModeCount, ControlMode::Alignment>;

InterruptResult _handleMotorFault(const mcpwm_fault_event_data_t& faultData, void* userInfo) {
    ESP_DRAM_LOGI(loggingTag, "Motor fault!");
    return InterruptResult::NoHighPriorityTaskWoken;
}

class ControlDelegate : public bldc::ControlStrategyDelegate {
public:
    ControlDelegate(MotorControllerTypePtr& motorController) : _controller(motorController) {}

    virtual void controlStrategyMotorDidStall(const MotorControlStrategy& controlStrategy) {
        esp_err_t err = ESP_OK;
        _controller->setControlMode(ControlMode::Stalled, true, err);
        if (err != ESP_OK) {
            ESP_LOGE(_loggingTag, "!!!!!!!!!!!! COULD NOT ABORT MOTOR CONTROL DURING STALL !!!!!!!!!!!");
            return;
        }
    }

    virtual void controlStrategyDidComplete(const MotorControlStrategy& controlStrategy) {
        esp_err_t err = ESP_OK;
        switch (_controller->controlMode()) {
            case PulseInjection:
                _controller->setControlMode(Alignment, false, err);
                break;
            case Alignment:
                _controller->setControlMode(Drag, false, err);
                break;
            case Drag:
                _controller->setControlMode(ClosedLoop, false, err);
                break;
            case ClosedLoop:
                _controller->setControlMode(Stopped, false, err);
                break;
            case Stalled:
            case Stopped:
            case Fault:
                break;
        }
        if (err != ESP_OK) {
            ESP_LOGE(_loggingTag, "Failed to set motor control mode: %s", esp_err_to_name(err));
            return;
        }
    }

private:
    MotorControllerTypePtr _controller = nullptr;

    static constexpr char _loggingTag[] = "ControlDelegate";
};

extern "C" {

void app_main(void) {
    esp_err_t err = ESP_OK;

    esp_task_wdt_deinit();

    MotorADCConfig motorADCConfig;

    static const std::array<gpio_num_t, 4> channelPins{kBEMFUPin, kBEMFVPin, kBEMFWPin, kBEMFVDDPin};
    for (auto [i, gpio] : channelPins | std::views::enumerate) {
        std::pair<adc_unit_t, adc_channel_t> channelPair = esp::ESP32::adcChannelForGPIO(gpio, err);
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        motorADCConfig.unit = channelPair.first;
        motorADCConfig.channels[i] = channelPair.second;
    }

    MotorConfig motorConfig{
        .inputGPIOs = {kMotorInUPin, kMotorInVPin, kMotorInWPin},
        .enableGPIOs = {kMotorEnUPin, kMotorEnVPin, kMotorEnWPin},
        .adcConfig = motorADCConfig,
    };
    MotorControlConfig motorControllerConfig{
        .sleepGPIONum = kNotSleepPin,
        .sleepValue = false,
    };

    err = esp_event_loop_create_default();
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);

    MotorPtr motor = std::make_shared<Motor>(motorConfig, err);
    if (err != ESP_OK) {
        ESP_LOGE(loggingTag, "Motor::Motor failed: %s", err);
        return;
    }

    PulseInjectionControlStrategyPtr pulseInjectionStrategy = std::make_shared<PulseInjectionControlStrategy>(motor);
    AlignmentControlStrategyPtr alignmentStrategy = std::make_shared<AlignmentControlStrategy>(motor);

    DragControlStrategyPtr dragControlStrategy = std::make_shared<DragControlStrategy>(motor, err);
    if (err != ESP_OK) {
        ESP_LOGE(loggingTag, "DragControlStrategy construction failed: %s", esp_err_to_name(err));
        return;
    }

    SensorlessControlStrategyPtr sensorlessControlStrategy = std::make_shared<SensorlessControlStrategy>(motor, err);
    if (err != ESP_OK) {
        ESP_LOGE(loggingTag, "SensorlessControlStrategy construction failed: %s", esp_err_to_name(err));
        return;
    }

    sensorlessControlStrategy->setPIDParameters(kPidControlConfig.init_param, err);
    if (err != ESP_OK) {
        ESP_LOGE(loggingTag, "SensorlessControlStrategy::setPIDParameters failed: %s", esp_err_to_name(err));
        return;
    }
    HaltControlStrategyPtr haltControlStrategy = std::make_shared<HaltControlStrategy>(motor);

    std::array<MotorControlStrategyPtr, kControlModeCount> controlStrategies{
        pulseInjectionStrategy,     // Pulse Injection
        alignmentStrategy,          // Alignment
        dragControlStrategy,        // Drag
        sensorlessControlStrategy,  // Closed Loop
        haltControlStrategy,        // Stalled
        haltControlStrategy,        // Stopped
        haltControlStrategy         // Fault
    };
    MotorControllerTypePtr controller = std::make_shared<MotorControllerType>(motorControllerConfig, motor, controlStrategies, err);

    ControlDelegate controlDelegate(controller);
    pulseInjectionStrategy->setDelegate(&controlDelegate);
    alignmentStrategy->setDelegate(&controlDelegate);
    dragControlStrategy->setDelegate(&controlDelegate);
    sensorlessControlStrategy->setDelegate(&controlDelegate);
    haltControlStrategy->setDelegate(&controlDelegate);

    ESP_ERROR_CHECK_WITHOUT_ABORT(err);

    controller->configureMotorFaultHandling(kNotFaultPin, true, _handleMotorFault);

    controller->start(2000, err);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);

    vTaskDelay(8000 / portTICK_PERIOD_MS);
    controller->stop(err);
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);

    while (1) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
}
