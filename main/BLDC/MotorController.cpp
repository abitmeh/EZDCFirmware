/*
 * DragControlStrategy.hpp
 *
 * (c) Tom Davie 30/11/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#include "BLDC/MotorController.hpp"
#include "BLDC/AlignmentControlStrategy.hpp"
#include "BLDC/DragControlStrategy.hpp"
#include "BLDC/HaltControlStrategy.hpp"
#include "BLDC/Motor.hpp"
#include "BLDC/MotorConfig.hpp"
#include "BLDC/PulseInjectionControlStrategy.hpp"
#include "BLDC/SensorlessControlStrategy.hpp"

#include "ESP32.hpp"

#include <esp_log.h>

using namespace bldc;
using namespace esp;

namespace bldc {
    void _controlTask(void* userInfo) {
        MotorController* motorController = reinterpret_cast<MotorController*>(userInfo);
        motorController->_controlTask();
    }
}  // namespace bldc

MotorController::MotorController(const MotorControlConfig& config, esp_err_t& err) {
    _sleepValue = config.sleepValue;
    esp::GPIOConfig sleepGPIOConfig(config.sleepGPIONum, GPIO_MODE_OUTPUT);
    _sleepGPIO = ESP32::sharedESP32()->gpio(sleepGPIOConfig, err);
    if (_sleepGPIO == nullptr || err != ESP_OK) {
        ESP_LOGE(_loggingTag, "GPIO construction failed: %s", esp_err_to_name(err));
        return;
    }

    // Init gptimer
    GPTimerConfig timerConfig = {.durationMicroseconds = kAlarmCountMicroseconds,
                                 .callback = [this] IRAM_ATTR(GPTimer & timer, const gptimer_alarm_event_data_t& eventData) -> bool {
                                     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                                     xSemaphoreGiveFromISR(_timerFiredSemaphore, &xHigherPriorityTaskWoken);
                                     return xHigherPriorityTaskWoken;
                                 }};

    _timer = std::make_shared<GPTimer>(timerConfig, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "GPIO construction failed: %s", esp_err_to_name(err));
        return;
    }

    _timerFiredSemaphore = xSemaphoreCreateBinary();
    if (_timerFiredSemaphore == nullptr) {
        ESP_LOGE(_loggingTag, "xSemaphoreCreateBinary failed");
        return;
    }

    _mcpwmTimerEventCallbacks = config.motorConfig.inputSwitchConfig._callbacks;
    _mcpwmTimerEventCallbacks.onFull = [this, config] IRAM_ATTR(const mcpwm_timer_event_data_t& eventData) -> bool {
        if (_controllers[_controlPhase] != nullptr) {
            _controllers[_controlPhase]->mcpwmTimerFull();
        }

        if (config.motorConfig.inputSwitchConfig._callbacks.onFull) {
            return config.motorConfig.inputSwitchConfig._callbacks.onFull(eventData);
        } else {
            return false;
        }
    };
    MotorConfig motorConfig = config.motorConfig;
    motorConfig.inputSwitchConfig._callbacks = _mcpwmTimerEventCallbacks;

    _motor = std::make_shared<Motor>(motorConfig, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "Motor::Motor failed: %s", err);
        return;
    }

    HaltControlStrategyPtr haltControlStrategy = std::make_shared<HaltControlStrategy>(*_motor);
    _controllers = {
        std::make_shared<PulseInjectionControlStrategy>(*_motor),                                   // Pulse Injection
        std::make_shared<AlignmentControlStrategy>(*_motor),                                        // Alignment
        std::make_shared<DragControlStrategy>(*_motor, err),                                        // Drag
        std::make_shared<SensorlessControlStrategy>(config.sensorlessControlConfig, *_motor, err),  // Closed Loop
        haltControlStrategy,                                                                        // Stalled
        haltControlStrategy,                                                                        // Stopped
        haltControlStrategy,                                                                        // Fault
    };

    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "ZeroCrossingDetector construction failed: %s", err);
        return;
    }

    xTaskCreate(bldc::_controlTask, "BLDC Motor Controller", 1024 * 4, this, 10, NULL);
}

MotorController::~MotorController() {
    if (_timerFiredSemaphore != nullptr) {
        vSemaphoreDelete(_timerFiredSemaphore);
    }
}

void MotorController::start(uint32_t targetRPM, esp_err_t& err) {
    if (_running) {
        return setTargetRPM(targetRPM);
    }

    _sleepGPIO->setLevel(!_sleepValue, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "GPIO::setLevel failed: %s", esp_err_to_name(err));
        return;
    }

    _motor->start(targetRPM);

    _setControlPhase(PulseInjection, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "MotorController::_setControlPhase failed: %s", esp_err_to_name(err));
        return;
    }
    _timer->start(err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "bldc_gptimer_start failed: %s", esp_err_to_name(err));
        return;
    }

    _running = true;
}

void MotorController::stop(esp_err_t& err) {
    _timer->stop(err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "GPTimer::stop failed: %s", esp_err_to_name(err));
        err = ESP_FAIL;
        return;
    }

    _motor->stop();

    _sleepGPIO->setLevel(_sleepValue, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "GPIO::setLevel failed: %s", esp_err_to_name(err));
        err = ESP_FAIL;
        return;
    }

    _running = false;
}

Direction MotorController::direction() const {
    return _motor->direction();
}

void MotorController::setDirection(Direction direction) {
    _motor->setDirection(direction);
}

uint32_t MotorController::dutyCycle() const {
    return _motor->dutyCycle();
}

void MotorController::setDutyCycle(uint32_t dutyCycle) {
    _motor->setDutyCycle(dutyCycle);
}

uint32_t MotorController::rpm() const {
    return _motor->currentRPM();
}

uint32_t MotorController::targetRPM() const {
    return _motor->targetRPM();
}

void MotorController::setTargetRPM(uint32_t targetRPM) {
    _motor->setTargetRPM(targetRPM);
}

void MotorController::_controlTask() {
    while (true) {
        if (xSemaphoreTake(_timerFiredSemaphore, portMAX_DELAY)) {
            _tick();
        }
    }
}

void MotorController::_setControlPhase(ControlPhase phase, esp_err_t& err) {
    ESP_LOGD(_loggingTag, "Entering control phase: %s", to_string(phase).c_str());
    _controlPhase = phase;
    if (_controllers[phase] != nullptr) {
        _controllers[phase]->start(err);
        if (err != ESP_OK) {
            ESP_LOGE(_loggingTag, "PhaseChangeTimer::start() failed: %s", esp_err_to_name(err));
            return;
        }
    }
}

bool MotorController::_checkForStall(esp_err_t& err) {
    if (_motor->isStalled()) {
        _motor->stop();
        _setControlPhase(Stalled, err);
        if (err != ESP_OK) {
            ESP_LOGE(_loggingTag, "MotorController::_setControlPhase failed: %s", esp_err_to_name(err));
            return true;
        }
        return true;
    }

    return false;
}

void MotorController::_tick() {
    esp_err_t err = ESP_OK;

    MotorControlStrategyPtr& controller = _controllers[_controlPhase];

    _motor->tick();

    const bool stalled = _checkForStall(err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "MotorController::_checkForStall failed: %s", esp_err_to_name(err));
        return;
    }
    if (!stalled && _timeToNextStep.has_value() && _timeToNextStep.value() == 0) {
        _motor->turnIfNecessary();
        _timeToNextStep = std::optional<uint32_t>();
    }

    if (!_timeToNextStep.has_value()) {
        std::optional<std::pair<uint16_t, MotorStep>> nextStep = controller->nextStepChange();
        if (nextStep.has_value()) {
            _timeToNextStep = nextStep->first;
            _motor->setNextStep(nextStep->second);
        }
    } else {
        (*_timeToNextStep)--;
    }

    _motor->setDutyCycle(controller->dutyCycle());

    const std::optional<ControlPhase> nextControlPhase = controller->nextControlPhase(_controlPhase);
    if (nextControlPhase.has_value()) {
        _setControlPhase(nextControlPhase.value(), err);
        if (err != ESP_OK) {
            ESP_LOGE(_loggingTag, "MotorController::_setControlPhase failed: %s", esp_err_to_name(err));
            return;
        }
    }
}
