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
#include "Utilities/Tracer.hpp"

#include "ESP32.hpp"

#include <esp_log.h>

using namespace bldc;
using namespace esp;

namespace bldc {
    void _controlTask(void* userInfo) {
        MotorController* motorController = reinterpret_cast<MotorController*>(userInfo);
        motorController->_controlTask();
    }

    InterruptResult _timerCallback(GPTimer& timer, const gptimer_alarm_event_data_t& eventData, void* userInfo) {
        MotorController* motorController = reinterpret_cast<MotorController*>(userInfo);
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(motorController->_timerFiredSemaphore, &higherPriorityTaskWoken);
        return higherPriorityTaskWoken ? InterruptResult::HighPriorityTaskWoken : InterruptResult::NoHighPriorityTaskWoken;
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
    GPTimerConfig timerConfig = {.durationMicroseconds = kTickUs,
                                 .callback = bldc::_timerCallback };

    _timer = std::make_shared<GPTimer>(timerConfig, this, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "GPIO construction failed: %s", esp_err_to_name(err));
        return;
    }

    _timerFiredSemaphore = xSemaphoreCreateBinary();
    if (_timerFiredSemaphore == nullptr) {
        ESP_LOGE(_loggingTag, "xSemaphoreCreateBinary failed");
        return;
    }

    _motor = std::make_shared<Motor>(config.motorConfig, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "Motor::Motor failed: %s", err);
        return;
    }

    DragControlStrategyPtr dragControlStrategy = std::make_shared<DragControlStrategy>(_motor, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "DragControlStrategy construction failed: %s", esp_err_to_name(err));
        return;
    }

    SensorlessControlStrategyPtr sensorlessControlStrategy = std::make_shared<SensorlessControlStrategy>(_motor, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "SensorlessControlStrategy construction failed: %s", esp_err_to_name(err));
        return;
    }

    sensorlessControlStrategy->setPIDParameters(kPidControlConfig.init_param, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "SensorlessControlStrategy::setPIDParameters failed: %s", esp_err_to_name(err));
        return;
    }
    HaltControlStrategyPtr haltControlStrategy = std::make_shared<HaltControlStrategy>(_motor);

    _controllers = {
        std::make_shared<PulseInjectionControlStrategy>(_motor),  // Pulse Injection
        std::make_shared<AlignmentControlStrategy>(_motor),       // Alignment
        dragControlStrategy,       // Drag
        sensorlessControlStrategy, // Closed Loop
        haltControlStrategy,                                       // Stalled
        haltControlStrategy,                                       // Stopped
        haltControlStrategy,                                       // Fault
    };

    xTaskCreate(bldc::_controlTask, "BLDC Motor Controller", 1024 * 4, this, tskIDLE_PRIORITY + 4, NULL);
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

    _sleepGPIO->setLevel(esp::Level(!_sleepValue), err);
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

    _sleepGPIO->setLevel(esp::Level(_sleepValue), err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "GPIO::setLevel failed: %s", esp_err_to_name(err));
        err = ESP_FAIL;
        return;
    }

    _running = false;
}

void MotorController::configureMotorFaultHandling(gpio_num_t gpio, bool inverted, esp::mcpwm::GPIOFault::Callback callback) {
    _motor->configureFaultHandling(gpio, inverted, callback);
}

Direction MotorController::direction() const {
    return _motor->direction();
}

void MotorController::setDirection(Direction direction) {
    _motor->setDirection(direction);
}

float MotorController::dutyCycle() const {
    return static_cast<float>(_motor->dutyCycle()) / static_cast<float>(kMaxDutyCycle);
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
    _controllers[_controlPhase]->stop(err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "MotorController::stop() failed: %s", esp_err_to_name(err));
        return;
    }
    _controlPhase = phase;
    _motor->setControlPhase(phase);
    if (_controllers[phase] != nullptr) {
        _controllers[phase]->start(err);
        if (err != ESP_OK) {
            ESP_LOGE(_loggingTag, "MotorController::start() failed: %s", esp_err_to_name(err));
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
    if (stalled) {
        std::optional<ControlPhase> nextControlPhase = controller->nextControlPhase(_controlPhase);
        if (nextControlPhase.has_value()) {
            _setControlPhase(nextControlPhase.value(), err);
            if (err != ESP_OK) {
                ESP_LOGE(_loggingTag, "MotorController::_setControlPhase failed: %s", esp_err_to_name(err));
                return;
            }
            controller = _controllers[_controlPhase];
        }
    } else if (_ticksToNextStep.has_value() && _ticksToNextStep.value() == 0) {
        _motor->commutateIfNecessary();
        _ticksToNextStep = std::optional<uint32_t>();

        std::optional<ControlPhase> nextControlPhase = controller->nextControlPhase(_controlPhase);
        if (nextControlPhase.has_value()) {
            _setControlPhase(nextControlPhase.value(), err);
            if (err != ESP_OK) {
                ESP_LOGE(_loggingTag, "MotorController::_setControlPhase failed: %s", esp_err_to_name(err));
                return;
            }
            controller = _controllers[_controlPhase];
        }
    }

    if (!_ticksToNextStep.has_value()) {
        std::optional<std::pair<uint16_t, MotorStep>> nextStep = controller->nextStepChange();
        if (nextStep.has_value()) {
            _ticksToNextStep = nextStep->first;
            _motor->setNextStep(nextStep->second);
        }
    } else {
        (*_ticksToNextStep)--;
    }

    float dutyCycle = controller->dutyCycle();
    _motor->setDutyCycle(dutyCycle * kMaxDutyCycle);

    Tracer* tracer = Tracer::sharedTracer();
    tracer->setControlMode(_controlPhase);
    tracer->setTicksToNextStep(_ticksToNextStep.value_or(0));
    tracer->setDutyCycle(dutyCycle);
    tracer->setCurrentRPM(_motor->currentRPM());
    tracer->setTargetRPM(_motor->targetRPM());
    tracer->setError(Error::None); // TO DO: Report the error correctly
    tracer->tick();
}
