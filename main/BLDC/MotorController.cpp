/*
 * DragControlStrategy.hpp
 *
 * (c) Tom Davie 30/11/2025
 * 
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#include "BLDC/Motor.hpp"
#include "BLDC/MotorConfig.hpp"
#include "BLDC/MotorController.hpp"
#include "BLDC/Strategies/AlignmentControlStrategy.hpp"
#include "BLDC/Strategies/DragControlStrategy.hpp"
#include "BLDC/Strategies/HaltControlStrategy.hpp"
#include "BLDC/Strategies/PulseInjectionControlStrategy.hpp"
#include "BLDC/Strategies/SensorlessConfig.hpp"
#include "BLDC/Strategies/SensorlessControlStrategy.hpp"
#include "BLDC/Types.hpp"
#include "Utilities/Tracer.hpp"

#include "ESP32.hpp"

#include <esp_log.h>
#include <chrono>

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
    GPTimerConfig timerConfig = {.durationMicroseconds = static_cast<std::chrono::microseconds>(1_tks).count(), .callback = bldc::_timerCallback};

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
        dragControlStrategy,                                      // Drag
        sensorlessControlStrategy,                                // Closed Loop
        haltControlStrategy,                                      // Stalled
        haltControlStrategy,                                      // Stopped
        haltControlStrategy,                                      // Fault
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

    _setControlMode(PulseInjection, err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "MotorController::_setControlMode failed: %s", esp_err_to_name(err));
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

void MotorController::_setControlMode(ControlMode phase, esp_err_t& err) {
    _controllers[_ControlMode]->stop(err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "MotorController::stop() failed: %s", esp_err_to_name(err));
        return;
    }
    _ControlMode = phase;
    _motor->setControlMode(phase);
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
        _setControlMode(Stalled, err);
        if (err != ESP_OK) {
            ESP_LOGE(_loggingTag, "MotorController::_setControlMode failed: %s", esp_err_to_name(err));
            return true;
        }
        return true;
    }

    return false;
}

void MotorController::_tick() {
    esp_err_t err = ESP_OK;

    MotorControlStrategyPtr& controller = _controllers[_ControlMode];

    _motor->tick();

    const bool stalled = _checkForStall(err);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "MotorController::_checkForStall failed: %s", esp_err_to_name(err));
        return;
    }
    if (stalled) {
        std::optional<ControlMode> nextControlMode = controller->nextControlMode(_ControlMode);
        if (nextControlMode.has_value()) {
            _setControlMode(nextControlMode.value(), err);
            if (err != ESP_OK) {
                ESP_LOGE(_loggingTag, "MotorController::_setControlMode failed: %s", esp_err_to_name(err));
                return;
            }
            controller = _controllers[_ControlMode];
        }
    } else if (_timeToNextCommutation.has_value() && _timeToNextCommutation.value() == 0_tks) {
        _motor->commutateIfNecessary();
        _timeToNextCommutation = std::optional<Ticks16>();

        std::optional<ControlMode> nextControlMode = controller->nextControlMode(_ControlMode);
        if (nextControlMode.has_value()) {
            _setControlMode(nextControlMode.value(), err);
            if (err != ESP_OK) {
                ESP_LOGE(_loggingTag, "MotorController::_setControlMode failed: %s", esp_err_to_name(err));
                return;
            }
            controller = _controllers[_ControlMode];
        }
    }

    if (!_timeToNextCommutation.has_value()) {
        std::optional<std::pair<Ticks16, PhaseAngle>> nextStep = controller->nextStepChange();
        if (nextStep.has_value()) {
            _timeToNextCommutation = nextStep->first;
            _motor->setNextStep(nextStep->second);
        }
    } else {
        (*_timeToNextCommutation)--;
    }

    float dutyCycle = controller->dutyCycle();
    _motor->setDutyCycle(dutyCycle * kMaxDutyCycle);

    Tracer* tracer = Tracer::sharedTracer();
    TraceSample* sample = tracer->currentSample();
    sample->controlMode = _ControlMode;
    sample->ticksToNextStep = _timeToNextCommutation.value_or(0_tks).count();
    sample->dutyCycle = static_cast<uint8_t>(dutyCycle * 255.0f);
    _motor->writeSample(sample);
    tracer->commitSample();
}
