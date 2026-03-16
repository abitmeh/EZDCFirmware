/*
 * MotorController.hpp
 *
 * (c) Tom Davie 30/11/2025
 *
 * Based in large part on Espressif's bldc motor driver found at
 * https://github.com/espressif/esp-iot-solution/tree/master/components/motor/esp_sensorless_bldc_control
 *
 */

#pragma once

#include "BLDC/McpwmContext.hpp"
#include "BLDC/Motor.hpp"
#include "BLDC/MotorConfig.hpp"
#include "BLDC/MotorController.hpp"
#include "BLDC/Strategies/AlignmentControlStrategy.hpp"
#include "BLDC/Strategies/DragControlStrategy.hpp"
#include "BLDC/Strategies/HaltControlStrategy.hpp"
#include "BLDC/Strategies/MotorControlStrategy.hpp"
#include "BLDC/Strategies/PulseInjectionControlStrategy.hpp"
#include "BLDC/Strategies/SensorlessConfig.hpp"
#include "BLDC/Strategies/SensorlessControlStrategy.hpp"
#include "BLDC/Types.hpp"
#include "Utilities/Tracer.hpp"

#include "ESP32.hpp"
#include "GPIO.hpp"
#include "GPTimer.hpp"
#include "Interrupt.hpp"

#include <esp_event.h>

#include <optional>
#include <type_traits>

namespace bldc {
    struct MotorControlConfig {
        gpio_num_t sleepGPIONum;
        bool sleepValue;
    };

    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    class MotorController;
    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    using MotorControllerPtr = std::shared_ptr<MotorController<CtrlMode, ModeCount, InitialMode>>;

    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    void _controlTask(void* userInfo);
    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    esp::InterruptResult _timerCallback(esp::GPTimer& timer, const gptimer_alarm_event_data_t& eventData, void* userInfo);

    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    class MotorController {
    public:
        MotorController(const MotorControlConfig& config, MotorPtr& motor, const std::array<MotorControlStrategyPtr, ModeCount>& controlStrategies,
                        esp_err_t& err);
        ~MotorController();

        void configureMotorFaultHandling(gpio_num_t gpio, bool inverted, esp::mcpwm::GPIOFault::Callback callback);

        void setControlMode(CtrlMode phase, bool now,
                            esp_err_t& err);  // If now is false, waits for the next commutation, and changes mode immediately after

        CtrlMode controlMode() const { return _controlMode; }

        void start(uint32_t targetRPM, esp_err_t& err);
        void stop(esp_err_t& err);

        float dutyCycle() const;

        uint32_t rpm() const;
        uint32_t targetRPM() const;
        void setTargetRPM(uint32_t targetRPM);

    private:
        void _controlTask();

        void _tick();

        bool _checkForStall(esp_err_t& err);

        esp::GPTimerPtr _timer;
        SemaphoreHandle_t _timerFiredSemaphore;

        MotorPtr _motor;

        CtrlMode _controlMode = InitialMode;
        CtrlMode _nextControlMode = InitialMode;
        std::array<MotorControlStrategyPtr, ModeCount> _strategies;
        std::optional<Ticks16> _timeToNextCommutation;
        bool _running = false;

        esp::mcpwm::Timer::EventCallbacks _mcpwmTimerEventCallbacks;

        esp::GPIOPtr _sleepGPIO;
        bool _sleepValue;

        esp::GPIOPtr _faultGPIO;
        bool _faultValue;
        esp::mcpwm::GPIOFault::Callback _faultCallback;

        static constexpr char _loggingTag[] = "bldc::MotorController";

        friend void bldc::_controlTask<CtrlMode, ModeCount, InitialMode>(void* userInfo);
        friend esp::InterruptResult _timerCallback<CtrlMode, ModeCount, InitialMode>(esp::GPTimer& timer, const gptimer_alarm_event_data_t& eventData,
                                                                                     void* userInfo);
    };

    // IMPLEMENTATION

    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    void _controlTask(void* userInfo) {
        MotorController<CtrlMode, ModeCount, InitialMode>* motorController = reinterpret_cast<MotorController<CtrlMode, ModeCount, InitialMode>*>(userInfo);
        motorController->_controlTask();
    }

    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    esp::InterruptResult _timerCallback(esp::GPTimer& timer, const gptimer_alarm_event_data_t& eventData, void* userInfo) {
        MotorController<CtrlMode, ModeCount, InitialMode>* motorController = reinterpret_cast<MotorController<CtrlMode, ModeCount, InitialMode>*>(userInfo);
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(motorController->_timerFiredSemaphore, &higherPriorityTaskWoken);
        return higherPriorityTaskWoken ? esp::InterruptResult::HighPriorityTaskWoken : esp::InterruptResult::NoHighPriorityTaskWoken;
    }

    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    MotorController<CtrlMode, ModeCount, InitialMode>::MotorController(const MotorControlConfig& config, MotorPtr& motor,
                                                                       const std::array<MotorControlStrategyPtr, ModeCount>& controlStrategies,
                                                                       esp_err_t& err) {
        _controlMode = InitialMode;
        _sleepValue = config.sleepValue;
        esp::GPIOConfig sleepGPIOConfig(config.sleepGPIONum, GPIO_MODE_OUTPUT);
        _sleepGPIO = esp::ESP32::sharedESP32()->gpio(sleepGPIOConfig, err);
        if (_sleepGPIO == nullptr || err != ESP_OK) {
            ESP_LOGE(_loggingTag, "GPIO construction failed: %s", esp_err_to_name(err));
            return;
        }

        // Init gptimer
        esp::GPTimerConfig timerConfig = {
            .durationMicroseconds = static_cast<std::chrono::microseconds>(1_tks).count(), .callback = bldc::_timerCallback<CtrlMode, ModeCount, InitialMode>};

        _timer = std::make_shared<esp::GPTimer>(timerConfig, this, err);
        if (err != ESP_OK) {
            ESP_LOGE(_loggingTag, "GPIO construction failed: %s", esp_err_to_name(err));
            return;
        }

        _timerFiredSemaphore = xSemaphoreCreateBinary();
        if (_timerFiredSemaphore == nullptr) {
            ESP_LOGE(_loggingTag, "xSemaphoreCreateBinary failed");
            return;
        }

        _motor = motor;

        _strategies = controlStrategies;

        xTaskCreate(bldc::_controlTask<CtrlMode, ModeCount, InitialMode>, "BLDC Motor Controller", 1024 * 4, this, tskIDLE_PRIORITY + 4, NULL);
    }

    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    MotorController<CtrlMode, ModeCount, InitialMode>::~MotorController() {
        if (_timerFiredSemaphore != nullptr) {
            vSemaphoreDelete(_timerFiredSemaphore);
        }
    }

    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    void MotorController<CtrlMode, ModeCount, InitialMode>::start(uint32_t targetRPM, esp_err_t& err) {
        if (_running) {
            return setTargetRPM(targetRPM);
        }

        _sleepGPIO->setLevel(esp::Level(!_sleepValue), err);
        if (err != ESP_OK) {
            ESP_LOGE(_loggingTag, "GPIO::setLevel failed: %s", esp_err_to_name(err));
            return;
        }

        _motor->start(targetRPM);

        _timer->start(err);
        if (err != ESP_OK) {
            ESP_LOGE(_loggingTag, "bldc_gptimer_start failed: %s", esp_err_to_name(err));
            return;
        }

        _running = true;
    }

    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    void MotorController<CtrlMode, ModeCount, InitialMode>::stop(esp_err_t& err) {
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

    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    void MotorController<CtrlMode, ModeCount, InitialMode>::configureMotorFaultHandling(gpio_num_t gpio, bool inverted,
                                                                                        esp::mcpwm::GPIOFault::Callback callback) {
        _motor->configureFaultHandling(gpio, inverted, callback);
    }

    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    float MotorController<CtrlMode, ModeCount, InitialMode>::dutyCycle() const {
        return static_cast<float>(_motor->dutyCycle());
    }

    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    uint32_t MotorController<CtrlMode, ModeCount, InitialMode>::rpm() const {
        return _motor->currentRPM();
    }

    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    uint32_t MotorController<CtrlMode, ModeCount, InitialMode>::targetRPM() const {
        return _motor->targetRPM();
    }

    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    void MotorController<CtrlMode, ModeCount, InitialMode>::setTargetRPM(uint32_t targetRPM) {
        _motor->setTargetRPM(targetRPM);
    }

    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    void MotorController<CtrlMode, ModeCount, InitialMode>::_controlTask() {
        while (true) {
            if (xSemaphoreTake(_timerFiredSemaphore, portMAX_DELAY)) {
                _tick();
            }
        }
    }

    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    void MotorController<CtrlMode, ModeCount, InitialMode>::setControlMode(CtrlMode newMode, bool now, esp_err_t& err) {
        if (now && _nextControlMode != _controlMode) {
            ControlStrategyTransferableState state = _strategies[_controlMode]->stop(err);
            if (err != ESP_OK) {
                ESP_LOGE(_loggingTag, "MotorController::stop() failed: %s", esp_err_to_name(err));
                return;
            }
            _controlMode = newMode;
            _nextControlMode = newMode;
            if (_strategies[newMode] != nullptr) {
                _strategies[newMode]->start(std::move(state), err);
                if (err != ESP_OK) {
                    ESP_LOGE(_loggingTag, "MotorController::start() failed: %s", esp_err_to_name(err));
                    return;
                }
            }
        } else {
            _nextControlMode = newMode;
        }
    }

    template <typename CtrlMode, size_t ModeCount, CtrlMode InitialMode>
        requires std::is_enum_v<CtrlMode>
    void MotorController<CtrlMode, ModeCount, InitialMode>::_tick() {
        esp_err_t err = ESP_OK;

        MotorControlStrategyPtr& strategy = _strategies[_controlMode];

        _motor->tick();

        if (_timeToNextCommutation.has_value() && _timeToNextCommutation.value() == 0_tks) {
            _motor->commutateIfNecessary();
            _timeToNextCommutation = std::optional<Ticks16>();

            if (_nextControlMode != _controlMode) {
                setControlMode(_nextControlMode, true, err);
                strategy = _strategies[_controlMode];
                if (err != ESP_OK) {
                    ESP_LOGE(_loggingTag, "MotorController::setControlMode failed: %s", esp_err_to_name(err));
                    return;
                }
            }
        }

        if (!_timeToNextCommutation.has_value()) {
            std::optional<Commutation> nextCommutation = strategy->tick();
            if (nextCommutation.has_value()) {
                _timeToNextCommutation = nextCommutation->first;
                _motor->setNextMotorState(nextCommutation->second);
            }
        } else {
            (*_timeToNextCommutation)--;
        }

        Tracer* tracer = Tracer::sharedTracer();
        TraceSample* sample = tracer->currentSample();
        sample->controlMode = static_cast<uint8_t>(_controlMode);
        sample->ticksToNextStep = _timeToNextCommutation.value_or(0_tks).count();
        sample->dutyCycle = static_cast<uint8_t>(dutyCycle() * 255.0f);
        _motor->writeSample(sample);
        tracer->commitSample();
    }
}  // namespace bldc

