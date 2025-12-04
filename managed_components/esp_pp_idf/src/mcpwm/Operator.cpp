/*
 * Operator.cpp
 *
 * (c) Tom Davie 03/12/2025 
 * 
 */

#include "mcpwm/Operator.hpp"

#include "mcpwm/Comparator.hpp"
#include "mcpwm/Generator.hpp"
#include "mcpwm/Timer.hpp"

#include <esp_log.h>

using namespace esp;
using namespace mcpwm;

Operator::Operator(TimerPtr timer, const OperatorConfig& config, esp_err_t& err) : _timer(timer) {
    const mcpwm_operator_config_t operatorConfig = {.group_id = _timer->_config.groupId,
                                                    .intr_priority = config.interuptPriority,
                                                    .flags{
                                                        .update_gen_action_on_tez = config.updateGeneratorActionOnTimerZero,
                                                        .update_gen_action_on_tep = config.updateGeneratorActionOnTimerPeak,
                                                        .update_gen_action_on_sync = config.updateGeneratorActionOnSync,
                                                        .update_dead_time_on_tez = false,
                                                        .update_dead_time_on_tep = false,
                                                        .update_dead_time_on_sync = false,
                                                    }};

    err = mcpwm_new_operator(&operatorConfig, &_operator);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_new_operator failed: %s", esp_err_to_name(err));
        return;
    }
    err = mcpwm_operator_connect_timer(_operator, timer->_timer);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_operator_connect_timer failed: %s", esp_err_to_name(err));
        return;
    }
}

Operator::~Operator() {
    esp_err_t err = ESP_OK;
    err = mcpwm_del_operator(_operator);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_del_operator failed: %s", esp_err_to_name(err));
        return;
    }
}

ComparatorPtr Operator::addComparator(const ComparatorConfig& config, esp_err_t& err) {
    ComparatorPtr comparator = std::shared_ptr<Comparator>(new Comparator(shared_from_this(), config, err));

    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm::Comparator::Comparator failed: %s", esp_err_to_name(err));
        return nullptr;
    }
    if (comparator == nullptr) {
        err = ESP_FAIL;
        ESP_LOGE(_loggingTag, "mcpwm::Comparator::Comparator failed: %s", esp_err_to_name(err));
        return nullptr;
    }

    return comparator;
}

GeneratorPtr Operator::addGenerator(const GeneratorConfig& config, esp_err_t& err) {
    GeneratorPtr generator = std::shared_ptr<Generator>(new Generator(shared_from_this(), config, err));

    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm::Generator::Generator failed: %s", esp_err_to_name(err));
        return nullptr;
    }
    if (generator == nullptr) {
        err = ESP_FAIL;
        ESP_LOGE(_loggingTag, "mcpwm::Generator::Generator failed: %s", esp_err_to_name(err));
        return nullptr;
    }

    return generator;
}
