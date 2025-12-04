/*
 * Comparator.cpp
 *
 * (c) Tom Davie 03/12/2025 
 * 
 */

#include "mcpwm/Comparator.hpp"

#include "mcpwm/Operator.hpp"

#include <esp_log.h>

using namespace esp;
using namespace mcpwm;

Comparator::Comparator(OperatorPtr oper, const ComparatorConfig& config, esp_err_t& err) : _operator(oper) {
    const mcpwm_comparator_config_t comparatorConfig = {.intr_priority = config.interuptPriority,
                                                        .flags = {
                                                            .update_cmp_on_tez = config.updateComparatorOnTimerZero,
                                                            .update_cmp_on_tep = config.updateComparatorOnTimerPeak,
                                                            .update_cmp_on_sync = config.updateComparatorOnSync,
                                                        }};

    err = mcpwm_new_comparator(oper->_operator, &comparatorConfig, &_comparator);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_new_comparator failed: %s", esp_err_to_name(err));
        return;
    }
    setCompareValue(0, err);
}

Comparator::~Comparator() {
    esp_err_t err = ESP_OK;
    err = mcpwm_del_comparator(_comparator);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_del_comparator failed: %s", esp_err_to_name(err));
        return;
    }
}

uint32_t Comparator::compareValue() const {
    return _compareValue;
}

void Comparator::setCompareValue(uint32_t compareValue, esp_err_t& err) {
    err = mcpwm_comparator_set_compare_value(_comparator, compareValue);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_comparator_set_compare_value failed: %s", esp_err_to_name(err));
        return;
    }
}
