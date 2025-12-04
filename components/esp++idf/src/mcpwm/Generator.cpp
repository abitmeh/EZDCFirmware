/*
 * Generator.cpp
 *
 * (c) Tom Davie 03/12/2025 
 * 
 */

#include "mcpwm/Generator.hpp"

#include "mcpwm/Comparator.hpp"
#include "mcpwm/Operator.hpp"

#include <esp_log.h>

using namespace esp;
using namespace mcpwm;

Generator::CompareEventAction::CompareEventAction(mcpwm_timer_direction_t dir, ComparatorPtr cmp, mcpwm_generator_action_t act)
    : direction(dir), comparator(cmp), action(act) {}

Generator::Generator(OperatorPtr oper, const GeneratorConfig& config, esp_err_t& err) : _operator(oper) {
    mcpwm_generator_config_t generatorConfig = {.gen_gpio_num = config.gpioNum,
                                                .flags{
                                                    .invert_pwm = false,
                                                    .io_loop_back = false,
                                                    .io_od_mode = false,
                                                    .pull_up = false,
                                                    .pull_down = false,
                                                }};
    err = mcpwm_new_generator(oper->_operator, &generatorConfig, &_generator);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_new_generator failed: %s", esp_err_to_name(err));
        return;
    }
}

Generator::~Generator() {
    esp_err_t err = ESP_OK;
    err = mcpwm_del_generator(_generator);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_del_generator failed: %s", esp_err_to_name(err));
        return;
    }
}

void Generator::setActionOnCompareEvent(const CompareEventAction& action, esp_err_t& err) {
    mcpwm_gen_compare_event_action_t mcpwmAction = {.direction = action.direction, .comparator = action.comparator->_comparator, .action = action.action};
    err = mcpwm_generator_set_action_on_compare_event(_generator, mcpwmAction);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "mcpwm_generator_set_action_on_compare_event failed: %s", esp_err_to_name(err));
        return;
    }
}

void Generator::setActionsOnCompareEvent(std::initializer_list<CompareEventAction> actions, esp_err_t& err) {
    for (const CompareEventAction& action : actions) {
        setActionOnCompareEvent(action, err);
        if (err != ESP_OK) {
            return;
        }
    }
}
