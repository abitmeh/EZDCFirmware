/*
 * ADC.cpp
 *
 * (c) Tom Davie 29/11/2025 
 * 
 */

#include "ADC.hpp"

#include <esp_err.h>
#include <esp_log.h>
#include <esp_private/adc_private.h>

using namespace esp;

ADCChannelConfig::ADCChannelConfig() {}

ADCChannelConfig::ADCChannelConfig(adc_channel_t chan, adc_oneshot_chan_cfg_t config) : channel(chan), configuration(config) {}

namespace esp {
    bool operator==(const ADCChannelConfig& a, const ADCChannelConfig& b) {
        return a.channel == b.channel && a.configuration.atten == b.configuration.atten && a.configuration.bitwidth == b.configuration.bitwidth;
    }
}  // namespace esp

uint16_t ADCOneshotChannel::read() {
    int adc_raw_value = 0;
    adc_oneshot_read(_adc->_handle, _config.channel, &adc_raw_value);
    return static_cast<uint16_t>(adc_raw_value);
}

uint16_t ADCOneshotChannel::readIsr() {
    int adc_raw_value = 0;
    adc_oneshot_read_isr(_adc->_handle, _config.channel, &adc_raw_value);
    return static_cast<uint16_t>(adc_raw_value);
}

ADCOneshotChannel::ADCOneshotChannel(ADCOneshotPtr adc, const ADCChannelConfig& config, esp_err_t& err) : _adc(adc), _config(config) {
    err = adc_oneshot_config_channel(_adc->_handle, config.channel, &config.configuration);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err));
    }
}

ADCOneshotChannelPtr ADCOneshot::channel(const ADCChannelConfig& config, esp_err_t& err) {
    ADCOneshotChannelPtr chan = _channels[config.channel].lock();
    if (chan == nullptr) {
        chan = std::shared_ptr<ADCOneshotChannel>(new ADCOneshotChannel(shared_from_this(), config, err));
        _channels[config.channel] = std::weak_ptr<ADCOneshotChannel>(chan);
        return chan;
    }

    if (chan->_config == config) {
        return chan;
    }

    return nullptr;
}

ADCOneshot::ADCOneshot(adc_unit_t unitId, esp_err_t& err) {
    adc_oneshot_unit_init_cfg_t init_config = {.unit_id = unitId, .clk_src = ADC_RTC_CLK_SRC_DEFAULT, .ulp_mode = ADC_ULP_MODE_DISABLE};
    err = adc_oneshot_new_unit(&init_config, &_handle);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
    }
}

ADCOneshot::~ADCOneshot() {
    esp_err_t err = ESP_OK;
    err = adc_oneshot_del_unit(_handle);
    if (err != ESP_OK) {
        ESP_LOGE(_loggingTag, "adc_oneshot_del_unit failed: %s", esp_err_to_name(err));
    }
}
