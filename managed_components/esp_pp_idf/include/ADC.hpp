/*
 * ADC.hpp
 *
 * (c) Tom Davie 29/11/2025 
 * 
 */

#pragma once

#include <esp_adc/adc_oneshot.h>

#include <array>
#include <memory>
#include <vector>

namespace esp {
    class ESP32;

    struct ADCChannelConfig {
        ADCChannelConfig();
        ADCChannelConfig(adc_channel_t channel, adc_oneshot_chan_cfg_t configuration);

        adc_channel_t channel;
        adc_oneshot_chan_cfg_t configuration;
    };

    bool operator==(const ADCChannelConfig& a, const ADCChannelConfig& b);

    class ADCOneshotChannel;
    using ADCOneshotChannelPtr = std::shared_ptr<ADCOneshotChannel>;

    class ADCOneshot;
    using ADCOneshotPtr = std::shared_ptr<ADCOneshot>;

    class ADCOneshotChannel : private std::enable_shared_from_this<ADCOneshotChannel> {
    public:
        // Construct an ADCOneshotChannel by requesting one from an ADCOneshot.

        uint16_t read();
        IRAM_ATTR uint16_t readIsr();

    private:
        ADCOneshotChannel(ADCOneshotPtr adc, const ADCChannelConfig& config, esp_err_t& err);

        ADCOneshotPtr _adc;
        ADCChannelConfig _config;

        static constexpr char _loggingTag[] = "esp::ADCOneshotChannel";

        friend class ADCOneshot;
    };

    static constexpr size_t kADCUnitCount = 2;
    static constexpr size_t kADCChannelCount = 10;

    class ADCOneshot : public std::enable_shared_from_this<ADCOneshot> {
    public:
        // Construct ADCOneshot by requesting one from an ESP32.
        ~ADCOneshot();

        ADCOneshotChannelPtr channel(const ADCChannelConfig& config, esp_err_t& err);

    private:
        ADCOneshot(adc_unit_t unitId, esp_err_t& err);

        adc_oneshot_unit_handle_t _handle;

        std::array<std::weak_ptr<ADCOneshotChannel>, kADCChannelCount> _channels;

        static constexpr char _loggingTag[] = "esp::ADCOneshot";

        friend class ESP32;
        friend class ADCOneshotChannel;
    };
}  // namespace esp
