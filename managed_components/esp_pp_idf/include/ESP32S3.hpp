/*
 * ESP32S3.hpp
 *
 * (c) Tom Davie 29/11/2025 
 * 
 */

#include "ESP32.hpp"

namespace esp {
    class ESP32S3 : public ESP32 {
    public:
        ESP32S3() {
            _adcs.resize(numADCs());
            _gpios.resize(numGPIOs());
        }

        virtual size_t numADCs() override { return kNumADCs; }

        virtual size_t numGPIOs() override { return kNumGPIOs; }

    private:
        static constexpr size_t kNumADCs = 2;
        static constexpr size_t kNumGPIOs = 49;
    };
}  // namespace esp
