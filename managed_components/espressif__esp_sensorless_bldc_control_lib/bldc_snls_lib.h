/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Obtain the current phase angle through the injected ADC value.
 *
 * @param injectADCValue adc values
 * @return uint8_t phase 0-7
 */
uint8_t inject_get_phase(uint32_t *injectADCValue);

/**
 * @brief ADC detection of zero-crossing.
 *
 * @param phase Current phase angle.
 * @param repeat_detect Repeat detection.
 * @param adc_value adc values
 * @return uint8_t phase 1-6
 */
uint8_t zero_cross_adc_get_phase(uint8_t phase, uint8_t repeat_detect, uint32_t *adc_value);

/**
 * @brief Comparator detection of zero-crossing.
 *
 * @param queueFilterState queueFilterState
 * @param dir motor direction
 * @return uint8_t phase 0-7
 */
uint8_t zero_cross_comparer_get_phase(uint16_t *queueFilterState, uint8_t dir);

#ifdef __cplusplus
}
#endif
