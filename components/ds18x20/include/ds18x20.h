/*
 * Modified
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include "onewire_device.h"
#include "ds18x20_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Type of DS18X20 device handle
 */
typedef struct ds18x20_device_t *ds18x20_device_handle_t;

/**
 * @brief DS18X20 configuration
 */
typedef struct {
} ds18x20_config_t;

/**
 * @brief Create a new DS18X20 device based on the general 1-Wire device
 *
 * @note The general 1-Wire device can be enumerated during the 1-Wire bus device search process,
 *       this function is going to check and upgrade that into a DS18X20 device.
 *
 * @param[in] device 1-Wire device basic information, including bus handle and device ROM ID
 * @param[in] config DS18X20 configuration
 * @param[out] ret_ds18x20 Returned DS18X20 device handle
 * @return
 *      - ESP_OK: Create DS18X20 device successfully
 *      - ESP_ERR_INVALID_ARG: Create DS18X20 device failed due to invalid argument
 *      - ESP_ERR_NO_MEM: Create DS18X20 device failed due to out of memory
 *      - ESP_ERR_NOT_SUPPORTED: Create DS18X20 device failed because the device is unknown (e.g. a wrong family ID code)
 *      - ESP_FAIL: Create DS18X20 device failed due to other reasons
 */
esp_err_t ds18x20_new_device(onewire_device_t *device, const ds18x20_config_t *config, ds18x20_device_handle_t *ret_ds18x20);

/**
 * @brief Delete DS18X20 device
 *
 * @param ds18x20 DS18X20 device handle returned by `ds18x20_new_device`
 * @return
 *      - ESP_OK: Delete DS18X20 device successfully
 *      - ESP_ERR_INVALID_ARG: Delete DS18X20 device failed due to invalid argument
 *      - ESP_FAIL: Delete DS18X20 device failed due to other reasons
 */
esp_err_t ds18x20_del_device(ds18x20_device_handle_t ds18x20);

/**
 * @brief Set DS18X20's temperature conversion resolution
 *
 * @param[in] ds18x20 DS18X20 device handle returned by `ds18x20_new_device`
 * @param[in] resolution resolution of DS18X20's temperature conversion
 * @return
 *      - ESP_OK: Set resolution successfully
 *      - ESP_ERR_INVALID_ARG: Set resolution failed due to invalid argument
 *      - ESP_FAIL: Set resolution failed due to other reasons
 */
esp_err_t ds18x20_set_resolution(ds18x20_device_handle_t ds18x20, ds18x20_resolution_t resolution);

/**
 * @brief Trigger temperature conversion of DS18X20
 *
 * @note After send the trigger command, the DS18X20 will start temperature conversion.
 *       This function will delay for some while, to ensure the temperature conversion won't be interrupted.
 *
 * @param[in] ds18x20 DS18X20 device handle returned by `ds18x20_new_device`
 * @return
 *      - ESP_OK: Trigger temperature conversion successfully
 *      - ESP_ERR_INVALID_ARG: Trigger temperature conversion failed due to invalid argument
 *      - ESP_FAIL: Trigger temperature conversion failed due to other reasons
 */
esp_err_t ds18x20_trigger_temperature_conversion(ds18x20_device_handle_t ds18x20);

/**
 * @brief Get temperature from DS18X20
 *
 * @param[in] ds18x20 DS18X20 device handle returned by `ds18x20_new_device`
 * @param[out] temperature conversion result from DS18X20
 * @return
 *      - ESP_OK: Get temperature successfully
 *      - ESP_ERR_INVALID_ARG: Get temperature failed due to invalid argument
 *      - ESP_ERR_INVALID_CRC: Get temperature failed due to CRC check error
 *      - ESP_FAIL: Get temperature failed due to other reasons
 */
esp_err_t ds18x20_get_temperature(ds18x20_device_handle_t ds18x20, float *temperature);

#ifdef __cplusplus
}
#endif
