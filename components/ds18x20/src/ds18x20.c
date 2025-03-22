/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "onewire_bus.h"
#include "onewire_cmd.h"
#include "onewire_crc.h"
#include "ds18x20.h"

static const char *TAG = "ds18x20";

#define DS18X20_CMD_CONVERT_TEMP      0x44
#define DS18X20_CMD_WRITE_SCRATCHPAD  0x4E
#define DS18X20_CMD_READ_SCRATCHPAD   0xBE

#define DS_OLD_FAMILY 0x10 // DS1820 and DS18S20 family code
#define DS_FAMILY 0x28     // DS18X20 family code

/**
 * @brief Structure of DS18X20's scratchpad
 */
typedef struct  {
    uint8_t temp_lsb;      /*!< lsb of temperature */
    uint8_t temp_msb;      /*!< msb of temperature */
    uint8_t th_user1;      /*!< th register or user byte 1 */
    uint8_t tl_user2;      /*!< tl register or user byte 2 */
    uint8_t configuration; /*!< resolution configuration register */
    uint8_t _reserved1;
    uint8_t _reserved2;
    uint8_t _reserved3;
    uint8_t crc_value;     /*!< crc value of scratchpad data */
} __attribute__((packed)) ds18x20_scratchpad_t;

typedef struct ds18x20_device_t {
    onewire_bus_handle_t bus;
    onewire_device_address_t addr;
    uint8_t th_user1;
    uint8_t tl_user2;
    ds18x20_resolution_t resolution;
} ds18x20_device_t;

esp_err_t ds18x20_new_device(onewire_device_t *device, const ds18x20_config_t *config, ds18x20_device_handle_t *ret_ds18x20)
{
    ds18x20_device_t *ds18x20 = NULL;
    ESP_RETURN_ON_FALSE(device && config && ret_ds18x20, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    // check ROM ID, the family code of old DS (DS1820 and DS18S20) is 0x10
    // the family code of DS18B20 is 0x28
    uint8_t FamilyCode = device->address & 0xFF;
    if ((FamilyCode != DS_FAMILY) && (FamilyCode != DS_OLD_FAMILY)) {
        ESP_LOGD(TAG, "%016llX is not a DS18X20 device", device->address);
        return ESP_ERR_NOT_SUPPORTED;
    }
    ds18x20 = calloc(1, sizeof(ds18x20_device_t));
    ESP_RETURN_ON_FALSE(ds18x20, ESP_ERR_NO_MEM, TAG, "no mem for ds18x20");
    ds18x20->bus = device->bus;
    ds18x20->addr = device->address;
    if (FamilyCode == DS_OLD_FAMILY){
        ds18x20->resolution = DS18B20_RESOLUTION_9B; // DS1820 resolution only 9 bits
    } else {
        ds18x20->resolution = DS18B20_RESOLUTION_12B; // DS18B20 default resolution is 12 bits
    }
    *ret_ds18x20 = ds18x20;
    return ESP_OK;
}

esp_err_t ds18x20_del_device(ds18x20_device_handle_t ds18x20)
{
    ESP_RETURN_ON_FALSE(ds18x20, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    free(ds18x20);
    return ESP_OK;
}

static esp_err_t ds18x20_send_command(ds18x20_device_handle_t ds18x20, uint8_t cmd)
{
    // send command
    uint8_t tx_buffer[10] = {0};
    tx_buffer[0] = ONEWIRE_CMD_MATCH_ROM;
    memcpy(&tx_buffer[1], &ds18x20->addr, sizeof(ds18x20->addr));
    tx_buffer[sizeof(ds18x20->addr) + 1] = cmd;

    return onewire_bus_write_bytes(ds18x20->bus, tx_buffer, sizeof(tx_buffer));
}

esp_err_t ds18x20_set_resolution(ds18x20_device_handle_t ds18x20, ds18x20_resolution_t resolution)
{
    ESP_RETURN_ON_FALSE(ds18x20, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    // reset bus and check if the ds18x20 is present
    ESP_RETURN_ON_ERROR(onewire_bus_reset(ds18x20->bus), TAG, "reset bus error");

    // send command: DS18X20_CMD_WRITE_SCRATCHPAD
    ESP_RETURN_ON_ERROR(ds18x20_send_command(ds18x20, DS18X20_CMD_WRITE_SCRATCHPAD), TAG, "send DS18X20_CMD_WRITE_SCRATCHPAD failed");

    // write new resolution to scratchpad
    const uint8_t resolution_data[] = {0x1F, 0x3F, 0x5F, 0x7F};
    uint8_t tx_buffer[3] = {0};
    tx_buffer[0] = ds18x20->th_user1;
    tx_buffer[1] = ds18x20->tl_user2;
    tx_buffer[2] = resolution_data[resolution];
    ESP_RETURN_ON_ERROR(onewire_bus_write_bytes(ds18x20->bus, tx_buffer, sizeof(tx_buffer)), TAG, "send new resolution failed");

    ds18x20->resolution = resolution;
    return ESP_OK;
}

esp_err_t ds18x20_trigger_temperature_conversion(ds18x20_device_handle_t ds18x20)
{
    ESP_RETURN_ON_FALSE(ds18x20, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    // reset bus and check if the ds18x20 is present
    ESP_RETURN_ON_ERROR(onewire_bus_reset(ds18x20->bus), TAG, "reset bus error");

    // send command: DS18X20_CMD_CONVERT_TEMP
    ESP_RETURN_ON_ERROR(ds18x20_send_command(ds18x20, DS18X20_CMD_CONVERT_TEMP), TAG, "send DS18X20_CMD_CONVERT_TEMP failed");

    // delay proper time for temperature conversion
    const uint32_t delays_ms[] = {100, 200, 400, 800};
    vTaskDelay(pdMS_TO_TICKS(delays_ms[ds18x20->resolution]));

    return ESP_OK;
}

esp_err_t ds18x20_get_temperature(ds18x20_device_handle_t ds18x20, float *ret_temperature)
{
    int16_t temperature_raw;
    ESP_RETURN_ON_FALSE(ds18x20 && ret_temperature, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    // reset bus and check if the ds18x20 is present
    ESP_RETURN_ON_ERROR(onewire_bus_reset(ds18x20->bus), TAG, "reset bus error");

    // send command: DS18X20_CMD_READ_SCRATCHPAD
    ESP_RETURN_ON_ERROR(ds18x20_send_command(ds18x20, DS18X20_CMD_READ_SCRATCHPAD), TAG, "send DS18X20_CMD_READ_SCRATCHPAD failed");

    // read scratchpad data
    ds18x20_scratchpad_t scratchpad;
    ESP_RETURN_ON_ERROR(onewire_bus_read_bytes(ds18x20->bus, (uint8_t *)&scratchpad, sizeof(scratchpad)),
                        TAG, "error while reading scratchpad data");
    // check crc
    ESP_RETURN_ON_FALSE(onewire_crc8(0, (uint8_t *)&scratchpad, 8) == scratchpad.crc_value, ESP_ERR_INVALID_CRC, TAG, "scratchpad crc error");

    if ((ds18x20->addr & 0xFF) == DS_OLD_FAMILY) {
        temperature_raw = ((scratchpad.temp_msb & ~0x07) << 8) | (scratchpad.temp_lsb << 3);
    } else {
        const uint8_t lsb_mask[4] = {0x07, 0x03, 0x01, 0x00}; // mask bits not used in low resolution
        uint8_t lsb_masked = scratchpad.temp_lsb & (~lsb_mask[scratchpad.configuration >> 5]);
        // Combine the MSB and masked LSB into a signed 16-bit integer
        temperature_raw = (((int16_t)scratchpad.temp_msb << 8) | lsb_masked);
    }
    // Convert the raw temperature to a float,
    *ret_temperature = temperature_raw / 16.0f;
    return ESP_OK;

}
