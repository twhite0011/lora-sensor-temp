#include "shtc3.h"

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lora_config.h"

#define SHTC3_I2C_ADDR 0x70
#define SHTC3_CMD_WAKEUP 0x3517
#define SHTC3_CMD_SLEEP 0xB098
#define SHTC3_CMD_MEASURE_TFIRST 0x7866

static const char *TAG = "shtc3";

static esp_err_t shtc3_write_cmd(uint16_t cmd)
{
    uint8_t data[2] = {(uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF)};
    return i2c_master_write_to_device(I2C_PORT_NUM, SHTC3_I2C_ADDR, data, sizeof(data), pdMS_TO_TICKS(100));
}

static uint8_t shtc3_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

esp_err_t shtc3_init(void)
{
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    ESP_RETURN_ON_ERROR(i2c_param_config(I2C_PORT_NUM, &cfg), TAG, "i2c_param_config failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(I2C_PORT_NUM, cfg.mode, 0, 0, 0), TAG, "i2c_driver_install failed");

    // Ensure sensor is awake before measurements.
    esp_err_t err = shtc3_write_cmd(SHTC3_CMD_WAKEUP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wake command failed: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(2));
    return ESP_OK;
}

esp_err_t shtc3_read(float *temperature_c, float *humidity_rh)
{
    if (temperature_c == NULL || humidity_rh == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(shtc3_write_cmd(SHTC3_CMD_WAKEUP), TAG, "wake command failed");
    vTaskDelay(pdMS_TO_TICKS(2));

    ESP_RETURN_ON_ERROR(shtc3_write_cmd(SHTC3_CMD_MEASURE_TFIRST), TAG, "measure command failed");
    vTaskDelay(pdMS_TO_TICKS(15));

    uint8_t raw[6] = {0};
    ESP_RETURN_ON_ERROR(
        i2c_master_read_from_device(I2C_PORT_NUM, SHTC3_I2C_ADDR, raw, sizeof(raw), pdMS_TO_TICKS(100)),
        TAG,
        "read failed");

    if (shtc3_crc8(raw, 2) != raw[2] || shtc3_crc8(&raw[3], 2) != raw[5]) {
        ESP_LOGE(TAG, "crc mismatch");
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t t_ticks = (uint16_t)((raw[0] << 8) | raw[1]);
    uint16_t h_ticks = (uint16_t)((raw[3] << 8) | raw[4]);

    *temperature_c = -45.0f + 175.0f * ((float)t_ticks / 65535.0f);
    *humidity_rh = 100.0f * ((float)h_ticks / 65535.0f);

    // Put sensor back to sleep between wakes.
    (void)shtc3_write_cmd(SHTC3_CMD_SLEEP);

    return ESP_OK;
}

void shtc3_deinit(void)
{
    (void)shtc3_write_cmd(SHTC3_CMD_SLEEP);
    (void)i2c_driver_delete(I2C_PORT_NUM);
}
