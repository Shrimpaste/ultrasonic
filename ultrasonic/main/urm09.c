#include "urm09.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "URM09";

#define I2C_TIMEOUT_MS  100
#define MEASURE_WAIT_MS 120

esp_err_t urm09_init(i2c_master_bus_handle_t *bus_handle,
                     i2c_master_dev_handle_t *dev_handle,
                     int sda_pin, int scl_pin)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(ret));
        return ret;
    }

    const i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = URM09_I2C_ADDR,
        .scl_speed_hz = URM09_I2C_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(ret));
        i2c_del_master_bus(*bus_handle);
        return ret;
    }

    ret = i2c_master_probe(*bus_handle, URM09_I2C_ADDR, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "URM09 not found at 0x%02X: %s", URM09_I2C_ADDR, esp_err_to_name(ret));
        urm09_deinit(*bus_handle, *dev_handle);
        return ret;
    }

    ESP_LOGI(TAG, "URM09  OK — I2C0 SDA=GPIO%d SCL=GPIO%d addr=0x%02X", sda_pin, scl_pin, URM09_I2C_ADDR);
    return ESP_OK;
}

esp_err_t urm09_read(i2c_master_dev_handle_t dev_handle, urm09_result_t *result)
{
    result->valid = false;
    result->distance_cm = 0;
    result->temperature = 0.0f;

    /* Trigger measurement: write 0x01 to CMD register (0x08) */
    uint8_t cmd_buf[2] = {URM09_REG_CMD, URM09_CMD_MEASURE};
    esp_err_t ret = i2c_master_transmit(dev_handle, cmd_buf, sizeof(cmd_buf), I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Measure trigger failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Wait for measurement to complete (120ms for 500cm range) */
    vTaskDelay(pdMS_TO_TICKS(MEASURE_WAIT_MS));

    /* Read 4 bytes starting from DIST_H (0x03): [DIST_H, DIST_L, TEMP_H, TEMP_L] */
    uint8_t write_buf = URM09_REG_DIST_H;
    uint8_t read_buf[4] = {0};

    ret = i2c_master_transmit_receive(dev_handle, &write_buf, 1, read_buf, sizeof(read_buf), I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    int distance = (read_buf[0] << 8) | read_buf[1];
    int16_t temp_raw = (int16_t)((read_buf[2] << 8) | read_buf[3]);
    float temperature = temp_raw / 10.0f;

    /* Filter anomalous values: 0xFFFF and readings exceeding 500cm range */
    if (distance == 0xFFFF || distance > 500) {
        ESP_LOGW(TAG, "Abnormal distance %d cm, discarded", distance);
        return ESP_ERR_INVALID_SIZE;
    }

    /* 0 cm means out of range */
    if (distance == 0) {
        ESP_LOGW(TAG, "Out of range");
        return ESP_ERR_NOT_FOUND;
    }

    result->distance_cm = distance;
    result->temperature = temperature;
    result->valid = true;
    return ESP_OK;
}

void urm09_deinit(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t dev_handle)
{
    i2c_master_bus_rm_device(dev_handle);
    i2c_del_master_bus(bus_handle);
    ESP_LOGI(TAG, "URM09 deinitialized");
}
