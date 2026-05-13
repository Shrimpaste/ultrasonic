/**
 * @file urm09.c
 * @brief URM09 超声波传感器 I2C 驱动实现（V3.0 寄存器布局）
 *
 * 寄存器（V3.0）：
 *   0x08 CMD    写 0x01 触发测距
 *   0x03 DIST_H 距离高字节 (cm)
 *   0x04 DIST_L 距离低字节 (cm)
 *   0x05 TEMP_H 温度高字节 (×0.1°C)
 *   0x06 TEMP_L 温度低字节
 */

#include "urm09.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "URM09";

#define URM09_ADDR          0x11
#define URM09_REG_CMD       0x08
#define URM09_REG_DIST_H    0x03
#define URM09_CMD_MEASURE   0x01

esp_err_t urm09_init(i2c_master_bus_handle_t *bus,
                     i2c_master_dev_handle_t *dev,
                     int sda, int scl)
{
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port   = I2C_NUM_0,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(ret));
        return ret;
    }

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = URM09_ADDR,
        .scl_speed_hz    = 100000,
    };

    ret = i2c_master_bus_add_device(*bus, &dev_cfg, dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(ret));
        i2c_del_master_bus(*bus);
        return ret;
    }

    ret = i2c_master_probe(*bus, URM09_ADDR, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "URM09 not found at 0x%02X: %s", URM09_ADDR, esp_err_to_name(ret));
        urm09_deinit(*bus, *dev);
        return ret;
    }

    ESP_LOGI(TAG, "URM09 OK — I2C0 SDA=GPIO%d SCL=GPIO%d", sda, scl);
    return ESP_OK;
}

esp_err_t urm09_read(i2c_master_dev_handle_t dev, urm09_result_t *result)
{
    result->distance_cm = 0;
    result->temperature = 0.0f;
    result->valid = false;

    /* 触发测量 */
    uint8_t cmd[2] = {URM09_REG_CMD, URM09_CMD_MEASURE};
    esp_err_t ret = i2c_master_transmit(dev, cmd, 2, 100);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "measure trigger failed: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(120));

    /* 读取距离(2B) + 温度(2B) */
    uint8_t reg = URM09_REG_DIST_H;
    uint8_t data[4] = {0};
    ret = i2c_master_transmit_receive(dev, &reg, 1, data, 4, 100);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    int dist = (data[0] << 8) | data[1];
    int16_t temp_raw = (int16_t)((data[2] << 8) | data[3]);

    /* 过滤异常值 */
    if (dist == 0xFFFF || dist > 500) {
        ESP_LOGW(TAG, "distance anomalous: %d cm", dist);
        return ESP_ERR_INVALID_SIZE;
    }

    result->distance_cm = dist;
    result->temperature = temp_raw / 10.0f;
    result->valid = true;
    return ESP_OK;
}

void urm09_deinit(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t dev)
{
    i2c_master_bus_rm_device(dev);
    i2c_del_master_bus(bus);
    ESP_LOGI(TAG, "URM09 deinitialized");
}
