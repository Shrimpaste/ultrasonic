/**
 * @file urm09.h
 * @brief URM09 超声波传感器 I2C 驱动（V3.0 寄存器布局）
 */

#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int distance_cm;
    float temperature;
    bool valid;
} urm09_result_t;

/**
 * @brief 初始化 URM09 I2C 驱动
 * @param bus  输出：I2C 总线句柄
 * @param dev  输出：URM09 设备句柄
 * @param sda  SDA GPIO 引脚号
 * @param scl  SCL GPIO 引脚号
 */
esp_err_t urm09_init(i2c_master_bus_handle_t *bus,
                     i2c_master_dev_handle_t *dev,
                     int sda, int scl);

/**
 * @brief 触发测量并读取结果（约 120ms）
 */
esp_err_t urm09_read(i2c_master_dev_handle_t dev, urm09_result_t *result);

/**
 * @brief 释放 I2C 总线和设备
 */
void urm09_deinit(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t dev);

#ifdef __cplusplus
}
#endif
