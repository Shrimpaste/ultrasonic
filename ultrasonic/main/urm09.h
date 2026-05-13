#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define URM09_I2C_ADDR      0x11
#define URM09_I2C_FREQ_HZ   100000

/* V3.0 Register Map (verified via hardware debug) */
#define URM09_REG_CMD       0x08    /* Command register: write 0x01 to trigger measurement */
#define URM09_REG_DIST_H    0x03    /* Distance high byte */
#define URM09_REG_DIST_L    0x04    /* Distance low byte */
#define URM09_REG_TEMP_H    0x05    /* Temperature high byte (×0.1°C, signed) */
#define URM09_REG_TEMP_L    0x06    /* Temperature low byte */

/* Measurement command */
#define URM09_CMD_MEASURE   0x01

typedef struct {
    int distance_cm;
    float temperature;
    bool valid;
} urm09_result_t;

/**
 * @brief Initialize I2C bus and add URM09 device
 * @param bus_handle Output: I2C bus handle
 * @param dev_handle Output: URM09 device handle
 * @param sda_pin SDA GPIO pin number (GPIO8)
 * @param scl_pin SCL GPIO pin number (GPIO9)
 * @return ESP_OK on success
 */
esp_err_t urm09_init(i2c_master_bus_handle_t *bus_handle,
                     i2c_master_dev_handle_t *dev_handle,
                     int sda_pin, int scl_pin);

/**
 * @brief Trigger measurement and read result
 * @param dev_handle URM09 device handle
 * @param result Pointer to result struct
 * @return ESP_OK on valid read
 */
esp_err_t urm09_read(i2c_master_dev_handle_t dev_handle, urm09_result_t *result);

/**
 * @brief Deinitialize I2C bus and remove device
 * @param bus_handle I2C bus handle
 * @param dev_handle URM09 device handle
 */
void urm09_deinit(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t dev_handle);

#ifdef __cplusplus
}
#endif
