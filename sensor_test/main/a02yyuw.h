/**
 * @file a02yyuw.h
 * @brief A02YYUW 超声波传感器 UART 驱动（V3.0，批量读取 + 帧扫描）
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int distance_mm;
    bool valid;
} a02yyuw_result_t;

/**
 * @brief 初始化 A02YYUW UART 驱动
 * @param rx_pin 传感器 TX 接 ESP32 的 GPIO（如 GPIO18）
 * @param tx_pin 传感器 RX 接 ESP32 的 GPIO（如 GPIO17），可悬空
 */
esp_err_t a02yyuw_init(int rx_pin, int tx_pin);

/**
 * @brief 读取一次距离（批量读取 + 帧扫描，约 100ms）
 */
esp_err_t a02yyuw_read(a02yyuw_result_t *result);

/**
 * @brief 释放 UART 资源
 */
void a02yyuw_deinit(void);

#ifdef __cplusplus
}
#endif
