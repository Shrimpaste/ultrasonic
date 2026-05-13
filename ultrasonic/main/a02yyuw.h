#pragma once

#include "driver/uart.h"
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
 * @brief Initialize UART1 for A02YYUW sensor
 * @param rx_pin UART RX GPIO pin number (GPIO18 = U1RXD)
 * @param tx_pin UART TX GPIO pin number (GPIO17 = U1TXD, keep HIGH for stable mode)
 * @return ESP_OK on success
 */
esp_err_t a02yyuw_init(int rx_pin, int tx_pin);

/**
 * @brief Read one distance measurement from A02YYUW
 * @param result Pointer to result struct
 * @return ESP_OK on valid read, ESP_FAIL on error
 */
esp_err_t a02yyuw_read(a02yyuw_result_t *result);

/**
 * @brief Deinitialize UART1
 */
void a02yyuw_deinit(void);

#ifdef __cplusplus
}
#endif
