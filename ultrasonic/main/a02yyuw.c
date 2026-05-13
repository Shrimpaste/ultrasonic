#include "a02yyuw.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "A02YYUW";

#define A02_UART_PORT   UART_NUM_1
#define A02_BAUDRATE    9600
#define A02_RX_BUF_SIZE 256

esp_err_t a02yyuw_init(int rx_pin, int tx_pin)
{
    const uart_config_t cfg = {
        .baud_rate  = A02_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_param_config(A02_UART_PORT, &cfg), TAG, "uart_param_config");
    ESP_RETURN_ON_ERROR(uart_set_pin(A02_UART_PORT, tx_pin, rx_pin,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "uart_set_pin");
    ESP_RETURN_ON_ERROR(uart_driver_install(A02_UART_PORT, A02_RX_BUF_SIZE, 0, 0, NULL, 0),
                        TAG, "uart_driver_install");
    ESP_LOGI(TAG, "A02YYUW OK — UART1 RX=GPIO%d TX=GPIO%d", rx_pin, tx_pin);
    return ESP_OK;
}

/**
 * @brief Read one A02YYUW frame using batch-read approach
 *
 * Flush old data → wait 100ms for a fresh frame → batch read → scan for 0xFF header → take last valid frame.
 * This avoids stale data and ensures we always get the most recent reading.
 */
esp_err_t a02yyuw_read(a02yyuw_result_t *result)
{
    result->valid = false;
    result->distance_mm = 0;

    /* Flush stale data in UART buffer */
    uart_flush_input(A02_UART_PORT);

    /* Wait for a fresh frame to arrive (sensor sends ~10Hz, 100ms is enough) */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Batch read all available bytes */
    uint8_t buf[64];
    int total = uart_read_bytes(A02_UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(30));
    if (total < 4) {
        return ESP_ERR_TIMEOUT;
    }

    /* Scan buffer for 0xFF frame headers, take the LAST valid frame (most recent) */
    int last_frame = -1;
    for (int i = 0; i <= total - 4; i++) {
        if (buf[i] == 0xFF) {
            uint8_t calc = (0xFF + buf[i + 1] + buf[i + 2]) & 0xFF;
            if (calc == buf[i + 3]) {
                last_frame = i;
            }
        }
    }

    if (last_frame < 0) {
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t dist = ((uint16_t)buf[last_frame + 1] << 8) | buf[last_frame + 2];
    if (dist == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    result->distance_mm = (int)dist;
    result->valid = true;
    return ESP_OK;
}

void a02yyuw_deinit(void)
{
    uart_driver_delete(A02_UART_PORT);
    ESP_LOGI(TAG, "A02YYUW deinitialized");
}
