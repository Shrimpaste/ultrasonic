/**
 * @file a02yyuw.c
 * @brief A02YYUW 超声波传感器 UART 驱动实现
 *
 * 帧格式（4字节）：[0xFF] [DATA_H] [DATA_L] [SUM]
 * SUM = (0xFF + DATA_H + DATA_L) & 0xFF
 * 距离 = DATA_H * 256 + DATA_L (mm)
 */

#include "a02yyuw.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "A02YYUW";

#define A02_UART_PORT   UART_NUM_1
#define A02_BAUDRATE    9600
#define A02_RX_BUF_SIZE 256

static int s_rx_pin = -1;

esp_err_t a02yyuw_init(int rx_pin, int tx_pin)
{
    s_rx_pin = rx_pin;

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

esp_err_t a02yyuw_read(a02yyuw_result_t *result)
{
    result->distance_mm = 0;
    result->valid = false;

    /* 清空旧数据 */
    uart_flush_input(A02_UART_PORT);

    /* 等一帧到达（传感器约 100ms 发一帧） */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 批量读取缓冲区所有数据 */
    uint8_t buf[64];
    int total = uart_read_bytes(A02_UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(30));
    if (total < 4) {
        return ESP_ERR_TIMEOUT;
    }

    /* 扫描 0xFF 帧头，取最后一帧（最新的） */
    int last_frame = -1;
    for (int i = 0; i <= total - 4; i++) {
        if (buf[i] == 0xFF) {
            uint8_t calc = (0xFF + buf[i+1] + buf[i+2]) & 0xFF;
            if (calc == buf[i+3]) {
                last_frame = i;
            }
        }
    }

    if (last_frame < 0) {
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t dist = ((uint16_t)buf[last_frame+1] << 8) | buf[last_frame+2];
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
