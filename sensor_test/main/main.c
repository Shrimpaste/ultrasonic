/**
 * @file main.c
 * @brief 超声波传感器硬件调试例程
 *
 * 功能：同时驱动 A02YYUW（UART）和 URM09（I2C）两个超声波模块，
 *       持续输出格式化读数表格，用于在正式实验前验证硬件是否正常。
 *
 * 接线：
 *   A02YYUW  VCC→3.3V  GND→GND  白线(TX)→GPIO18  黄线(RX)→GPIO17
 *   URM09    VCC→3.3V  GND→GND  SDA→GPIO8  SCL→GPIO9
 */

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "a02yyuw.h"
#include "urm09.h"

static const char *TAG = "SENSOR_TEST";

/* ═══════════════════════════════════════════════════
 *  GPIO Pin Assignments（应用层配置）
 * ═══════════════════════════════════════════════════ */
#define A02_RX_PIN      18
#define A02_TX_PIN      17
#define URM09_SDA_PIN   8
#define URM09_SCL_PIN   9

/* ═══════════════════════════════════════════════════
 *  表格输出
 * ═══════════════════════════════════════════════════ */

static void print_header(void)
{
    printf("\n");
    printf("╔════════╦═══════════════════════╦═══════════════════════════╗\n");
    printf("║  序号  ║    A02YYUW (UART)     ║      URM09 (I2C)         ║\n");
    printf("║        ║  距离(mm)    状态     ║  距离(cm)  温度(°C) 状态  ║\n");
    printf("╠════════╬═══════════════════════╬═══════════════════════════╣\n");
    fflush(stdout);
}

static void print_row(int index, const a02yyuw_result_t *a02,
                      const urm09_result_t *u)
{
    if (a02->valid)
        printf("║  %3d   ║  %6d mm    ✓ OK  ║", index, a02->distance_mm);
    else
        printf("║  %3d   ║     ---      ✗ FAIL║", index);

    if (u->valid)
        printf("  %4d cm  %5.1f°C  ✓ OK ║\n", u->distance_cm, u->temperature);
    else
        printf("    ---     ---°C  ✗ FAIL ║\n");

    fflush(stdout);
}

static void print_footer(void)
{
    printf("╚════════╩═══════════════════════╩═══════════════════════════╝\n");
    fflush(stdout);
}

/* ═══════════════════════════════════════════════════
 *  Main
 * ═══════════════════════════════════════════════════ */
void app_main(void)
{
    printf("\n");
    printf("┌─────────────────────────────────────────────────┐\n");
    printf("│     超声波传感器硬件调试工具  v2.0 [模块化]     │\n");
    printf("│     A02YYUW (UART) + URM09 (I2C)               │\n");
    printf("└─────────────────────────────────────────────────┘\n\n");

    /* ── 初始化 ── */
    int a02_ok = 0, urm09_ok = 0;
    i2c_master_bus_handle_t urm09_bus = NULL;
    i2c_master_dev_handle_t urm09_dev = NULL;

    if (a02yyuw_init(A02_RX_PIN, A02_TX_PIN) == ESP_OK) {
        a02_ok = 1;
    } else {
        ESP_LOGE(TAG, "A02YYUW init FAIL — check white wire → GPIO%d", A02_RX_PIN);
    }

    if (urm09_init(&urm09_bus, &urm09_dev, URM09_SDA_PIN, URM09_SCL_PIN) == ESP_OK) {
        urm09_ok = 1;
    } else {
        ESP_LOGE(TAG, "URM09 init FAIL — check SDA=GPIO%d SCL=GPIO%d + pull-ups",
                 URM09_SDA_PIN, URM09_SCL_PIN);
    }

    printf("\n");
    if (a02_ok && urm09_ok) {
        printf("  ✓ Both sensors ready.\n\n");
    } else {
        printf("  ⚠ Some sensors failed. Check wiring above.\n\n");
    }

    print_header();

    /* ── 持续读取 ── */
    int count = 0;
    int a02_last_mm = -1;

    while (1) {
        a02yyuw_result_t a02 = {.distance_mm = 0, .valid = false};
        urm09_result_t u = {.distance_cm = 0, .temperature = 0, .valid = false};

        if (a02_ok) {
            if (a02yyuw_read(&a02) == ESP_OK) {
                a02_last_mm = a02.distance_mm;
            } else if (a02_last_mm > 0) {
                a02.distance_mm = a02_last_mm;
                a02.valid = true;
            }
        }

        if (urm09_ok) {
            urm09_read(urm09_dev, &u);
        }

        count++;
        print_row(count, &a02, &u);

        if (count % 20 == 0) {
            print_footer();
            print_header();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
