#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "a02yyuw.h"
#include "urm09.h"

static const char *TAG = "MAIN";

/* ─── GPIO Pin Assignments ─── */
#define A02_YYUW_RX_PIN     18  /* 白线=传感器TX → ESP32 RX (U1RXD) */
#define A02_YYUW_TX_PIN     17  /* 黄线=传感器RX → ESP32 TX (U1TXD) */
#define URM09_SDA_PIN       8   /* D (SDA) → GPIO8 */
#define URM09_SCL_PIN       9   /* C (SCL) → GPIO9 */

/* ─── Test Phase Definitions ─── */
typedef struct {
    int phase;
    const char *name;
    int target_cm;
    int samples;
} test_phase_t;

static const test_phase_t phases[] = {
    {0, "Static_20cm",    20,  30},
    {1, "Static_100cm",  100,  30},
    {2, "Static_300cm",  300,  30},
    {3, "Angle_0deg",     20,  30},
    {4, "Angle_30deg",    20,  30},
    {5, "Angle_45deg",    20,  30},
    {6, "Stability_100x", 50, 100},
};
#define PHASE_COUNT (sizeof(phases) / sizeof(phases[0]))

/* ─── Statistics Helper ─── */
typedef struct {
    double mean;
    double std;
    int min;
    int max;
    int err_count;
} stats_t;

static void compute_stats(const int *values, int count, int err_count, stats_t *out)
{
    double sum = 0;
    for (int i = 0; i < count; i++) {
        sum += values[i];
    }
    out->mean = sum / count;

    double var = 0;
    for (int i = 0; i < count; i++) {
        double diff = values[i] - out->mean;
        var += diff * diff;
    }
    out->std = (count > 1) ? sqrt(var / (count - 1)) : 0;
    out->err_count = err_count;

    out->min = values[0];
    out->max = values[0];
    for (int i = 1; i < count; i++) {
        if (values[i] < out->min) out->min = values[i];
        if (values[i] > out->max) out->max = values[i];
    }
}

/* ─── JSON Output Helpers ─── */
static void emit_phase_start(const test_phase_t *phase)
{
    printf("{\"event\":\"phase_start\",\"phase\":%d,\"name\":\"%s\",\"samples\":%d}\n",
           phase->phase, phase->name, phase->samples);
    fflush(stdout);
}

static void emit_sample(int timestamp_ms, int phase, int index,
                        const a02yyuw_result_t *a, const urm09_result_t *u)
{
    printf("{\"t\":%d,\"ph\":%d,\"i\":%d,"
           "\"a\":{\"d\":%d,\"ok\":%s},"
           "\"u\":{\"d\":%d,\"ok\":%s,\"temp\":%.1f}}\n",
           timestamp_ms, phase, index,
           a->distance_mm, a->valid ? "true" : "false",
           u->distance_cm, u->valid ? "true" : "false",
           u->temperature);
    fflush(stdout);
}

static void emit_phase_stats(const test_phase_t *phase,
                             const stats_t *a_stats, const stats_t *u_stats)
{
    printf("{\"event\":\"phase_stats\",\"phase\":%d,\"name\":\"%s\","
           "\"a\":{\"mean\":%.0f,\"std\":%.1f,\"min\":%d,\"max\":%d,\"err\":%d},"
           "\"u\":{\"mean\":%.1f,\"std\":%.1f,\"min\":%d,\"max\":%d,\"err\":%d}}\n",
           phase->phase, phase->name,
           a_stats->mean, a_stats->std, a_stats->min, a_stats->max, a_stats->err_count,
           u_stats->mean, u_stats->std, u_stats->min, u_stats->max, u_stats->err_count);
    fflush(stdout);
}

/* ─── Console Input ─── */
/**
 * @brief 清空输入缓冲区，等待用户在串口监视器按 ENTER
 */
static void wait_for_enter(const char *prompt)
{
    printf("%s", prompt);
    fflush(stdout);

    /* 清空残留输入 */
    int c;
    while ((c = getchar()) != '\n' && c != EOF) {}

    /* 等待用户按下 ENTER */
    while (1) {
        c = getchar();
        if (c == '\n' || c == '\r') {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ─── Main ─── */
void app_main(void)
{
    ESP_LOGI(TAG, "=== Ultrasonic Sensor Comparison Test ===");

    /* Initialize A02YYUW */
    esp_err_t ret = a02yyuw_init(A02_YYUW_RX_PIN, A02_YYUW_TX_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "A02YYUW init failed: %s", esp_err_to_name(ret));
    }

    /* Initialize URM09 */
    i2c_master_bus_handle_t urm09_bus = NULL;
    i2c_master_dev_handle_t urm09_dev = NULL;
    ret = urm09_init(&urm09_bus, &urm09_dev, URM09_SDA_PIN, URM09_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "URM09 init failed: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Initialization complete. Starting test phases...");

    printf("\n  >> 每个阶段结束后，程序会暂停等待您按 ENTER 继续\n");
    printf("  >> 请在阶段间调整好反射板位置和角度\n\n");

    /* ─── Run Test Phases ─── */
    for (int p = 0; p < PHASE_COUNT; p++) {
        const test_phase_t *phase = &phases[p];
        emit_phase_start(phase);

        /* Allocate sample buffers */
        int *a_values = calloc(phase->samples, sizeof(int));
        int *u_values = calloc(phase->samples, sizeof(int));
        int a_err = 0, u_err = 0;

        for (int i = 0; i < phase->samples; i++) {
            a02yyuw_result_t a_result = {0};
            urm09_result_t u_result = {0};

            /* Read A02YYUW first */
            if (a02yyuw_read(&a_result) != ESP_OK || !a_result.valid) {
                a_values[i] = -1;
                a_err++;
            } else {
                a_values[i] = a_result.distance_mm;
            }

            /* Read URM09 */
            if (urm09_dev && (urm09_read(urm09_dev, &u_result) == ESP_OK) && u_result.valid) {
                u_values[i] = u_result.distance_cm;
            } else {
                u_values[i] = -1;
                u_err++;
            }

            /* Emit sample JSON */
            int ts = (int)(esp_timer_get_time() / 1000);
            emit_sample(ts, phase->phase, i, &a_result, &u_result);

            /* Small delay between samples */
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        /* Compute and emit phase statistics */
        stats_t a_stats = {0}, u_stats = {0};

        /* Filter valid values for stats */
        int *a_valid = malloc((phase->samples - a_err) * sizeof(int));
        int *u_valid = malloc((phase->samples - u_err) * sizeof(int));
        int a_valid_cnt = 0, u_valid_cnt = 0;

        for (int i = 0; i < phase->samples; i++) {
            if (a_values[i] >= 0) a_valid[a_valid_cnt++] = a_values[i];
            if (u_values[i] >= 0) u_valid[u_valid_cnt++] = u_values[i];
        }

        if (a_valid_cnt > 0) {
            compute_stats(a_valid, a_valid_cnt, a_err, &a_stats);
        }
        a_stats.mean = (a_valid_cnt > 0) ? a_stats.mean : 0;
        a_stats.std = (a_valid_cnt > 0) ? a_stats.std : 0;
        a_stats.min = (a_valid_cnt > 0) ? a_stats.min : 0;
        a_stats.max = (a_valid_cnt > 0) ? a_stats.max : 0;

        if (u_valid_cnt > 0) {
            compute_stats(u_valid, u_valid_cnt, u_err, &u_stats);
        }
        u_stats.mean = (u_valid_cnt > 0) ? u_stats.mean : 0;
        u_stats.std = (u_valid_cnt > 0) ? u_stats.std : 0;
        u_stats.min = (u_valid_cnt > 0) ? u_stats.min : 0;
        u_stats.max = (u_valid_cnt > 0) ? u_stats.max : 0;

        emit_phase_stats(phase, &a_stats, &u_stats);

        /* Print human-readable summary */
        printf("\n");
        printf("═══════════════════════════════════════════════════\n");
        printf("  Phase %d: %s  (target %d cm, %d samples)\n",
               phase->phase, phase->name, phase->target_cm, phase->samples);
        printf("───────────────────────────────────────────────────\n");
        if (a_valid_cnt > 0) {
            printf("  A02YYUW : mean=%6.1f mm  std=%4.1f mm  range=[%d, %d]  err=%d/%d\n",
                   a_stats.mean, a_stats.std, a_stats.min, a_stats.max,
                   a_err, phase->samples);
        } else {
            printf("  A02YYUW : ALL FAILED (%d/%d)\n", a_err, phase->samples);
        }
        if (u_valid_cnt > 0) {
            printf("  URM09   : mean=%6.1f cm  std=%4.1f cm  range=[%d, %d]  err=%d/%d  temp=%.1f°C\n",
                   u_stats.mean, u_stats.std, u_stats.min, u_stats.max,
                   u_err, phase->samples, u_stats.mean);
        } else {
            printf("  URM09   : ALL FAILED (%d/%d)\n", u_err, phase->samples);
        }
        printf("═══════════════════════════════════════════════════\n");

        free(a_values);
        free(u_values);
        free(a_valid);
        free(u_valid);

        /* Wait for user to adjust setup, then press ENTER */
        if (p < PHASE_COUNT - 1) {
            printf("\n  >> 请调整好下一阶段的实验设置，然后按 ENTER 继续...\n");
            wait_for_enter("  >> ");
            printf("\n");
        }
    }

    /* Test complete */
    printf("{\"event\":\"test_complete\"}\n");
    fflush(stdout);
    ESP_LOGI(TAG, "=== All test phases complete ===");

    /* Cleanup */
    a02yyuw_deinit();
    if (urm09_dev) {
        urm09_deinit(urm09_bus, urm09_dev);
    }
}
