#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"

#include "lidar.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "servo.h"

#define LIDAR_UART        UART_NUM_1
#define LIDAR_TX_PIN      GPIO_NUM_17
#define LIDAR_RX_PIN      GPIO_NUM_16

#define RAW_BUFFER_SIZE   512
#define NODE_BUFFER_SIZE  128

#define SCAN_QUEUE_DEPTH  4

#define SCAN_TIMEOUT_MS   200    /* give up on an angle after this long */

static const char *TAG = "MAIN";

/* One full 360 degree scan at one servo angle */
typedef struct {
    int16_t  servo_angle;
    uint16_t dist_mm[360];      /* index = lidar degree */
} scan_t;

static QueueHandle_t scan_queue;

sg90_t servo;
sg90_config_t cfg = {
    .gpio    = GPIO_NUM_18,
    .channel = LEDC_CHANNEL_0,
    .timer   = LEDC_TIMER_0,
};

static void dispatch_task(void *arg)
{
    static scan_t scan;

    while (1) {
        if (xQueueReceive(scan_queue, &scan, portMAX_DELAY) != pdTRUE) continue;

        /* One line per scan: servo_angle,d0,d1,...,d359 */
        printf("%d", scan.servo_angle);
        for (int i = 0; i < 360; i++) {
            printf(",%u", scan.dist_mm[i]);
        }
        printf("\n-----------\n");
    }
}

static void lidar_task(void *arg)
{
    static uint8_t      raw_data[RAW_BUFFER_SIZE];
    static lidar_node_t nodes[NODE_BUFFER_SIZE];
    static scan_t       scan;
    static bool         seen[360];

    lidar_decoder_t decoder;
    lidar_decoder_init(&decoder);

    int angle  = -90;
    int step   = 2;
    int filled = 0;                       /* how many of the 360 degrees we have */

    sg90_set_angle(&servo, angle);
    memset(&scan, 0, sizeof(scan));
    memset(seen, 0, sizeof(seen));

    int64_t scan_start = esp_timer_get_time();   /* add next to the other variables */

    while (1)
    {
        /* Read UART and fold any new nodes into the scan */
        int n = uart_read_bytes(LIDAR_UART, raw_data, sizeof(raw_data), pdMS_TO_TICKS(100));
        if (n > 0) {
            size_t count = 0;
            if (lidar_decode_data(&decoder, raw_data, (size_t)n,
                                  nodes, NODE_BUFFER_SIZE, &count) == ESP_OK) {
                for (size_t i = 0; i < count; i++) {
                    int deg = (int)nodes[i].angle_deg % 360;
                    if (deg < 0) deg += 360;

                    scan.dist_mm[deg] = (uint16_t)nodes[i].distance_mm;
                    if (!seen[deg]) {
                        seen[deg] = true;
                        filled++;
                    }
                }
            }
        }

        /* Done when all 360 degrees are in, or when we have waited too long */
        bool complete  = (filled >= 360);
        bool timed_out = ((esp_timer_get_time() - scan_start) / 1000) > SCAN_TIMEOUT_MS;

        if (!complete && !timed_out) continue;

        if (timed_out && !complete) {
            ESP_LOGW(TAG, "\nTimeout at %d deg: got %d/360, sending partial scan", angle, filled);
        }

        scan.servo_angle = (int16_t)angle;
        if (xQueueSend(scan_queue, &scan, 0) != pdTRUE) {
            ESP_LOGW(TAG, "\nQueue full, scan at %d dropped", angle);
        }

        /* Move servo and start over */
        angle += step;
        if (angle >= 90 || angle <= -90) step = -step;
        sg90_set_angle(&servo, angle);

        memset(&scan, 0, sizeof(scan));
        memset(seen, 0, sizeof(seen));
        filled = 0;
        scan_start = esp_timer_get_time();
    }
}

void app_main(void)
{
    esp_err_t err = lidar_uart_init(LIDAR_UART, LIDAR_TX_PIN, LIDAR_RX_PIN);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %s", esp_err_to_name(err));
        return;
    }

    err = sg90_init(&servo, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Servo init failed: %s", esp_err_to_name(err));
        return;
    }

    scan_queue = xQueueCreate(SCAN_QUEUE_DEPTH, sizeof(scan_t));
    if (!scan_queue) {
        ESP_LOGE(TAG, "Queue create failed");
        return;
    }

    lidar_set_motor_max(LIDAR_UART);
    vTaskDelay(pdMS_TO_TICKS(2000));

    err = lidar_start_scan(LIDAR_UART);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Scan start failed: %s", esp_err_to_name(err));
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(5000));      /* lidar spin-up */

    

    xTaskCreatePinnedToCore(dispatch_task, "dispatch", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(lidar_task,    "lidar",    4096, NULL, 6, NULL, 1);

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}

// W (15675) MAIN: 
// Timeout at -16 deg: got 331/360, sending partial scan
// -16,95,95,93,101,103,105,106,110,112,110,111,111,107,106,105,107,107,105,103,97,89,86,84,77,0,67,66,63,62,62,61,60,60,55,53,53,52,0,54,50,51,47,48,44,45,46,45,45,46,0,48,46,45,44,46,43,44,0,40,38,37,36,35,35,39,39,43,45,45,44,43,0,33,33,33,35,0,35,35,36,0,0,34,34,33,34,0,0,38,0,38,0,40,38,38,39,0,36,34,0,0,0,36,33,32,32,31,0,39,0,41,40,39,37,35,34,33,33,33,33,33,34,35,36,39,39,42,41,42,42,43,45,46,46,47,46,48,47,47,49,0,59,60,60,60,60,60,0,46,46,47,48,52,52,54,0,0,61,60,56,55,0,0,0,109,117,116,117,119,0,190,187,0,136,115,0,104,736,2570,2586,2572,2573,2573,2574,2578,2582,2585,2590,2598,2603,2611,2627,2636,2651,2657,2672,2679,2688,2707,2717,2740,2753,2768,2802,2824,2861,2866,2876,3137,3164,3220,3248,3312,3344,3375,3445,3480,3497,3446,3325,3311,3305,3296,3290,3283,3281,3279,3274,3272,3273,3262,3231,3239,2507,2500,2485,2412,2399,3036,3026,3008,2973,2955,2920,2904,2874,2859,2846,2817,2804,2783,2772,2753,2743,2735,2719,2709,2694,2688,2677,2673,2669,2660,2657,2652,2650,2648,2648,2649,2649,2649,2652,2654,2658,2663,2666,2674,2679,2688,2655,2627,2604,2573,2522,2501,310,321,314,299,296,298,299,302,301,301,299,263,277,277,275,273,270,262,265,265,268,267,255,255,261,261,260,252,252,253,0,79,80,81,79,79,78,78,77,77,77,77,78,78,77,76,76,75,74,74,73,73,73,73,73,73,76,76,77,76,77,78,78,79,79,80,81,82,82,83,82,80,90,91,93
// -----------