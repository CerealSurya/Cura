#include "lidar.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"

#define LIDAR_UART       UART_NUM_1

#define LIDAR_TX_PIN     GPIO_NUM_17
#define LIDAR_RX_PIN     GPIO_NUM_16

#define RAW_BUFFER_SIZE  512
#define NODE_BUFFER_SIZE 100

static const char *TAG = "MAIN";

static void lidar_task(void *arg)
{
    uint8_t raw_data[RAW_BUFFER_SIZE];
    lidar_node_t nodes[NODE_BUFFER_SIZE];

    lidar_decoder_t decoder;
    lidar_decoder_init(&decoder);

    while (1)
    {
        int bytes_read = uart_read_bytes(
            LIDAR_UART, 
            raw_data, 
            sizeof(raw_data), 
            pdMS_TO_TICKS(100)
        );

        if (bytes_read <= 0) { continue; }

        size_t decoded_count = 0;

        esp_err_t err = lidar_decode_data(
            &decoder,
            raw_data,
            (size_t)bytes_read,
            nodes,
            NODE_BUFFER_SIZE,
            &decoded_count
        );

        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "Decode failed: %s", esp_err_to_name(err));
            continue;
        }

        for (size_t i = 0; i < decoded_count; i++)
        {
            ESP_LOGI(
                TAG,
                "r=%.1f mm, theta=%.2f deg, quality=%u",
                nodes[i].distance_mm,
                nodes[i].angle_deg,
                nodes[i].quality
            );
        }
    }

    /* Unreachable in normal operation; retain for future shutdown support. */
    lidar_stop_scan(LIDAR_UART);
    vTaskDelete(NULL);
}


void app_main(void)
{
    esp_err_t err = lidar_uart_init(
        LIDAR_UART,
        LIDAR_TX_PIN,
        LIDAR_RX_PIN
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART initialization failed: %s",
                 esp_err_to_name(err));
        return;
    }

    err = lidar_start_scan(LIDAR_UART);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Scan startup failed: %s",
                 esp_err_to_name(err));
        return;
    }

    BaseType_t task_created = xTaskCreate(
        lidar_task,
        "lidar_task",
        4096,
        NULL,
        5,
        NULL
    );

    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LIDAR task");
        lidar_stop_scan(LIDAR_UART);
    }
}