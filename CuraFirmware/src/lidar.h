#ifndef LIDAR_H
#define LIDAR_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "driver/uart.h"

/* ==================== UART Constants ==================== */

#define LIDAR_BAUD_RATE             460800
#define LIDAR_UART_RX_BUFFER_SIZE   4096

/* ==================== Command Constants ==================== */

#define LIDAR_CMD_SYNC              0xA5
#define LIDAR_CMD_START_SCAN        0x20
#define LIDAR_CMD_STOP_SCAN         0x25

/* ==================== Response Constants ==================== */

#define LIDAR_DESC_SYNC_1           0xA5
#define LIDAR_DESC_SYNC_2           0x5A
#define LIDAR_DESC_LENGTH           7

#define LIDAR_DESC_SIZE_LOW         0x05
#define LIDAR_DESC_SIZE_HIGH        0x00
#define LIDAR_DESC_SIZE_UPPER       0x00
#define LIDAR_DESC_SIZE_MODE        0x40
#define LIDAR_DESC_TYPE             0x81

#define LIDAR_COMMAND_TIMEOUT_MS    1000

/* ==================== Measurement Constants ==================== */

#define LIDAR_NODE_SIZE             5
#define LIDAR_NODE_SYNC_MASK        0x01
#define LIDAR_NODE_ANGLE_CHECK_MASK 0x01

#define LIDAR_ANGLE_SCALE           64.0f
#define LIDAR_DISTANCE_SCALE        4.0f
#define LIDAR_FULL_ROTATION_DEG     360.0f

/* ==================== Data Structures ==================== */

typedef struct {
    float angle_deg;
    float distance_mm;
    uint8_t quality;
} lidar_node_t;

typedef struct {
    uint8_t window[LIDAR_NODE_SIZE];
    size_t window_len;
} lidar_decoder_t;

/* ==================== Public API ==================== */

/* Configure and install the ESP-IDF UART driver. */
esp_err_t lidar_uart_init(
    uart_port_t uart_num,
    int tx_pin,
    int rx_pin
);

/* Start or stop scanning. */
esp_err_t lidar_start_scan(uart_port_t uart_num);
esp_err_t lidar_stop_scan(uart_port_t uart_num);

/* Initialize decoder state. */
void lidar_decoder_init(lidar_decoder_t *decoder);

/*
 * Decode raw UART bytes into measurements.
 * decoded_count reports how many measurements were produced.
 */
esp_err_t lidar_decode_data(
    lidar_decoder_t *decoder,
    const uint8_t *data,
    size_t data_len,
    lidar_node_t *nodes,
    size_t capacity,
    size_t *decoded_count
);

#endif