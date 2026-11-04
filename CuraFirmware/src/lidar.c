#include "lidar.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ==================== Private UART Constants ==================== */

#define LIDAR_UART_DATA_BITS       UART_DATA_8_BITS
#define LIDAR_UART_PARITY          UART_PARITY_DISABLE
#define LIDAR_UART_STOP_BITS       UART_STOP_BITS_1
#define LIDAR_UART_FLOW_CONTROL    UART_HW_FLOWCTRL_DISABLE

/* ==================== UART Initialization ==================== */

esp_err_t lidar_uart_init(
    uart_port_t uart_num,
    int tx_pin,
    int rx_pin)
{
    const uart_config_t config = {
        .baud_rate = LIDAR_BAUD_RATE,
        .data_bits = LIDAR_UART_DATA_BITS,
        .parity = LIDAR_UART_PARITY,
        .stop_bits = LIDAR_UART_STOP_BITS,
        .flow_ctrl = LIDAR_UART_FLOW_CONTROL,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(uart_num, &config);
    if (err != ESP_OK) {
        return err;
    }

    err = uart_set_pin(
        uart_num,
        tx_pin,
        rx_pin,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );
    if (err != ESP_OK) {
        return err;
    }

    return uart_driver_install(
        uart_num,
        LIDAR_UART_RX_BUFFER_SIZE,
        0,
        0,
        NULL,
        0
    );
}

/* ==================== Private Protocol Helpers ==================== */

static esp_err_t lidar_send_command(
    uart_port_t uart_num,
    uint8_t command)
{
    const uint8_t packet[] = {
        LIDAR_CMD_SYNC,
        command
    };

    int written = uart_write_bytes(
        uart_num,
        packet,
        sizeof(packet)
    );

    if (written != sizeof(packet)) {
        return ESP_FAIL;
    }

    return uart_wait_tx_done(
        uart_num,
        pdMS_TO_TICKS(LIDAR_COMMAND_TIMEOUT_MS)
    );
}


static esp_err_t lidar_read_exact(
    uart_port_t uart_num,
    uint8_t *buffer,
    size_t length)
{
    size_t received = 0;

    TickType_t start = xTaskGetTickCount();
    TickType_t timeout =
        pdMS_TO_TICKS(LIDAR_COMMAND_TIMEOUT_MS);

    while (received < length) {
        TickType_t elapsed = xTaskGetTickCount() - start;

        if (elapsed >= timeout) {
            return ESP_ERR_TIMEOUT;
        }

        TickType_t remaining = timeout - elapsed;

        int count = uart_read_bytes(
            uart_num,
            buffer + received,
            length - received,
            remaining
        );

        if (count > 0) {
            received += (size_t)count;
        }
    }

    return ESP_OK;
}


static esp_err_t lidar_read_scan_descriptor(
    uart_port_t uart_num,
    uint8_t *descriptor)
{
    return lidar_read_exact(
        uart_num,
        descriptor,
        LIDAR_DESC_LENGTH
    );
}


static bool lidar_validate_scan_descriptor(
    const uint8_t *d)
{
    return d[0] == LIDAR_DESC_SYNC_1 &&
           d[1] == LIDAR_DESC_SYNC_2 &&
           d[2] == LIDAR_DESC_SIZE_LOW &&
           d[3] == LIDAR_DESC_SIZE_HIGH &&
           d[4] == LIDAR_DESC_SIZE_UPPER &&
           d[5] == LIDAR_DESC_SIZE_MODE &&
           d[6] == LIDAR_DESC_TYPE;
}

/* ==================== Public Scan Control ==================== */

esp_err_t lidar_start_scan(uart_port_t uart_num)
{
    uint8_t descriptor[LIDAR_DESC_LENGTH];

    /* Clear stale UART input before starting a new scan. */
    esp_err_t err = uart_flush_input(uart_num);
    if (err != ESP_OK) {
        return err;
    }

    /* Send scan-start command. */
    err = lidar_send_command(
        uart_num,
        LIDAR_CMD_START_SCAN
    );
    if (err != ESP_OK) {
        return err;
    }

    /* Read the response descriptor. */
    err = lidar_read_scan_descriptor(
        uart_num,
        descriptor
    );
    if (err != ESP_OK) {
        return err;
    }

    /* Validate the descriptor. */
    if (!lidar_validate_scan_descriptor(descriptor)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}


esp_err_t lidar_stop_scan(uart_port_t uart_num)
{
    return lidar_send_command(
        uart_num,
        LIDAR_CMD_STOP_SCAN
    );
}

/* ==================== Decoder Helpers ==================== */

void lidar_decoder_init(lidar_decoder_t *decoder)
{
    if (decoder == NULL) {
        return;
    }

    memset(decoder, 0, sizeof(*decoder));
}


static bool lidar_node_is_valid(const uint8_t *b)
{
    uint8_t start_flag =
        b[0] & LIDAR_NODE_SYNC_MASK;

    uint8_t inverse_flag =
        (b[0] >> 1) & LIDAR_NODE_SYNC_MASK;

    if (start_flag == inverse_flag) {
        return false;
    }

    if ((b[1] & LIDAR_NODE_ANGLE_CHECK_MASK) == 0) {
        return false;
    }

    return true;
}


static void lidar_decode_node(
    const uint8_t *b,
    lidar_node_t *node)
{
    uint16_t angle_q6 =
        ((uint16_t)b[1] >> 1) |
        ((uint16_t)b[2] << 7);

    uint16_t distance_q2 =
        (uint16_t)b[3] |
        ((uint16_t)b[4] << 8);

    node->angle_deg = 
        (int)angle_q6 >> LIDAR_ANGLE_SCALE; // /64
    //Both scales are powers of 2, we could bit shift over division, look into this
    //Also do we even need decimal level accuracy if we are simply encoding vibrations
    node->distance_mm =
        (int)distance_q2 >> LIDAR_DISTANCE_SCALE; // /4

    node->quality = b[0] >> 2;
}

/* ==================== Public Decoder ==================== */

esp_err_t lidar_decode_data(
    lidar_decoder_t *decoder,
    const uint8_t *data,
    size_t data_len,
    lidar_node_t *nodes,
    size_t capacity,
    size_t *decoded_count)
{
    if (decoder == NULL ||
        (data == NULL && data_len > 0) ||
        (nodes == NULL && capacity > 0) ||
        decoded_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *decoded_count = 0;

    for (size_t i = 0; i < data_len; i++) {
        decoder->window[decoder->window_len++] = data[i];

        if (decoder->window_len < LIDAR_NODE_SIZE) {
            continue;
        }

        if (lidar_node_is_valid(decoder->window)) {
            lidar_node_t node;

            lidar_decode_node(decoder->window, &node);

            decoder->window_len = 0;

            if (node.distance_mm > 0.0f &&
                node.angle_deg < LIDAR_FULL_ROTATION_DEG) {

                if (*decoded_count >= capacity) {
                    return ESP_ERR_INVALID_SIZE;
                }

                nodes[*decoded_count] = node;
                (*decoded_count)++;
            }
        } else {
            /* Slide one byte to recover synchronization. */
            memmove(
                decoder->window,
                decoder->window + 1,
                LIDAR_NODE_SIZE - 1
            );

            decoder->window_len = LIDAR_NODE_SIZE - 1;
        }
    }

    return ESP_OK;
}