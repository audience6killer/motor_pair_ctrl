#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

#include "esp32_uart_task_common.h"
#include "esp32_uart_unit.h"

static QueueHandle_t communication_esp32_queue = NULL;
static QueueHandle_t g_esp32_transmit_data_queue = NULL;
static QueueHandle_t g_esp32_received_data_queue = NULL;

static const char TAG[] = "ESP32_uart";

// Protocol constants
#define SOWER_FRAME_START 0xAA
#define SOWER_FRAME_END 0x55
#define SOWER_SERIALIZED_SIZE 8 // 4 bytes for enum + 4 bytes for float

// Serialized frame structure
typedef struct
{
    uint8_t start_byte;
    uint8_t length;
    uint8_t data[SOWER_SERIALIZED_SIZE];
    uint8_t checksum;
    uint8_t end_byte;
} sower_frame_t;

/**
 * @brief Calculate XOR checksum
 */
uint8_t calculate_checksum(const uint8_t *data, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * @brief Serialize sower_cmd_t into byte array
 * @param cmd Pointer to command struct
 * @param buffer Output buffer (must be at least SOWER_SERIALIZED_SIZE bytes)
 * @return Number of bytes written
 */
size_t serialize_sower_cmd(const sower_cmd_t *cmd, uint8_t *buffer)
{
    size_t offset = 0;

    // Serialize command code (4 bytes - treating enum as uint32_t for consistency)
    uint32_t cmd_code = (uint32_t)cmd->code;
    memcpy(buffer + offset, &cmd_code, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    // Serialize argument (4 bytes)
    memcpy(buffer + offset, &cmd->arg, sizeof(float));
    offset += sizeof(float);

    return offset;
}

esp_err_t esp32_uart_get_transmit_data_queue(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_esp32_transmit_data_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue not initialized");

    *handle = g_esp32_transmit_data_queue;
    return ESP_OK;
}

esp_err_t esp32_uart_get_received_data_queue(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_esp32_received_data_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue not initialized");

    *handle = g_esp32_received_data_queue;
    return ESP_OK;
}

esp_err_t esp32_uart_handshake(void)
{
    ESP_LOGI(TAG, "Executing uart handshake");

    sower_cmd_t echo_sower = {
        .code = SOWER_CMD_ECHO_SOWER,
        .arg = 0.0f,
    };

    if (xQueueSend(g_esp32_transmit_data_queue, &echo_sower, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error: Failed to send echo msg to queue");
        return ESP_FAIL;
    }

    sower_event_t response;
    if (xQueueReceive(g_esp32_received_data_queue, &response, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Error: Response receiver Timeout");
        return ESP_FAIL;
    }

    if (response.event == SOWER_EVENT_ECHO_MSG)
        ESP_LOGI(TAG, "Handshake successful!");
    else
    {
        ESP_LOGE(TAG, "Handshake failed!");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void esp32_uart_receive_task(void *pvParameters)
{
    uart_event_t event_uart_rx;

    for (;;)
    {
        sower_event_t cutter_event;
        if (xQueueReceive(communication_esp32_queue, (void *)&event_uart_rx, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            switch (event_uart_rx.type)
            {
            case UART_DATA:
                // read from uart
                uart_read_bytes(ESP32_UART_PORT, (uint8_t *)&cutter_event, event_uart_rx.size, pdMS_TO_TICKS(100));

                const char *event_name = sower_event_name(cutter_event.event);
                ESP_LOGI(TAG, "Received sower event: %s, error=%d", event_name, cutter_event.error);

                // send queue
                if (xQueueSend(g_esp32_received_data_queue, &cutter_event, pdMS_TO_TICKS(100)) != pdTRUE)
                {
                    ESP_LOGE(TAG, "Error sending data to queue");
                }

                // Clean input.
                uart_flush(ESP32_UART_PORT);
                break;

            default:
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t esp32_send_frame_to_sower(const sower_cmd_t cmd)
{
    sower_frame_t frame;
    // Writes information to the UART port
    // Build frame
    frame.start_byte = SOWER_FRAME_START;
    frame.length = serialize_sower_cmd(&cmd, frame.data);
    frame.checksum = calculate_checksum(frame.data, frame.length);
    frame.end_byte = SOWER_FRAME_END;

    // Calculate total frame size
    size_t frame_size = sizeof(frame.start_byte) + sizeof(frame.length) +
                        frame.length + sizeof(frame.checksum) + sizeof(frame.end_byte);

    // Send frame
    int bytes_written = uart_write_bytes(ESP32_UART_PORT, (const char *)&frame, frame_size);

    return (bytes_written == frame_size) ? ESP_OK : ESP_FAIL;
}

static void esp32_uart_transmit_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing esp32_uart_transmit_task");

    for (;;)
    {
        sower_cmd_t cmd;
        // Manages the information and send to master.
        if (xQueueReceive(g_esp32_transmit_data_queue,
                          &cmd,
                          pdMS_TO_TICKS(500)) == pdPASS)
        {
            const char *cmd_name = sower_cmd_name(cmd.code);
            printf("Event to send to sower: %s, %f\n", cmd_name, cmd.arg);
            if (esp32_send_frame_to_sower((const sower_cmd_t)cmd) != ESP_OK)
            {
                ESP_LOGE(TAG, "Error: Failed to send serialized data");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t esp32_uart_task_init(void)
{
    ESP_LOGI(TAG, "Initializing ESP32 communication task");
    uart_config_t uart_RF_configuration = {
        .baud_rate = ESP32_UART_BAUDRATE_RF,
        .data_bits = ESP32_UART_DATA_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT};

    // Creates UART port.
    ESP_ERROR_CHECK(uart_param_config(ESP32_UART_PORT, &uart_RF_configuration));

    // Configurates PINS for UART.
    ESP_ERROR_CHECK(uart_set_pin(ESP32_UART_PORT, ESP32_UART_TX_PIN, ESP32_UART_RX_PIN, ESP32_UART_RTS_PIN, ESP32_UART_CTS_PIN));

    // Prepares UART.
    ESP_ERROR_CHECK(uart_driver_install(ESP32_UART_PORT,
                                        ESP32_UART_BUFFER_SIZE,
                                        ESP32_UART_BUFFER_SIZE,
                                        ESP32_UART_QUEUE_SIZE,
                                        &communication_esp32_queue,
                                        ESP_INTR_FLAG_LEVEL1));

    return ESP_OK;
}

void esp32_uart_task_start(void)
{
    ESP_LOGI(TAG, "Starting ESP32/cutter communication task");

    ESP_ERROR_CHECK(esp32_uart_task_init());

    g_esp32_transmit_data_queue = xQueueCreate(10, sizeof(sower_cmd_t));
    g_esp32_received_data_queue = xQueueCreate(5, sizeof(sower_event_t));

    // Creates RF task to receive information.
    xTaskCreatePinnedToCore(esp32_uart_receive_task,
                            "esp32_receive_task",
                            ESP32_UART_TASK_STACK_SIZE,
                            NULL,
                            ESP32_UART_TASK_RECEIVE_PRIORITY,
                            NULL,
                            ESP32_UART_TASK_CORE_ID);

    // Creates RF task to transmit information.
    xTaskCreatePinnedToCore(esp32_uart_transmit_task,
                            "esp32_transmit_task",
                            ESP32_UART_TASK_STACK_SIZE,
                            NULL,
                            ESP32_UART_TASK_TRANSFER_PRIORITY,
                            NULL,
                            ESP32_UART_TASK_CORE_ID);
}
// ESP32_uart: Error sending data to queue