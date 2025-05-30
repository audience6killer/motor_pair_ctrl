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

const char TAG[] = "ESP32_uart";

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

static void esp32_uart_receive_task(void *pvParameters)
{
    uart_event_t event_uart_rx;

    for (;;)
    {
        cutter_disk_event_t cutter_event;
        if (xQueueReceive(communication_esp32_queue, (void *)&event_uart_rx, pdMS_TO_TICKS(100)) == pdTRUE)
        {

            switch (event_uart_rx.type)
            {
            case UART_DATA:
                // read from uart
                uart_read_bytes(ESP32_UART_PORT, (uint8_t *)&cutter_event, event_uart_rx.size, pdMS_TO_TICKS(100));

                // send queue
                if (xQueueSend(g_esp32_received_data_queue, &cutter_event, pdMS_TO_TICKS(100)) == pdFAIL)
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

static void esp32_uart_transmit_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing esp32_uart_transmit_task");

    for (;;)
    {
        esp32_uart_cmd_t cmd;
        // Manages the information and send to master.
        if (xQueueReceive(g_esp32_transmit_data_queue,
                          &cmd,
                          pdMS_TO_TICKS(100)) == pdPASS)
        {
            // Writes information to the UART port
            uart_write_bytes(ESP32_UART_PORT, (const char*)&cmd, sizeof(esp32_uart_cmd_t));

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
                                        ESP_INTR_FLAG_LEVEL3));

    return ESP_OK;
}

void esp32_uart_task_start(void)
{
    ESP_LOGI(TAG, "Starting ESP32/cutter communication task");

    ESP_ERROR_CHECK(esp32_uart_task_init());

    g_esp32_transmit_data_queue = xQueueCreate(5, sizeof(esp32_uart_cmd_t));
    g_esp32_received_data_queue = xQueueCreate(5, sizeof(cutter_disk_event_t));

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
