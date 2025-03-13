
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

#include "lora_rf_task_common.h"
#include "lora_rf_unit.h"

static QueueHandle_t communication_RF_queue = NULL; 
static QueueHandle_t g_queue_data2send = NULL;
static QueueHandle_t g_queue_data_received = NULL;   

const char TAG[] = "LoRa";

esp_err_t lora_get_queue_data2send(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_queue_data2send != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue not initialized");

    *handle = g_queue_data2send;
    return ESP_OK;
}

esp_err_t lora_get_queue_data_received(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_queue_data_received != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue not initialized");

    *handle = g_queue_data_received;
    return ESP_OK;
}

static void lora_receive_task(void *pvParameters)
{
    uart_event_t event_uart_rx;

    uint8_t *data = (uint8_t *)malloc(RF_UART_BUFFER_SIZE);

    char message_to_queue[RF_DATA_LENGTH];

    for(;;)
    {
        if (xQueueReceive(communication_RF_queue, (void *)&event_uart_rx, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // Clean buffer.
            for(size_t i = 0; i < RF_UART_BUFFER_SIZE; i++)
                data[i] = 0;

            switch (event_uart_rx.type)
            {
            case UART_DATA:
                // read from uart
                uart_read_bytes(RF_UART_PORT, (char *)data, event_uart_rx.size, pdMS_TO_TICKS(100));

                // copy data into variable and add terminator
                strncpy(message_to_queue, (char *)data, sizeof(message_to_queue) - 1);
                message_to_queue[sizeof(message_to_queue) - 1] = '\0';

                // send queue
                if(xQueueSend(g_queue_data_received, message_to_queue, pdMS_TO_TICKS(100)) == pdFAIL)
                {
                    ESP_LOGE(TAG, "Error sending data to queue");
                }

                // Clean input.
                uart_flush(RF_UART_PORT);
                break;

            default:
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    free(data);
    data = NULL;
}

static void lora_transmit_task(void *pvParameters)
{
    BaseType_t status_queue;
    char received_data[RF_DATA_LENGTH];

    // gpio for aux
    gpio_set_direction(RF_AUX_PIN, GPIO_MODE_DEF_INPUT);
    gpio_set_pull_mode(RF_AUX_PIN, GPIO_PULLUP_ONLY);

    for(;;)
    {
        // If it detects a queue.
        status_queue = xQueueReceive(g_queue_data2send, received_data, pdMS_TO_TICKS(WAIT_QUEUE_SEND_RF));

        // detects pin Aux in 1 -> lora module is available
        if (gpio_get_level(RF_AUX_PIN) == 1)
        {
            // Manages the information and send to master.
            if (status_queue == pdPASS)
            {
                // Writes information to the UART port
                uart_write_bytes(RF_UART_PORT, received_data, strlen(received_data));

                // Cleans result.
                memset(received_data, 0, sizeof(received_data));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t lora_task_init(void)
{
    ESP_LOGI(TAG, "Initializing RF communication task");
    uart_config_t uart_RF_configuration = {
        .baud_rate = RF_UART_BAUDRATE_RF,
        .data_bits = RF_UART_DATA_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    // Creates UART port.
    ESP_ERROR_CHECK(uart_param_config(RF_UART_PORT, &uart_RF_configuration));

    // Configurates PINS for UART.
    ESP_ERROR_CHECK(uart_set_pin(RF_UART_PORT, RF_UART_TX_PIN, RF_UART_RX_PIN, RF_UART_RTS_PIN, RF_UART_CTS_PIN));

    // Prepares UART.
    ESP_ERROR_CHECK(uart_driver_install(RF_UART_PORT,
                                        RF_UART_BUFFER_SIZE,
                                        RF_UART_BUFFER_SIZE,
                                        RF_UART_QUEUE_SIZE,
                                        &communication_RF_queue,
                                        ESP_INTR_FLAG_LEVEL3));

    return ESP_OK;
}

void lora_task_start(void)
{
    ESP_LOGI(TAG, "Starting RF communication task");

    ESP_ERROR_CHECK(lora_task_init());

    // Creates RF task to receive information.
    xTaskCreatePinnedToCore(lora_receive_task,
                            "lora_receive",
                            RF_TASK_STACK_SIZE,
                            NULL,
                            RF_TASK_RECEIVE_PRIORITY,
                            NULL,
                            RF_TASK_CORE_ID);

    // Creates RF task to transmit information.
    xTaskCreatePinnedToCore(lora_transmit_task,
                            "lora_transmit",
                            RF_TASK_STACK_SIZE,
                            NULL,
                            RF_TASK_TRANSFER_PRIORITY,
                            NULL,
                            RF_TASK_CORE_ID);
}
