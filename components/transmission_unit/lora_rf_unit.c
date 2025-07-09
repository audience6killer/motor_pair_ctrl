
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>

#include "data_center.h"

#include "lora_rf_task_common.h"
#include "lora_rf_unit.h"

static QueueHandle_t communication_RF_queue = NULL;
static QueueHandle_t g_lora_received_data_queue = NULL;
static QueueHandle_t g_lora_transmit_data_queue = NULL;

const char TAG[] = "LoRa";

esp_err_t lora_get_received_data_queue(QueueHandle_t *queue)
{
    ESP_RETURN_ON_FALSE(g_lora_received_data_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "received_queue is null while retriving");

    *queue = g_lora_received_data_queue;

    return ESP_OK;
}

esp_err_t lora_get_transmit_data_queue(QueueHandle_t *queue)
{
    ESP_RETURN_ON_FALSE(g_lora_transmit_data_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "transmit_queue is null while retriving");

    *queue = g_lora_transmit_data_queue;

    return ESP_OK;
}

esp_err_t lora_send_to_data_center_queue(char *msg)
{
    // printf("TO SEND: %s\n", msg);
    if (xQueueSend(g_lora_received_data_queue, msg, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending data to queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

inline bool lora_is_available()
{
    return gpio_get_level(RF_AUX_PIN) == 1 ? true : false;
}

static void lora_transmit_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting transmit task");
    char received_data[RF_DATA_LENGTH];

    for (;;)
    {
        if (xQueueReceive(g_lora_transmit_data_queue, received_data, pdMS_TO_TICKS(WAIT_QUEUE_SEND_RF)) == pdPASS)
        {
            printf("%s\n", received_data);
            // Writes information to the UART port
            int len = uart_write_bytes(RF_UART_PORT, received_data, strlen(received_data));

            printf("Bytes sended: %d\n", len);

            // Cleans result.
            memset(received_data, 0, sizeof(received_data));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void lora_receive_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting receive task");
    uart_event_t event_uart_rx;
    uint8_t *data = (uint8_t *)malloc(RF_UART_BUFFER_SIZE);

    char message_to_decode[RF_DATA_LENGTH];
    char *msg = malloc(200);

    for (;;)
    {
        if (xQueueReceive(communication_RF_queue, (void *)&event_uart_rx, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // Clean buffer.
            memset(data, 0, RF_UART_BUFFER_SIZE);

            switch (event_uart_rx.type)
            {
            case UART_DATA:
                // read from uart
                ESP_LOGI(TAG, "Data received");
                uart_read_bytes(RF_UART_PORT, (char *)data, event_uart_rx.size, pdMS_TO_TICKS(100));

                // copy data into variable and add terminator
                strncpy(message_to_decode, (char *)data, sizeof(message_to_decode) - 1);
                message_to_decode[sizeof(message_to_decode) - 1] = '\0';
                strncpy(msg, message_to_decode, 200 - 1);
                msg[200 - 1] = '\0';
                
                //printf("%s\n", msg);

                if (lora_send_to_data_center_queue(msg) != ESP_OK)
                {
                    ESP_LOGE(TAG, "Failed to send message to queue");
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

esp_err_t lora_task_init(void)
{
    ESP_LOGI(TAG, "Initializing RF communication task");

    uart_config_t uart_RF_configuration = {
        .baud_rate = RF_UART_BAUDRATE_RF,
        .data_bits = RF_UART_DATA_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT};

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

    /* Configure GPIO for busy pin */
    gpio_set_direction(RF_AUX_PIN, GPIO_MODE_DEF_INPUT);
    gpio_set_pull_mode(RF_AUX_PIN, GPIO_PULLUP_ONLY);

    g_lora_received_data_queue = xQueueCreate(5, 200);
    g_lora_transmit_data_queue = xQueueCreate(5, 200);

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

    xTaskCreatePinnedToCore(lora_transmit_task,
                            "lora_send",
                            RF_TASK_STACK_SIZE,
                            NULL,
                            RF_TASK_RECEIVE_PRIORITY,
                            NULL,
                            RF_TASK_CORE_ID);
}
