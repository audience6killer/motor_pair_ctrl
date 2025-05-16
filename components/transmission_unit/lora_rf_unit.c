
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
static QueueHandle_t g_data_center_data_queue = NULL;

const char TAG[] = "LoRa";

esp_err_t data_center_get_queue_handle(QueueHandle_t *queue)
{
    ESP_RETURN_ON_FALSE(queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue is NULL");

    *queue = g_data_center_data_queue;

    return ESP_OK;
}

esp_err_t data_center_send2queue(data_center_msg_t *msg)
{
    if (xQueueSend(g_data_center_data_queue, msg, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending data to queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t lora_parse_received_data(char *data, data_center_msg_t *msg)
{
    char code[4];

    // Check if the data is valid
    if (strstr(data, "/*") == NULL || strstr(data, "*/") == NULL)
    {
        ESP_LOGE(TAG, "Invalid data format");
        return ESP_FAIL;
    }

    // Extract the code
    sscanf(data, "/*%3s", code);

    float x, y, theta;
    x = y = theta = 0.0f;

    msg->args[0] = msg->args[1] = msg->args[2] = 0.0f;

    if (strcmp(code, "SPN") == 0) // SM_CMD_STOP_NAV
    {
        msg->code = SM_CMD_STOP_NAV;
        ESP_LOGI(TAG, "Command received: STOP_NAV");
    }
    else if (strcmp(code, "STN") == 0) // SM_CMD_START_NAV
    {
        msg->code = SM_CMD_START_NAV;
        ESP_LOGI(TAG, "Command received: START_NAV");
    }
    else if (strcmp(code, "PSN") == 0) // SM_CMD_PAUSE_NAV
    {
        msg->code = SM_CMD_PAUSE_NAV;
        ESP_LOGI(TAG, "Command received: PAUSE_NAV");
    }
    else if (strcmp(code, "RMN") == 0) // SM_CMD_RESUME_NAV
    {
        msg->code = SM_CMD_RESUME_NAV;
        ESP_LOGI(TAG, "Command received: RESUME_NAV");
    }
    else if (strcmp(code, "NVP") == 0) // SM_CMD_ADD_WAYPOINT
    {
        msg->code = SM_CMD_ADD_WAYPOINT;
        // sscanf(data, "/%*[^,],%f,%f,%f*/", &x, &y, &theta);
        // ESP_LOGI(TAG, "Command received: ADD_WAYPOINT, x = %.2f, y = %.2f, theta = %.2f", x, y, theta);
        sscanf(data, "/%*[^,],%f,%f,%f*/", &msg->args[0], &msg->args[1], &msg->args[2]);
        ESP_LOGI(TAG, "Command received: ADD_WAYPOINT, x = %.2f, y = %.2f, theta = %.2f", msg->args[0], msg->args[1], msg->args[2]);
    }
    else if (strcmp(code, "RST") == 0) // SM_CMD_RESET
    {
        msg->code = SM_CMD_RESET;
        ESP_LOGI(TAG, "Command received: RESET");
    }
    else if (strcmp(code, "ECH") == 0) // SM_CMD_ECHO
    {
        msg->code = SM_CMD_ECHO;
        ESP_LOGI(TAG, "Command received: ECHO");
    }
    else
    {
        msg->code = SM_CMD_EMPTY;
        ESP_LOGW(TAG, "Unknown command received: %s", code);
    }

    return ESP_OK;
}

static void lora_receive_task(void *pvParameters)
{

    uart_event_t event_uart_rx;

    uint8_t *data = (uint8_t *)malloc(RF_UART_BUFFER_SIZE);

    char message_to_decode[RF_DATA_LENGTH];

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
                uart_read_bytes(RF_UART_PORT, (char *)data, event_uart_rx.size, pdMS_TO_TICKS(100));

                // copy data into variable and add terminator
                strncpy(message_to_decode, (char *)data, sizeof(message_to_decode) - 1);
                message_to_decode[sizeof(message_to_decode) - 1] = '\0';

                // Parse the received data
                data_center_msg_t msg;
                if (lora_parse_received_data(message_to_decode, &msg) == ESP_OK)
                {
                    // Send the parsed message to the queue
                    ESP_LOGI(TAG, "msg: %d, dat1: %.2f, dat2: %.2f, dat3: %.2f", msg.code, msg.args[0], msg.args[1], msg.args[2]);

                    if (data_center_send2queue(&msg) != pdPASS)
                    {
                        ESP_LOGE(TAG, "Failed to send message to queue");
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to parse received data");
                }

                /*
                // send queue
                if(xQueueSend(g_queue_data_received, message_to_decode, pdMS_TO_TICKS(100)) == pdFAIL)
                {
                    ESP_LOGE(TAG, "Error sending data to queue");
                }
                */

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
    g_data_center_data_queue = xQueueCreate(5, sizeof(data_center_msg_t));

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
}
