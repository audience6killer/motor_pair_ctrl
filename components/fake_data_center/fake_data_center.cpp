extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"

#include "state_machine.h"

#include "fake_data_center.h"
#include "fake_data_center_task_common.h"
}

#include "Arduino.h"
#include <cstring>

const char TAG[] = "fake_datac";

static QueueHandle_t g_data_center_data_queue = NULL;

esp_err_t data_center_get_queue_handle(QueueHandle_t *queue)
{
    ESP_RETURN_ON_FALSE(queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue is NULL");

    *queue = g_data_center_data_queue;

    return ESP_OK;
}

static void data_center_send_data(void *args)
{
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

/**
 * @brief The data received have the format: / * code,(char[3]),(args) * /
 * depending on the code there will be different args or none.
 * Examples: / *NVP,15.00,17.00,35.00* / -> Navigation point, x = 15.00, y = 17.00, theta = 35°
 *
 * @param data
 * @return esp_err_t
 */
esp_err_t data_center_parse_data(char *data, data_center_msg_t *msg)
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

void data_center_receive_from_serial(void)
{
    static char buffer[128]; // Buffer to store the received line
    static int index = 0;
    // Check if data is available on the serial port
    if (Serial.available())
    {
        char c = Serial.read(); // Read a single character

        // ESP_LOGI(TAG, "Received %c", c);

        // If newline or carriage return is encountered, process the line
        if (c == '\n' || c == '\r')
        {
            buffer[index] = '\0'; // Null-terminate the string

            if (index > 0) // Ensure the buffer is not empty
            {
                ESP_LOGI(TAG, "Received line: %s", buffer);

                // Parse the received data
                data_center_msg_t msg;
                if (data_center_parse_data(buffer, &msg) == ESP_OK)
                {
                    // Send the parsed message to the queue
                    if (data_center_send2queue(&msg) != ESP_OK)
                    {
                        ESP_LOGE(TAG, "Failed to send message to queue");
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Failed to parse received data");
                }
            }

            // Reset the buffer for the next line
            index = 0;
        }
        else
        {
            // Add the character to the buffer if there's space
            if (index < sizeof(buffer) - 1)
            {
                buffer[index++] = c;
            }
            else
            {
                ESP_LOGW(TAG, "Input line too long, discarding");
                index = 0; // Reset the buffer
            }
        }
    }
}

static void data_center_receive_task(void *args)
{
    ESP_LOGI(TAG, "Initializing receiver task");

    for (;;)
    {
        data_center_receive_from_serial();

        // Add a small delay to avoid busy-waiting
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void data_center_send_task(void *args)
{
    ESP_LOGI(TAG, "Initializing sender task");

    for (;;)
    {
    }
}

void fake_data_center_task_start(void)
{
    ESP_LOGI(TAG, "Fake Data center task started");

    g_data_center_data_queue = xQueueCreate(5, sizeof(data_center_msg_t));

    // xTaskCreatePinnedToCore(data_center_send_task, "dc_send_task", FAKE_DC_TASK_STACK_SIZE, NULL, FAKE_DC_TASK_PRIORITY, NULL, FAKE_DC_TASK_CORE_ID);

    xTaskCreatePinnedToCore(data_center_receive_task, "dc_receive_task", FAKE_DC_TASK_STACK_SIZE, NULL, FAKE_DC_TASK_PRIORITY, NULL, FAKE_DC_TASK_CORE_ID);
}
