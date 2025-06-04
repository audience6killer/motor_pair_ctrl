
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"

#include "traction_control.h"
#include "kalman_filter.h"
#include "traction_control.h"
#include "waypoint_controller.h"
#include "lora_rf_task_common.h"
#include "lora_rf_unit.h"
#include "esp32_uart_unit.h"
}

#include "ArduinoJson.h"
#include "data_center.h"
#include "data_center_task_common.h"
#include <cstring>

const char TAG[] = "data_center";

static QueueHandle_t g_data_center_data_queue = NULL;
static QueueHandle_t g_lora_received_data_queue = NULL;
static QueueHandle_t g_lora_transmit_data_queue = NULL;
static JsonDocument json;

esp_err_t data_center_get_data_queue(QueueHandle_t *queue)
{
    ESP_RETURN_ON_FALSE(g_data_center_data_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue is NULL");

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

esp_err_t data_center_send_vehicle_data(char *msg)
{
    ESP_RETURN_ON_FALSE(msg != NULL, ESP_ERR_INVALID_STATE, TAG, "Trying to send null msg to lora!");
    
    if(xQueueSend(g_lora_transmit_data_queue, msg, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sendig vehicle data to queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* Colect data while */
void data_center_recolect_data(char *msg)
{
    // Clear the json document 
    json.clear();

    /* Get waypoint data */ 
    json["wp"]["st"] = waypoint_get_state_string();
    json["wp"]["no_p"] = waypoint_get_point_number();
    
    /* Get diff drive data */
    json["dd"]["st"] = diff_drive_get_state_string();
    navigation_point_t current_point;
    diff_drive_get_current_point(&current_point);
    json["dd"]["cpo"][0] = current_point.x;
    json["dd"]["cpo"][1] = current_point.y;
    json["dd"]["cpo"][2] = current_point.theta;
    kalman_info_t current_pose;
    diff_drive_get_current_pose(&current_pose);
    json["dd"]["cpop"][0] = current_pose.x;
    json["dd"]["cpop"][1] = current_pose.y;
    json["dd"]["cpop"][2] = current_pose.theta;
    json["dd"]["cpop"][3] = current_pose.x_p;
    json["dd"]["cpop"][4] = current_pose.y_p;
    json["dd"]["cpop"][5] = current_pose.theta_p;

    /* Get traction data */
    json["tc"]["st"] = tract_ctrl_get_state_string();

    /* Get state machine */
    json["sm"]["st"] = state_machine_get_state_string();
    if(strcmp("SM_STATE_ERROR", json["sm"]["st"]) == 0)
        json["sm"]["er"] = state_machine_get_error_string();

    serializeJson(json, msg, 200);
    strcat(msg, "\n");
    //printf("%s\n", msg);
    
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
        printf("%s\n", data);
        return ESP_FAIL;
    }

    // Extract the code
    sscanf(data, "/*%3s", code);

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

        char msg[200];
        data_center_recolect_data(msg);
        data_center_send_vehicle_data(msg);
    }
    else if (strcmp(code, "ECE") == 0) // SM_CMD_ECHO_ESP32
    {
        msg->code = SM_CMD_ECHO_ESP32;
        ESP_LOGI(TAG, "Command received: ECHO ESP32");

    }
    else
    {
        msg->code = SM_CMD_EMPTY;
        ESP_LOGW(TAG, "Unknown command received: %s", code);
    }

    return ESP_OK;
}


esp_err_t data_center_receive_lora_data(void)
{
    char received_data[200];
    data_center_msg_t msg;
    //memset(received_data, 0, 200);


    if (xQueueReceive(g_lora_received_data_queue, received_data, pdMS_TO_TICKS(100)) == pdPASS)
    {
        //printf("DATA_CENTER: %s\n", received_data);
        data_center_parse_data(received_data, &msg);
        ESP_ERROR_CHECK(data_center_send2queue(&msg));
    }

    return ESP_OK;
}

esp_err_t data_center_send_lora_data(void)
{
    if(lora_is_available())
    {
        char msg[200];
        data_center_recolect_data(msg);
        data_center_send_vehicle_data(msg);
    }

    return ESP_OK;
}

static void data_center_task(void *args)
{
    ESP_LOGI(TAG, "Iniatiliazing data center receiving task");

    g_data_center_data_queue = xQueueCreate(5, sizeof(data_center_msg_t));

    while(lora_get_received_data_queue(&g_lora_received_data_queue))
    {
        ESP_LOGE(TAG, "Cannot get the lora_received_data_queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while(lora_get_transmit_data_queue(&g_lora_transmit_data_queue))
    {
        ESP_LOGE(TAG, "Cannot get the lora_transmit_data_queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    for (;;)
    {
        data_center_receive_lora_data();
        //data_center_send_lora_data();
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void data_center_task_start(void)
{
    ESP_LOGI(TAG, "Data center task started");

    xTaskCreatePinnedToCore(data_center_task, "data_center_task", DATA_CENTER_TASK_STACK_SIZE, NULL, DATA_CENTER_TASK_PRIORITY, NULL, DATA_CENTER_TASK_CORE_ID);
}