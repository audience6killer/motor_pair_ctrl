
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "waypoint_controller.h"
#include "kalman_filter.h"
}

#include "Arduino.h"

#include "test_waypoint_follower.h"

#define CORE_ID 0
#define STACK_SIZE 4096
#define PRIORITY 2

static const char TAG[] = "test_waypoint_task";
static QueueHandle_t g_waypoint_cmd_handle = NULL;
static QueueHandle_t g_waypoint_state_handle = NULL;
static QueueHandle_t g_kalman_cmd_handle = NULL;

void start_test_waypoint_follower(void)
{
    ESP_LOGW(TAG, "Starting test_waypoint_task");

    /* Kalman commands */
    kalman_cmd_e cmd_k1 = KALMAN_CMD_START;

    if(xQueueSend(g_kalman_cmd_handle, &cmd_k1, pdMS_TO_TICKS(100)) == pdPASS)
    {
        ESP_LOGI(TAG, "Kalman process started");
    }

    /* Waypoint commands */ 
    navigation_point_t point_1 = (navigation_point_t){
        .x = 0.0f,
        .y = 0.0f,
        .theta = 0.0f,
    };

    navigation_point_t point_2 = (navigation_point_t){
        .x = 1.0f,
        .y = 7.0f,
        .theta = 0.0f,
    };
    waypoint_cmd_t cmd_1 = {
        .cmd = WP_CMD_RECEIVE_POINT,
        .point = &point_1,
    };

    waypoint_cmd_t cmd_2 = {
        .cmd = WP_CMD_RECEIVE_POINT,
        .point = &point_2,
    };

    waypoint_cmd_t cmd_start = {
        .cmd = WP_CMD_START_TRAJ,
        .point = NULL,
    };
    
    if(xQueueSend(g_waypoint_cmd_handle, &cmd_1, pdMS_TO_TICKS(100)) == pdPASS)
    {
        ESP_LOGI(TAG, "Sended first point");
    }

    if(xQueueSend(g_waypoint_cmd_handle, &cmd_1, pdMS_TO_TICKS(100)) == pdPASS)
    {
        ESP_LOGI(TAG, "Sended second point");
    }

    if(xQueueSend(g_waypoint_cmd_handle, &cmd_1, pdMS_TO_TICKS(100)) == pdPASS)
    {
        ESP_LOGI(TAG, "Sended third point");
    }

    ESP_LOGI(TAG, "WAITING TO START TRAJECTORY\n");
    vTaskDelay(pdMS_TO_TICKS(5000));

    if(xQueueSend(g_waypoint_cmd_handle, &cmd_start, pdMS_TO_TICKS(100)) == pdPASS)
    {
        ESP_LOGI(TAG, "Sended start command");
    }
}

void receive_from_waypoint(void)
{
    static waypoint_state_e wp_state;
    if(xQueueReceive(g_waypoint_state_handle, &wp_state, pdMS_TO_TICKS(100)) == pdPASS)
    {
        const char *state_name = waypoint_state_to_string(wp_state);
        printf("Waypoint state: %s\n", state_name);
    }
}

static void test_waypoint_task(void *pvParemeters)
{
    ESP_LOGI(TAG, "Configuring waypoints for test");

    while(waypoint_get_cmd_queue_handle(&g_waypoint_cmd_handle) != ESP_OK)
    {
        ESP_LOGW(TAG, "Error geting waypoint cmd queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while(waypoint_get_state_queue_handle(&g_waypoint_state_handle) != ESP_OK)
    {
        ESP_LOGW(TAG, "Error geting waypoint state queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    while(kalman_get_cmd_queue(&g_kalman_cmd_handle) != ESP_OK)
    {

        ESP_LOGW(TAG, "Error geting kalman cmd queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // ESP_ERROR_CHECK(waypoint_controller_add_point(0.0f, 0.0f, 3.1516f));
    // ESP_ERROR_CHECK(waypoint_ctrl_add_point(10.0f, 0.0f, 0.0f));

    ESP_LOGI(TAG, "Waiting for start signal");

    for (;;)
    {
        if(Serial.available())
        {
            char c = Serial.read();
            if(c == 's')
            {
                start_test_waypoint_follower();
            }
        }
        receive_from_waypoint();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void test_waypoint_follower_task_start(void)
{

    ESP_LOGI(TAG, "Iniatilazing waypoint task");

    xTaskCreatePinnedToCore(&test_waypoint_task, "test_waypoint", STACK_SIZE, NULL, PRIORITY, NULL, CORE_ID);
}
