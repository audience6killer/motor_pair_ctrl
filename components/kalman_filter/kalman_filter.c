#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

#include "kalman_filter.h"
#include "kalman_filter_task_common.h"

#include "odometry_unit.h"

/* Static variables */
static const char TAG[] = "kalman_filter_task";
static QueueHandle_t g_kalman_data_queue = NULL;
static QueueHandle_t g_kalman_cmd_queue = NULL;
static QueueHandle_t g_odometry_data_queue = NULL;
static QueueHandle_t g_odometry_cmd_queue = NULL;
static kalman_state_e g_kalman_state = KALMAN_STATE_STOPPED;

esp_err_t kalman_initialize_info(kalman_info_t *data)
{
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_STATE, TAG, "Null data to initialize");

    data->x = 0.0f;
    data->y = 0.0f;
    data->z = 0.0f;
    data->theta = 0.0f;
    data->x_p = 0.0f;
    data->y_p = 0.0f;
    data->z_p = 0.0f;
    data->theta_p = 0.0f;

    return ESP_OK;
}

esp_err_t kalman_get_cmd_queue(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_kalman_cmd_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Retriving queue handle uniniatilized");

    *handle = g_kalman_cmd_queue;
    return ESP_OK;
}

esp_err_t kalman_get_data_queue(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_kalman_data_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Retriving queue handle uniniatilized");

    *handle = g_kalman_data_queue;
    return ESP_OK;
}

esp_err_t kalman_send2queue(kalman_info_t *data)
{
    ESP_RETURN_ON_FALSE(g_kalman_data_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue handle is null");

    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_STATE, TAG, "Null data to send to queue");

    //printf("/*x,%.4f,y,%.4f,z,%.4f,theta,%.4f*/\n",
    //         data->x, data->y, data->z, data->theta);

    if (xQueueSend(g_kalman_data_queue, data, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending queue");
    }

    return ESP_OK;
}

/* Sources reader */
esp_err_t kalman_get_sources_data(void)
{
    static odometry_data_t odo_robot_pose;
    static kalman_info_t nav_point = (kalman_info_t){
        .x = 0.0f,
        .y = 0.0f,
        .z = 0.0f,
        .theta = 0.0f,
        .x_p = 0.0f,
        .y_p = 0.0f,
        .z_p = 0.0f,
        .theta_p = 0.0f,
    };

    if (xQueueReceive(g_odometry_data_queue, &odo_robot_pose, pdMS_TO_TICKS(20)) == pdPASS)
    {
        nav_point = (kalman_info_t){
            .x = odo_robot_pose.x.cur_value,
            .y = odo_robot_pose.y.cur_value,
            .z = 0.0f,
            .theta = odo_robot_pose.theta.cur_value,
            .x_p = odo_robot_pose.x.diff_value,
            .y_p = odo_robot_pose.y.diff_value,
            .z_p = 0.0f,
            .theta_p = odo_robot_pose.theta.diff_value,
        };

        kalman_send2queue(&nav_point);
    }

    return ESP_OK;
}

/* Event handlers */
// TODO: Kalman and waypoint will communicate with state machine through event groups
esp_err_t kalman_start_event_handler(void)
{
    if (g_kalman_state == KALMAN_STATE_STARTED)
    {
        ESP_LOGW(TAG, "Kalman process already started");
        return ESP_OK;
    }

    odometry_cmd_e cmd_odo = ODO_CMD_START;
    if(xQueueSend(g_odometry_cmd_queue, &cmd_odo, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error: Sending start cmd to odometry");
        return ESP_FAIL;
    }

    g_kalman_state = KALMAN_STATE_STARTED;
    ESP_LOGI(TAG, "Process started");
    return ESP_OK;
}

esp_err_t kalman_stop_event_handler(void)
{
    if (g_kalman_state == KALMAN_STATE_STOPPED)
    {
        ESP_LOGW(TAG, "Kalman process already stopped");
        return ESP_OK;
    }

    /* Stop sources process*/
    odometry_cmd_e cmd_odo = ODO_CMD_STOP;
    if(xQueueSend(g_odometry_cmd_queue, &cmd_odo, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error: Sending stop cmd to odometry");
        return ESP_FAIL;
    }

    g_kalman_state = KALMAN_STATE_STOPPED;
    ESP_LOGI(TAG, "Process stopped");
    return ESP_OK;
}

void kalman_event_handler(void)
{
    static kalman_cmd_e cmd;
    if (xQueueReceive(g_kalman_cmd_queue, &cmd, pdMS_TO_TICKS(50)) == pdPASS)
    {
        switch (cmd)
        {
        case KALMAN_CMD_STOP:
            ESP_LOGI(TAG, "Event: Stop event");
            kalman_stop_event_handler();
            break;

        case KALMAN_CMD_START:
            ESP_LOGI(TAG, "Event: Start event");
            kalman_start_event_handler();
            break;

        default:
            break;
        }
    }
}

static void kalman_filter_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initilizing task");

    /* Initialize queues */
    g_kalman_data_queue = xQueueCreate(4, sizeof(kalman_info_t));
    g_kalman_cmd_queue = xQueueCreate(4, sizeof(kalman_cmd_e));

    /* Get sources data queues */
    while (odometry_get_data_queue(&g_odometry_data_queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error getting odometry data queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    while (odometry_get_cmd_queue(&g_odometry_cmd_queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error getting odometry CMD queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    for (;;)
    {
        kalman_event_handler();

        if (g_kalman_state == KALMAN_STATE_STARTED)
        {
            kalman_get_sources_data();
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void kalman_filter_start_task(void)
{
    ESP_LOGI(TAG, "Starting kalman filter task");

    xTaskCreatePinnedToCore(&kalman_filter_task, "kalman_filter", KALMAN_FILTER_STACK_SIZE, NULL, KALMAN_FILTER_TASK_PRIORITY, NULL, KALMAN_FILTER_CORE_ID);
}
