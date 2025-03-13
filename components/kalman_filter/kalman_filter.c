#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

#include "kalman_filter.h"
#include "kalman_filter_task_common.h"

#include "odometry_unit.h"

static const char TAG[] = "kalman_filter_task";
static QueueHandle_t g_kalman_queue_handle = NULL;

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

QueueHandle_t kalman_fiter_get_queue(void)
{
    //ESP_RETURN_ON_FALSE(g_kalman_queue_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Retriving queue handle uniniatilized");

    return g_kalman_queue_handle;
}

esp_err_t kalman_send2queue(kalman_info_t *data)
{
    ESP_RETURN_ON_FALSE(g_kalman_queue_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue handle is null");

    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_STATE, TAG, "Null data to send to queue");

    if (xQueueSend(g_kalman_queue_handle, data, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending queue");
    }

    return ESP_OK;
}

static void kalman_filter_task(void *pvParameters)
{
    QueueHandle_t odometry_queue_pv = odometry_unit_get_queue_handle();
    odometry_data_t odo_robot_pose;

    kalman_info_t nav_point = (kalman_info_t){
        .x = 0.0f,
        .y = 0.0f,
        .z = 0.0f,
        .theta = 0.0f,
        .x_p = 0.0f,
        .y_p = 0.0f,
        .z_p = 0.0f,
        .theta_p = 0.0f,
    };

    g_kalman_queue_handle = xQueueCreate(4, sizeof(kalman_info_t));

    for(;;)
    {
        if(xQueueReceive(odometry_queue_pv, &odo_robot_pose, portMAX_DELAY) == pdPASS)
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

            ESP_ERROR_CHECK(kalman_send2queue(&nav_point));
        
        }
        else
        {
            ESP_LOGE(TAG, "Error receiving queue");
        }
    }
}

void kalman_filter_start_task(void)
{
    ESP_LOGI(TAG, "Initializing kalman filter task");

    xTaskCreatePinnedToCore(&kalman_filter_task, "kalman_filter", KALMAN_FILTER_STACK_SIZE, NULL, KALMAN_FILTER_TASK_PRIORITY, NULL, KALMAN_FILTER_CORE_ID);
}
