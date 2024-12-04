#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "kalman_filter.h"
#include "kalman_filter_task_common.h"

#include "odometry_unit.h"

static const char TAG[] = "kalman_filter_task";
static QueueHandle_t g_kalman_queue_handle = NULL;

QueueHandle_t kalman_fiter_get_queue(void)
{
    return g_kalman_queue_handle;
}

static void kalman_filter_task(void *pvParameters)
{
    QueueHandle_t odometry_queue_pv = odometry_unit_get_queue_handle();
    odometry_robot_pose_t odo_robot_pose;

    navigation_point_t nav_point;

    g_kalman_queue_handle = xQueueCreate(4, sizeof(navigation_point_t));

    for(;;)
    {
        if(xQueueReceive(odometry_queue_pv, &odo_robot_pose, portMAX_DELAY) == pdPASS)
        {
            nav_point = (navigation_point_t){
                .x = odo_robot_pose.x,
                .y = odo_robot_pose.y,
                .theta = odo_robot_pose.theta,
            };

            if( xQueueSend(g_kalman_queue_handle, &nav_point, portMAX_DELAY) != pdPASS )
            {
                ESP_LOGE(TAG, "Error sending queue");
            }
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
