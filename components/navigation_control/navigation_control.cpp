#include "queue"

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"

#include "navigation_control.h"
#include "navigation_control_task_common.h"

#include "odometry_unit.h"
#include "differential_drive_ctrl.h"

#define NAVIGATION_POINT_NO 2
}

static const char TAG[] = "navigation_control";

static QueueHandle_t navigation_queue_handle;

static std::queue<navigation_point_t> g_navigation_points;

esp_err_t navigation_add_nav_point(navigation_point_t point)
{
    g_navigation_points.push(point);
    return ESP_OK;
}

esp_err_t navigation_kalman_filter(navigation_point_t navigation_point)
{
    if(xQueueSend(diff_drive_get_queue_handle(), &navigation_point, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(TAG, "error sending queue");
    }

    return ESP_OK;
}

static void navigation_control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "navigation control task");
    // ESP_RETURN_ON_FALSE(g_navigation_points == NULL, ESP_ERR_INVALID_STATE, TAG, "navigation point already initialized");

    QueueHandle_t odometry_queue_handle = odometry_unit_get_queue_handle();
    odometry_robot_pose_t odo_vehicle_pose;

    ESP_LOGI(TAG, "Navigation create queue");
    navigation_queue_handle = xQueueCreate(4, sizeof(navigation_point_t));

    for (;;)
    {
        if (xQueueReceive(odometry_queue_handle, &odo_vehicle_pose, portMAX_DELAY) == pdPASS)
        {
#if false
            printf("/*x,%f,y,%f,theta,%f*/\r\n", vehicle_pose.x, vehicle_pose.y, vehicle_pose.theta);
#endif
            // g_navigation_points.emplace(odo_vehicle_pose.x, odo_vehicle_pose.y, odo_vehicle_pose.theta);

            ESP_ERROR_CHECK(navigation_kalman_filter((navigation_point_t){.x = odo_vehicle_pose.x, .y = odo_vehicle_pose.y, .theta = odo_vehicle_pose.theta}));
        }
        else
        {
            ESP_LOGE(TAG, "Failed to receive queue");
        }
    }
}

void navigation_control_start_task(void)
{

    ESP_LOGI(TAG, "Iniatilizing task");

    xTaskCreatePinnedToCore(&navigation_control_task, "navigation_task", NAVIGATION_TASK_STACK_SIZE, NULL, NAVIGATION_TASK_PRIORITY, NULL, NAVIGATION_TASK_CORE_ID);
}