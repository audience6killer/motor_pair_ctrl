#include "queue"

extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"

#include "waypoint_controller.h"
#include "waypoint_controller_task_common.h"

#include "differential_drive_ctrl.h"
}

static const char TAG[] = "waypoint_controller";
static QueueHandle_t g_waypoint_queue_handle = NULL;
static std::queue<navigation_point_t> g_navigation_points;
static waypoint_ctrl_state_e g_waypoint_state = WP_STOPPED;

esp_err_t waypoint_ctrl_send2queue(waypoint_ctrl_state_e state)
{
    ESP_RETURN_ON_FALSE(g_waypoint_queue_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "trying to send msg when queue is null");

    g_waypoint_state = state;
    if (xQueueSend(g_waypoint_queue_handle, &state, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending to queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t waypoint_controller_add_point(float x, float y, float theta)
{
    g_navigation_points.emplace(x, y, theta);
    return ESP_OK;
}

esp_err_t waypoint_controller_trajectory_ctrl(diff_drive_state_t state)
{
    if (state == POINT_REACHED)
    {
        if (g_navigation_points.size() > 0)
        {
            navigation_point_t point = g_navigation_points.front();
            g_navigation_points.pop();
            diff_drive_set_navigation_point(point);
            ESP_LOGI(TAG, "Next point in trajectory sended: (%.4f, %.4f, %.4f)", point.x, point.y, point.theta);
        }

        else
        {
            ESP_LOGI(TAG, "Trajectory finished!");
            ESP_ERROR_CHECK(waypoint_ctrl_send2queue(WP_TRJ_FINISHED));
        }
    }

    return ESP_OK;
}

esp_err_t waypoint_controller_start_trajectory(void)
{
    if (g_navigation_points.size() > 0)
    {
        if (g_waypoint_state == WP_NAVIGATING)
        {
            ESP_LOGW(TAG, "Trajectory already started!");
            return ESP_OK;
        }

        waypoint_ctrl_send2queue(WP_NAVIGATING);
        ESP_LOGI(TAG, "Starting trajectory");
        return ESP_OK;
    }
    else
    {
        waypoint_ctrl_send2queue(WP_STOPPED);
        ESP_LOGE(TAG, "Cannot start trajectory because it is empty!");
        return ESP_FAIL;
    }
}

static void waypoint_controller_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Waypoint controller task");

    g_waypoint_queue_handle = xQueueCreate(4, sizeof(navigation_point_t));

    QueueHandle_t diff_drive_queue_pv = diff_drive_get_queue_handle();

    diff_drive_state_t diff_drive_state;

    for (;;)
    {
        if (g_waypoint_state == WP_NAVIGATING)
        {
            if (xQueueReceive(diff_drive_queue_pv, &diff_drive_state, portMAX_DELAY) == pdPASS)
            {
                ESP_ERROR_CHECK(waypoint_controller_trajectory_ctrl(diff_drive_state));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void waypoint_controller_start_task(void)
{

    ESP_LOGI(TAG, "Iniatilizing task");

    xTaskCreatePinnedToCore(&waypoint_controller_task, "waypoint_controller", WAYPOINT_CONTROLLER_STACK_SIZE, NULL, WAYPOINT_CONTROLLER_PRIORITY, NULL, WAYPOINT_CONTROLLER_CORE_ID);
}