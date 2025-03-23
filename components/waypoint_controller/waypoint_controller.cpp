#include "queue"

extern "C"
{
#include "esp_log.h"
#include "esp_check.h"

#include "esp_timer.h"

#include "waypoint_controller.h"
#include "waypoint_controller_task_common.h"

#include "differential_drive_ctrl.h"
}

static const char TAG[] = "waypoint_ctrl";
static QueueHandle_t g_waypoint_queue_handle = NULL;
static std::queue<navigation_point_t> g_navigation_points;
static waypoint_ctrl_state_e g_waypoint_state = WP_STOPPED;
static esp_timer_handle_t g_next_point_timer;

esp_err_t waypoint_ctrl_get_queue_handle(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_waypoint_queue_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue not initialized");

    *handle = g_waypoint_queue_handle;
    return ESP_OK;
}

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

esp_err_t waypoint_ctrl_add_point(float x, float y, float theta)
{
    g_navigation_points.emplace(x, y, theta);
    return ESP_OK;
}

void waypoint_ctrl_trajectory_ctrl(void *args)
{
    diff_drive_state_t state = *(diff_drive_state_t *)args;

    if (state.state == POINT_REACHED)
    {
        if (g_navigation_points.size() > 0)
        {
            navigation_point_t point = g_navigation_points.front();
            g_navigation_points.pop();
            diff_drive_set_navigation_point(point);
            ESP_ERROR_CHECK(waypoint_ctrl_send2queue(WP_NAVIGATING));
            ESP_LOGI(TAG, "Next point in trajectory sended: (%.4f, %.4f, %.4f)", point.x, point.y, point.theta);
        }

        else
        {
            ESP_LOGI(TAG, "Trajectory finished!");
            ESP_ERROR_CHECK(waypoint_ctrl_send2queue(WP_TRJ_FINISHED));
        }
    }
}

esp_err_t waypoint_ctrl_start_trajectory(void)
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

static void waypoint_ctrl_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Waypoint ctrl task");

    g_waypoint_queue_handle = xQueueCreate(4, sizeof(navigation_point_t));

    QueueHandle_t diff_drive_queue_pv = NULL;
    ESP_ERROR_CHECK(diff_drive_get_queue_handle(&diff_drive_queue_pv));

    diff_drive_state_t diff_drive_state;

    // Timer to wait for the next point
    esp_timer_create_args_t next_point_timer_args = {
        .callback = &waypoint_ctrl_trajectory_ctrl,
        .arg = &diff_drive_state,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "next_point_timer",
        .skip_unhandled_events = false
    };

    ESP_ERROR_CHECK(esp_timer_create(&next_point_timer_args, &g_next_point_timer));

    for (;;)
    {
        if (g_waypoint_state == WP_NAVIGATING)
        {
            if (xQueueReceive(diff_drive_queue_pv, &diff_drive_state, portMAX_DELAY) == pdPASS)
            {
                if (diff_drive_state.state == POINT_REACHED)
                {
                    if (!esp_timer_is_active(g_next_point_timer))
                    {
                        ESP_LOGI(TAG, "Point reached. Starting timer");
                        ESP_ERROR_CHECK(esp_timer_start_once(g_next_point_timer, WAYPOINT_CTRL_NXT_POINT_WAIT));
                        ESP_ERROR_CHECK(waypoint_ctrl_send2queue(WP_WAITING));
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void waypoint_ctrl_start_task(void)
{

    ESP_LOGI(TAG, "Iniatilizing task");

    xTaskCreatePinnedToCore(&waypoint_ctrl_task, "waypoint_ctrl", WAYPOINT_CTRL_STACK_SIZE, NULL, WAYPOINT_CTRL_PRIORITY, NULL, WAYPOINT_CTRL_CORE_ID);
}