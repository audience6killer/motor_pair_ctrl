#include "queue"

extern "C"
{
#include "esp_log.h"
#include "esp_check.h"

#include "esp_timer.h"

#include "waypoint_controller.h"
#include "waypoint_controller_task_common.h"

#include "diff_drive_ctrl.h"
}

static const char TAG[] = "waypoint";
static QueueHandle_t g_waypoint_data_queue = NULL;
static QueueHandle_t g_waypoint_cmd_queue = NULL;
static QueueHandle_t g_diff_drive_cmd_queue = NULL;
static QueueHandle_t g_diff_drive_data_queue = NULL;
static std::queue<navigation_point_t> g_navigation_points;
static waypoint_state_e g_waypoint_state = WP_STOPPED;
static esp_timer_handle_t g_next_point_timer;

esp_err_t waypoint_get_data_queue_handle(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_waypoint_data_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue not initialized");

    *handle = g_waypoint_data_queue;
    return ESP_OK;
}

esp_err_t waypoint_get_cmd_queue_handle(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_waypoint_cmd_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "CMD queue not initialized");

    *handle = g_waypoint_cmd_queue;
    return ESP_OK;
}

esp_err_t waypoint_send2queue(waypoint_state_e state)
{
    ESP_RETURN_ON_FALSE(g_waypoint_data_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "trying to send msg when queue is null");

    g_waypoint_state = state;
    if (xQueueSend(g_waypoint_data_queue, &state, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending to queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t waypoint_add_point(float x, float y, float theta)
{
    g_navigation_points.emplace(x, y, theta);
    return ESP_OK;
}

void waypoint_trajectory_ctrl(void *args)
{
    diff_drive_state_t state = *(diff_drive_state_t *)args;

    if (state.state == DD_STATE_POINT_REACHED)
    {
        if (g_navigation_points.size() > 0)
        {
            navigation_point_t point = g_navigation_points.front();
            g_navigation_points.pop();

            if (xQueueSend(g_diff_drive_cmd_queue, &point, pdMS_TO_TICKS(100)) != pdPASS)
            {
                ESP_LOGE(TAG, "Error sending point to diff drive");
                return;
            }
            // diff_drive_set_navigation_point(point);

            ESP_ERROR_CHECK(waypoint_send2queue(WP_NAVIGATING));
            ESP_LOGI(TAG, "Next point in trajectory sended: (%.4f, %.4f, %.4f)", point.x, point.y, point.theta);
        }

        else
        {
            ESP_LOGI(TAG, "Trajectory finished!");
            ESP_ERROR_CHECK(waypoint_send2queue(WP_TRJ_FINISHED));
        }
    }
}

esp_err_t waypoint_start_trajectory(void)
{
    if (g_navigation_points.size() > 0)
    {
        if (g_waypoint_state == WP_NAVIGATING)
        {
            ESP_LOGW(TAG, "Trajectory already started!");
            return ESP_OK;
        }

        waypoint_send2queue(WP_NAVIGATING);
        ESP_LOGI(TAG, "Starting trajectory");
        return ESP_OK;
    }
    else
    {
        waypoint_send2queue(WP_STOPPED);
        ESP_LOGE(TAG, "Cannot start trajectory because it is empty!");
        return ESP_FAIL;
    }
}

void waypoint_receive_from_diff_drive(void)
{
    static diff_drive_state_t diff_drive_state;

    if (g_waypoint_state == WP_NAVIGATING)
    {
        if (xQueueReceive(g_diff_drive_data_queue, &diff_drive_state, portMAX_DELAY) == pdPASS)
        {
            if (diff_drive_state.state == DD_STATE_POINT_REACHED)
            {
                if (!esp_timer_is_active(g_next_point_timer))
                {
                    ESP_LOGI(TAG, "Point reached. Starting timer");
                    ESP_ERROR_CHECK(esp_timer_start_once(g_next_point_timer, WAYPOINT_NXT_POINT_WAIT));
                    ESP_ERROR_CHECK(waypoint_send2queue(WP_WAITING));
                }
            }
        }
    }
}

/* Event handlers */
void waypoint_event_handler(void)
{
    static waypoint_cmd_t cmd;
    if(xQueueReceive(g_waypoint_cmd_queue, &cmd, pdMS_TO_TICKS(20)) == pdPASS)
    {
        switch (cmd.cmd)
        {
        case WP_CMD_START_TRAJ:
            waypoint_start_event_handler();
            break;
        
        case WP_CMD_STOP_TRAJ:
            waypoint_stop_event_handler();
            break;

        case WP_CMD_RECEIVE_POINT:
            waypoint_receive_point_event_handler(cmd.point);
            break;
        default:
            break;
        }
    }
}

void waypoint_start_event_handler(void)
{
    ESP_LOGI(TAG, "Start event received");
    if (g_waypoint_state == WP_NAVIGATING)
    {
        ESP_LOGW(TAG, "Trajectory already started!");
        return;
    }
    waypoint_start_trajectory();
    
}

void waypoint_stop_event_handler(void)
{
    ESP_LOGI(TAG, "Stop event received");
    if (g_waypoint_state == WP_STOPPED)
    {
        ESP_LOGW(TAG, "Trajectory has already stopped!");
        return;
    }
    
    waypoint_send2queue(WP_STOPPED);
}

void waypoint_receive_point_event_handler(navigation_point_t *point)
{
    ESP_LOGI(TAG, "Receive point event received");
}

/* Main task */
static void waypoint_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing waypoint task");

    /* INitialize queues */
    g_waypoint_data_queue = xQueueCreate(4, sizeof(navigation_point_t));
    g_waypoint_cmd_queue = xQueueCreate(4, sizeof(waypoint_cmd_t));

    /* Get queue handles */
    while (diff_drive_get_data_queue_handle(&g_diff_drive_data_queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error retraving diff drive data queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(40));
    }

    /* Initialize timer to wait for sending next point*/
    esp_timer_create_args_t next_point_timer_args = {
        .callback = &waypoint_trajectory_ctrl,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "nxt_pnt_tmr",
        .skip_unhandled_events = false};

    ESP_ERROR_CHECK(esp_timer_create(&next_point_timer_args, &g_next_point_timer));

    for (;;)
    {
        waypoint_event_handler();
        waypoint_receive_from_diff_drive();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void waypoint_start_task(void)
{

    ESP_LOGI(TAG, "Iniatilizing task");

    xTaskCreatePinnedToCore(&waypoint_task, "waypoint", WAYPOINT_STACK_SIZE, NULL, WAYPOINT_PRIORITY, NULL, WAYPOINT_CORE_ID);
}