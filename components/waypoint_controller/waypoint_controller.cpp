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
static QueueHandle_t g_waypoint_state_queue = NULL;
static QueueHandle_t g_waypoint_cmd_queue = NULL;
static QueueHandle_t g_diff_drive_cmd_queue = NULL;
static QueueHandle_t g_diff_drive_data_queue = NULL;
static std::queue<navigation_point_t> g_navigation_points;
static waypoint_state_e g_waypoint_state = WP_STOPPED;
static esp_timer_handle_t g_next_point_timer;
static EventGroupHandle_t g_event_group_handle = NULL;
static EventGroupHandle_t g_error_group_handle = NULL;

esp_err_t waypoint_get_state_queue_handle(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_waypoint_state_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue not initialized");

    *handle = g_waypoint_state_queue;
    return ESP_OK;
}

esp_err_t waypoint_get_cmd_queue_handle(QueueHandle_t *handle)
{

    *handle = g_waypoint_cmd_queue;
    return ESP_OK;
}

esp_err_t waypoint_get_event_group(EventGroupHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_event_group_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Event group is null while retraving");

    *handle = g_event_group_handle;

    return ESP_OK;
}

esp_err_t waypoint_get_error_group(EventGroupHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_error_group_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Error event group is null while retraving");

    *handle = g_error_group_handle;

    return ESP_OK;
}

esp_err_t waypoint_send2queue(waypoint_state_e state)
{
    g_waypoint_state = state;
    if (xQueueSend(g_waypoint_state_queue, &state, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending to queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t waypoint_add_point(float x, float y, float theta)
{
    g_navigation_points.emplace(x, y, theta);
    ESP_LOGI(TAG, "Point -> x:%.4f, y:%.4f, theta:%.4f", x, y, theta);

    return ESP_OK;
}

/* ISR Routine */
void waypoint_trajectory_ctrl(void *args)
{
    if (g_waypoint_state == WP_WAITING)
    {
        if (g_navigation_points.size() > 0)
        {
            navigation_point_t point = g_navigation_points.front();
            g_navigation_points.pop();

            diff_drive_cmd_t point_cmd = {
                .cmd = DD_CMD_RECEIVE_POINT,
                .point = &point,
            };

            xQueueSendFromISR(g_diff_drive_cmd_queue, &point_cmd, NULL);

            g_waypoint_state = WP_NAVIGATING;
            waypoint_state_e state = g_waypoint_state;
            xQueueSendFromISR(g_waypoint_state_queue, &state, NULL);
        }

        else
        {
            g_waypoint_state = WP_TRJ_FINISHED;
            waypoint_state_e state = g_waypoint_state;
            xQueueSendFromISR(g_waypoint_state_queue, &state, NULL);
        }
    }
}

void waypoint_receive_from_diff_drive(void)
{
    static diff_drive_state_e diff_drive_state;

    if (xQueueReceive(g_diff_drive_data_queue, &diff_drive_state, pdMS_TO_TICKS(100)) == pdPASS)
    {
        if (diff_drive_state == DD_STATE_POINT_REACHED)
        {
            if (!esp_timer_is_active(g_next_point_timer))
            {
                ESP_LOGI(TAG, "Point reached. Starting timer");
                ESP_ERROR_CHECK(esp_timer_start_once(g_next_point_timer, WAYPOINT_NXT_POINT_WAIT));

                ESP_ERROR_CHECK(waypoint_send2queue(WP_WAITING));
                // waypoint_trajectory_ctrl(NULL);
            }
        }
    }
}

/* Event handlers */
esp_err_t waypoint_start_event_handler(void)
{
    if (g_waypoint_state == WP_NAVIGATING)
    {
        //ESP_LOGW(TAG, "Trajectory already started!");
        xEventGroupSetBits(g_event_group_handle, WP_NAVIGATING);
        return ESP_OK;
    }
    if (g_navigation_points.size() == 0)
    {
        //ESP_LOGE(TAG, "There are no navigation points!");
        xEventGroupSetBits(g_event_group_handle, WP_ERROR);
        xEventGroupSetBits(g_error_group_handle, WP_ERROR_EMPTY_NAV_POINTS);
        return ESP_FAIL;
    }

    /* Start diff drive */
    diff_drive_cmd_t cmd = {
        .cmd = DD_CMD_START,
        .point = NULL,
    };

    if (xQueueSend(g_diff_drive_cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdPASS)
    {
        //ESP_LOGE(TAG, "Error: Cannot send start command to diff_drive");
        xEventGroupSetBits(g_event_group_handle, WP_ERROR);
        xEventGroupSetBits(g_error_group_handle, WP_ERROR_CANNOT_START_TRACT);
        return ESP_FAIL;
    }

    /* Send first point */
    navigation_point_t point = g_navigation_points.front();
    g_navigation_points.pop();

    diff_drive_cmd_t point_cmd = {
        .cmd = DD_CMD_RECEIVE_POINT,
        .point = &point,
    };

    if (xQueueSend(g_diff_drive_cmd_queue, &point_cmd, pdMS_TO_TICKS(100)) != pdPASS)
    {
        //ESP_LOGE(TAG, "Error: Cannot send first point to diff_drive");
        xEventGroupSetBits(g_event_group_handle, WP_ERROR);
        xEventGroupSetBits(g_error_group_handle, WP_ERROR_CANNOT_SEND_FPOINT);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "First point in trajectory sended: (%.4f, %.4f, %.4f)", point.x, point.y, point.theta);
    if (waypoint_send2queue(WP_NAVIGATING) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot send state to queue");
        //return ESP_FAIL;
    }

    /* Notify to parent task */
    xEventGroupSetBits(g_event_group_handle, WP_NAVIGATING);

    return ESP_OK;
}

esp_err_t waypoint_stop_event_handler(void)
{
    if (g_waypoint_state == WP_STOPPED)
    {
        ESP_LOGW(TAG, "Trajectory has already stopped!");
        return ESP_OK;
    }

    if (waypoint_send2queue(WP_STOPPED) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot set stopped state");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Trayectory stopped with %d points left", g_navigation_points.size());

    return ESP_OK;
}

esp_err_t waypoint_receive_point_event_handler(navigation_point_t *point)
{
    ESP_RETURN_ON_FALSE(point != NULL, ESP_ERR_INVALID_STATE, TAG, "Point received was null");

    waypoint_add_point(point->x, point->y, point->theta);
    ESP_LOGI(TAG, "Point added successfully");

    xEventGroupSetBits(g_event_group_handle, WP_POINT_ADDED);

    return ESP_OK;
}

void waypoint_event_handler(void)
{
    static waypoint_cmd_t cmd;
    if (xQueueReceive(g_waypoint_cmd_queue, &cmd, pdMS_TO_TICKS(20)) == pdPASS)
    {
        // TODO Error handling in the state machine
        switch (cmd.cmd)
        {
        case WP_CMD_START_TRAJ:
            ESP_LOGI(TAG, "Event: Start Trajectory");
            waypoint_start_event_handler();
            break;

        case WP_CMD_STOP_TRAJ:
            ESP_LOGI(TAG, "Event: Stop Trajectory");
            waypoint_stop_event_handler();
            break;

        case WP_CMD_RECEIVE_POINT:
            ESP_LOGI(TAG, "Event: Receive Point");
            waypoint_receive_point_event_handler(cmd.point);
            break;
        default:
            break;
        }
    }
}

/* Main task */
static void waypoint_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing waypoint task");

    /* Initialize queues */
    g_waypoint_state_queue = xQueueCreate(4, sizeof(waypoint_state_e));
    g_waypoint_cmd_queue = xQueueCreate(4, sizeof(waypoint_cmd_t));

    /* Get queue handles */
    while (diff_drive_get_state_queue_handle(&g_diff_drive_data_queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error retraving diff drive data queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    while (diff_drive_get_cmd_queue_handle(&g_diff_drive_cmd_queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error retraving diff drive cmd queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(40));
    }

    /* Initialize timer to wait for sending next point*/
    esp_timer_create_args_t next_point_timer_args = {
        .callback = waypoint_trajectory_ctrl,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_ISR,
        .name = "nxt_pnt_tmr",
    };

    ESP_ERROR_CHECK(esp_timer_create(&next_point_timer_args, &g_next_point_timer));

    /* Initialize event groups */
    g_event_group_handle = xEventGroupCreate();
    g_error_group_handle = xEventGroupCreate();

    for (;;)
    {
        if (g_waypoint_state == WP_NAVIGATING)
        {
            waypoint_receive_from_diff_drive();
        }

        waypoint_event_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void waypoint_start_task(void)
{

    ESP_LOGI(TAG, "Starting task");

    xTaskCreatePinnedToCore(&waypoint_task, "waypoint", WAYPOINT_STACK_SIZE, NULL, WAYPOINT_PRIORITY, NULL, WAYPOINT_CORE_ID);
}