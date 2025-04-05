/**
 * @file waypoint_controller.cpp
 * @author your name (you@domain.com)
 * @brief Controller that provides points to the differential drive controller
 * @version 0.1
 * @date 2025-03-26
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "queue"

extern "C"
{
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_event.h"

#include "waypoint_controller.h"
#include "waypoint_controller_task_common.h"

#include "diff_drive_ctrl.h"
}

ESP_EVENT_DEFINE_BASE(WAYPOINT_EVENT_BASE);

static const char TAG[] = "waypoint_ctrl";
static QueueHandle_t g_waypoint_queue_handle = NULL;
static QueueHandle_t g_diff_drive_queue = NULL;
static diff_drive_state_t g_diff_drive_state;
static TaskHandle_t g_waypoint_task_pv = NULL;
static TaskHandle_t g_parent_ptr = NULL;
static esp_event_loop_handle_t g_waypoint_event_handle = NULL;
static esp_event_loop_handle_t g_diff_drive_event_handle = NULL;
static std::queue<navigation_point_t> g_navigation_points;
static waypoint_ctrl_state_e g_waypoint_state = WP_STOPPED;
static esp_timer_handle_t g_next_point_timer;

void waypoint_ctrl_set_state(waypoint_ctrl_state_e state)
{
    g_waypoint_state = state;
}

esp_err_t waypoint_ctrl_get_event_loop(esp_event_loop_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_waypoint_event_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Event loop not initialized");

    *handle = g_waypoint_event_handle;
    return ESP_OK;
}

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

    if (state.state == DD_POINT_REACHED)
    {
        if (g_navigation_points.size() > 0)
        {
            navigation_point_t point = g_navigation_points.front();
            g_navigation_points.pop();
            ESP_ERROR_CHECK(esp_event_post_to(g_diff_drive_event_handle, DIFF_DRIVE_EVENT_BASE, DIFF_DRIVE_RECEIVE_POINT_EVENT, &point, sizeof(point), portMAX_DELAY));
            // diff_drive_set_navigation_point(point);
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

        ESP_LOGI(TAG, "Starting trajectory");
        ESP_ERROR_CHECK(esp_event_post_to(g_diff_drive_event_handle, DIFF_DRIVE_EVENT_BASE, DIFF_DRIVE_START_EVENT, NULL, 0, portMAX_DELAY));
        waypoint_ctrl_send2queue(WP_NAVIGATING);

        return ESP_OK;
    }
    else
    {
        waypoint_ctrl_send2queue(WP_STOPPED);
        ESP_LOGE(TAG, "Cannot start trajectory because it is empty!");
        return ESP_FAIL;
    }
}

/* Event handlers */
static void waypoint_add_point_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    ESP_LOGI(TAG, "Adding point to trajectory");
    if (event_data == NULL)
    {
        ESP_LOGE(TAG, "event data is NULL");
        return;
    }

    navigation_point_t *point = (navigation_point_t *)event_data;
    ESP_ERROR_CHECK(waypoint_ctrl_add_point(point->x, point->y, point->theta));
}

static void waypoint_stop_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    ESP_LOGI(TAG, "Stopping waypoint controller");
    if (g_waypoint_state == WP_STOPPED)
    {
        ESP_LOGW(TAG, "Waypoint controller already stopped!");
        return;
    }

    ESP_LOGI(TAG, "Stopping waypoint controller");
    ESP_ERROR_CHECK(esp_event_post_to(g_diff_drive_event_handle, DIFF_DRIVE_EVENT_BASE, DIFF_DRIVE_STOP_EVENT, NULL, 0, portMAX_DELAY));
    waypoint_ctrl_set_state(WP_STOPPED);
    waypoint_ctrl_send2queue(g_waypoint_state);
}

static void waypoint_start_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    ESP_LOGI(TAG, "Starting waypoint controller");

    esp_err_t ret = waypoint_ctrl_start_trajectory();

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error starting trajectory. Code: %s", esp_err_to_name(ret));
        return;
    }
}

esp_err_t waypoint_ctrl_receive_from_diff_drive(void)
{
    if (g_waypoint_state == WP_NAVIGATING)
    {
        if (xQueueReceive(g_diff_drive_queue, &g_diff_drive_state, portMAX_DELAY) == pdPASS)
        {
            if (g_diff_drive_state.state == DD_POINT_REACHED)
            {
                if (!esp_timer_is_active(g_next_point_timer))
                {
                    ESP_LOGI(TAG, "Point reached. Starting timer");
                    ESP_ERROR_CHECK(esp_timer_start_once(g_next_point_timer, WAYPOINT_CTRL_NXT_POINT_WAIT));
                    ESP_ERROR_CHECK(waypoint_ctrl_send2queue(WP_WAITING));
                }
            }
        }
        else
        {
            ESP_LOGE(TAG, "Error receiving from diff drive queue");
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

static void waypoint_ctrl_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Waypoint ctrl task");

    g_waypoint_queue_handle = xQueueCreate(4, sizeof(navigation_point_t));

    /* wait for diff drive to finish initializing */
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Traction finished initializing");

    /* Get a reference to the diff drive data queue */
    ESP_ERROR_CHECK(diff_drive_get_queue_handle(&g_diff_drive_queue));
    ESP_ERROR_CHECK(diff_drive_get_event_loop(&g_diff_drive_event_handle));

    /* Timer to wait for the next point */
    esp_timer_create_args_t next_point_timer_args = {
        .callback = &waypoint_ctrl_trajectory_ctrl,
        .arg = &g_diff_drive_state,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "next_point_timer",
        .skip_unhandled_events = false};

    ESP_ERROR_CHECK(esp_timer_create(&next_point_timer_args, &g_next_point_timer));

    /* Setting up event loop */
    esp_event_loop_args_t event_loop_args = {
        .queue_size = 10,
        .task_name = "waypoint_event_loop",
        .task_priority = 5,
        .task_stack_size = 4084,
        .task_core_id = 0,
    };

    ESP_ERROR_CHECK(esp_event_loop_create(&event_loop_args, &g_waypoint_event_handle));

    /* Register events */
    ESP_ERROR_CHECK(esp_event_handler_register_with(g_waypoint_event_handle, WAYPOINT_EVENT_BASE, WP_STOP_EVENT, waypoint_stop_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(g_waypoint_event_handle, WAYPOINT_EVENT_BASE, WP_START_EVENT, waypoint_start_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(g_waypoint_event_handle, WAYPOINT_EVENT_BASE, WP_ADD_POINT_EVENT, waypoint_add_point_event_handler, NULL));

    /* Notify end of initialization */
    if (g_parent_ptr != NULL)
        xTaskNotifyGive(g_parent_ptr);

    waypoint_ctrl_set_state(WP_READY);
    waypoint_ctrl_send2queue(g_waypoint_state);
    
    for (;;)
    {
        waypoint_ctrl_receive_from_diff_drive();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void waypoint_ctrl_start_task(TaskHandle_t parent)
{
    ESP_LOGI(TAG, "Iniatilizing task");

    if (parent != NULL)
        g_parent_ptr = parent;

    xTaskCreatePinnedToCore(&waypoint_ctrl_task, "waypoint_ctrl", WAYPOINT_CTRL_STACK_SIZE, NULL, WAYPOINT_CTRL_PRIORITY, &g_waypoint_task_pv, WAYPOINT_CTRL_CORE_ID);

    diff_drive_task_start(g_waypoint_task_pv);
}