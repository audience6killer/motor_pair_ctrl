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
#include "freertos/event_groups.h"

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
    if (xQueueSend(g_waypoint_queue_handle, &state, pdMS_TO_TICKS(100)) != pdPASS)
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
    if (g_navigation_points.size() > 0)
    {
        navigation_point_t point = g_navigation_points.front();
        g_navigation_points.pop();
        ESP_ERROR_CHECK(esp_event_post_to(g_diff_drive_event_handle, DIFF_DRIVE_EVENT_BASE, DIFF_DRIVE_RECEIVE_POINT_EVENT, &point, sizeof(point), portMAX_DELAY));
        // diff_drive_set_navigation_point(point);
        if(g_waypoint_state != WP_NAVIGATING)
            ESP_ERROR_CHECK_WITHOUT_ABORT( waypoint_ctrl_send2queue(WP_NAVIGATING) );

        ESP_LOGI(TAG, "Next point in trajectory sended: (%.4f, %.4f, %.4f)", point.x, point.y, point.theta);
    }

    else
    {
        ESP_LOGI(TAG, "Trajectory finished!");
        waypoint_ctrl_send2queue(WP_TRJ_FINISHED);
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
        ESP_ERROR_CHECK(esp_event_post_to(g_diff_drive_event_handle, DIFF_DRIVE_EVENT_BASE, DIFF_DRIVE_START_EVENT, NULL, 0, pdMS_TO_TICKS(200)));
        waypoint_ctrl_send2queue(WP_NAVIGATING);

        // Send first point
        //waypoint_ctrl_trajectory_ctrl(NULL);

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
    ESP_ERROR_CHECK(esp_event_post_to(g_diff_drive_event_handle, DIFF_DRIVE_EVENT_BASE, DIFF_DRIVE_STOP_EVENT, NULL, 0, pdMS_TO_TICKS(200)));
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

    /* Wait for started bit on the diff drive task */
    EventBits_t bits = xEventGroupWaitBits(diff_drive_event_group_handle, DD_STARTED, pdTRUE, pdTRUE, pdMS_TO_TICKS(200));

    if ((bits & DD_STARTED) == 0)
    {
        ESP_LOGE(TAG, "Error receiving started flag from diff drive");
        return;
    }

    ESP_LOGI(TAG, "Sending first point!");
    waypoint_ctrl_trajectory_ctrl(NULL);

}

esp_err_t waypoint_ctrl_receive_from_diff_drive(void)
{
    if (g_waypoint_state == WP_NAVIGATING)
    {
        EventBits_t bits = xEventGroupWaitBits(diff_drive_event_group_handle, DD_POINT_REACHED, pdTRUE, pdTRUE, 0);
        if ((bits & DD_POINT_REACHED) != 0)
        {
            ESP_LOGW(TAG, "Point reached. Starting timer");

            waypoint_ctrl_trajectory_ctrl(NULL);
            // ESP_ERROR_CHECK(esp_timer_start_once(g_next_point_timer, WAYPOINT_CTRL_NXT_POINT_WAIT));

            // uint64_t current_time = esp_timer_get_time(); // Get the current time
            // uint64_t expiry_time = 0;
            // ESP_ERROR_CHECK(esp_timer_get_expiry_time(g_next_point_timer, &expiry_time)); // Get the expiry time

            // if (expiry_time > current_time)
            //{
            //     uint64_t remaining_time = expiry_time - current_time;       // Calculate remaining time
            //     printf("Remaining time: %llu ms\n", remaining_time / 1000); // Convert to milliseconds
            // }
            // else
            //{
            //     printf("Timer has already expired or invalid expiry time\n");
            // }

            //waypoint_ctrl_send2queue(WP_NAVIGATING);
        }
    }
    return ESP_OK;
}

static void waypoint_ctrl_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Waypoint ctrl task");

    g_waypoint_queue_handle = xQueueCreate(4, sizeof(navigation_point_t));

    /* wait for diff drive to finish initializing */
    xEventGroupWaitBits(diff_drive_event_group_handle, DD_READY, pdTRUE, pdTRUE, portMAX_DELAY);
    // ESP_LOGI(TAG, "Traction finished initializing");

    /* Get a reference to the diff drive data queue */
    ESP_ERROR_CHECK(diff_drive_get_queue_handle(&g_diff_drive_queue));
    ESP_ERROR_CHECK(diff_drive_get_event_loop(&g_diff_drive_event_handle));

    /* Timer to wait for the next point */
    esp_timer_create_args_t next_point_timer_args = {
        .callback = &waypoint_ctrl_trajectory_ctrl,
        .name = "next_point_timer",
    };

    ESP_ERROR_CHECK(esp_timer_create(&next_point_timer_args, &g_next_point_timer));

    /* Setting up event loop */
    esp_event_loop_args_t event_loop_args = {
        .queue_size = 10,
        .task_name = "wp_event_loop",
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

    //waypoint_ctrl_set_state(WP_READY);
    waypoint_ctrl_send2queue(WP_READY);

    for (;;)
    {
        waypoint_ctrl_receive_from_diff_drive();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void waypoint_ctrl_start_task(void)
{
    ESP_LOGI(TAG, "Iniatilizing task");

    xTaskCreatePinnedToCore(&waypoint_ctrl_task, "waypoint_ctrl", WAYPOINT_CTRL_STACK_SIZE, NULL, WAYPOINT_CTRL_PRIORITY, NULL, WAYPOINT_CTRL_CORE_ID);
}