
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_check.h"

// #include "data_center.h"
#include "fake_data_center.h"
// #include "lora_rf_unit.h"
#include "waypoint_controller.h"

#include "state_machine.h"
#include "state_machine_task_common.h"
}

static const char TAG[] = "state_machine";
static QueueHandle_t g_waypoint_cmd_queue = NULL;
static QueueHandle_t g_waypoint_status_queue = NULL;
static QueueHandle_t g_kalman_cmd_handle = NULL;
static QueueHandle_t g_data_center_data_queue = NULL;
static bool g_is_running_traj = false;

static EventGroupHandle_t g_waypoint_event_group = NULL;
static EventGroupHandle_t g_waypoint_error_group = NULL;

/* Event handlers */
esp_err_t state_machine_start_event_handler(void)
{
    /* Start kalman process */
    kalman_cmd_e cmd_k = KALMAN_CMD_START;

    if (xQueueSend(g_kalman_cmd_handle, &cmd_k, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error: Cannot send start command to kalman task");
        return ESP_FAIL;
    }

    /* Start waypoint trajectory */
    waypoint_cmd_t cmd_start = {
        .cmd = WP_CMD_START_TRAJ,
        .point = NULL,
    };
    if (xQueueSend(g_waypoint_cmd_queue, &cmd_start, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error: Cannot send start trajectory command to waypoint task");
        return ESP_FAIL;
    }

    EventBits_t wp_status_flag = xEventGroupWaitBits(g_waypoint_event_group, WP_NAVIGATING | WP_ERROR, pdTRUE, pdFALSE, pdMS_TO_TICKS(50));

    if ((wp_status_flag & WP_NAVIGATING) != 0)
    {
        ESP_LOGI(TAG, "Waypoint trajectory start was successfull!");
        g_is_running_traj = true;
    }
    else if ((wp_status_flag & WP_ERROR) != 0)
    {
        /*Error handling*/
        ESP_LOGE(TAG, "Error: Cannot start trajectory");

        EventBits_t error_flag = xEventGroupWaitBits(g_waypoint_error_group, WP_ERROR_EMPTY_NAV_POINTS | WP_ERROR_CANNOT_START_TRACT | WP_ERROR_CANNOT_SEND_FPOINT, pdTRUE, pdFALSE, pdMS_TO_TICKS(50));

        if ((error_flag & WP_ERROR_EMPTY_NAV_POINTS) != 0)
        {
            ESP_LOGE(TAG, "Error: the navigation points are empty");
        }
        else if ((error_flag & WP_ERROR_CANNOT_START_TRACT) != 0)
        {
            ESP_LOGE(TAG, "Error: Cannot start tract");
        }
        else if ((error_flag & WP_ERROR_CANNOT_SEND_FPOINT) != 0)
        {
            ESP_LOGE(TAG, "Error: Cannot send first point tp diff drive");
        }
        else
        {
            ESP_LOGE(TAG, "Error: Cannot get error bits");
        }
    }
    else
    {
        /* error handling */
        ESP_LOGE(TAG, "Error: Waypoint statuts bits were not received");
    }

    return ESP_OK;
}

esp_err_t state_machine_stop_event_handler(void)
{
    /* Stop waypoint trajectory */
    waypoint_cmd_t cmd_stop = {
        .cmd = WP_CMD_STOP_TRAJ,
        .point = NULL,
    };

    if (xQueueSend(g_waypoint_cmd_queue, &cmd_stop, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error: Cannot send stop trajectory command to waypoint task");
        return ESP_FAIL;
    }

    EventBits_t wp_status_flag = xEventGroupWaitBits(g_waypoint_event_group, WP_STOPPED | WP_ERROR, pdTRUE, pdFALSE, pdMS_TO_TICKS(50));

    if ((wp_status_flag & WP_STOPPED) != 0)
    {
        ESP_LOGI(TAG, "Trajectory stopped successfully");
        g_is_running_traj = false;
    }
    else if ((wp_status_flag & WP_ERROR) != 0)
    {
        ESP_LOGE(TAG, "Error: Error stopping trajectory");
    }
    else
    {
        ESP_LOGE(TAG, "Trajectory stopped successfully");
    }

    return ESP_OK;
}

esp_err_t state_machine_nav_point_event_handler(float x, float y, float theta)
{
    navigation_point_t point = {
        .x = x,
        .y = y,
        .theta = theta,
    };
    waypoint_cmd_t cmd = {
        .cmd = WP_CMD_RECEIVE_POINT,
        .point = &point,
    };

    if (xQueueSend(g_waypoint_cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Error: Cannot send point command to waypoint ctrl");
    }

    EventBits_t wp_flag_bits = xEventGroupWaitBits(g_waypoint_event_group, WP_POINT_ADDED | WP_ERROR, pdTRUE, pdFALSE, pdMS_TO_TICKS(100));

    if ((wp_flag_bits & WP_POINT_ADDED) != 0)
    {
        ESP_LOGI(TAG, "Point added successfully");
    }
    else if ((wp_flag_bits & WP_ERROR) != 0)
    {
        ESP_LOGE(TAG, "Error: Point was not added");
    }
    else
    {
        ESP_LOGE(TAG, "Error: No flags were received");
    }

    return ESP_OK;
}

esp_err_t state_machine_echo_event_handler(void)
{
    return ESP_OK;
}

void state_machine_event_handler(void)
{
    data_center_msg_t data_center_msg;
    if (xQueueReceive(g_data_center_data_queue, &data_center_msg, pdMS_TO_TICKS(10)) == pdPASS)
    {
        switch (data_center_msg.code)
        {
            // TODO: Missing cases
        case SM_CMD_EMPTY:
            ESP_LOGW(TAG, "CMD: Empty message received");
            break;
        case SM_CMD_STOP_NAV:
            ESP_LOGI(TAG, "CMD: Stop trajectory");
            ESP_ERROR_CHECK(state_machine_stop_event_handler());
            break;
        case SM_CMD_START_NAV:
            ESP_LOGI(TAG, "CMD: Start trajectory");
            ESP_ERROR_CHECK(state_machine_start_event_handler());
            break;
        case SM_CMD_ADD_WAYPOINT:
            ESP_LOGI(TAG, "CMD: Navigation point added");
            ESP_ERROR_CHECK(state_machine_nav_point_event_handler(data_center_msg.args[0], data_center_msg.args[1], data_center_msg.args[2]));
            break;
        case SM_CMD_ECHO:
            ESP_LOGI(TAG, "CMD: Echo");
            ESP_ERROR_CHECK(state_machine_echo_event_handler());
            break;
        default:
            ESP_LOGE(TAG, "CMD ERROR: Invalid message received");
            break;
        }
    }
}

void state_machine_receive_waypoint_state(void)
{
    waypoint_state_e wp_state;
    if(xQueueReceive(g_waypoint_status_queue, &wp_state, pdMS_TO_TICKS(10)) == pdPASS)
    {
        const char *state = waypoint_state_to_string(wp_state);
        ESP_LOGI(TAG, "Waypoint State: %s", state);
    }
}

/* Main task */
static void state_machine_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initilizing State machine task started");

    /* Get data center data queue */
    while (data_center_get_queue_handle(&g_data_center_data_queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot get data_center data queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    while (waypoint_get_cmd_queue_handle(&g_waypoint_cmd_queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot get waypoint cmd queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    while (waypoint_get_state_queue_handle(&g_waypoint_status_queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot get waypoint status queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    while (kalman_get_cmd_queue(&g_kalman_cmd_handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot get kalman cmd queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    /* Get event group bit handler */
    while (waypoint_get_event_group(&g_waypoint_event_group) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot get waypoint event group handler. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    while (waypoint_get_error_group(&g_waypoint_error_group) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot get waypoint error group handler. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    for (;;)
    {
        state_machine_event_handler();

        if(g_is_running_traj)
        {
            state_machine_receive_waypoint_state();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void state_machine_task_start(void)
{
    ESP_LOGI(TAG, "State machine task started");

    xTaskCreatePinnedToCore(state_machine_task, "state_machine_task", STATE_MACHINE_TASK_STACK_SIZE, NULL, STATE_MACHINE_TASK_PRIORITY, NULL, STATE_MACHINE_TASK_CORE_ID);
}