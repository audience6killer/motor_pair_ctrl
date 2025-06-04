
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_check.h"

#include "data_center.h"
#include "esp32_uart_unit.h"
// #include "fake_data_center.h"
//  #include "lora_rf_unit.h"
#include "waypoint_controller.h"

#include "state_machine.h"
#include "state_machine_task_common.h"
}

#include <string.h>

static const char TAG[] = "state_machine";
static QueueHandle_t g_waypoint_cmd_queue = NULL;
static QueueHandle_t g_waypoint_status_queue = NULL;
static QueueHandle_t g_kalman_cmd_handle = NULL;
static QueueHandle_t g_data_center_data_queue = NULL;
static QueueHandle_t g_esp32_uart_cmd_queue = NULL;
static QueueHandle_t g_esp32_uart_event_queue = NULL;
static bool g_is_running_traj = false;
static state_machine_state_e g_state_machine_state = SM_STATE_IDLE;
static char g_error_string[100];

static EventGroupHandle_t g_waypoint_event_group = NULL;
static EventGroupHandle_t g_waypoint_error_group = NULL;

const char *state_machine_get_state_string(void)
{
    return state_machine_state_to_string(g_state_machine_state);
}

esp_err_t state_machine_receive_sower_event(sower_event_t *event, uint32_t time_to_wait)
{
    if (xQueueReceive(g_esp32_uart_event_queue, &event, pdMS_TO_TICKS(time_to_wait)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error: Failed to receive sower event");
        return ESP_FAIL;
    }

    return ESP_OK;
}

bool state_machine_wait_for_sower_event(sower_events_e evt_to_wait, uint32_t time_to_wait)
{
    sower_event_t event;
    if (state_machine_receive_sower_event(&event, time_to_wait) == ESP_OK)
    {
        return event.event == evt_to_wait;
    }

    return false;
}

const char *state_machine_get_error_string(void)
{
    // Return the error string and clear it after reading
    static char temp[sizeof(g_error_string)];
    strncpy(temp, g_error_string, sizeof(g_error_string));
    temp[sizeof(g_error_string) - 1] = '\0'; // Ensure null-termination
    memset(g_error_string, 0, sizeof(g_error_string));
    return temp;
}

esp_err_t state_machine_set_error(char *error_msg)
{
    g_state_machine_state = SM_STATE_ERROR;
    strncpy(g_error_string, error_msg, sizeof(*error_msg));

    return ESP_OK;
}

/* Event handlers */
esp_err_t state_machine_start_event_handler(void)
{
    /* Start kalman process */
    kalman_cmd_e cmd_k = KALMAN_CMD_START;

    if (xQueueSend(g_kalman_cmd_handle, &cmd_k, pdMS_TO_TICKS(100)) != pdPASS)
    {
        char msg[] = "Error: Cannot send start command to kalman task";
        ESP_LOGE(TAG, "%s", msg);
        state_machine_set_error(msg);
        return ESP_FAIL;
    }

    /* Start cutter disk */
    ESP_LOGI(TAG, "Starting cutter disk");
    sower_cmd_t cmd_cutter = {
        .code = SOWER_CMD_START_CUTTER,
        .arg = 0.0f,
    };
    if (xQueueSend(g_esp32_uart_cmd_queue, &cmd_cutter, pdMS_TO_TICKS(100) != pdPASS))
    {
        char msg[] = "Error: Cannot send start cutter command";
        ESP_LOGE(TAG, "%s", msg);

        state_machine_set_error(msg);
        return ESP_FAIL;
    }

    if (!state_machine_wait_for_sower_event(SOWER_EVENT_CUTTER_STARTED, 1000))
    {
        ESP_LOGE(TAG, "Error: Cannot start cutter disk");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Cutter disk started successfully");

    sower_cmd_t cmd_linear_motor = {
        .code = SOWER_CMD_LINEAR_MOTOR_DOWN,
        .arg = 0.0f,
    };
    if (xQueueSend(g_esp32_uart_cmd_queue, &cmd_linear_motor, pdMS_TO_TICKS(100) != pdPASS))
    {
        char msg[] = "Error: Cannot send descend linear motor";
        ESP_LOGE(TAG, "%s", msg);

        state_machine_set_error(msg);
        return ESP_FAIL;
    }

    if (!state_machine_wait_for_sower_event(SOWER_EVENT_CUTTER_DOWN, 200))
    {
        ESP_LOGE(TAG, "Error: Cannot descend cutter disk");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Cutter disk descended correctly");

    /* Start waypoint trajectory */
    waypoint_cmd_t cmd_start = {
        .cmd = WP_CMD_START_TRAJ,
        .point = NULL,
    };
    if (xQueueSend(g_waypoint_cmd_queue, &cmd_start, pdMS_TO_TICKS(100)) != pdPASS)
    {
        char msg[] = "Error: Cannot send start trajectory command to waypoint task";
        ESP_LOGE(TAG, "%s", msg);

        state_machine_set_error(msg);
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
            char msg[] = "Error: the navigation points are empty";
            ESP_LOGE(TAG, "%s", msg);
            state_machine_set_error(msg);
        }
        else if ((error_flag & WP_ERROR_CANNOT_START_TRACT) != 0)
        {
            char msg[] = "Error: Cannot start tract";
            ESP_LOGE(TAG, "%s", msg);
            state_machine_set_error(msg);
        }
        else if ((error_flag & WP_ERROR_CANNOT_SEND_FPOINT) != 0)
        {
            char msg[] = "Error: Cannot send first point tp diff drive";
            ESP_LOGE(TAG, "%s", msg);
            state_machine_set_error(msg);
        }
        else
        {
            char msg[] = "Error: Cannot get error bits";
            ESP_LOGE(TAG, "%s", msg);
            state_machine_set_error(msg);
        }
    }
    else
    {
        /* error handling */
        char msg[] = "Error: Waypoint statuts bits were not received";
        ESP_LOGE(TAG, "%s", msg);
        state_machine_set_error(msg);
    }

    g_state_machine_state = SM_STATE_STARTED;
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

    /* Stop and lift the cutter */
    sower_cmd_t cmd_linear_motor = {
        .code = SOWER_CMD_LINEAR_MOTOR_UP,
        .arg = 0.0f,
    };
    if (xQueueSend(g_esp32_uart_cmd_queue, &cmd_linear_motor, pdMS_TO_TICKS(100) != pdPASS))
    {
        char msg[] = "Error: Cannot send lift linear motor";
        ESP_LOGE(TAG, "%s", msg);

        state_machine_set_error(msg);
        return ESP_FAIL;
    }

    if (!state_machine_wait_for_sower_event(SOWER_EVENT_CUTTER_UP, 200))
    {
        ESP_LOGE(TAG, "Error: Cannot lift cutter disk");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Cutter disk rised correctly");


    sower_cmd_t cmd_cutter = {
        .code = SOWER_CMD_STOP_CUTTER,
        .arg = 0.0f,
    };
    if (xQueueSend(g_esp32_uart_cmd_queue, &cmd_cutter, pdMS_TO_TICKS(100) != pdPASS))
    {
        char msg[] = "Error: Cannot send stop cutter command";
        ESP_LOGE(TAG, "%s", msg);

        state_machine_set_error(msg);
        return ESP_FAIL;
    }
    if (!state_machine_wait_for_sower_event(SOWER_EVENT_CUTTER_STOPPED, 500))
    {
        ESP_LOGE(TAG, "Error: Cannot stop cutter disk");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Cutter disk stopped correctly");

    g_state_machine_state = SM_STATE_STOPPED;
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

    g_state_machine_state = SM_STATE_WAYPOINT_ADDED;
    return ESP_OK;
}

esp_err_t state_machine_echo_event_handler(void)
{
    return ESP_OK;
}

esp_err_t state_machine_echo_esp32_event_handler()
{
    sower_cmd_t cmd_cutter = {
        .code = SOWER_CMD_ECHO_SOWER,
        .arg = 0.0f,
    };

    if (xQueueSend(g_esp32_uart_cmd_queue, &cmd_cutter, pdMS_TO_TICKS(100) != pdPASS))
    {
        char msg[] = "Error: Cannot send echo sower command";
        ESP_LOGE(TAG, "%s", msg);

        state_machine_set_error(msg);
        return ESP_FAIL;
    }
    if (!state_machine_wait_for_sower_event(SOWER_EVENT_ECHO_MSG, 2000))
    {
        ESP_LOGE(TAG, "Error: Cannot comm with sower module");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sower module says HELLOWWW!");
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
            ESP_ERROR_CHECK_WITHOUT_ABORT(state_machine_start_event_handler());
            break;
        case SM_CMD_ADD_WAYPOINT:
            ESP_LOGI(TAG, "CMD: Navigation point added");
            ESP_ERROR_CHECK(state_machine_nav_point_event_handler(data_center_msg.args[0], data_center_msg.args[1], data_center_msg.args[2]));
            break;
        case SM_CMD_ECHO:
            ESP_LOGI(TAG, "CMD: Echo");
            ESP_ERROR_CHECK(state_machine_echo_event_handler());
            break;
        case SM_CMD_ECHO_ESP32:
            ESP_LOGI(TAG, "CMD: Echo ESP32");
            state_machine_echo_esp32_event_handler();
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
    if (xQueueReceive(g_waypoint_status_queue, &wp_state, pdMS_TO_TICKS(10)) == pdPASS)
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
    while (data_center_get_data_queue(&g_data_center_data_queue) != ESP_OK)
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
    while (esp32_uart_get_transmit_data_queue(&g_esp32_uart_cmd_queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot get esp32_uart_transmit queue. Retrying...");
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

    /* Get sower state queue */
    while (esp32_uart_get_received_data_queue(&g_esp32_uart_event_queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error: Cannot get sower_event_queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    /* Initialize error string */
    memset(g_error_string, 0, sizeof(g_error_string));

    for (;;)
    {
        state_machine_event_handler();

        if (g_is_running_traj)
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