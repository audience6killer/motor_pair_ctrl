#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

#include "kalman_filter.h"
#include "kalman_filter_task_common.h"

#include "odometry_unit.h"

ESP_EVENT_DEFINE_BASE(KALMAN_EVENT_BASE);

static const char TAG[] = "kalman_task";
static QueueHandle_t g_kalman_data_queue = NULL;
static esp_event_loop_handle_t g_kalman_event_loop_handle = NULL;
static QueueHandle_t g_odometry_queue_handle = NULL;
static kalman_info_t g_current_state;
static bool g_is_running = false;

esp_err_t kalman_initialize_info(kalman_info_t *data)
{
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_STATE, TAG, "Null data to initialize");

    data->x = 0.0f;
    data->y = 0.0f;
    data->z = 0.0f;
    data->theta = 0.0f;
    data->x_p = 0.0f;
    data->y_p = 0.0f;
    data->z_p = 0.0f;
    data->theta_p = 0.0f;

    return ESP_OK;
}

esp_err_t kalman_get_data_queue(QueueHandle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_kalman_data_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Retriving queue handle uniniatilized");

    *handle = g_kalman_data_queue;
    return ESP_OK;
}

esp_err_t kalman_send2queue(kalman_info_t *data)
{
    ESP_RETURN_ON_FALSE(g_kalman_data_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue handle is null");

    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_STATE, TAG, "Null data to send to queue");

    if (xQueueSend(g_kalman_data_queue, data, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending queue");
    }

    return ESP_OK;
}

void kalman_set_state(kalman_state_e state)
{
    g_current_state.state = state;
}

static void kalman_stop_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    ESP_LOGI(TAG, "Stopping");

    if (g_is_running == false)
    {
        ESP_LOGW(TAG, "Kalman filter is already stopped");
        return;
    }

    g_is_running = false;

    return;
}

static void kalman_start_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    ESP_LOGI(TAG, "Starting up");

    if (g_is_running == true)
    {
        ESP_LOGW(TAG, "Kalman filter is already running");
        return;
    }

    g_is_running = true;

    return;
}

void kalman_receive_odometry_data(void)
{
    static odometry_data_t odo_robot_pose;


    if (xQueueReceive(g_odometry_queue_handle, &odo_robot_pose, portMAX_DELAY) == pdPASS)
    {
        g_current_state = (kalman_info_t){
            .x = odo_robot_pose.x.cur_value,
            .y = odo_robot_pose.y.cur_value,
            .z = 0.0f,
            .theta = odo_robot_pose.theta.cur_value,
            .x_p = odo_robot_pose.x.diff_value,
            .y_p = odo_robot_pose.y.diff_value,
            .z_p = 0.0f,
            .theta_p = odo_robot_pose.theta.diff_value,
        };

        ESP_ERROR_CHECK(kalman_send2queue(&g_current_state));
    }
    else
    {
        ESP_LOGE(TAG, "Error receiving queue");
    }
}

static void kalman_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing kalman filter task");
    odometry_get_data_queue(&g_odometry_queue_handle);

    g_kalman_data_queue = xQueueCreate(4, sizeof(kalman_info_t));


    g_current_state = (kalman_info_t){
        .x = 0.0f,
        .y = 0.0f,
        .z = 0.0f,
        .theta = 0.0f,
        .x_p = 0.0f,
        .y_p = 0.0f,
        .z_p = 0.0f,
        .theta_p = 0.0f,
    };

    /* Set up event loop */
    esp_event_loop_args_t event_loop_args = {
        .task_name = "kalman_event_lp",
        .queue_size = 10,
        .task_core_id = 0,
        .task_priority = 5,
        .task_stack_size = 4098,
    };
    ESP_ERROR_CHECK(esp_event_loop_create(&event_loop_args, &g_kalman_event_loop_handle));

    /* Register events */
    ESP_ERROR_CHECK(esp_event_handler_register_with(g_kalman_event_loop_handle, KALMAN_EVENT_BASE, KALMAN_STOP_EVENT, kalman_stop_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(g_kalman_event_loop_handle, KALMAN_EVENT_BASE, KALMAN_START_EVENT, kalman_start_event_handler, NULL));

    kalman_set_state(KALMAN_READY);
    kalman_send2queue(&g_current_state);

    for (;;)
    {
        if (g_is_running)
        {
            kalman_receive_odometry_data();
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void kalman_start_task(void)
{
    ESP_LOGI(TAG, "Initializing kalman filter task");

    xTaskCreatePinnedToCore(&kalman_task, "kalman", KALMAN_FILTER_STACK_SIZE, NULL, KALMAN_FILTER_TASK_PRIORITY, NULL, KALMAN_FILTER_CORE_ID);
}
