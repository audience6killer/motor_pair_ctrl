#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"

#include "odometry_unit.h"
#include "odometry_task_common.h"

#include "traction_control.h"
#include "traction_task_common.h"

#define IN_VECINITY(x, setpoint) (abs(setpoint - x) <= 30 ? setpoint : x)

static const char TAG[] = "odometry_task";
ESP_EVENT_DEFINE_BASE(ODOMETRY_EVENT_BASE);

static QueueHandle_t g_odometry_data_queue;
static QueueHandle_t g_traction_data_queue = NULL;
static esp_event_loop_handle_t g_odometry_event_loop_handle;
static odometry_data_t g_odometry_data;
static esp_timer_handle_t g_odometry_timer_handle = NULL;
static TaskHandle_t g_parent_ptr = NULL;

esp_err_t odometry_get_data_queue(QueueHandle_t *queue)
{
    ESP_RETURN_ON_FALSE(g_odometry_data_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue handle is NULL");

    *queue = g_odometry_data_queue;
    return ESP_OK;
}

esp_err_t odometry_get_event_loop(esp_event_loop_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_odometry_event_loop_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Event loop is NULL");

    *handle = g_odometry_event_loop_handle;
    return ESP_OK;
}

esp_err_t initialize_odometry_data(odometry_data_t *data)
{
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_STATE, TAG, "Null data to initialize");

    data->x.cur_value = 0.0f;
    data->y.cur_value = 0.0f;
    data->phi_l.cur_value = 0.0f;
    data->phi_r.cur_value = 0.0f;
    data->theta.cur_value = 0.0f;

    data->x.past_value = 0.0f;
    data->y.past_value = 0.0f;
    data->phi_l.past_value = 0.0f;
    data->phi_r.past_value = 0.0f;
    data->theta.past_value = 0.0f;

    data->x.diff_value = 0.0f;
    data->y.diff_value = 0.0f;
    data->phi_l.diff_value = 0.0f;
    data->phi_r.diff_value = 0.0f;
    data->theta.diff_value = 0.0f;

    return ESP_OK;
}

esp_err_t calculate_diffential(odometry_data_t *data)
{
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_STATE, TAG, "Null data to calculate diffential");

    data->x.diff_value = (data->x.cur_value - data->x.past_value) / 10E-2;
    data->y.diff_value = (data->y.cur_value - data->y.past_value) / 10E-2;
    data->phi_l.diff_value = (data->phi_l.cur_value - data->phi_l.past_value) / 10E-2;
    data->phi_r.diff_value = (data->phi_r.cur_value - data->phi_r.past_value) / 10E-2;
    data->theta.diff_value = (data->theta.cur_value - data->theta.past_value) / 10E-2;

    return ESP_OK;
}

esp_err_t odometry_send_data_to_queue(odometry_data_t *data)
{
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_STATE, TAG, "Null data to send to queue");

    xQueueSend(g_odometry_data_queue, data, pdMS_TO_TICKS(50));

    return ESP_OK;
}

static void odometry_update_pose_loop(void *args)
{
    float phi_diff = g_odometry_data.phi_r.cur_value - g_odometry_data.phi_l.cur_value;

    phi_diff = TRACT_CONV_PULSES2RAD(phi_diff);

    float delta_phi_r = g_odometry_data.phi_r.cur_value - g_odometry_data.phi_r.past_value;
    float delta_phi_l = g_odometry_data.phi_l.cur_value - g_odometry_data.phi_l.past_value;

    float phi_sum = delta_phi_l + delta_phi_r;
    phi_sum = TRACT_CONV_PULSES2RAD(phi_sum);

    // Update N-1 pose
    g_odometry_data.phi_l.past_value = g_odometry_data.phi_l.cur_value;
    g_odometry_data.phi_r.past_value = g_odometry_data.phi_r.cur_value;

    // Update pose parameters
    g_odometry_data.theta.past_value = g_odometry_data.theta.cur_value;
    g_odometry_data.theta.cur_value = WHEEL_RADIUS * phi_diff / (2 * WHEEL_DISTANCE_TO_CM);
    g_odometry_data.x.cur_value += WHEEL_RADIUS * (phi_sum / 2) * cos(g_odometry_data.theta.cur_value);
    g_odometry_data.y.cur_value += WHEEL_RADIUS * (phi_sum / 2) * sin(g_odometry_data.theta.cur_value);

    ESP_ERROR_CHECK(calculate_diffential(&g_odometry_data));

    ESP_ERROR_CHECK(odometry_send_data_to_queue(&g_odometry_data));
}

esp_err_t odometry_calculate_pose(motor_pair_data_t r_data)
{
    int delta_phi_l = r_data.mleft_pulses;
    int delta_phi_r = r_data.mright_pulses;

    // Correct for angle
    delta_phi_l = IN_VECINITY(delta_phi_l, r_data.mleft_set_point);
    delta_phi_r = IN_VECINITY(delta_phi_r, r_data.mright_set_point);

    // Update vehicle's wheel angle. Rolling average
    g_odometry_data.phi_l.cur_value += delta_phi_l;
    g_odometry_data.phi_r.cur_value += delta_phi_r;

    return ESP_OK;
}

esp_err_t odometry_set_current_state(odometry_state_e state)
{
    g_odometry_data.odometry_state = state;
    return ESP_OK;
}

esp_err_t odometry_stop(void)
{
    ESP_RETURN_ON_FALSE(g_odometry_timer_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Timer handle is NULL");

    if (!esp_timer_is_active(g_odometry_timer_handle))
    {
        ESP_LOGW(TAG, "Odometry timer already stopped");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Timer is being stopped!");
    ESP_ERROR_CHECK(esp_timer_stop(g_odometry_timer_handle));

    return ESP_OK;
}

esp_err_t odometry_start(void)
{
    ESP_RETURN_ON_FALSE(g_odometry_timer_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Timer handle is NULL");

    if (esp_timer_is_active(g_odometry_timer_handle))
    {
        ESP_LOGW(TAG, "Odometry timer already started");
        return ESP_OK;
    }

    ESP_ERROR_CHECK(esp_timer_start_periodic(g_odometry_timer_handle, ODOMETRY_LOOP_MS * 100));
    odometry_set_current_state(ODO_RUNNING);
    odometry_send_data_to_queue(&g_odometry_data);
    ESP_LOGI(TAG, "Odometry is running");

    return ESP_OK;
}

/* Event handlers */
static void odometry_start_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    ESP_LOGI(TAG, "Starting odometry");

    esp_err_t ret = odometry_start();

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error starting odometry. Code: %s", esp_err_to_name(ret));
        return;
    }
}

static void odometry_stop_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    ESP_LOGI(TAG, "Stopping odometry");

    esp_err_t ret = odometry_stop();

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error stopping odometry. Code: %s", esp_err_to_name(ret));
        return;
    }
}

void odometry_receive_from_traction(void)
{
    static motor_pair_data_t r_data;

    if (g_odometry_data.odometry_state == ODO_RUNNING)
    {
        if (xQueueReceive(g_traction_data_queue, &r_data, pdMS_TO_TICKS(30)) == pdTRUE)
        {
            ESP_ERROR_CHECK(odometry_calculate_pose(r_data));
        }
    }
}

static void odometry_unit_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing odometry task");

    /* Setting up data queue */
    g_odometry_data_queue = xQueueCreate(4, sizeof(odometry_data_t));

    /* Wait for traction to end initializing*/
    xEventGroupWaitBits(tract_ctrl_event_group_handle, TRACT_EVENT_BIT_READY, pdFALSE, pdTRUE, portMAX_DELAY);

    /* Setting up timer for odometry */
    esp_timer_create_args_t odometry_timer_args = {
        .callback = odometry_update_pose_loop,
        .arg = NULL,
        .name = "odometry_timer_loop",
    };
    ESP_ERROR_CHECK(esp_timer_create(&odometry_timer_args, &g_odometry_timer_handle));

    /* Initialize global data */
    initialize_odometry_data(&g_odometry_data);

    /* Setting up event loop */
    esp_event_loop_args_t event_loop_args = {
        .queue_size = 10,
        .task_core_id = 0,
        .task_name = "odo_event_loop",
        .task_priority = 5,
        .task_stack_size = 4098,
    };
    ESP_ERROR_CHECK(esp_event_loop_create(&event_loop_args, &g_odometry_event_loop_handle));

    /* Register events */
    ESP_ERROR_CHECK(esp_event_handler_register_with(g_odometry_event_loop_handle, ODOMETRY_EVENT_BASE, ODOMETRY_START_EVENT, odometry_start_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(g_odometry_event_loop_handle, ODOMETRY_EVENT_BASE, ODOMETRY_STOP_EVENT, odometry_stop_event_handler, NULL));

    /* Setting up traction queue and variable to save msg */
    while (tract_ctrl_get_data_queue(&g_traction_data_queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error getting traction control queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    /* Notify parent task end of initialization */
    if (g_parent_ptr != NULL)
        xTaskNotifyGive(g_parent_ptr);

    odometry_set_current_state(ODO_READY);
    odometry_send_data_to_queue(&g_odometry_data);

    for (;;)
    {
        odometry_receive_from_traction();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void odometry_start_task(TaskHandle_t parent)
{
    ESP_LOGI(TAG, "Iniatilazing task");

    if (parent != NULL)
        g_parent_ptr = parent;

    xTaskCreatePinnedToCore(&odometry_unit_task, "odometry_task", ODOMETRY_TASK_STACK_SIZE, NULL, ODOMETRY_TASK_PRIORITY, NULL, ODOMETRY_TASK_CORE_ID);
}
