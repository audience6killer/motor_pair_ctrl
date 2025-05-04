#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"

#include "odometry_unit.h"
#include "odometry_task_common.h"

#include "traction_control.h"
#include "traction_task_common.h"

#define IN_VECINITY(x, setpoint) (abs(setpoint - x) <= 30 ? setpoint : x)

static const char TAG[] = "odometry_task";

static QueueHandle_t g_odometry_data_queue = NULL;
static QueueHandle_t g_odometry_cmd_queue = NULL;
static QueueHandle_t g_traction_data_queue = NULL;
static odometry_data_t g_vehicle_pose;

esp_err_t odometry_get_cmd_queue(QueueHandle_t *queue)
{
    ESP_RETURN_ON_FALSE(g_odometry_cmd_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue handle is NULL");
    *queue = g_odometry_cmd_queue;

    return ESP_OK;
}

esp_err_t odometry_get_data_queue(QueueHandle_t *queue)
{
    ESP_RETURN_ON_FALSE(g_odometry_data_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue handle is NULL");

    *queue = g_odometry_data_queue;
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

esp_err_t odometry_calculate_differential(odometry_data_t *data, uint32_t delta_t)
{
    //ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_STATE, TAG, "Null data to calculate diffential");

    data->x.diff_value = (data->x.cur_value - data->x.past_value) / (float)delta_t;
    data->y.diff_value = (data->y.cur_value - data->y.past_value) / (float)delta_t;
    data->phi_l.diff_value = (data->phi_l.cur_value - data->phi_l.past_value) / (float)delta_t;
    data->phi_r.diff_value = (data->phi_r.cur_value - data->phi_r.past_value) / (float)delta_t;
    data->theta.diff_value = (data->theta.cur_value - data->theta.past_value) / (float)delta_t;

    return ESP_OK;
}

esp_err_t odometry_send_to_data_queue(odometry_data_t *data)
{
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_STATE, TAG, "Null data to send to queue");

    if (xQueueSend(g_odometry_data_queue, data, pdMS_TO_TICKS(20)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending queue");
    }

    return ESP_OK;
}

/* ISR routine */
static void odometry_update_pose_loop(void *args)
{
    static int64_t prev_time = 0;
    int64_t c_time = esp_timer_get_time();
    int64_t delta_t = c_time - prev_time;
    prev_time = c_time;

    ESP_LOGI(TAG, "Time elapsed since last execution: %lld microseconds", delta_t);

    // ESP_LOGI(TAG, "IN LOOPPPPPP!");
    float phi_diff = g_vehicle_pose.phi_r.cur_value - g_vehicle_pose.phi_l.cur_value;

    phi_diff = TRACT_CONV_PULSES2RAD(phi_diff);

    float delta_phi_r = g_vehicle_pose.phi_r.cur_value - g_vehicle_pose.phi_r.past_value;
    float delta_phi_l = g_vehicle_pose.phi_l.cur_value - g_vehicle_pose.phi_l.past_value;

    float phi_sum = delta_phi_l + delta_phi_r;
    phi_sum = TRACT_CONV_PULSES2RAD(phi_sum);

    // Update N-1 pose
    g_vehicle_pose.phi_l.past_value = g_vehicle_pose.phi_l.cur_value;
    g_vehicle_pose.phi_r.past_value = g_vehicle_pose.phi_r.cur_value;

    // Update pose parameters
    g_vehicle_pose.theta.past_value = g_vehicle_pose.theta.cur_value;
    g_vehicle_pose.theta.cur_value = WHEEL_RADIUS * phi_diff / (2 * WHEEL_DISTANCE_TO_CM);
    g_vehicle_pose.x.cur_value += WHEEL_RADIUS * (phi_sum / 2) * cos(g_vehicle_pose.theta.cur_value);
    g_vehicle_pose.y.cur_value += WHEEL_RADIUS * (phi_sum / 2) * sin(g_vehicle_pose.theta.cur_value);

    //odometry_calculate_differential(&g_vehicle_pose);

    //odometry_send_to_data_queue(&g_vehicle_pose);
    odometry_data_t data = g_vehicle_pose;
    xQueueSendFromISR(g_odometry_data_queue, &data, NULL);
}

esp_err_t odometry_update_pose(motor_pair_data_t r_data)
{
    /* Get time differential between executions */
    static int64_t prev_time = 0;
    int64_t c_time = esp_timer_get_time();
    int64_t delta_t = c_time - prev_time;
    prev_time = c_time;

    // ESP_LOGI(TAG, "Time elapsed since last execution: %lld microseconds", delta_t);

    int delta_phil = r_data.mleft_pulses;
    int delta_phir = r_data.mright_pulses;

    // Correct for angle
    delta_phil = IN_VECINITY(delta_phil, r_data.mleft_set_point);
    delta_phir = IN_VECINITY(delta_phir, r_data.mright_set_point);

    // Update vehicle's wheel angle. Rolling average
    g_vehicle_pose.phi_l.cur_value += delta_phil;
    g_vehicle_pose.phi_r.cur_value += delta_phir;

    // ESP_LOGI(TAG, "IN LOOPPPPPP!");
    float phi_diff = g_vehicle_pose.phi_r.cur_value - g_vehicle_pose.phi_l.cur_value;

    phi_diff = TRACT_CONV_PULSES2RAD(phi_diff);

    float delta_phi_r = g_vehicle_pose.phi_r.cur_value - g_vehicle_pose.phi_r.past_value;
    float delta_phi_l = g_vehicle_pose.phi_l.cur_value - g_vehicle_pose.phi_l.past_value;

    float phi_sum = delta_phi_l + delta_phi_r;
    phi_sum = TRACT_CONV_PULSES2RAD(phi_sum);

    // Update N-1 pose
    g_vehicle_pose.phi_l.past_value = g_vehicle_pose.phi_l.cur_value;
    g_vehicle_pose.phi_r.past_value = g_vehicle_pose.phi_r.cur_value;

    // Update pose parameters
    g_vehicle_pose.theta.past_value = g_vehicle_pose.theta.cur_value;
    g_vehicle_pose.theta.cur_value = WHEEL_RADIUS * phi_diff / (2 * WHEEL_DISTANCE_TO_CM);
    g_vehicle_pose.x.cur_value += WHEEL_RADIUS * (phi_sum / 2) * cos(g_vehicle_pose.theta.cur_value);
    g_vehicle_pose.y.cur_value += WHEEL_RADIUS * (phi_sum / 2) * sin(g_vehicle_pose.theta.cur_value);

    float delta_t_seconds = delta_t / 1000000.0f;
    odometry_calculate_differential(&g_vehicle_pose, delta_t_seconds);

    odometry_data_t data = g_vehicle_pose;
    odometry_send_to_data_queue(&data);

    return ESP_OK;
}

esp_err_t odometry_set_current_state(odometry_state_e state)
{
    g_vehicle_pose.odometry_state = state;
    return ESP_OK;
}

void odometry_get_traction_data(void)
{
    static motor_pair_data_t traction_data;
    // initialize_odometry_data(&g_vehicle_pose);

    if (xQueueReceive(g_traction_data_queue, &traction_data, pdMS_TO_TICKS(10)))
    {
        odometry_update_pose(traction_data);

        /*ESP_LOGI(TAG, "Traction Data: mleft_pulses=%d, mright_pulses=%d, mleft_set_point=%d, mright_set_point=%d\n",
                 traction_data.mleft_pulses, traction_data.mright_pulses, traction_data.mleft_set_point, traction_data.mright_set_point);*/
    }
}

/* Event handlers */
esp_err_t odometry_stop_event_handler(void)
{
    ESP_LOGI(TAG, "Timer is being stopped!");
    odometry_set_current_state(ODO_STOPPED);


    return ESP_OK;
}

esp_err_t odometry_start_event_handler(void)
{

    ESP_LOGI(TAG, "Timer is being started!");
    odometry_set_current_state(ODO_RUNNING);

    return ESP_OK;
}

/* Main task */
void odometry_event_handler(void)
{
    static odometry_cmd_e cmd;

    if (xQueueReceive(g_odometry_cmd_queue, &cmd, pdMS_TO_TICKS(100)) == pdPASS)
    {
        switch (cmd)
        {
        case ODO_CMD_START:
            ESP_LOGI(TAG, "Event: Start process");
            if (odometry_start_event_handler() != ESP_OK)
                ESP_LOGE(TAG, "Error starting odometry loop");
            break;
        case ODO_CMD_STOP:
            ESP_LOGI(TAG, "Event: Stop process");
            if (odometry_stop_event_handler() != ESP_OK)
                ESP_LOGE(TAG, "Error stopping odometry loop");
            break;
        default:
            break;
        }
    }
}

/* Odometry task */
static void odometry_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing odometry task");

    // Setting up traction queue and variable to save msg
    while (tract_ctrl_get_data_queue(&g_traction_data_queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Error getting traction control queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Setting up queues
    g_odometry_cmd_queue = xQueueCreate(4, sizeof(odometry_cmd_e));
    g_odometry_data_queue = xQueueCreate(4, sizeof(odometry_data_t));

    odometry_set_current_state(ODO_STOPPED);

    for (;;)
    {
        if(g_vehicle_pose.odometry_state == ODO_RUNNING)
        {
            odometry_get_traction_data();
        }
        odometry_event_handler();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void odometry_start_task(void)
{
    ESP_LOGI(TAG, "Iniatilazing task");

    xTaskCreatePinnedToCore(&odometry_task, "odometry_task", ODOMETRY_TASK_STACK_SIZE, NULL, ODOMETRY_TASK_PRIORITY, NULL, ODOMETRY_TASK_CORE_ID);
}
