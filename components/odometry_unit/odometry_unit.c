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

#define IN_VECINITY(x, setpoint) (abs(setpoint - x) <= 2 ? setpoint : x)

static const char TAG[] = "odometry_task";

static QueueHandle_t odometry_queue_handle;
static odometry_data_t g_vehicle_pose;
static odometry_data_t g_vehicle_pose_past;
static uint32_t g_reading_count = 0;

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

esp_err_t odometry_send_msg2queue(odometry_data_t *data)
{
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_STATE, TAG, "Null data to send to queue");

    if (xQueueSend(odometry_queue_handle, data, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending queue");
    }

    return ESP_OK;
}

static void odometry_update_pose_loop(void *args)
{
    float phi_diff = g_vehicle_pose.phi_r - g_vehicle_pose.phi_l;

    phi_diff = TRACTION_MR_PULSES2RAD(phi_diff);

    float delta_phi_r = g_vehicle_pose.phi_r - g_vehicle_pose_past.phi_r;
    float delta_phi_l = g_vehicle_pose.phi_l - g_vehicle_pose_past.phi_l;

    float phi_sum = delta_phi_l + delta_phi_r;
    phi_sum = TRACTION_MR_PULSES2RAD(phi_sum);

    // Update N-1 pose
    g_vehicle_pose_past.phi_l = g_vehicle_pose.phi_l;
    g_vehicle_pose_past.phi_r = g_vehicle_pose.phi_r;

    // Update pose parameters
    g_vehicle_pose.theta = WHEEL_RADIUS * phi_diff / (2 * WHEEL_DISTANCE_TO_CM);
    g_vehicle_pose.x = g_vehicle_pose.x + WHEEL_RADIUS * (phi_sum / 2) * cos(g_vehicle_pose.theta);
    g_vehicle_pose.y = g_vehicle_pose.y + WHEEL_RADIUS * (phi_sum / 2) * sin(g_vehicle_pose.theta);

    ESP_ERROR_CHECK(odometry_send_msg2queue(&g_vehicle_pose));

}

esp_err_t odometry_calculate_pose(motor_pair_data_t r_data)
{
    int delta_phi_l = r_data.mleft_real_pulses;
    int delta_phi_r = r_data.mright_real_pulses;

    // Correct for angle
    delta_phi_l = IN_VECINITY(delta_phi_l, r_data.mleft_set_point);
    delta_phi_r = IN_VECINITY(delta_phi_r, r_data.mright_set_point);

    int phi_l_in_r = map(delta_phi_l,
                         -TRACTION_ML_MAX_PULSES, TRACTION_ML_MAX_PULSES,
                         -TRACTION_MR_MAX_PULSES, TRACTION_MR_MAX_PULSES);

    // Update vehicle's wheel angle. Rolling average
    g_vehicle_pose.phi_l += phi_l_in_r;
    g_vehicle_pose.phi_r += delta_phi_r;

    return ESP_OK;
}

static void odometry_unit_task(void *pvParameters)
{
    QueueHandle_t traction_control_queue = traction_control_get_queue_handle();

    motor_pair_data_t traction_data;

    g_vehicle_pose = (odometry_data_t){
        .x = 0.0f,
        .y = 0.0f,
        .theta = 0.0f,
        .phi_l = 0.0f,
        .phi_r = 0.0f,
        .odometry_state = ODO_STOPPED
    };

    g_vehicle_pose_past = (odometry_data_t){
        .x = 0.0f,
        .y = 0.0f,
        .theta = 0.0f,
        .phi_l = 0.0f,
        .phi_r = 0.0f,
    };

    esp_timer_create_args_t odometry_timer_args = {
        .callback = odometry_update_pose_loop,
        .arg = NULL,
        .name = "odometry_timer_loop",
    };

    esp_timer_handle_t odometry_timer_handle = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&odometry_timer_args, &odometry_timer_handle));

    ESP_ERROR_CHECK(esp_timer_start_periodic(odometry_timer_handle, 10E3));

    odometry_queue_handle = xQueueCreate(4, sizeof(odometry_data_t));

    g_vehicle_pose.odometry_state = ODO_READING;

    for (;;)
    {
        if (xQueueReceive(traction_control_queue, &traction_data, portMAX_DELAY) == pdPASS)
        {
            ESP_ERROR_CHECK(odometry_calculate_pose(traction_data));

#if false 
            printf("/*left_setpoint,%d,speed_left,%d,right_setpoint,%d,speed_right,%d,state,%d,x,%.3f,y,%.3f,theta,%.3f*/\r\n", traction_data.mleft_set_point, traction_data.mleft_real_pulses, traction_data.mright_set_point, traction_data.mright_real_pulses, traction_data.state, g_vehicle_pose.x, g_vehicle_pose.y, g_vehicle_pose.theta);
#endif
        }
        else
        {
            ESP_LOGE(TAG, "Error receiving the queue");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

QueueHandle_t odometry_unit_get_queue_handle(void)
{
    return odometry_queue_handle;
}

void odometry_unit_start_task(void)
{
    ESP_LOGI(TAG, "Iniatilazing task");

    xTaskCreatePinnedToCore(&odometry_unit_task, "odometry_task", ODOMETRY_TASK_STACK_SIZE, NULL, ODOMETRY_TASK_PRIORITY, NULL, ODOMETRY_TASK_CORE_ID);
}
