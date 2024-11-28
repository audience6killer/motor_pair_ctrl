#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "odometry_unit.h"
#include "odometry_task_common.h"

#include "traction_control.h"
#include "traction_task_common.h"

static const char TAG[] = "odometry_task";

static QueueHandle_t odometry_queue_handle;
static odometry_robot_pose_t g_vehicle_pose;
static odometry_robot_pose_t g_vehicle_pose_past;

/**
 * @brief Re-maps a number from one range to another. That is, a value of fromLow would get mapped to toLow, a value of fromHigh to toHigh, values in-between to values in-between, etc.
 *
 * @param x
 * @param in_min
 * @param in_max
 * @param out_min
 * @param out_max
 * @return int
 */
int map(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

esp_err_t odometry_calculate_pose(motor_pair_data_t r_data)
{
    int delta_phi_l = r_data.mleft_real_pulses;
    int delta_phi_r = r_data.mright_real_pulses;

    // Average filter
    float phi_l_avg = (g_vehicle_pose_past.phi_l + delta_phi_l) / 2.00f;
    float phi_r_avg = (g_vehicle_pose_past.phi_r + delta_phi_r) / 2.00f;

    // Update previous phi value
    g_vehicle_pose_past.phi_l = delta_phi_l;
    g_vehicle_pose_past.phi_r = delta_phi_r;

    // Update vehicle's wheel angle
    g_vehicle_pose.phi_l += phi_l_avg;
    g_vehicle_pose.phi_r += phi_r_avg;

    float phi_l_in_r = map((int)g_vehicle_pose.phi_l,
                           -TRACTION_ML_MAX_PULSES, TRACTION_ML_MAX_PULSES,
                           -TRACTION_MR_MAX_PULSES, TRACTION_MR_MAX_PULSES);

    float delta_phi_total = TRACTION_MR_PULSES2RAD(TRACTION_MR_PULSES2RAD(g_vehicle_pose.phi_r - phi_l_in_r));
    g_vehicle_pose.theta = WHEEL_RADIUS * delta_phi_total / (2 * WHEEL_DISTANCE_TO_CM);

    float phi_l_avg_in_r = map((int)phi_l_avg,
                               -TRACTION_ML_MAX_PULSES, TRACTION_ML_MAX_PULSES,
                               -TRACTION_MR_MAX_PULSES, TRACTION_MR_MAX_PULSES);

    float phi_sum = TRACTION_MR_PULSES2RAD(phi_r_avg + phi_l_avg_in_r);
    g_vehicle_pose.x = g_vehicle_pose.x + WHEEL_RADIUS * (phi_sum / 2) * cos(g_vehicle_pose.theta);
    g_vehicle_pose.y = g_vehicle_pose.y + WHEEL_RADIUS * (phi_sum / 2) * sin(g_vehicle_pose.theta);

    return ESP_OK;
}

static void odometry_unit_task(void *pvParameters)
{
    QueueHandle_t traction_control_queue = traction_control_get_queue_handle();

    motor_pair_data_t traction_data;

    g_vehicle_pose = (odometry_robot_pose_t){
        .x = 0.0f,
        .y = 0.0f,
        .theta = 0.0f,
        .phi_l = 0.0f,
        .phi_r = 0.0f,
    };

    g_vehicle_pose_past = (odometry_robot_pose_t){
        .x = 0.0f,
        .y = 0.0f,
        .theta = 0.0f,
        .phi_l = 0.0f,
        .phi_r = 0.0f,
    };

    odometry_queue_handle = xQueueCreate(4, sizeof(odometry_robot_pose_t));

    for (;;)
    {
        if (xQueueReceive(traction_control_queue, &traction_data, portMAX_DELAY) == pdPASS)
        {
            ESP_ERROR_CHECK(odometry_calculate_pose(traction_data));

            // if (xQueueSend(odometry_queue_handle, &g_vehicle_pose, portMAX_DELAY) != pdPASS)
            // {
            //     ESP_LOGE(TAG, "Error sending queue");
            // }
#if true
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
