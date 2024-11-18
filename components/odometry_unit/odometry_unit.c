#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "odometry_unit.h"
#include "odometry_task_common.h"

#include "traction_control.h"

#ifndef SERIAL_DEBUG_ENABLE
#define SERIAL_DEBUG_ENABLE true
#endif

static const char TAG[] = "odometry_task";

// static QueueHandle_t odometry_queue_handle;
static odometry_robot_pose_t g_vehicle_pose;

esp_err_t odometry_calculate_pose(motor_pair_data_t r_data)
{
    float delta_phi_l = r_data.motor_left_angle_measured;
    float delta_phi_r = r_data.motor_right_angle_measured;

    g_vehicle_pose.phi_l += delta_phi_l;
    g_vehicle_pose.phi_r += delta_phi_r;

    g_vehicle_pose.theta = WHEEL_RADIUS * (g_vehicle_pose.phi_r - g_vehicle_pose.phi_l) / (2 * WHEEL_DISTANCE_TO_CM);

    g_vehicle_pose.x = g_vehicle_pose.x + WHEEL_RADIUS * ((delta_phi_l + delta_phi_r) / 2) * cos(g_vehicle_pose.theta);
    g_vehicle_pose.y = g_vehicle_pose.y + WHEEL_RADIUS * ((delta_phi_l + delta_phi_r) / 2) * sin(g_vehicle_pose.theta);

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

    for(;;)
    {
        if(xQueueReceive(traction_control_queue, &traction_data, portMAX_DELAY) == pdPASS)
        {
            ESP_ERROR_CHECK(odometry_calculate_pose(traction_data));
            #if SERIAL_DEBUG_ENABLE
            printf("/*x,%f,y,%f,theta,%f*/\r\n", g_vehicle_pose.x, g_vehicle_pose.y, g_vehicle_pose.theta);
            #endif 
        }
        else
        {
            ESP_LOGE(TAG, "Error receiving the queue");
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }

}

void odometry_unit_start_task(void)
{
    ESP_LOGI(TAG, "Iniatilazing task");

    xTaskCreatePinnedToCore(&odometry_unit_task, "odometry_task", ODOMETRY_TASK_STACK_SIZE, NULL, ODOMETRY_TASK_PRIORITY, NULL, ODOMETRY_TASK_CORE_ID);

}
