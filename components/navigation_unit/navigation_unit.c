#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

#include "navigation_unit.h"
#include "navigation_unit_task_common.h"

#include "odometry_unit.h"
#include "traction_control.h"
#include "pid_ctrl.h"

#define X_D 2.00f
#define Y_D 2.00f
#define THETA_D M_PI / 2
#define V_COMM 1.688f
#define WHEEL_RADIUS 0.033
#define DISTANCE_TH     0.05   // 5 cm
#define RADS2REVS(b) (b * 0.1592f)

static const char TAG[] = "navigation_unit";
static QueueHandle_t navigation_queue_handle;
static navigation_unit_handle_t *navigation_handle = NULL;

esp_err_t navigation_orientation_control(float theta_error)
{
    ESP_RETURN_ON_FALSE(navigation_handle != NULL, ESP_ERR_INVALID_STATE, "TAG", "navigation_handle is null when calculating pos control");

    float omega_comm = 0.0f;
    ESP_ERROR_CHECK(pid_compute(navigation_handle->orientation_pid_ctrl, theta_error, &omega_comm));

    float phi_lp = (-1.00f) * omega_comm / WHEEL_RADIUS;
    float phi_rp = omega_comm / WHEEL_RADIUS;

    ESP_ERROR_CHECK(traction_control_speed_controlled_direction((float)RADS2REVS(phi_lp), (float)RADS2REVS(phi_rp)));

    return ESP_OK;
}

esp_err_t navigation_position_control(float x_error, float y_error, float theta_error)
{
    ESP_RETURN_ON_FALSE(navigation_handle != NULL, ESP_ERR_INVALID_STATE, "TAG", "navigation_handle is null when calculating pos control");

    float omega_comm = 0.0f;

    ESP_ERROR_CHECK(pid_compute(navigation_handle->position_pid_ctrl, theta_error, &omega_comm));

    float phi_lp = (V_COMM - omega_comm) / WHEEL_RADIUS;
    float phi_rp = (V_COMM + omega_comm) / WHEEL_RADIUS;

    ESP_ERROR_CHECK(traction_control_speed_controlled_direction((float)RADS2REVS(phi_lp), (float)RADS2REVS(phi_rp)));

    return ESP_OK;
}

esp_err_t navigation_position_follower(odometry_robot_pose_t *c_pose)
{
    float y_error = Y_D - c_pose->y;
    float x_error = X_D - c_pose->x;

    float theta_m = atan2f(y_error, x_error);
    float theta_error = theta_m - c_pose->theta;

    float dist_error = sqrtf(powf(x_error, 2) + powf(y_error, 2));
    
    if(fabs(dist_error) <= DISTANCE_TH)
        ESP_ERROR_CHECK( navigation_orientation_control(theta_error) );
    else
        ESP_ERROR_CHECK( navigation_position_control(x_error, y_error, theta_error) );

    return ESP_OK; 
}

esp_err_t navigation_unit_init(void)
{
    ESP_RETURN_ON_FALSE(navigation_handle != NULL, ESP_ERR_INVALID_STATE, "TAG", "navigation_handle is null when calculating pos control");

    pid_ctrl_parameter_t navigation_pos_pid_runtime_param = {
        .kp = NAVIGATION_UNIT_POS_KP,
        .kd = NAVIGATION_UNIT_POS_KD,
        .ki = 0.0,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_integral = 10,
        .min_integral = -10,
        .min_output = -200,
    };

    pid_ctrl_parameter_t navigation_ori_pid_runtime_param = {
        .kp = NAVIGATION_UNIT_ORI_KP,
        .kd = NAVIGATION_UNIT_ORI_KD,
        .ki = 0.0,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_integral = 10,
        .min_integral = -10,
        .min_output = -200,
    };

    pid_ctrl_block_handle_t navigation_pos_pid_ctrl = NULL;
    pid_ctrl_block_handle_t navigation_ori_pid_ctrl = NULL;

    const pid_ctrl_config_t navigation_pos_pid_config = {
        .init_param = navigation_pos_pid_runtime_param,
    };

    const pid_ctrl_config_t navigation_ori_pid_config = {
        .init_param = navigation_ori_pid_runtime_param,
    };

    ESP_ERROR_CHECK(pid_new_control_block(&navigation_pos_pid_config, navigation_pos_pid_ctrl));
    ESP_ERROR_CHECK(pid_new_control_block(&navigation_ori_pid_config, navigation_ori_pid_ctrl));

    navigation_handle->position_pid_ctrl = navigation_pos_pid_ctrl;
    navigation_handle->orientation_pid_ctrl = navigation_pos_pid_ctrl;

    return ESP_OK;
}

static void navigation_unit_task(void *pvParameters)
{

    navigation_handle = (navigation_unit_handle_t *)malloc(sizeof(navigation_unit_handle_t));

    ESP_ERROR_CHECK(navigation_unit_init());

    QueueHandle_t odometry_queue_handle = odometry_unit_get_queue_handle();

    odometry_robot_pose_t vehicle_pose;

    for (;;)
    {
        if (xQueueReceive(odometry_queue_handle, &vehicle_pose, portMAX_DELAY) == pdPASS)
        {
            printf("/*x,%f,y,%f,theta,%f*/\r\n", vehicle_pose.x, vehicle_pose.y, vehicle_pose.theta);

            ESP_ERROR_CHECK(navigation_position_follower(&vehicle_pose));

        }
        else
        {
            ESP_LOGE(TAG, "Failed to receive queue");
        }

        vTaskDelay(30);
    }
}

void navigation_unit_task_start(void)
{
    ESP_LOGI(TAG, "Initializing navigation task");

    xTaskCreatePinnedToCore(&navigation_unit_task, "navigation_unit", NAVIGATION_UNIT_STACK_SIZE, NULL, NAVIGATION_UNIT_TASK_PRIORITY, NULL, NAVIGATION_UNIT_CORE_ID);
}
