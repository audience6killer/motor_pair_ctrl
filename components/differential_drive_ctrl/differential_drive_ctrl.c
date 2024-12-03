#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

#include "differential_drive_ctrl.h"
#include "differential_drive_ctrl_task_common.h"

#include "odometry_unit.h"
#include "traction_control.h"
#include "pid_ctrl.h"

#ifndef MIN
#define MIN(X, Y) ((X) < (Y) ? (X) : (Y))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

static const char TAG[] = "diff_drive_ctrl";
static QueueHandle_t diff_drive_queue_handle;
static diff_drive_ctrl_handle_t *diff_drive_handle = NULL;

static bool g_point_reached = false;
static bool g_position_control = false;

esp_err_t diff_drive_orientation_control(float theta_error)
{
    ESP_RETURN_ON_FALSE(diff_drive_handle != NULL, ESP_ERR_INVALID_STATE, "TAG", "diff_drive_handle is null when calculating pos control");

    float omega_comm = 0.0f;
    ESP_ERROR_CHECK(pid_compute(diff_drive_handle->orientation_pid_ctrl, theta_error, &omega_comm));

    // printf("e_theta, %f, omega_comm,%f,", theta_error, omega_comm);

    float phi_lp = (-1.00f) * omega_comm / WHEEL_RADIUS;
    float phi_rp = omega_comm / WHEEL_RADIUS;

    phi_lp = MIN(MAX(phi_lp, -V_MAX), V_MAX);
    phi_rp = MIN(MAX(phi_rp, -V_MAX), V_MAX);

    ESP_ERROR_CHECK(traction_control_speed_controlled_direction((float)RADS2REVS(phi_lp), (float)RADS2REVS(phi_rp)));

    return ESP_OK;
}

esp_err_t diff_drive_position_control(float theta_error)
{
    ESP_RETURN_ON_FALSE(diff_drive_handle != NULL, ESP_ERR_INVALID_STATE, "TAG", "diff_drive_handle is null when calculating pos control");

    float omega_comm = 0.0f;

    ESP_ERROR_CHECK(pid_compute(diff_drive_handle->position_pid_ctrl, theta_error, &omega_comm));

    float phi_lp = V_COMM - (omega_comm / WHEEL_RADIUS);
    float phi_rp = V_COMM + (omega_comm / WHEEL_RADIUS);

    phi_lp = MIN(MAX(phi_lp, -V_MAX), V_MAX);
    phi_rp = MIN(MAX(phi_rp, -V_MAX), V_MAX);

    // printf("omega_comm,%f,", omega_comm);
    //  printf("phi_lp: %f, phi_rp: %f\t", phi_lp, phi_rp);

    ESP_ERROR_CHECK(traction_control_speed_controlled_direction(phi_lp, phi_rp));

    return ESP_OK;
}

esp_err_t diff_drive_point_follower(odometry_robot_pose_t *c_pose)
{
    float y_error = Y_D - c_pose->y;
    float x_error = X_D - c_pose->x;

    float theta_m = atan2f(y_error, x_error);
    float theta_error = theta_m - c_pose->theta;

    float dist_error = sqrtf(powf(x_error, 2) + powf(y_error, 2));
    float ori_e = THETA_D - c_pose->theta;

    // printf(",ori_e,%f\n", ori_e);

    if (dist_error > DISTANCE_TH)
    {
        if (fabs(theta_error) >= ORIENTATION_TH)
        {
            //ESP_LOGI(TAG, "ORIENTATION ONLY");
            ESP_ERROR_CHECK(diff_drive_orientation_control(theta_error));
        }
        else
        {
            g_position_control = true;
            //ESP_LOGI(TAG, "POSITON ONLY");
            ESP_ERROR_CHECK(diff_drive_position_control(theta_error));
        }

        // printf("distance,%f,theta_error,%f,", dist_error, theta_error);
    }
    else if (fabs(ori_e) >= ORIENTATION_TH)
    {
        //ESP_LOGI(TAG, "POSITON ONLY");
        ESP_ERROR_CHECK(diff_drive_orientation_control(ori_e));
    }
    else
    {
        ESP_LOGI(TAG, "Vehicle reached point: (%f, %f, theta:%f)\n", X_D, Y_D, THETA_D);
        ESP_ERROR_CHECK(traction_control_speed_controlled_direction(0.0f, 0.0f));
        g_point_reached = true;
    }

    return ESP_OK;
}

esp_err_t diff_drive_ctrl_init(void)
{
    ESP_RETURN_ON_FALSE(diff_drive_handle != NULL, ESP_ERR_INVALID_STATE, "TAG", "diff_drive_handle is null when calculating pos control");

    ESP_LOGI(TAG, "Initatilizing navigation unit");
    pid_ctrl_parameter_t diff_drive_pos_pid_runtime_param = {
        .kp = DIFF_DRIVE_POS_KP,
        .kd = DIFF_DRIVE_POS_KD,
        .ki = 0.1,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_integral = 10,
        .min_integral = -10,
        .min_output = -200,
    };

    pid_ctrl_parameter_t diff_drive_ori_pid_runtime_param = {
        .kp = DIFF_DRIVE_ORI_KP,
        .kd = DIFF_DRIVE_ORI_KD,
        .ki = 0.1,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_integral = 10,
        .min_integral = -10,
        .min_output = -200,
        .max_output = 5,
    };

    pid_ctrl_block_handle_t diff_drive_pos_pid_ctrl = NULL;
    pid_ctrl_block_handle_t diff_drive_ori_pid_ctrl = NULL;

    pid_ctrl_config_t diff_drive_pos_pid_config = {
        .init_param = diff_drive_pos_pid_runtime_param,
    };

    pid_ctrl_config_t diff_drive_ori_pid_config = {
        .init_param = diff_drive_ori_pid_runtime_param,
    };

    ESP_ERROR_CHECK(pid_new_control_block(&diff_drive_pos_pid_config, &diff_drive_pos_pid_ctrl));
    ESP_ERROR_CHECK(pid_new_control_block(&diff_drive_ori_pid_config, &diff_drive_ori_pid_ctrl));

    diff_drive_handle->position_pid_ctrl = diff_drive_pos_pid_ctrl;
    diff_drive_handle->orientation_pid_ctrl = diff_drive_ori_pid_ctrl;

    return ESP_OK;
}

static void diff_drive_ctrl_task(void *pvParameters)
{

    diff_drive_handle = (diff_drive_ctrl_handle_t *)malloc(sizeof(diff_drive_ctrl_handle_t));

    ESP_ERROR_CHECK(diff_drive_ctrl_init());

    QueueHandle_t odometry_queue_handle = odometry_unit_get_queue_handle();

    odometry_robot_pose_t vehicle_pose;

    for (;;)
    {
        if (xQueueReceive(odometry_queue_handle, &vehicle_pose, portMAX_DELAY) == pdPASS)
        {
#if false
            printf("/*x,%f,y,%f,theta,%f*/\r\n", vehicle_pose.x, vehicle_pose.y, vehicle_pose.theta);
#endif
            if (!traction_control_is_busy() && !g_point_reached)
                ESP_ERROR_CHECK(diff_drive_point_follower(&vehicle_pose));
        }
        else
        {
            ESP_LOGE(TAG, "Failed to receive queue");
        }

        // vTaskDelay(10);
    }
}

void diff_drive_ctrl_task_start(void)
{
    ESP_LOGI(TAG, "Initializing navigation task");

    xTaskCreatePinnedToCore(&diff_drive_ctrl_task, "navigation_unit", DIFF_DRIVE_STACK_SIZE, NULL, DIFF_DRIVE_TASK_PRIORITY, NULL, DIFF_DRIVE_CORE_ID);
}
