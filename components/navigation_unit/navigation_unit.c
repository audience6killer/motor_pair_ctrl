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

#ifndef MIN
#define MIN(X, Y) ((X) < (Y) ? (X) : (Y))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#define X_D 4.00f
#define Y_D 4.00f
#define THETA_D M_PI
#define V_COMM 1.00f
#define V_MAX 2.00f
#define WHEEL_RADIUS 0.033f
#define DISTANCE_TH 0.05f        // 5 cm
#define ORIENTATION_TH 0.0872665 // 5°
#define RADS2REVS(b) (b * 0.1592f)

static const char TAG[] = "navigation_unit";
static QueueHandle_t navigation_queue_handle;
static navigation_unit_handle_t *navigation_handle = NULL;

static bool g_point_reached = false;
static bool g_position_control = false;

esp_err_t navigation_orientation_control(float theta_error)
{
    ESP_RETURN_ON_FALSE(navigation_handle != NULL, ESP_ERR_INVALID_STATE, "TAG", "navigation_handle is null when calculating pos control");

    float omega_comm = 0.0f;
    ESP_ERROR_CHECK(pid_compute(navigation_handle->orientation_pid_ctrl, theta_error, &omega_comm));

    // printf("e_theta, %f, omega_comm,%f,", theta_error, omega_comm);

    float phi_lp = (-1.00f) * omega_comm / WHEEL_RADIUS;
    float phi_rp = omega_comm / WHEEL_RADIUS;

    phi_lp = MIN(MAX(phi_lp, -V_MAX), V_MAX);
    phi_rp = MIN(MAX(phi_rp, -V_MAX), V_MAX);

    ESP_ERROR_CHECK(traction_control_speed_controlled_direction((float)RADS2REVS(phi_lp), (float)RADS2REVS(phi_rp)));

    return ESP_OK;
}

esp_err_t navigation_position_control(float theta_error)
{
    ESP_RETURN_ON_FALSE(navigation_handle != NULL, ESP_ERR_INVALID_STATE, "TAG", "navigation_handle is null when calculating pos control");

    float omega_comm = 0.0f;

    ESP_ERROR_CHECK(pid_compute(navigation_handle->position_pid_ctrl, theta_error, &omega_comm));

    // ERROR HERE!
    float phi_lp = V_COMM - (omega_comm / WHEEL_RADIUS);
    float phi_rp = V_COMM + (omega_comm / WHEEL_RADIUS);

    phi_lp = MIN(MAX(phi_lp, -V_MAX), V_MAX);
    phi_rp = MIN(MAX(phi_rp, -V_MAX), V_MAX);

    // printf("omega_comm,%f,", omega_comm);
    //  printf("phi_lp: %f, phi_rp: %f\t", phi_lp, phi_rp);

    ESP_ERROR_CHECK(traction_control_speed_controlled_direction(phi_lp, phi_rp));

    return ESP_OK;
}

esp_err_t navigation_position_follower(odometry_robot_pose_t *c_pose)
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
            ESP_ERROR_CHECK(navigation_orientation_control(theta_error));
        }
        else
        {
            g_position_control = true;
            //ESP_LOGI(TAG, "POSITON ONLY");
            ESP_ERROR_CHECK(navigation_position_control(theta_error));
        }

        // printf("distance,%f,theta_error,%f,", dist_error, theta_error);
    }
    else if (fabs(ori_e) >= ORIENTATION_TH)
    {
        //ESP_LOGI(TAG, "POSITON ONLY");
        ESP_ERROR_CHECK(navigation_orientation_control(ori_e));
    }
    else
    {
        ESP_LOGI(TAG, "Vehicle reached point: (%f, %f, theta:%f)\n", X_D, Y_D, THETA_D);
        ESP_ERROR_CHECK(traction_control_speed_controlled_direction(0.0f, 0.0f));
        g_point_reached = true;
    }

    return ESP_OK;
}

esp_err_t navigation_unit_init(void)
{
    ESP_RETURN_ON_FALSE(navigation_handle != NULL, ESP_ERR_INVALID_STATE, "TAG", "navigation_handle is null when calculating pos control");

    ESP_LOGI(TAG, "Initatilizing navigation unit");
    pid_ctrl_parameter_t navigation_pos_pid_runtime_param = {
        .kp = NAVIGATION_UNIT_POS_KP,
        .kd = NAVIGATION_UNIT_POS_KD,
        .ki = 0.1,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_integral = 10,
        .min_integral = -10,
        .min_output = -200,
    };

    pid_ctrl_parameter_t navigation_ori_pid_runtime_param = {
        .kp = NAVIGATION_UNIT_ORI_KP,
        .kd = NAVIGATION_UNIT_ORI_KD,
        .ki = 0.1,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_integral = 10,
        .min_integral = -10,
        .min_output = -200,
        .max_output = 5,
    };

    pid_ctrl_block_handle_t navigation_pos_pid_ctrl = NULL;
    pid_ctrl_block_handle_t navigation_ori_pid_ctrl = NULL;

    pid_ctrl_config_t navigation_pos_pid_config = {
        .init_param = navigation_pos_pid_runtime_param,
    };

    pid_ctrl_config_t navigation_ori_pid_config = {
        .init_param = navigation_ori_pid_runtime_param,
    };

    ESP_ERROR_CHECK(pid_new_control_block(&navigation_pos_pid_config, &navigation_pos_pid_ctrl));
    ESP_ERROR_CHECK(pid_new_control_block(&navigation_ori_pid_config, &navigation_ori_pid_ctrl));

    navigation_handle->position_pid_ctrl = navigation_pos_pid_ctrl;
    navigation_handle->orientation_pid_ctrl = navigation_ori_pid_ctrl;

    return ESP_OK;
}

static void navigation_unit_task(void *pvParameters)
{

    navigation_handle = (navigation_unit_handle_t *)malloc(sizeof(navigation_unit_handle_t));

    ESP_ERROR_CHECK(navigation_unit_init());

    QueueHandle_t odometry_queue_handle = odometry_unit_get_queue_handle();

    // Start up traction soft start
    // const float target_speed = V_COMM;
    // const int tf = 100;
    // traction_control_soft_start(V_COMM, tf);

    odometry_robot_pose_t vehicle_pose;

    for (;;)
    {
        if (xQueueReceive(odometry_queue_handle, &vehicle_pose, portMAX_DELAY) == pdPASS)
        {
#if false
            printf("/*x,%f,y,%f,theta,%f*/\r\n", vehicle_pose.x, vehicle_pose.y, vehicle_pose.theta);
#endif
            if (!traction_control_is_busy() && !g_point_reached)
                ESP_ERROR_CHECK(navigation_position_follower(&vehicle_pose));
        }
        else
        {
            ESP_LOGE(TAG, "Failed to receive queue");
        }

        // vTaskDelay(10);
    }
}

void navigation_unit_task_start(void)
{
    ESP_LOGI(TAG, "Initializing navigation task");

    xTaskCreatePinnedToCore(&navigation_unit_task, "navigation_unit", NAVIGATION_UNIT_STACK_SIZE, NULL, NAVIGATION_UNIT_TASK_PRIORITY, NULL, NAVIGATION_UNIT_CORE_ID);
}
