#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

#include "differential_drive_ctrl.h"
#include "differential_drive_ctrl_task_common.h"

#include "kalman_filter.h"
#include "traction_control.h"
#include "pid_ctrl.h"

#ifndef MIN
#define MIN(X, Y) ((X) < (Y) ? (X) : (Y))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

static const char TAG[] = "diff_drive_ctrl";
static QueueHandle_t diff_drive_queue_handle = NULL;
static diff_drive_ctrl_handle_t *diff_drive_handle = NULL;

static diff_drive_state_t g_current_state = POINT_REACHED;

static navigation_point_t g_current_point;

esp_err_t diff_drive_orientation_control(float theta_error)
{
    ESP_RETURN_ON_FALSE(diff_drive_handle != NULL, ESP_ERR_INVALID_STATE, "TAG", "diff_drive_handle is null when calculating pos control");

    float omega_comm = 0.0f;
    ESP_ERROR_CHECK(pid_compute(diff_drive_handle->orientation_pid_ctrl, theta_error, &omega_comm));

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

esp_err_t diff_drive_send2queue(diff_drive_state_t state)
{
    diff_drive_state_t s = state;

    ESP_RETURN_ON_FALSE(diff_drive_queue_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "trying to send msg to queue before init");

    if (xQueueSend(diff_drive_queue_handle, &s, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t diff_drive_point_follower(navigation_point_t *c_pose)
{
    float y_error = g_current_point.y - c_pose->y;
    float x_error = g_current_point.x - c_pose->x;

    float theta_m = atan2f(y_error, x_error);
    float theta_error = theta_m - c_pose->theta;

    float dist_error = sqrtf(powf(x_error, 2) + powf(y_error, 2));
    float ori_e = g_current_point.theta - c_pose->theta;

    if (dist_error > DISTANCE_TH)
    {
        if (fabs(theta_error) >= ORIENTATION_TH)
        {
            ESP_ERROR_CHECK(diff_drive_orientation_control(theta_error));
        }
        else
        {
            ESP_ERROR_CHECK(diff_drive_position_control(theta_error));
        }
    }
    else if (fabs(ori_e) >= ORIENTATION_TH)
    {
        ESP_ERROR_CHECK(diff_drive_orientation_control(ori_e));
    }
    else
    {
        ESP_LOGI(TAG, "Vehicle reached point: (%f, %f, theta:%f)\n", g_current_point.x, g_current_point.y, g_current_point.theta);
        ESP_ERROR_CHECK(traction_control_speed_controlled_direction(0.0f, 0.0f));
        g_current_state = POINT_REACHED;

        ESP_ERROR_CHECK(diff_drive_send2queue(g_current_state));
    }

    return ESP_OK;
}

esp_err_t diff_drive_set_navigation_point(navigation_point_t point)
{
    ESP_RETURN_ON_FALSE(g_current_state == POINT_REACHED, ESP_ERR_INVALID_STATE, TAG, "Trying to change nav point before trajectory is compleated");

    ESP_LOGI(TAG, "New desired pose: (%.4f, %.4f, %.2f)\r\n", point.x, point.y, point.theta);

    g_current_point = point;
    g_current_state = NAVIGATING;

    ESP_ERROR_CHECK(diff_drive_send2queue(g_current_state));

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

QueueHandle_t diff_drive_get_queue_handle(void)
{
    return diff_drive_queue_handle;
}

static void diff_drive_ctrl_task(void *pvParameters)
{
    ESP_LOGI(TAG, "running diff drive task");

    diff_drive_handle = (diff_drive_ctrl_handle_t *)malloc(sizeof(diff_drive_ctrl_handle_t));

    ESP_ERROR_CHECK(diff_drive_ctrl_init());

    g_current_point = (navigation_point_t){
        .theta = 0.0f,
        .x = 0.0f,
        .y = 0.0f,
    };

    diff_drive_queue_handle = xQueueCreate(4, sizeof(diff_drive_state_t));

    QueueHandle_t kalman_filter_queue_pv = kalman_fiter_get_queue();

    navigation_point_t vehicle_pose;

    ESP_ERROR_CHECK(diff_drive_send2queue(g_current_state));

    for (;;)
    {
        if (xQueueReceive(kalman_filter_queue_pv, &vehicle_pose, portMAX_DELAY) == pdPASS)
        {
#if false 
            printf("/*x,%f,xd,%f,y,%f,yd,%f,theta,%f,thetad,%f,state,%d*/\r\n", vehicle_pose.x, g_current_point.x, vehicle_pose.y, g_current_point.y, vehicle_pose.theta, g_current_point.theta, g_current_state);
#endif
            if (g_current_state != POINT_REACHED)
                ESP_ERROR_CHECK(diff_drive_point_follower(&vehicle_pose));
        }
        else
        {
            ESP_LOGE(TAG, "Failed to receive queue");
        }
    }
}

void diff_drive_ctrl_task_start(void)
{
    ESP_LOGI(TAG, "Initializing navigation task");

    xTaskCreatePinnedToCore(&diff_drive_ctrl_task, "navigation_unit", DIFF_DRIVE_STACK_SIZE, NULL, DIFF_DRIVE_TASK_PRIORITY, NULL, DIFF_DRIVE_CORE_ID);
}
