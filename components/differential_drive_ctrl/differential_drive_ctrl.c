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
static QueueHandle_t g_traction_cmd_queue = NULL;

static diff_drive_state_t g_current_state;
static navigation_point_t g_current_point;

esp_err_t diff_drive_send2traction(tract_ctrl_cmd_t cmd);


esp_err_t diff_drive_orientation_control(float theta_error)
{
    ESP_RETURN_ON_FALSE(diff_drive_handle != NULL, ESP_ERR_INVALID_STATE, "TAG", "diff_drive_handle is null when calculating pos control");

    float wheel_angular_vel = 0.0f; // rad/s
    ESP_ERROR_CHECK(pid_compute(diff_drive_handle->orientation_pid_ctrl, theta_error, &wheel_angular_vel));

    // The lines below will be necessary if the linear velocity would be used,
    // but the traction module just accepts angular velocity
    // float phi_lp = (-1.00f) * wheel_angular_vel / WHEEL_RADIUS; // m/s
    // float phi_rp = wheel_angular_vel / WHEEL_RADIUS;

    // Ensure that the velocity is within the limits
    float phi_p = MIN(MAX(wheel_angular_vel, -V_MAX_RADS), V_MAX_RADS);
    

    float left_speed = (float)RADS2REVS((-1.0f) * phi_p);
    float right_speed = (float)RADS2REVS(phi_p);
    tract_ctrl_cmd_t cmd = {
        .cmd = TRACT_CTRL_CMD_SET_SPEED,
        .motor_left_speed = &left_speed,
        .motor_right_speed = &right_speed,
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(diff_drive_send2traction(cmd));
    

    return ESP_OK;
}

esp_err_t diff_drive_position_control(float theta_error)
{
    ESP_RETURN_ON_FALSE(diff_drive_handle != NULL, ESP_ERR_INVALID_STATE, "TAG", "diff_drive_handle is null when calculating pos control");

    float wheel_angular_vel = 0.0f;

    ESP_ERROR_CHECK(pid_compute(diff_drive_handle->position_pid_ctrl, theta_error, &wheel_angular_vel));

    float phi_lp = V_COMM - (wheel_angular_vel / WHEEL_RADIUS);
    float phi_rp = V_COMM + (wheel_angular_vel / WHEEL_RADIUS);

    phi_lp = MIN(MAX(phi_lp, -V_MAX_RADS), V_MAX_RADS);
    phi_rp = MIN(MAX(phi_rp, -V_MAX_RADS), V_MAX_RADS);

    // printf("wheel_angular_vel,%f,", wheel_angular_vel);
    // printf("phi_lp: %f, phi_rp: %f\n", phi_lp, phi_rp);

    float left_speed = (float)RADS2REVS(phi_lp);
    float right_speed = (float)RADS2REVS(phi_rp);

    tract_ctrl_cmd_t cmd = {
        .cmd = TRACT_CTRL_CMD_SET_SPEED,
        .motor_left_speed = &left_speed,
        .motor_right_speed = &right_speed,
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(diff_drive_send2traction(cmd));
    // ESP_ERROR_CHECK(traction_control_speed_controlled_direction(phi_lp, phi_rp));

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

esp_err_t diff_drive_point_follower(kalman_info_t *c_pose)
{
    g_current_state.err_y = g_current_point.y - c_pose->y;
    g_current_state.err_x = g_current_point.x - c_pose->x;

    float y_error = g_current_state.err_y;
    float x_error = g_current_state.err_x;

    float theta_m = atan2f(y_error, x_error);
    g_current_state.err_theta = theta_m - c_pose->theta;

    float theta_error = g_current_state.err_theta; 

    g_current_state.err_dist = sqrtf(powf(x_error, 2) + powf(y_error, 2));
    g_current_state.err_ori = g_current_point.theta - c_pose->theta;

    float dist_error = g_current_state.err_dist; 
    float ori_e = g_current_state.err_ori; 

    // printf("theta_error:%f,d_error:%f,ori_e:%f*/\r\n", theta_error, dist_error, ori_e);

    if (dist_error > DISTANCE_TH)
    {
        g_current_state.state = NAVIGATING;
        ESP_ERROR_CHECK(diff_drive_position_control(theta_error));
    }
    else if (fabs(ori_e) >= ORIENTATION_TH)
    {
        ESP_ERROR_CHECK(diff_drive_orientation_control(ori_e));
    }
    else
    {
        ESP_LOGI(TAG, "Vehicle reached point: (%f, %f, theta:%f)\n", g_current_point.x, g_current_point.y, g_current_point.theta);
        ESP_ERROR_CHECK(diff_drive_send2traction((tract_ctrl_cmd_t){
            .cmd = TRACT_CTRL_CMD_STOP,
            .motor_left_speed = NULL,
            .motor_right_speed = NULL,
        }));
        g_current_state.state = POINT_REACHED;

        ESP_ERROR_CHECK(diff_drive_send2queue(g_current_state));
    }

    return ESP_OK;
}

esp_err_t diff_drive_set_navigation_point(navigation_point_t point)
{
    ESP_RETURN_ON_FALSE(g_current_state.state == POINT_REACHED, ESP_ERR_INVALID_STATE, TAG, "Trying to change nav point before trajectory is compleated");

    ESP_LOGI(TAG, "New desired pose: (%.4f, %.4f, %.2f)\r\n", point.x, point.y, point.theta);

    g_current_point = point;
    g_current_state.state = ORIENTING;

    ESP_ERROR_CHECK(diff_drive_send2queue(g_current_state));
    ESP_ERROR_CHECK(diff_drive_send2traction((tract_ctrl_cmd_t){
        .cmd = TRACT_CTRL_CMD_START,
        .motor_left_speed = NULL,
        .motor_right_speed = NULL,
    }));

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
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_integral = 10,
        .min_integral = -10,
        .min_output = -12,
        .max_output = 12,
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

esp_err_t diff_drive_get_queue_handle(QueueHandle_t *queue)
{
    ESP_RETURN_ON_FALSE(diff_drive_queue_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue handle is NULL");

    *queue = diff_drive_queue_handle;

    return ESP_OK;
}

esp_err_t diff_drive_send2traction(tract_ctrl_cmd_t cmd)
{
    ESP_RETURN_ON_FALSE(g_traction_cmd_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Traction control queue is NULL");

    if (xQueueSend(g_traction_cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending to traction control queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void diff_drive_ctrl_task(void *pvParameters)
{
    ESP_LOGI(TAG, "running diff drive task");

    diff_drive_handle = (diff_drive_ctrl_handle_t *)malloc(sizeof(diff_drive_ctrl_handle_t));

    ESP_ERROR_CHECK(diff_drive_ctrl_init());

    // Initialize inicial point
    g_current_point = (navigation_point_t){
        .theta = 0.0f,
        .x = 0.0f,
        .y = 0.0f,
    };

    diff_drive_queue_handle = xQueueCreate(4, sizeof(diff_drive_state_t));

    QueueHandle_t kalman_filter_queue_pv = kalman_fiter_get_queue();
    
    while(tract_ctrl_get_cmd_queue(&g_traction_cmd_queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get traction control queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(40));
    }

    kalman_info_t vehicle_pose = (kalman_info_t){
        .x = 0.0f,
        .y = 0.0f,
        .z = 0.0f,
        .theta = 0.0f,
        .x_p = 0.0f,
        .y_p = 0.0f,
        .z_p = 0.0f,
        .theta_p = 0.0f,
    };

    g_current_state = (diff_drive_state_t){
        .err_dist = 0.0f,
        .err_ori = 0.0f,
        .err_theta = 0.0f,
        .err_x = 0.0f,
        .err_y = 0.0f,
        .state = POINT_REACHED,
    };

    ESP_ERROR_CHECK(diff_drive_send2queue(g_current_state));

    for (;;)
    {
        if (xQueueReceive(kalman_filter_queue_pv, &vehicle_pose, portMAX_DELAY) == pdPASS)
        {
            
#if false 
            printf("/*x,%f,xd,%f,y,%f,yd,%f,theta,%f,thetad,%f,state,%d*/\r\n", vehicle_pose.x, g_current_point.x, vehicle_pose.y, g_current_point.y, vehicle_pose.theta, g_current_point.theta, g_current_state);
#endif
            if (g_current_state.state != POINT_REACHED)
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
