#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

#include "diff_drive_ctrl.h"
#include "diff_drive_ctrl_task_common.h"

#include "kalman_filter.h"
#include "traction_control.h"
#include "pid_ctrl.h"

#ifndef MIN
#define MIN(X, Y) ((X) < (Y) ? (X) : (Y))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

typedef struct
{
    pid_ctrl_block_handle_t position_pid_ctrl;
    pid_ctrl_block_handle_t orientation_pid_ctrl;
    float vel_l; // rad/s -> rev/s
    float vel_r;
} diff_drive_ctrl_handle_t;

/* Static variables */
static const char TAG[] = "diff_drive_ctrl";
static QueueHandle_t g_diff_drive_state_queue = NULL;
static QueueHandle_t g_diff_drive_cmd_queue = NULL;
static diff_drive_ctrl_handle_t *diff_drive_handle = NULL;
static QueueHandle_t g_traction_cmd_queue = NULL;
static QueueHandle_t g_kalman_data_queue;

static diff_drive_state_e g_diff_drive_state;
static diff_drive_err_t g_diff_drive_error;
static navigation_point_t g_current_point;

/* Function declarations */
esp_err_t diff_drive_send2traction(tract_ctrl_cmd_t cmd);

esp_err_t diff_drive_update_state(diff_drive_state_e state)
{
    ESP_RETURN_ON_FALSE(g_diff_drive_state_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "trying to send msg to queue before init");

    g_diff_drive_state = state;
    diff_drive_state_e s = g_diff_drive_state;

    if (xQueueSend(g_diff_drive_state_queue, &s, pdMS_TO_TICKS(20)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending state queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}


esp_err_t diff_drive_orientation_control(float theta_error)
{
    ESP_RETURN_ON_FALSE(diff_drive_handle != NULL, ESP_ERR_INVALID_STATE, "TAG", "diff_drive_handle is null when calculating pos control");

    float wheel_angular_vel = 0.0f; // rad/s
    ESP_ERROR_CHECK(pid_compute(diff_drive_handle->orientation_pid_ctrl, theta_error, &wheel_angular_vel));
    printf("ORIENTATION CONTROL  ");

    // The lines below will be necessary if the linear velocity would be used,
    // but the traction module only accepts angular velocity
    // float phi_lp = (-1.00f) * wheel_angular_vel / WHEEL_RADIUS; // m/s
    // float phi_rp = wheel_angular_vel / WHEEL_RADIUS;

    // Ensure that the velocity is within the limits
    float phi_p = MIN(MAX(wheel_angular_vel, -V_MAX_RADS), V_MAX_RADS);

    float left_speed = (float)RADS2REVS((-1.0f) * phi_p);
    float right_speed = (float)RADS2REVS(phi_p);
    // printf("phi_lpo: %f, phi_rpo: %f\n", phi_p, phi_p);
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

    printf("POSITION CONTROL  ");
    float wheel_angular_vel = 0.0f;

    ESP_ERROR_CHECK(pid_compute(diff_drive_handle->position_pid_ctrl, theta_error, &wheel_angular_vel));

    float phi_lp = V_COMM - wheel_angular_vel;
    float phi_rp = V_COMM + wheel_angular_vel;

    // Check for angular velocity saturation
    phi_lp = MIN(MAX(phi_lp, -V_MAX_RADS), V_MAX_RADS);
    phi_rp = MIN(MAX(phi_rp, -V_MAX_RADS), V_MAX_RADS);

    float left_speed = (float)RADS2REVS(phi_lp);
    float right_speed = (float)RADS2REVS(phi_rp);

    // printf("phi_lp: %f, phi_rp: %f, phi_lpp: %f, phi_rpp: %f\n", phi_lp, phi_rp, left_speed, right_speed);

    tract_ctrl_cmd_t cmd = {
        .cmd = TRACT_CTRL_CMD_SET_SPEED,
        .motor_left_speed = &left_speed,
        .motor_right_speed = &right_speed,
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(diff_drive_send2traction(cmd));

    return ESP_OK;
}

esp_err_t diff_drive_point_follower(kalman_info_t *c_pose)
{
    g_diff_drive_error.err_y = g_current_point.y - c_pose->y;
    g_diff_drive_error.err_x = g_current_point.x - c_pose->x;

    float y_error = g_diff_drive_error.err_y;
    float x_error = g_diff_drive_error.err_x;

    // theta_m is the angle between the current position and the desired position
    float theta_m = atan2f(y_error, x_error);
    g_diff_drive_error.err_theta = theta_m - c_pose->theta;

    float theta_error = g_diff_drive_error.err_theta;

    g_diff_drive_error.err_dist = sqrtf(powf(x_error, 2) + powf(y_error, 2));
    // Err_ori is the error between the desired final orientation and the current orientation
    g_diff_drive_error.err_ori = g_current_point.theta - c_pose->theta;

    float dist_error = g_diff_drive_error.err_dist;
    float ori_e = g_diff_drive_error.err_ori;

    // printf("theta_error:%f,d_error:%f,ori_e:%f*/\r\n", theta_error, dist_error, ori_e);
    // printf("%.4f\n", dist_error);

    #if true
        printf("/*x,%.4f,xd,%.4f,y,%.4f,yd,%.4f,theta,%.4f,thetad,%.4f,dist_error,%.4f,ori_error,%.4f*/\n",c_pose->x, g_current_point.x, c_pose->y, g_current_point.y, c_pose->theta, g_current_point.theta, dist_error, ori_e);
    #endif

    if (dist_error >= DISTANCE_TH)
    {
        g_diff_drive_state = DD_STATE_NAVIGATING;
        ESP_ERROR_CHECK(diff_drive_position_control(theta_error));
    }
    else if (fabs(ori_e) > ORIENTATION_TH)
    {
        g_diff_drive_state = DD_STATE_ORIENTING;
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

        diff_drive_update_state(DD_STATE_POINT_REACHED);
    }

    return ESP_OK;
}

esp_err_t diff_drive_set_navigation_point(navigation_point_t point)
{
    // printf("STATE: %d\n", g_diff_drive_state);
    ESP_RETURN_ON_FALSE(g_diff_drive_state == DD_STATE_POINT_REACHED || g_diff_drive_state == DD_STATE_STARTED, ESP_ERR_INVALID_STATE, TAG, "Trying to change nav point before trajectory is compleated");

    ESP_LOGI(TAG, "New desired pose: (%.4f, %.4f, %.2f)\r\n", point.x, point.y, point.theta);

    g_current_point = point;

    if (g_diff_drive_state != DD_STATE_ORIENTING)
    {
        diff_drive_update_state(DD_STATE_ORIENTING);
    }

    return ESP_OK;
}

esp_err_t diff_drive_get_state_queue_handle(QueueHandle_t *queue)
{
    ESP_RETURN_ON_FALSE(g_diff_drive_state_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Data queue handle is NULL");

    *queue = g_diff_drive_state_queue;

    return ESP_OK;
}

esp_err_t diff_drive_get_cmd_queue_handle(QueueHandle_t *queue)
{
    ESP_RETURN_ON_FALSE(g_diff_drive_cmd_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "CMD queue handle is NULL");

    *queue = g_diff_drive_cmd_queue;

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

void diff_drive_receive_kalman_data(void)
{
    static kalman_info_t vehicle_pose = (kalman_info_t){
        .x = 0.0f,
        .y = 0.0f,
        .z = 0.0f,
        .theta = 0.0f,
        .x_p = 0.0f,
        .y_p = 0.0f,
        .z_p = 0.0f,
        .theta_p = 0.0f,
    };
    static int counter = 0;

    if (xQueueReceive(g_kalman_data_queue, &vehicle_pose, pdMS_TO_TICKS(5)) == pdPASS)
    {
        // printf("kalman received in diff_drive\n");
        // if (counter % 100 == 0)
        //{
        // counter = 0;
#if false 
            printf("/*x,%f,xd,%f,y,%f,yd,%f,theta,%f,thetad,%f,state,%d*/\n", vehicle_pose.x, g_current_point.x, vehicle_pose.y, g_current_point.y, vehicle_pose.theta, g_current_point.theta, g_diff_drive_state);
#endif
        // }

        // counter++;

        if (g_diff_drive_state != DD_STATE_POINT_REACHED)
            ESP_ERROR_CHECK(diff_drive_point_follower(&vehicle_pose));
    }
}

/* Event handlers */
esp_err_t diff_drive_start_event_handle(void)
{
    if (g_diff_drive_state == DD_STATE_STARTED)
    {
        ESP_LOGW(TAG, "Process already started!");
        return ESP_OK;
    }

    diff_drive_update_state(DD_STATE_STARTED);
    ESP_ERROR_CHECK(diff_drive_send2traction((tract_ctrl_cmd_t){
        .cmd = TRACT_CTRL_CMD_START,
        .motor_left_speed = NULL,
        .motor_right_speed = NULL,
    }));
    ESP_LOGW(TAG, "Process started");

    return ESP_OK;
}

esp_err_t diff_drive_stop_event_handle(void)
{
    if (g_diff_drive_state == DD_STATE_STOPPED)
    {
        ESP_LOGW(TAG, "Process already stopped!");
        return ESP_OK;
    }

    diff_drive_update_state(DD_STATE_STOPPED);
    ESP_ERROR_CHECK(diff_drive_send2traction((tract_ctrl_cmd_t){
        .cmd = TRACT_CTRL_CMD_STOP,
        .motor_left_speed = NULL,
        .motor_right_speed = NULL,
    }));
    ESP_LOGW(TAG, "Process Stopped");

    return ESP_OK;
}

esp_err_t diff_drive_receive_point_event_handler(navigation_point_t *point)
{
    ESP_RETURN_ON_FALSE(point != NULL, ESP_ERR_INVALID_STATE, TAG, "Navigation point is null");

    esp_err_t ret = diff_drive_set_navigation_point(*point);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error setting navigation point. Code: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    return ESP_OK;
}

void diff_drive_cmd_handler(void)
{
    static diff_drive_cmd_t cmd;
    if (xQueueReceive(g_diff_drive_cmd_queue, &cmd, pdMS_TO_TICKS(10)))
    {
        switch (cmd.cmd)
        {
        case DD_CMD_STOP:
            ESP_LOGI(TAG, "Event: Stop Process");
            diff_drive_stop_event_handle();
            break;

        case DD_CMD_START:
            ESP_LOGI(TAG, "Event: Start Process");
            diff_drive_start_event_handle();
            break;

        case DD_CMD_RECEIVE_POINT:
            ESP_LOGI(TAG, "Event: Point received");
            diff_drive_receive_point_event_handler(cmd.point);
            break;

        default:
            ESP_LOGE(TAG, "Wrong CMD format");
            break;
        }
    }
}

esp_err_t diff_drive_ctrl_init(void)
{
    ESP_RETURN_ON_FALSE(diff_drive_handle != NULL, ESP_ERR_INVALID_STATE, "TAG", "diff_drive_handle is null when calculating pos control");

    ESP_LOGI(TAG, "Initatilizing navigation unit");
    pid_ctrl_parameter_t diff_drive_pos_pid_runtime_param = {
        .kp = DIFF_DRIVE_POS_KP,
        .kd = DIFF_DRIVE_POS_KD,
        .ki = DIFF_DRIVE_POS_KI,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_integral = 1000,
        .min_integral = -1000,
        .max_output = 100,
        .min_output = -100,
    };

    pid_ctrl_parameter_t diff_drive_ori_pid_runtime_param = {
        .kp = DIFF_DRIVE_ORI_KP,
        .kd = DIFF_DRIVE_ORI_KD,
        .ki = 0.0f,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_integral = 100,
        .min_integral = -100,
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
    ESP_LOGI(TAG, "Initializing task");

    /* Initialize task handle */
    diff_drive_handle = (diff_drive_ctrl_handle_t *)malloc(sizeof(diff_drive_ctrl_handle_t));

    /* Initialize queues */
    g_diff_drive_state_queue = xQueueCreate(4, sizeof(diff_drive_state_e));
    g_diff_drive_cmd_queue = xQueueCreate(4, sizeof(diff_drive_cmd_t));

    ESP_ERROR_CHECK(diff_drive_ctrl_init());

    g_current_point = (navigation_point_t){
        .theta = 0.0f,
        .x = 0.0f,
        .y = 0.0f,
    };

    /* Get queue reference */
    while (kalman_get_data_queue(&g_kalman_data_queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get kalman data queue. Retraying...");
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    while (tract_ctrl_get_cmd_queue(&g_traction_cmd_queue) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get traction control queue. Retrying...");
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    g_diff_drive_error = (diff_drive_err_t){
        .err_dist = 0.0f,
        .err_ori = 0.0f,
        .err_theta = 0.0f,
        .err_x = 0.0f,
        .err_y = 0.0f,
    };

    // ESP_ERROR_CHECK(diff_drive_update_state());
    diff_drive_update_state(DD_STATE_READY);

    for (;;)
    {
        if (g_diff_drive_state == DD_STATE_ORIENTING || g_diff_drive_state == DD_STATE_NAVIGATING)
        {
            diff_drive_receive_kalman_data();
        }

        diff_drive_cmd_handler();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void diff_drive_ctrl_task_start(void)
{
    ESP_LOGI(TAG, "Starting navigation task");

    xTaskCreatePinnedToCore(&diff_drive_ctrl_task, "navigation_unit", DIFF_DRIVE_STACK_SIZE, NULL, DIFF_DRIVE_TASK_PRIORITY, NULL, DIFF_DRIVE_CORE_ID);
}
