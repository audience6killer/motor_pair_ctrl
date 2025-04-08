#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_timer.h"

#include "kalman_filter.h"
#include "traction_control.h"
#include "pid_ctrl.h"

#include "diff_drive_ctrl.h"
#include "diff_drive_ctrl_task_common.h"

static const char TAG[] = "diff_drive";

ESP_EVENT_DEFINE_BASE(DIFF_DRIVE_EVENT_BASE);
EventGroupHandle_t diff_drive_event_group_handle = NULL;

static diff_drive_handle_t *diff_drive_handle = NULL;
static QueueHandle_t diff_drive_queue_handle = NULL;
static esp_timer_handle_t g_diff_queue_timer = NULL;
static esp_event_loop_handle_t g_tract_event_handle = NULL;
static esp_event_loop_handle_t g_diff_drive_event_handle = NULL;

static diff_drive_state_t g_diff_drive_state;
static navigation_point_t g_current_point;

esp_err_t diff_drive_post_to_tract(tract_ctrl_cmd_t cmd);

esp_err_t diff_drive_send2queue(diff_drive_state_t *state)
{
    ESP_RETURN_ON_FALSE(diff_drive_queue_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "trying to send msg to queue before init");

    if (xQueueSend(diff_drive_queue_handle, state, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t diff_drive_set_state(diff_drive_state_e state)
{
    g_diff_drive_state.state = state;
    return ESP_OK;
}

esp_err_t diff_drive_send_event_bit(diff_drive_state_e event)
{
    xEventGroupSetBits(diff_drive_event_group_handle, event);

    return ESP_OK;
}

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
        .motor_left_speed = left_speed,
        .motor_right_speed = right_speed,
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(diff_drive_post_to_tract(cmd));

    return ESP_OK;
}

esp_err_t diff_drive_position_control(float theta_error)
{
    ESP_RETURN_ON_FALSE(diff_drive_handle != NULL, ESP_ERR_INVALID_STATE, "TAG", "diff_drive_handle is null when calculating pos control");

    float wheel_angular_vel = 0.0f;

    ESP_ERROR_CHECK(pid_compute(diff_drive_handle->position_pid_ctrl, theta_error, &wheel_angular_vel));

    //printf("angular vel: %.4f rad/s\r\n", wheel_angular_vel);

    // In rad/s, so it's not necessary to divide the wheel radius
    float phi_lp = V_COMM - wheel_angular_vel;
    float phi_rp = V_COMM + wheel_angular_vel;

    phi_lp = MIN(MAX(phi_lp, -V_MAX_RADS), V_MAX_RADS);
    phi_rp = MIN(MAX(phi_rp, -V_MAX_RADS), V_MAX_RADS);

    // printf("wheel_angular_vel,%f,", wheel_angular_vel);
    //printf("phi_lp: %f, phi_rp: %f\r\n", phi_lp, phi_rp);

    float left_speed = (float)RADS2REVS(phi_lp);
    float right_speed = (float)RADS2REVS(phi_rp);

    tract_ctrl_cmd_t cmd = {
        .cmd = TRACT_CTRL_CMD_SET_SPEED,
        .motor_left_speed = left_speed,
        .motor_right_speed = right_speed,
    };

    //printf("sending speed to tract\r\n");
    // ESP_ERROR_CHECK_WITHOUT_ABORT(diff_drive_post_to_tract(cmd));
    ESP_ERROR_CHECK(tract_ctrl_set_speed(left_speed, right_speed));

    return ESP_OK;
}

esp_err_t diff_drive_point_follower(kalman_info_t *c_pose)
{
    g_diff_drive_state.err_y = g_current_point.y - c_pose->y;
    g_diff_drive_state.err_x = g_current_point.x - c_pose->x;

    float y_error = g_diff_drive_state.err_y;
    float x_error = g_diff_drive_state.err_x;

    float theta_m = atan2f(y_error, x_error);
    g_diff_drive_state.err_theta = theta_m - c_pose->theta;

    float theta_error = g_diff_drive_state.err_theta;

    g_diff_drive_state.err_dist = sqrtf(powf(x_error, 2) + powf(y_error, 2));
    g_diff_drive_state.err_ori = g_current_point.theta - c_pose->theta;

    float dist_error = g_diff_drive_state.err_dist;
    float ori_e = g_diff_drive_state.err_ori;

    // printf("theta_error:%f,d_error:%f,ori_e:%f*/\r\n", theta_error, dist_error, ori_e);

    if (dist_error > DISTANCE_TH)
    {
        g_diff_drive_state.state = DD_NAVIGATING;
        ESP_ERROR_CHECK(diff_drive_position_control(theta_error));
    }
    else if (fabs(ori_e) >= ORIENTATION_TH)
    {
        ESP_ERROR_CHECK(diff_drive_orientation_control(ori_e));
    }
    else
    {
        ESP_LOGI(TAG, "Vehicle reached point: (%f, %f, theta:%f)\n", g_current_point.x, g_current_point.y, g_current_point.theta);
        ESP_ERROR_CHECK(diff_drive_post_to_tract((tract_ctrl_cmd_t){
            .cmd = TRACT_CTRL_CMD_SET_SPEED,
            .motor_left_speed = 0.0f,
            .motor_right_speed = 0.0f,
        }));
        g_diff_drive_state.state = DD_POINT_REACHED;
        diff_drive_send_event_bit(DD_POINT_REACHED);
        // ESP_ERROR_CHECK(diff_drive_send2queue(&g_diff_drive_state));
    }

    return ESP_OK;
}

esp_err_t diff_drive_set_navigation_point(const navigation_point_t *point)
{
    ESP_RETURN_ON_FALSE(g_diff_drive_state.state != DD_ORIENTING || g_diff_drive_state.state != DD_NAVIGATING, ESP_ERR_INVALID_STATE, TAG, "Trying to change nav point before trajectory is compleated");

    ESP_LOGI(TAG, "New desired pose: (%.4f, %.4f, %.2f)", point->x, point->y, point->theta);

    g_current_point = *point;

    if (g_diff_drive_state.state != DD_ORIENTING)
    {

        g_diff_drive_state.state = DD_ORIENTING;

        //tract_ctrl_cmd_t cmd = (tract_ctrl_cmd_t){
        //    .cmd = TRACT_CTRL_CMD_START,
        //    .motor_left_speed = 0.0f,
        //    .motor_right_speed = 0.0f,
        //};
        //// ESP_ERROR_CHECK(diff_drive_send2queue(&g_diff_drive_state));
        //ESP_ERROR_CHECK(diff_drive_post_to_tract(cmd));
    }

    return ESP_OK;
}

esp_err_t diff_drive_get_queue_handle(QueueHandle_t *queue)
{
    ESP_RETURN_ON_FALSE(diff_drive_queue_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue handle is NULL");

    *queue = diff_drive_queue_handle;

    return ESP_OK;
}

esp_err_t diff_drive_get_event_loop(esp_event_loop_handle_t *handle)
{
    ESP_RETURN_ON_FALSE(g_diff_drive_event_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Event loop is NULL");

    *handle = g_diff_drive_event_handle;

    return ESP_OK;
}

esp_err_t diff_drive_post_to_tract(tract_ctrl_cmd_t cmd)
{
    ESP_RETURN_ON_FALSE(g_tract_event_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "Traction control event loop handle is NULL");

    float *data = NULL;
    if (cmd.cmd == TRACT_CTRL_CMD_SET_SPEED)
    {
        data = (float *)malloc(2 * sizeof(float));
        data[0] = cmd.motor_left_speed;
        data[1] = cmd.motor_right_speed;
    }

    esp_err_t ret = esp_event_post_to(g_tract_event_handle, TRACT_EVENT_BASE, cmd.cmd, data, sizeof(data), pdMS_TO_TICKS(20));

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error posting data to event loop. Error: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* Event handlers */
static void diff_drive_start_event_handle(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    if (g_diff_drive_state.state == DD_STOPPED || g_diff_drive_state.state == DD_READY)
    {
        ESP_ERROR_CHECK(esp_event_post_to(g_tract_event_handle, TRACT_EVENT_BASE, TRACT_CTRL_CMD_START, NULL, 0, pdMS_TO_TICKS(10)));
        //tract_ctrl_cmd_t cmd = (tract_ctrl_cmd_t){
        //    .cmd = TRACT_CTRL_CMD_START,
        //    .motor_left_speed = 0.0f,
        //    .motor_right_speed = 0.0f,
        //};
        //// ESP_ERROR_CHECK(diff_drive_send2queue(&g_diff_drive_state));
        //ESP_ERROR_CHECK(diff_drive_post_to_tract(cmd));
        diff_drive_set_state(DD_STARTED);
        diff_drive_send_event_bit(DD_STARTED);
        // diff_drive_send2queue(&g_diff_drive_state);
        ESP_LOGI(TAG, "Diff drive started");
    }
    else
    {
        ESP_LOGW(TAG, "Diff drive has already started!");
    }
}

static void diff_drive_stop_event_handle(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    if (g_diff_drive_state.state == DD_STOPPED)
    {
        ESP_LOGW(TAG, "Diff drive is already stopped!");
        return;
    }

    diff_drive_set_state(DD_STOPPED);
    // diff_drive_send2queue(&g_diff_drive_state);
    ESP_LOGI(TAG, "Diff drive stopped");
}

static void diff_drive_receive_point_event_handle(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    if (event_data == NULL)
    {
        ESP_LOGE(TAG, "event data is NULL");
        return;
    }

    esp_err_t ret = diff_drive_set_navigation_point((const navigation_point_t *)event_data);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed setting the nav point. Code error: %s", esp_err_to_name(ret));

        return;
    }
}

esp_err_t diff_drive_receive_kalman_info(QueueHandle_t *kalman_queue_handle)
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

    if (xQueueReceive(*kalman_queue_handle, &vehicle_pose, pdMS_TO_TICKS(100)) == pdPASS)
    {

#if false 
        const char *state = kalman_state_to_name(vehicle_pose.state);
        printf("/*x,%f,xd,%f,y,%f,yd,%f,theta,%f,thetad,%f,state,%s*/\r\n", vehicle_pose.x, g_current_point.x, vehicle_pose.y, g_current_point.y, vehicle_pose.theta, g_current_point.theta, state);
#endif
        if (g_diff_drive_state.state == DD_ORIENTING || g_diff_drive_state.state == DD_NAVIGATING)
            ESP_ERROR_CHECK(diff_drive_point_follower(&vehicle_pose));
    }
    else
    {
        ESP_LOGE(TAG, "Failed to receive kalman data");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t diff_drive_init(void)
{
    ESP_RETURN_ON_FALSE(diff_drive_handle != NULL, ESP_ERR_INVALID_STATE, "TAG", "diff_drive_handle is null when calculating pos control");

    ESP_LOGI(TAG, "Initatilizing PID parameters");
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

static void diff_drive_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing diff drive task. Waiting for traction to finish initializing.");

    diff_drive_queue_handle = xQueueCreate(4, sizeof(diff_drive_state_t));

    /* Wait for traction task to be initialized */
    xEventGroupWaitBits(tract_ctrl_event_group_handle, TRACT_EVENT_BIT_READY, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Traction finished initializing");

    diff_drive_handle = (diff_drive_handle_t *)malloc(sizeof(diff_drive_handle_t));
    ESP_ERROR_CHECK(diff_drive_init());

    // Initialize initial point
    g_current_point = (navigation_point_t){
        .theta = 0.0f,
        .x = 0.0f,
        .y = 0.0f,
    };

    QueueHandle_t kalman_data_queue_handle = NULL;

    while (kalman_get_data_queue(&kalman_data_queue_handle) != ESP_OK)
    {
        ESP_LOGW(TAG, "Retriying getting kalman data queue...");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "Kalman data queue obtained");

    ESP_ERROR_CHECK(tract_ctrl_get_event_loop_handle(&g_tract_event_handle));

    g_diff_drive_state = (diff_drive_state_t){
        .err_dist = 0.0f,
        .err_ori = 0.0f,
        .err_theta = 0.0f,
        .err_x = 0.0f,
        .err_y = 0.0f,
        .state = DD_STOPPED,
    };

    // Setting up the event loop
    esp_event_loop_args_t event_loop_args = {
        .queue_size = 10,
        .task_name = "diff_drive_event_loop",
        .task_priority = 5,
        .task_stack_size = 4084,
        .task_core_id = 0,
    };

    ESP_ERROR_CHECK(esp_event_loop_create(&event_loop_args, &g_diff_drive_event_handle));

    /* Register events */
    ESP_ERROR_CHECK(esp_event_handler_register_with(g_diff_drive_event_handle, DIFF_DRIVE_EVENT_BASE, DIFF_DRIVE_STOP_EVENT, diff_drive_stop_event_handle, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(g_diff_drive_event_handle, DIFF_DRIVE_EVENT_BASE, DIFF_DRIVE_START_EVENT, diff_drive_start_event_handle, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register_with(g_diff_drive_event_handle, DIFF_DRIVE_EVENT_BASE, DIFF_DRIVE_RECEIVE_POINT_EVENT, diff_drive_receive_point_event_handle, NULL));

    /* Update and communicate the status of the task */
    diff_drive_set_state(DD_READY);
    // diff_drive_send2queue(&g_diff_drive_state);

    /* Setup queue timer */

    /* Notify parent task the end of initialization */
    diff_drive_send_event_bit(DD_READY);

    for (;;)
    {
        if (g_diff_drive_state.state != DD_STOPPED && g_diff_drive_state.state != DD_READY)
        {
            diff_drive_receive_kalman_info(&kalman_data_queue_handle);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void diff_drive_task_start(void)
{
    ESP_LOGI(TAG, "Initializing navigation task");

    /* Initialize event group */
    diff_drive_event_group_handle = xEventGroupCreate();

    xTaskCreatePinnedToCore(&diff_drive_task, "navigation_unit", DIFF_DRIVE_STACK_SIZE, NULL, DIFF_DRIVE_TASK_PRIORITY, NULL, DIFF_DRIVE_CORE_ID);
}
