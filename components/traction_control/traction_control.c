#include "stdio.h"
#include "math.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_check.h"
#include "stdbool.h"
#include "string.h"
#include "driver/gptimer.h"

#include "traction_control.h"
#include "traction_task_common.h"
#include "motor_pair_ctrl.h"

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

static const char TAG[] = "tract_ctrl";

static motor_pair_handle_t *g_traction_handle = NULL;
static QueueHandle_t g_traction_cmd_queue = NULL;
static QueueHandle_t g_traction_data_queue = NULL;
static QueueHandle_t g_traction_isr_data_queue = NULL;
static motor_pair_state_e g_traction_state = STOPPED;
static motor_pair_data_t traction_data;
static esp_timer_handle_t g_traction_pid_timer = NULL;
static gptimer_handle_t g_traction_pid_gptimer = NULL;

/**
 * @brief Control direction and speed of the motors
 *
 * @param mleft_speed_pv
 * @param mright_speed_pv
 * @return esp_err_t
 */
esp_err_t tract_ctrl_set_speed_event_handler(float *mleft_speed_pv, float *mright_speed_pv);

/**
 * @brief Sends data to the data queue
 *
 * @param data
 * @return esp_err_t
 */
esp_err_t tract_ctrl_send2data_queue(motor_pair_data_t *data);

motor_pair_state_e tract_ctrl_get_current_state(void)
{
    return g_traction_state;
}

esp_err_t tract_ctrl_get_data_queue(QueueHandle_t *queue)
{
    ESP_RETURN_ON_FALSE(g_traction_data_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue handle is NULL");

    *queue = g_traction_data_queue;
    return ESP_OK;
}

esp_err_t tract_ctrl_get_cmd_queue(QueueHandle_t *queue)
{
    ESP_RETURN_ON_FALSE(g_traction_cmd_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue handle is NULL");

    *queue = g_traction_cmd_queue;
    return ESP_OK;
}

static bool IRAM_ATTR traction_pid_isr_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *event_data, void *user_ctx)
{
    // printf("In loop");
    static int motor_left_last_pulse_count = 0;
    static int motor_right_last_pulse_count = 0;

    // Read current encoder counts
    int motor_left_cur_pulse_count = 0;
    int motor_right_cur_pulse_count = 0;
    pcnt_unit_get_count(g_traction_handle->motor_left_ctx.pcnt_encoder, &motor_left_cur_pulse_count);
    pcnt_unit_get_count(g_traction_handle->motor_right_ctx.pcnt_encoder, &motor_right_cur_pulse_count);

    // Calculate pulses since the last callback
    int motor_left_real_pulses = motor_left_cur_pulse_count - motor_left_last_pulse_count;
    int motor_right_real_pulses = motor_right_cur_pulse_count - motor_right_last_pulse_count;

    motor_left_last_pulse_count = motor_left_cur_pulse_count;
    motor_right_last_pulse_count = motor_right_cur_pulse_count;

    // Prepare data to send to the queue
    motor_pair_data_t isr_traction_data = {
        .mleft_pulses = motor_left_real_pulses,
        .mright_pulses = motor_right_real_pulses,
        .state = g_traction_state,
    };

    // Send data to the queue (use ISR-safe function)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(g_traction_isr_data_queue, &isr_traction_data, &xHigherPriorityTaskWoken);

    // Request a context switch if a higher-priority task was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    return true; // Return true to indicate the timer should continue
}

/*
static bool IRAM_ATTR traction_pid_loop_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *event_data, void *user_ctx)
{
    static int motor_left_last_pulse_count = 0;
    static int motor_right_last_pulse_count = 0;
    static motor_pair_state_e last_traction_state = STOPPED;

    static int64_t last_execution_time = 0;
    int64_t current_time = esp_timer_get_time();
    int64_t time_diff = current_time - last_execution_time;
    // if (last_execution_time != 0)
    //{
    //     int64_t time_diff = current_time - last_execution_time;
    //     // ESP_LOGI(TAG, "TractionPID_Period: %lld ms", time_diff / 1000);
    // }
    last_execution_time = current_time;

    // ESP_LOGI(TAG, "IN LOOPPPPPP!");

    // Calculate current speed
    int motor_left_cur_pulse_count = 0;
    int motor_right_cur_pulse_count = 0;
    pcnt_unit_get_count(g_traction_handle->motor_left_ctx.pcnt_encoder, &motor_left_cur_pulse_count);
    pcnt_unit_get_count(g_traction_handle->motor_right_ctx.pcnt_encoder, &motor_right_cur_pulse_count);

    // The sign of the speed doesn't matter, as the forward and reverse of the motor will control the direction
    int motor_left_real_pulses = motor_left_cur_pulse_count - motor_left_last_pulse_count;
    int motor_right_real_pulses = motor_right_cur_pulse_count - motor_right_last_pulse_count;

    int motor_left_abs_pulses = abs(motor_left_real_pulses);
    int motor_right_abs_pulses = abs(motor_right_real_pulses);

    // Save the real value of the speed
    traction_data.mleft_pulses = motor_left_real_pulses;
    traction_data.mright_pulses = motor_right_real_pulses;

    motor_right_last_pulse_count = motor_right_cur_pulse_count;
    motor_left_last_pulse_count = motor_left_cur_pulse_count;

    // Check whether the state has changed
    // TODO Add Stopped state action
    if (last_traction_state != g_traction_state)
    {
        last_traction_state = g_traction_state;
        traction_data.state = g_traction_state;

        switch (g_traction_state)
        {
        case BRAKE:
            ESP_ERROR_CHECK(bdc_motor_brake(g_traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_brake(g_traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACT_DIR: BREAK");
            break;
        case COAST:
            ESP_ERROR_CHECK(bdc_motor_coast(g_traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_coast(g_traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACT_DIR: COAST");
            break;
        case STARTING:
            ESP_ERROR_CHECK(bdc_motor_forward(g_traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_forward(g_traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACT_DIR: STARTING");
            break;
        case FORWARD:
            ESP_ERROR_CHECK(bdc_motor_forward(g_traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_forward(g_traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACT_DIR: FORWARD");
            break;
        case REVERSE:
            ESP_ERROR_CHECK(bdc_motor_reverse(g_traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_reverse(g_traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACT_DIR: REVERSE");
            break;
        case TURN_LEFT_FORWARD:
            ESP_ERROR_CHECK(bdc_motor_reverse(g_traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_forward(g_traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACT_DIR: LEFT FORWARD");
            break;
        case TURN_RIGHT_FORWARD:
            ESP_ERROR_CHECK(bdc_motor_forward(g_traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_reverse(g_traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACT_DIR: RIGHT FORWARD");
            break;
        case TURN_LEFT_REVERSE:
            ESP_ERROR_CHECK(bdc_motor_forward(g_traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_reverse(g_traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACT_DIR: LEFT REVERSE");
            break;
        case TURN_RIGHT_REVERSE:
            ESP_ERROR_CHECK(bdc_motor_reverse(g_traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_forward(g_traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACT_DIR: RIGHT REVERSE");
            break;
        default:
            ESP_LOGI(TAG, "TRACT_DIR: ERRORRRRRRRRR");
            break;
        }
    }

    float motor_right_new_speed = 0;
    float motor_left_new_speed = 0;
    // If the vehicle is in break or coast state, its not necessary to calculate the PID value
    if (last_traction_state != BRAKE && last_traction_state != COAST)
    {
        // Calculate speed error
        float motor_left_error = g_traction_handle->motor_left_ctx.desired_speed - motor_left_abs_pulses;
        float motor_right_error = g_traction_handle->motor_right_ctx.desired_speed - motor_right_abs_pulses;

        // Set the new speed
        pid_compute(g_traction_handle->motor_right_ctx.pid_ctrl, motor_right_error, &motor_right_new_speed);
        pid_compute(g_traction_handle->motor_left_ctx.pid_ctrl, motor_left_error, &motor_left_new_speed);

        // printf("(%.4f, %.4f)\n", motor_left_new_speed, motor_right_new_speed);

        ESP_ERROR_CHECK(bdc_motor_set_speed(g_traction_handle->motor_right_ctx.motor, (uint32_t)motor_right_new_speed));
        ESP_ERROR_CHECK(bdc_motor_set_speed(g_traction_handle->motor_left_ctx.motor, (uint32_t)motor_left_new_speed));
    }

    // Save information
    if (g_traction_state == REVERSE || g_traction_state == TURN_LEFT_FORWARD || g_traction_state == TURN_RIGHT_REVERSE)
        traction_data.mleft_set_point = (-1) * g_traction_handle->motor_left_ctx.desired_speed;
    else
        traction_data.mleft_set_point = g_traction_handle->motor_left_ctx.desired_speed;

    if (g_traction_state == REVERSE || g_traction_state == TURN_RIGHT_FORWARD || g_traction_state == TURN_LEFT_REVERSE)
        traction_data.mright_set_point = (-1) * g_traction_handle->motor_right_ctx.desired_speed;
    else
        traction_data.mright_set_point = g_traction_handle->motor_right_ctx.desired_speed;

    // Send data to the queue
    tract_ctrl_send2data_queue(&traction_data);

    // printf("%d,%d,%d,%d,%.4f,%.4f,%llu\n", traction_data.mleft_set_point, traction_data.mleft_pulses, traction_data.mright_set_point, traction_data.mright_pulses, motor_left_new_speed, motor_right_new_speed, time_diff / 1000);

    // if (xQueueSend(g_traction_data_queue, &traction_data, portMAX_DELAY) != pdPASS)
    //{
    //     ESP_LOGE(TAG, "Error sending data to queue");
    // }

    return true;
}*/

esp_err_t tract_ctrl_send2data_queue(motor_pair_data_t *data)
{
    ESP_RETURN_ON_FALSE(g_traction_data_queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue handle is NULL");
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "Data is NULL");

    if (xQueueSend(g_traction_data_queue, data, pdMS_TO_TICKS(10)) != pdPASS)
    {
        // ESP_LOGE(TAG, "Error sending data to queue");
        // return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t tract_ctrl_set_direction(const motor_pair_state_e state)
{
    // TODO: Some kind of verification might be necessary
    g_traction_state = state;
    return ESP_OK;
}

/* Event handlers */
esp_err_t tract_ctrl_stop_event_handler(void)
{
    // ESP_RETURN_ON_FALSE(g_traction_pid_timer != NULL, ESP_ERR_INVALID_STATE, TAG, "PID timer is NULL");

    // if (!esp_timer_is_active(g_traction_pid_timer))
    // {
    //     ESP_LOGW(TAG, "PID loop already stopped");
    //     return ESP_OK;
    // }

    float zero_speed = 0.0f;
    tract_ctrl_set_speed_event_handler(&zero_speed, &zero_speed);
    ESP_ERROR_CHECK(bdc_motor_coast(g_traction_handle->motor_left_ctx.motor));
    ESP_ERROR_CHECK(bdc_motor_coast(g_traction_handle->motor_right_ctx.motor));
    pid_reset_ctrl_block(g_traction_handle->motor_left_ctx.pid_ctrl);
    pid_reset_ctrl_block(g_traction_handle->motor_right_ctx.pid_ctrl);

    // esp_err_t ret = esp_timer_stop(g_traction_pid_timer);
    // if (ret != ESP_OK)
    //     ESP_LOGE(TAG, "Error stopping speed loop");
    // else
    //     ESP_LOGI(TAG, "Speed loop stopped successfully");

    return ESP_OK;
}

esp_err_t tract_ctrl_start_event_handler(void)
{
    // ESP_RETURN_ON_FALSE(g_traction_pid_timer != NULL, ESP_ERR_INVALID_STATE, TAG, "PID timer is NULL");
    // Starting pid loop
    // if (esp_timer_is_active(g_traction_pid_timer))
    // {
    //     ESP_LOGW(TAG, "PID loop already started");
    //     return ESP_OK;
    // }
    // esp_err_t ret = esp_timer_start_periodic(g_traction_pid_timer, BDC_PID_LOOP_PERIOD_MS * 1000);

    // if (ret != ESP_OK)
    //     ESP_LOGE(TAG, "Error starting pid loop timer");
    // else
    ESP_LOGI(TAG, "PID loop timer started correctly");

    return ESP_OK;
}

esp_err_t tract_ctrl_set_speed_event_handler(float *mleft_speed_pv, float *mright_speed_pv)
{
    ESP_RETURN_ON_FALSE(mleft_speed_pv != NULL, ESP_ERR_INVALID_ARG, TAG, "mleft_speed is NULL");
    ESP_RETURN_ON_FALSE(mright_speed_pv != NULL, ESP_ERR_INVALID_ARG, TAG, "mright_speed is NULL");

    // if (!esp_timer_is_active(g_traction_pid_timer))
    //{
    //     ESP_LOGE(TAG, "PID loop is not running");
    //     return ESP_FAIL;
    // }

    float mleft_speed = *mleft_speed_pv;
    float mright_speed = *mright_speed_pv;
    float mleft_abs = fabs(mleft_speed);
    float mright_abs = fabs(mright_speed);

    // printf("mleft_abs: %f, mright_abs: %f\n", mleft_abs, mright_abs);

    if (mleft_abs <= TRACT_MOTOR_MAX_REVS && mright_abs <= TRACT_MOTOR_MAX_REVS)
    {
        if (mleft_speed < 0.0f && mright_speed < 0.0f)
            tract_ctrl_set_direction(REVERSE);
        else if (mleft_speed < 0.0f && mright_speed >= 0.0f)
            tract_ctrl_set_direction(TURN_LEFT_FORWARD);
        else if (mleft_speed >= 0.0f && mright_speed < 0.0f)
            tract_ctrl_set_direction(TURN_RIGHT_FORWARD);
        else if (mleft_speed > 0.0f && mright_speed > 0.0f)
            tract_ctrl_set_direction(FORWARD);
        else
        {
            // printf("(%.4f, %.4f)\n", mleft_speed, mright_speed);
            //   tract_ctrl_set_direction(BRAKE);
        }
        int mleft_int = roundf(TRACT_CONV_REV2PULSES(mleft_abs));
        int mright_int = roundf(TRACT_CONV_REV2PULSES(mright_abs));
        // printf("m_lefta, %f, m_righta, %f, left_int:%d, right_int:%d   ", mleft_abs, mright_abs, mleft_int, mright_int);
        if(mleft_int > 0 && mright_int > 0)
            motor_pair_set_speed(mleft_int, mright_int, g_traction_handle);
    }
    else
    {
        ESP_LOGE(TAG, "Speed Controller: Speed set was out of bounds!");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void tract_ctrl_event_loop(void)
{
    tract_ctrl_cmd_t tract_ctrl_cmd;
    if (xQueueReceive(g_traction_cmd_queue, &tract_ctrl_cmd, pdMS_TO_TICKS(40)) == pdTRUE)
    {
        switch (tract_ctrl_cmd.cmd)
        {
        case TRACT_CTRL_CMD_STOP:
            ESP_LOGI(TAG, "Event: Stop Event");
            if (tract_ctrl_stop_event_handler() != ESP_OK)
            {
                ESP_LOGE(TAG, "Event Error: Error stopping the process");
            }
            break;
        case TRACT_CTRL_CMD_START:
            ESP_LOGI(TAG, "Event: Start Event");
            if (tract_ctrl_start_event_handler() != ESP_OK)
            {
                ESP_LOGE(TAG, "Event Error: Error starting the process");
            }
            break;
        case TRACT_CTRL_CMD_SET_SPEED:
            // ESP_LOGI(TAG, "Set speed controlled direction command received");
            esp_err_t ret = tract_ctrl_set_speed_event_handler(tract_ctrl_cmd.motor_left_speed, tract_ctrl_cmd.motor_right_speed);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Event Error: Error setting up the speed controlled direction");
            }
            break;
        default:
            ESP_LOGE(TAG, "Invalid command received");
            break;
        }
    }
}

static void tract_ctrl_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing task");

    // Initialize g_traction_handle
    g_traction_handle = (motor_pair_handle_t *)malloc(sizeof(motor_pair_handle_t));
    strcpy(g_traction_handle->id, "tract_ctrl_pair");

    // Configuration parameters
    motor_pair_config_t tract_ctrl_config = {
        .motor_left_config = (motor_config_t){
            .motor_encodera_gpio_num = TRACT_ML_ENCODER_A,
            .motor_encoderb_gpio_num = TRACT_ML_ENCODER_B,
            .motor_pwma_gpio_num = TRACT_ML_PWMA,
            .motor_pwmb_gpio_num = TRACT_ML_PWMB,
            .pwm_freq_hz = TRACT_MOTORS_PWM_FREQ,
            .motor_id = "traction_left",
            .pid_config = {
                .kp = TRACT_ML_KP,
                .kd = TRACT_ML_KD,
                .ki = TRACT_ML_KI,
            },
        },
        .motor_right_config = (motor_config_t){
            .motor_encodera_gpio_num = TRACT_MR_ENCODER_A,
            .motor_encoderb_gpio_num = TRACT_MR_ENCODER_B,
            .motor_pwma_gpio_num = TRACT_MR_PWMA,
            .motor_pwmb_gpio_num = TRACT_MR_PWMB,
            .pwm_freq_hz = TRACT_MOTORS_PWM_FREQ,
            .motor_id = "traction_right",
            .pid_config = {
                .kp = TRACT_MR_KP,
                .kd = TRACT_MR_KD,
                .ki = TRACT_MR_KI,
            },
        },
        .bdc_config = (motor_pair_bdc_config_t){
            .bdc_encoder_pcnt_high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
            .bdc_encoder_pcnt_low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
            .mcpwm_group = TRACT_MOTORS_MCPWM_GROUP,
            .pid_loop_period = BDC_PID_LOOP_PERIOD_MS,
            .bdc_mcpwm_timer_resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
        },
    };

    /* Init motor pair unit */
    ESP_ERROR_CHECK(motor_pair_init(&tract_ctrl_config, g_traction_handle));

    /* Setting up timer */
    // const esp_timer_create_args_t traction_timer_args = {
    //     .callback = &traction_pid_loop_cb,
    //     .arg = NULL,
    //     .name = "traction_pid_loop",
    //     .dispatch_method = ESP_TIMER_TASK,
    //     .skip_unhandled_events = true,
    // };

    // ESP_ERROR_CHECK(esp_timer_create(&traction_timer_args, &g_traction_pid_timer));

    // GPTimer
    gptimer_config_t gptimer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, // Use the default clock source
        .direction = GPTIMER_COUNT_UP,      // Count up
        .resolution_hz = 1000000,           // 1 MHz resolution (1 tick = 1 microsecond)
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&gptimer_config, &g_traction_pid_gptimer));

    gptimer_event_callbacks_t gptimer_callbacks = {
        .on_alarm = traction_pid_isr_cb, // Callback function
    };

    ESP_ERROR_CHECK(gptimer_register_event_callbacks(g_traction_pid_gptimer, &gptimer_callbacks, NULL));
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,                            // Start counting from 0
        .alarm_count = BDC_PID_LOOP_PERIOD_MS * 1000, // Alarm period in microseconds
        .flags.auto_reload_on_alarm = true,           // Enable auto-reload
    };

    ESP_ERROR_CHECK(gptimer_set_alarm_action(g_traction_pid_gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(g_traction_pid_gptimer));
    // Enable motors
    ESP_ERROR_CHECK(motor_pair_enable_motors(g_traction_handle));

    // Initialize traction_data
    traction_data = (motor_pair_data_t){
        .state = STOPPED,
        .mleft_pulses = 0.0f,
        .mright_pulses = 0.0f,
        .mleft_set_point = 0.0f,
        .mright_set_point = 0.0f,
    };

    // Setting up queue
    g_traction_data_queue = xQueueCreate(4, sizeof(motor_pair_data_t));
    g_traction_cmd_queue = xQueueCreate(4, sizeof(tract_ctrl_cmd_t));

    ESP_LOGI(TAG, "Starting motor speed loop");
    ESP_ERROR_CHECK(gptimer_start(g_traction_pid_gptimer));
    // esp_err_t ret = esp_timer_start_periodic(g_traction_pid_timer, BDC_PID_LOOP_PERIOD_MS * 1000);

    // Set initial speed
    float initial_speed = 0.0f;
    tract_ctrl_set_speed_event_handler(&initial_speed, &initial_speed);

    // if (ret != ESP_OK)
    //     ESP_LOGE(TAG, "Error starting pid loop timer");
    // else
    //     ESP_LOGI(TAG, "PID loop timer started correctly");

    for (;;)
    {
        // Check for new commands received
        tract_ctrl_event_loop();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void tract_speed_update_task(void *pvParameters)
{
    motor_pair_data_t traction_data;

    static motor_pair_state_e last_traction_state = STOPPED;

    static int64_t last_execution_time = 0;
    for (;;)
    {
        // Wait for data from the ISR
        if (xQueueReceive(g_traction_isr_data_queue, &traction_data, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            // printf("IN loop\n");
            int64_t current_time = esp_timer_get_time();
            int64_t time_diff = current_time - last_execution_time;
            last_execution_time = current_time;
            if (last_traction_state != g_traction_state)
            {
                last_traction_state = g_traction_state;
                traction_data.state = g_traction_state;

                switch (g_traction_state)
                {
                case BRAKE:
                    ESP_ERROR_CHECK(bdc_motor_brake(g_traction_handle->motor_left_ctx.motor));
                    ESP_ERROR_CHECK(bdc_motor_brake(g_traction_handle->motor_right_ctx.motor));
                    ESP_LOGI(TAG, "TRACT_DIR: BREAK");
                    break;
                case COAST:
                    ESP_ERROR_CHECK(bdc_motor_coast(g_traction_handle->motor_left_ctx.motor));
                    ESP_ERROR_CHECK(bdc_motor_coast(g_traction_handle->motor_right_ctx.motor));
                    ESP_LOGI(TAG, "TRACT_DIR: COAST");
                    break;
                case STARTING:
                    ESP_ERROR_CHECK(bdc_motor_forward(g_traction_handle->motor_left_ctx.motor));
                    ESP_ERROR_CHECK(bdc_motor_forward(g_traction_handle->motor_right_ctx.motor));
                    ESP_LOGI(TAG, "TRACT_DIR: STARTING");
                    break;
                case FORWARD:
                    ESP_ERROR_CHECK(bdc_motor_forward(g_traction_handle->motor_left_ctx.motor));
                    ESP_ERROR_CHECK(bdc_motor_forward(g_traction_handle->motor_right_ctx.motor));
                    ESP_LOGI(TAG, "TRACT_DIR: FORWARD");
                    break;
                case REVERSE:
                    ESP_ERROR_CHECK(bdc_motor_reverse(g_traction_handle->motor_left_ctx.motor));
                    ESP_ERROR_CHECK(bdc_motor_reverse(g_traction_handle->motor_right_ctx.motor));
                    ESP_LOGI(TAG, "TRACT_DIR: REVERSE");
                    break;
                case TURN_LEFT_FORWARD:
                    ESP_ERROR_CHECK(bdc_motor_reverse(g_traction_handle->motor_left_ctx.motor));
                    ESP_ERROR_CHECK(bdc_motor_forward(g_traction_handle->motor_right_ctx.motor));
                    ESP_LOGI(TAG, "TRACT_DIR: LEFT FORWARD");
                    break;
                case TURN_RIGHT_FORWARD:
                    ESP_ERROR_CHECK(bdc_motor_forward(g_traction_handle->motor_left_ctx.motor));
                    ESP_ERROR_CHECK(bdc_motor_reverse(g_traction_handle->motor_right_ctx.motor));
                    ESP_LOGI(TAG, "TRACT_DIR: RIGHT FORWARD");
                    break;
                case TURN_LEFT_REVERSE:
                    ESP_ERROR_CHECK(bdc_motor_forward(g_traction_handle->motor_left_ctx.motor));
                    ESP_ERROR_CHECK(bdc_motor_reverse(g_traction_handle->motor_right_ctx.motor));
                    ESP_LOGI(TAG, "TRACT_DIR: LEFT REVERSE");
                    break;
                case TURN_RIGHT_REVERSE:
                    ESP_ERROR_CHECK(bdc_motor_reverse(g_traction_handle->motor_left_ctx.motor));
                    ESP_ERROR_CHECK(bdc_motor_forward(g_traction_handle->motor_right_ctx.motor));
                    ESP_LOGI(TAG, "TRACT_DIR: RIGHT REVERSE");
                    break;
                default:
                    ESP_LOGI(TAG, "TRACT_DIR: ERRORRRRRRRRR");
                    break;
                }
            }
            // If the vehicle is in break or coast state, its not necessary to calculate the PID value
            float motor_left_new_speed = 0;
            float motor_right_new_speed = 0;
            if (last_traction_state != BRAKE && last_traction_state != COAST)
            {
                // Perform PID calculations and update motor speeds
                float motor_left_error = g_traction_handle->motor_left_ctx.desired_speed - abs(traction_data.mleft_pulses);
                float motor_right_error = g_traction_handle->motor_right_ctx.desired_speed - abs(traction_data.mright_pulses);

                pid_compute(g_traction_handle->motor_left_ctx.pid_ctrl, motor_left_error, &motor_left_new_speed);
                pid_compute(g_traction_handle->motor_right_ctx.pid_ctrl, motor_right_error, &motor_right_new_speed);

                bdc_motor_set_speed(g_traction_handle->motor_left_ctx.motor, (uint32_t)motor_left_new_speed);
                bdc_motor_set_speed(g_traction_handle->motor_right_ctx.motor, (uint32_t)motor_right_new_speed);
            }

            // Save information
            if (g_traction_state == REVERSE || g_traction_state == TURN_LEFT_FORWARD || g_traction_state == TURN_RIGHT_REVERSE)
                traction_data.mleft_set_point = (-1) * g_traction_handle->motor_left_ctx.desired_speed;
            else
                traction_data.mleft_set_point = g_traction_handle->motor_left_ctx.desired_speed;

            if (g_traction_state == REVERSE || g_traction_state == TURN_RIGHT_FORWARD || g_traction_state == TURN_LEFT_REVERSE)
                traction_data.mright_set_point = (-1) * g_traction_handle->motor_right_ctx.desired_speed;
            else
                traction_data.mright_set_point = g_traction_handle->motor_right_ctx.desired_speed;

            // Send data to the queue
            tract_ctrl_send2data_queue(&traction_data);

            // printf("/*%d,%d,%d,%d,%.4f,%.4f,%llu*/\n", traction_data.mleft_set_point, traction_data.mleft_pulses, traction_data.mright_set_point, traction_data.mright_pulses, motor_left_new_speed, motor_right_new_speed, time_diff / 1000);
        }
    }
}

void tract_ctrl_start_task(void)
{
    ESP_LOGI(TAG, "Starting tract_ctrl task");

    g_traction_isr_data_queue = xQueueCreate(5, sizeof(motor_pair_data_t));

    xTaskCreatePinnedToCore(&tract_ctrl_task,
                            "tract_ctrl",
                            TRACT_CONTROL_STACK_SIZE,
                            NULL,
                            TRACT_CONTROL_TASK_PRIORITY,
                            NULL,
                            TRACT_CONTROL_CORE_ID);
    xTaskCreatePinnedToCore(&tract_speed_update_task,
                            "tract_update", 2048, NULL, 15, NULL, 0);
}
