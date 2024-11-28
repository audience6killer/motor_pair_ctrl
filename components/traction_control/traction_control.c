#include "stdio.h"
#include "math.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "stdbool.h"
#include "string.h"

#include "traction_control.h"
#include "traction_task_common.h"
#include "motor_pair_ctrl.h"

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

static const char TAG[] = "traction_control";

static motor_pair_handle_t *traction_handle = NULL;
static QueueHandle_t traction_queue_handle;
static motor_pair_state_e g_traction_state = STOPPED;
static motor_pair_data_t traction_data;

static bool g_soft_start_traction_active = false;
static float g_soft_start_target_speed = 0.0f;
static int g_soft_start_tf = 0;

static void traction_pid_loop_cb(void *args)
{
    static int motor_left_last_pulse_count = 0;
    static int motor_right_last_pulse_count = 0;
    static motor_pair_state_e last_traction_state = STOPPED;
    static int soft_start_counter = 0;

    // const unsigned int MEASUREMENTS = 500;

    // Calculate current speed
    int motor_left_cur_pulse_count = 0;
    int motor_right_cur_pulse_count = 0;
    pcnt_unit_get_count(traction_handle->motor_left_ctx.pcnt_encoder, &motor_left_cur_pulse_count);
    pcnt_unit_get_count(traction_handle->motor_right_ctx.pcnt_encoder, &motor_right_cur_pulse_count);

    // The sign of the speed doesn't matter, as the forward and reverse of the motor will control the direction
    int motor_left_real_pulses = motor_left_cur_pulse_count - motor_left_last_pulse_count;
    int motor_right_real_pulses = motor_right_cur_pulse_count - motor_right_last_pulse_count;

    int motor_left_abs_pulses = abs(motor_left_real_pulses);
    int motor_right_abs_pulses = abs(motor_right_real_pulses);

    // Save the real value of the speed
    traction_data.mleft_real_pulses = motor_left_real_pulses;
    traction_data.mright_real_pulses = motor_right_real_pulses;

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
            ESP_ERROR_CHECK(bdc_motor_brake(traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_brake(traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACTION_CONTROL_STATE: BREAK");
            break;
        case COAST:
            // TODO: Reset or not the desired speeds?
            ESP_ERROR_CHECK(bdc_motor_coast(traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_coast(traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACTION_CONTROL_STATE: COAST");
            break;
        case STARTING:
            ESP_ERROR_CHECK(bdc_motor_forward(traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_forward(traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACTION_CONTROL_STATE: STARTING");
            g_soft_start_traction_active = true;
            break;
        case FORWARD:
            ESP_ERROR_CHECK(bdc_motor_forward(traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_forward(traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACTION_CONTROL_STATE: FORWARD");
            break;
        case REVERSE:
            ESP_ERROR_CHECK(bdc_motor_reverse(traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_reverse(traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACTION_CONTROL_STATE: REVERSE");
            break;
        case TURN_LEFT_FORWARD:
            ESP_ERROR_CHECK(bdc_motor_reverse(traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_forward(traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACTION_CONTROL_STATE: LEFT FORWARD");
            break;
        case TURN_RIGHT_FORWARD:
            ESP_ERROR_CHECK(bdc_motor_forward(traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_reverse(traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACTION_CONTROL_STATE: RIGHT FORWARD");
            break;
        case TURN_LEFT_REVERSE:
            ESP_ERROR_CHECK(bdc_motor_forward(traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_reverse(traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACTION_CONTROL_STATE: LEFT REVERSE");
            break;
        case TURN_RIGHT_REVERSE:
            ESP_ERROR_CHECK(bdc_motor_reverse(traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_forward(traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACTION_CONTROL_STATE: RIGHT REVERSE");
            break;
        default:
            ESP_LOGI(TAG, "TRACTION_CONTROL_STATE: ERRORRRRRRRRR");
            break;
        }
    }

    if (g_soft_start_traction_active)
    {
        // During the soft start the queue will be ignored
        float point = 0.0f;
        // float target_speed = (float)traction_handle->motor_left_ctx.desired_speed / TRACTION_M_LEFT_REV2PULSES;
        soft_start_counter++;

        calculate_lspb_speed_point(g_soft_start_tf, soft_start_counter, g_soft_start_target_speed, &point);
        // traction_control_speed_controlled_direction(point, point);
        motor_pair_set_speed((int)roundf(TRACTION_ML_REV2PULSES(point)),
                             (int)roundf(TRACTION_MR_REV2PULSES(point)),
                             traction_handle);

        // ESP_LOGI(TAG, "soft start point: %f", point);
        if (soft_start_counter == g_soft_start_tf)
        {
            soft_start_counter = 0;
            g_soft_start_traction_active = false;
            traction_control_set_direction(FORWARD);
        }
    }

    // If the vehicle is in break or coast state, its not necessary to calculate the PID value
    if (last_traction_state != BRAKE && last_traction_state != COAST)
    {
        // Calculate speed error
        float motor_left_error = traction_handle->motor_left_ctx.desired_speed - motor_left_abs_pulses;
        float motor_right_error = traction_handle->motor_right_ctx.desired_speed - motor_right_abs_pulses;

        float motor_right_new_speed = 0;
        float motor_left_new_speed = 0;

        // Set the new speed
        pid_compute(traction_handle->motor_right_ctx.pid_ctrl, motor_right_error, &motor_right_new_speed);
        pid_compute(traction_handle->motor_left_ctx.pid_ctrl, motor_left_error, &motor_left_new_speed);

        ESP_ERROR_CHECK(bdc_motor_set_speed(traction_handle->motor_right_ctx.motor, (uint32_t)motor_right_new_speed));
        ESP_ERROR_CHECK(bdc_motor_set_speed(traction_handle->motor_left_ctx.motor, (uint32_t)motor_left_new_speed));
    }

    // Save information
    if (g_traction_state == REVERSE || g_traction_state == TURN_LEFT_FORWARD || g_traction_state == TURN_RIGHT_REVERSE)
        traction_data.mleft_set_point = (-1) * traction_handle->motor_left_ctx.desired_speed;
    else
        traction_data.mleft_set_point = traction_handle->motor_left_ctx.desired_speed;

    if (g_traction_state == REVERSE || g_traction_state == TURN_RIGHT_FORWARD || g_traction_state == TURN_LEFT_REVERSE)
        traction_data.mright_set_point = (-1) * traction_handle->motor_right_ctx.desired_speed;
    else
        traction_data.mright_set_point = traction_handle->motor_right_ctx.desired_speed;

    // Send new data to queue
    //uint64_t start = esp_timer_get_time();
    if (xQueueSend(traction_queue_handle, &traction_data, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending data to queue");
    }

    //uint64_t end = esp_timer_get_time();

    //const unsigned int MEASUREMENTS = 500;
    //printf("%u iterations took %llu milliseconds (%llu microseconds per invocation)\n",
    //       MEASUREMENTS, (end - start) / 1000, (end - start) / MEASUREMENTS);
}

esp_err_t traction_control_set_direction(const motor_pair_state_e state)
{
    // TODO: Some kind of verification might be necessary
    g_traction_state = state;
    return ESP_OK;
}

esp_err_t traction_control_speed_controlled_direction(float motor_left_speed, float motor_right_speed)
{
    float mleft_abs = fabs(motor_left_speed);
    float mright_abs = fabs(motor_right_speed);

    if (mleft_abs <= TRACTION_M_RIGHT_MAX_SPEED_REVS && mright_abs <= TRACTION_M_RIGHT_MAX_SPEED_REVS)
    {
        if (motor_left_speed < 0.0f && motor_right_speed < 0.0f)
            traction_control_set_direction(REVERSE);
        else if (motor_left_speed < 0.0f && motor_right_speed >= 0.0f)
            traction_control_set_direction(TURN_LEFT_FORWARD);
        else if (motor_left_speed >= 0.0f && motor_right_speed < 0.0f)
            traction_control_set_direction(TURN_RIGHT_FORWARD);
        else if (motor_left_speed > 0.0f && motor_right_speed > 0.0f)
            traction_control_set_direction(FORWARD);
        else
            traction_control_set_direction(BRAKE);

        // motor_pair_add_speed_to_queue(motor_left_abs, motor_right_abs, traction_handle);
        motor_pair_set_speed((int)roundf(TRACTION_ML_REV2PULSES(mleft_abs)), (int)roundf(TRACTION_MR_REV2PULSES(mright_abs)), traction_handle);
    }
    else
    {
        ESP_LOGE(TAG, "Speed Controller: Speed set was out of bounds!");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void traction_control_task(void *pvParameters)
{
    // Initialize traction_handle
    traction_handle = (motor_pair_handle_t *)malloc(sizeof(motor_pair_handle_t));
    strcpy(traction_handle->id, "traction_control_pair");

    // Configuration parameters
    motor_pair_config_t traction_control_config = {
        .motor_left_config = (motor_config_t){
            .motor_encodera_gpio_num = TRACTION_MOTOR_LEFT_ENCODER_A,
            .motor_encoderb_gpio_num = TRACTION_MOTOR_LEFT_ENCODER_B,
            .motor_pwma_gpio_num = TRACTION_MOTOR_LEFT_PWMA,
            .motor_pwmb_gpio_num = TRACTION_MOTOR_LEFT_PWMB,
            .pwm_freq_hz = TRACTION_MOTORS_PWM_FREQ,
            .motor_id = "traction_left",
            .pid_config = {
                .kp = TRACTION_MOTOR_LEFT_KP,
                .kd = TRACTION_MOTOR_LEFT_KD,
                .ki = TRACTION_MOTOR_LEFT_KI,
            },
        },
        .motor_right_config = (motor_config_t){
            .motor_encodera_gpio_num = TRACTION_MOTOR_RIGHT_ENCODER_A,
            .motor_encoderb_gpio_num = TRACTION_MOTOR_RIGHT_ENCODER_B,
            .motor_pwma_gpio_num = TRACTION_MOTOR_RIGHT_PWMA,
            .motor_pwmb_gpio_num = TRACTION_MOTOR_RIGHT_PWMB,
            .pwm_freq_hz = TRACTION_MOTORS_PWM_FREQ,
            .motor_id = "traction_right",
            .pid_config = {
                .kp = TRACTION_MOTOR_RIGHT_KP,
                .kd = TRACTION_MOTOR_RIGHT_KD,
                .ki = TRACTION_MOTOR_RIGHT_KI,
            },
        },
        .bdc_config = (motor_pair_bdc_config_t){
            .bdc_encoder_pcnt_high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
            .bdc_encoder_pcnt_low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
            .mcpwm_group = TRACTION_MOTORS_MCPWM_GROUP,
            .pid_loop_period = BDC_PID_LOOP_PERIOD_MS,
            .bdc_mcpwm_timer_resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
        },
    };

    // Init motor pair unit
    ESP_ERROR_CHECK(motor_pair_init(&traction_control_config, traction_handle));

    // Setting up timer
    // motor_pair_data_t traction_data;
    const esp_timer_create_args_t traction_timer_args = {
        .callback = traction_pid_loop_cb,
        .arg = NULL,
        .name = "traction_pid_loop",
    };

    esp_timer_handle_t traction_pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&traction_timer_args, &traction_pid_loop_timer));

    // Enable motors
    ESP_ERROR_CHECK(motor_pair_enable_motors(traction_handle));

    // Intinialize traction_data
    traction_data = (motor_pair_data_t){
        .state = STOPPED,
        .mleft_real_pulses = 0.0f,
        .mright_real_pulses = 0.0f,
        .mleft_set_point = 0.0f,
        .mright_set_point = 0.0f,
    };

    // Set initial speed
    float initial_speed = 0.0f;
    traction_control_speed_controlled_direction(initial_speed, initial_speed);
    // Setting up queue
    traction_queue_handle = xQueueCreate(4, sizeof(motor_pair_data_t));

    // Starting pid loop
    ESP_LOGI(TAG, "Starting motor speed loop");
    ESP_ERROR_CHECK(esp_timer_start_periodic(traction_pid_loop_timer, BDC_PID_LOOP_PERIOD_MS * 1000));

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t traction_control_soft_start(float target_speed, int tf)
{
    if (traction_handle == NULL)
        return ESP_FAIL;

    ESP_LOGI(TAG, "Starting soft start");
    // g_soft_start_traction_active = true;
    traction_control_set_direction(STARTING);
    g_soft_start_target_speed = target_speed;
    g_soft_start_tf = tf;

    return ESP_OK;
}

QueueHandle_t traction_control_get_queue_handle(void)
{
    return traction_queue_handle;
}

bool traction_control_is_busy(void)
{
    return g_soft_start_traction_active;
}

void traction_control_start_task(void)
{
    ESP_LOGI(TAG, "Initializing traction_control task");

    xTaskCreatePinnedToCore(&traction_control_task,
                            "traction_control",
                            TRACTION_CONTROL_STACK_SIZE,
                            NULL,
                            TRACTION_CONTROL_TASK_PRIORITY,
                            NULL,
                            TRACTION_CONTROL_CORE_ID);
}
