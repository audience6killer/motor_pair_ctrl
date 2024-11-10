#include "stdio.h"
#include "math.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "stdbool.h"

#include "traction_control.h"
#include "traction_task_common.h"
#include "motor_pair_ctrl.h"

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

static const char TAG[] = "traction_control";

static motor_pair_handle_t *traction_handle = NULL;
static QueueHandle_t traction_queue_handle;
static motor_pair_state_e g_traction_state = BRAKE;
static motor_pair_data_t traction_data;

static bool soft_start_traction_active = false;

static void traction_pid_loop_cb(void *args)
{
    static int motor_left_last_pulse_count = 0;
    static int motor_right_last_pulse_count = 0;
    static motor_pair_state_e last_traction_state = BRAKE;

    // Where will be saved the current info for the motor pair
    // motor_pair_data_t *pair_data = (motor_pair_data_t *)args;

    // Calculate current speed
    int motor_left_cur_pulse_count = 0;
    int motor_right_cur_pulse_count = 0;
    pcnt_unit_get_count(traction_handle->motor_left_ctx.pcnt_encoder, &motor_left_cur_pulse_count);
    pcnt_unit_get_count(traction_handle->motor_right_ctx.pcnt_encoder, &motor_right_cur_pulse_count);

    // The sign of the speed doesn't matter, as the forward and reverse of the motor will control the direction of the spin

    int motor_left_real_pulses = abs(motor_left_cur_pulse_count - motor_left_last_pulse_count);
    int motor_right_real_pulses = abs(motor_right_cur_pulse_count - motor_right_last_pulse_count);

    motor_right_last_pulse_count = motor_right_cur_pulse_count;
    motor_left_last_pulse_count = motor_left_cur_pulse_count;

    traction_handle->motor_right_ctx.report_pulses = motor_right_real_pulses;
    traction_handle->motor_left_ctx.report_pulses = motor_left_real_pulses;

    // Check whether the state has changed

    if (last_traction_state != g_traction_state)
    {
        last_traction_state = g_traction_state;

        switch (g_traction_state)
        {
        case BRAKE:
            ESP_ERROR_CHECK(bdc_motor_brake(traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_brake(traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "BREAK");
            break;
        case COAST:
            // TODO: Reset or not the desired speeds?
            ESP_ERROR_CHECK(bdc_motor_coast(traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_coast(traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "COAST");
            break;
        case FORWARD:
            ESP_ERROR_CHECK(bdc_motor_forward(traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_forward(traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "TRACTION_CONTROL_STATE: FORWARD");
            break;
        case REVERSE:
            ESP_ERROR_CHECK(bdc_motor_reverse(traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_reverse(traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "REVERSE");
            break;
        case TURN_LEFT_FORWARD:
            ESP_ERROR_CHECK(bdc_motor_reverse(traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_forward(traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "LEFT FORWARD");
            break;
        case TURN_RIGHT_FORWARD:
            ESP_ERROR_CHECK(bdc_motor_forward(traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_reverse(traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "RIGHT FORWARD");
            break;
        case TURN_LEFT_REVERSE:
            ESP_ERROR_CHECK(bdc_motor_forward(traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_reverse(traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "LEFT REVERSE");
            break;
        case TURN_RIGHT_REVERSE:
            ESP_ERROR_CHECK(bdc_motor_reverse(traction_handle->motor_left_ctx.motor));
            ESP_ERROR_CHECK(bdc_motor_forward(traction_handle->motor_right_ctx.motor));
            ESP_LOGI(TAG, "RIGHT REVERSE");
            break;
        default:
            ESP_LOGI(TAG, "ERRORRRRRRRRR");
            break;
        }
    }

    // If the vehicle is in break or coast state, the speed PID should not interfere
    if (last_traction_state == BRAKE || last_traction_state == COAST)
        return;

    // Check wheater there is another speed value in the queue
    if (!is_queue_empty(traction_handle->motor_left_ctx.speed_queue) && !is_queue_empty(traction_handle->motor_right_ctx.speed_queue))
    {
        traction_control_set_speed(dequeue(traction_handle->motor_left_ctx.speed_queue), dequeue(traction_handle->motor_right_ctx.speed_queue));
        //traction_handle->motor_left_ctx.desired_speed = (int)floor( dequeue(traction_handle->motor_left_ctx.speed_queue) * TRACTION_M_LEFT_REV2PULSES );
        //ESP_LOGI(TAG, "PID LOOP: Left Motor DS: %d", traction_handle->motor_left_ctx.desired_speed);
        //ESP_LOGI(TAG, "PID LOOP: Right Motor DS: %d", traction_handle->motor_right_ctx.desired_speed);
    }
    //if (is_queue_empty(traction_handle->motor_right_ctx.speed_queue) == 0)
    //{
    //    traction_handle->motor_right_ctx.desired_speed = dequeue(traction_handle->motor_right_ctx.speed_queue);
    //    ESP_LOGI(TAG, "PID LOOP: Right Motor DS: %d", traction_handle->motor_right_ctx.desired_speed);
    //}

    // Calculate speed error
    float motor_left_error = traction_handle->motor_left_ctx.desired_speed - motor_left_real_pulses;
    float motor_right_error = traction_handle->motor_right_ctx.desired_speed - motor_right_real_pulses;

    float motor_right_new_speed = 0;
    float motor_left_new_speed = 0;

    // Set the new speed
    pid_compute(traction_handle->motor_right_ctx.pid_ctrl, motor_right_error, &motor_right_new_speed);
    pid_compute(traction_handle->motor_left_ctx.pid_ctrl, motor_left_error, &motor_left_new_speed);

    ESP_ERROR_CHECK(bdc_motor_set_speed(traction_handle->motor_right_ctx.motor, (uint32_t)motor_right_new_speed));
    ESP_ERROR_CHECK(bdc_motor_set_speed(traction_handle->motor_left_ctx.motor, (uint32_t)motor_left_new_speed));
    // bdc_motor_forward(traction_handle->motor_right_ctx.motor);
    //  ESP_ERROR_CHECK(bdc_motor_set_speed(traction_handle->motor_right_ctx.motor, 32));

    // Save information
    traction_data.motor_left_error = motor_left_error;
    traction_data.motor_right_error = motor_right_error;
    traction_data.motor_left_real_pulses = motor_left_real_pulses;
    traction_data.motor_right_real_pulses = motor_right_real_pulses;
    traction_data.motor_left_desired_speed = traction_handle->motor_left_ctx.desired_speed;
    traction_data.motor_right_desired_speed = traction_handle->motor_right_ctx.desired_speed;
    // ESP_LOGI(TAG, "left_new_speed: %f, right_new_speed: %f", motor_left_new_speed, motor_right_new_speed);
    // ESP_LOGI(TAG, "INSIDE LOOP");
}

esp_err_t traction_control_set_direction(const motor_pair_state_e state)
{
    // TODO: Some kind of verification might be necessary
    g_traction_state = state;

    return ESP_OK;
}

/**
 * @brief Set the speed for both motors. Both in rad/s
 *
 * @param motor_left_speed
 * @param motor_right_speed
 * @return esp_err_t
 */
esp_err_t traction_control_set_speed(float motor_left_speed, float motor_right_speed)
{
    if (motor_left_speed <= TRACTION_M_LEFT_MAX_SPEED_REVS && motor_right_speed <= TRACTION_M_RIGHT_MAX_SPEED_REVS)
    {
        int pulses_left_speed = (int)(motor_left_speed * TRACTION_M_LEFT_REV2PULSES);
        int pulses_right_speed = (int)(motor_right_speed * TRACTION_M_RIGHT_REV2PULSES);
        ESP_ERROR_CHECK(motor_pair_set_speed(pulses_left_speed, pulses_right_speed, traction_handle));
        //ESP_LOGI(TAG, "Speed set left: %f, Speed set right: %f", motor_left_speed, motor_right_speed);
    }
    else
    {
        ESP_LOGE(TAG, "Speed set was out of bounds!");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void traction_control_task(void *pvParameters)
{
    // Initialize traction_handle
    traction_handle = (motor_pair_handle_t *)malloc(sizeof(motor_pair_handle_t));

    motor_config_t traction_left_config = {
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
    };
    motor_config_t traction_right_config = {
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
    };

    motor_pair_bdc_config_t traction_bdc_conf = {
        .bdc_encoder_pcnt_high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .bdc_encoder_pcnt_low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .mcpwm_group = TRACTION_MOTORS_MCPWM_GROUP,
        .pwm_freq_hz = TRACTION_MOTORS_PWM_FREQ,
        .pid_loop_period = BDC_PID_LOOP_PERIOD_MS,
        .bdc_mcpwm_timer_resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };

    motor_pair_config_t traction_control_config = {
        .motor_left_config = traction_left_config,
        .motor_right_config = traction_right_config,
        .bdc_config = traction_bdc_conf,
    };

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

    ESP_LOGI(TAG, "Enabling motor: %s", traction_handle->motor_left_ctx.motor_id);
    ESP_ERROR_CHECK(bdc_motor_enable(traction_handle->motor_left_ctx.motor));

    ESP_LOGI(TAG, "Enabling motor: %s", traction_handle->motor_right_ctx.motor_id);
    ESP_ERROR_CHECK(bdc_motor_enable(traction_handle->motor_right_ctx.motor));

    ESP_ERROR_CHECK(bdc_motor_brake(traction_handle->motor_left_ctx.motor));
    ESP_ERROR_CHECK(bdc_motor_brake(traction_handle->motor_right_ctx.motor));

    ESP_LOGI(TAG, "Starting motor speed loop");
    ESP_ERROR_CHECK(esp_timer_start_periodic(traction_pid_loop_timer, traction_bdc_conf.pid_loop_period * 1000));

    // Set initial speed
    float initial_speed = 0.0f;
    traction_control_set_speed(initial_speed, initial_speed);
    // Setting up queue
    traction_queue_handle = xQueueCreate(4, sizeof(motor_pair_data_t));

    for (;;)
    {
        // #if SERIAL_DEBUG_ENABLE
        // printf()
        printf("/*left_desired_speed,%d,speed_left,%d,right_des_speed,%d, speed_right,%d*/\r\n", traction_data.motor_left_desired_speed, traction_data.motor_left_real_pulses, traction_data.motor_right_desired_speed, traction_data.motor_right_real_pulses);
        // #endif
        // ESP_LOGI(TAG, "exec task!!!");
        // if (xQueueSend(traction_queue_handle, &traction_data, portMAX_DELAY) != pdPASS)
        //{
        //     ESP_LOGE(TAG, "Error sending data to queue");
        // }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t traction_control_smooth_start(float target_speed)
{
    if (traction_handle == NULL)
        return ESP_FAIL;

    traction_control_set_direction(FORWARD);

    ESP_LOGI(TAG, "Beginning smooth start");
    ESP_ERROR_CHECK(motor_pair_smooth_start(traction_handle, target_speed));

    return ESP_OK;
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
