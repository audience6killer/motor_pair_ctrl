#include "stdio.h"
#include "math.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "motor_pair_ctrl.h"
#include "seed_planter_control.h"
#include "seed_planter_task_common.h"

static const char TAG[] = "seed_planter_control";

static motor_pair_handle_t *seed_planter_handle = NULL;

static QueueHandle_t seed_planter_queue_handle;

static motor_pair_state_e g_cutter_disc_state = BREAK;

static motor_pair_state_e g_seed_dispenser_state = BREAK;



static void seed_planter_pid_loop_cb(void *args)
{
    static int cutter_disc_last_pulse_count = 0;
    static int seed_dispenser_last_pulse_count = 0;
    static motor_pair_state_e last_cutter_disc_state = BREAK;
    static motor_pair_state_e last_seed_dispenser_state = BREAK;

    // Where will be saved the current info for the motor pair
    motor_pair_data_t *pair_data = (motor_pair_data_t *)args;

    motor_control_context_t *cutter_disc = &seed_planter_handle->motor_left_ctx;
    motor_control_context_t *seed_dispenser = &seed_planter_handle->motor_right_ctx;

    // Calculate current speed
    int cutter_disc_cur_pulse_count = 0;
    int seed_dispenser_cur_pulse_count = 0;
    pcnt_unit_get_count(cutter_disc->pcnt_encoder, &cutter_disc_cur_pulse_count);
    pcnt_unit_get_count(seed_dispenser->pcnt_encoder, &seed_dispenser_cur_pulse_count);

    // The sign of the speed doesn't matter, as the forward and reverse of the motor will control the direction of the spin
    int cutter_disc_real_pulses = abs(cutter_disc_cur_pulse_count - cutter_disc_last_pulse_count);
    int seed_dispenser_real_pulses = abs(seed_dispenser_cur_pulse_count - seed_dispenser_last_pulse_count);

    seed_dispenser_last_pulse_count = seed_dispenser_cur_pulse_count;
    cutter_disc_last_pulse_count = cutter_disc_cur_pulse_count;

    seed_dispenser->report_pulses = seed_dispenser_real_pulses;
    cutter_disc->report_pulses = cutter_disc_real_pulses;

    // Check whether the state for the cutter disc has changed
    if (last_cutter_disc_state != g_cutter_disc_state)
    {
        last_cutter_disc_state = g_cutter_disc_state;

        switch (g_cutter_disc_state)
        {
        case BREAK:
            ESP_ERROR_CHECK(bdc_motor_brake(cutter_disc->motor));
            ESP_LOGI(TAG, "Cutter disc break");
            break;
        case COAST:
            // TODO: Reset or not the desired speeds?
            ESP_ERROR_CHECK(bdc_motor_coast(cutter_disc->motor));
            ESP_LOGI(TAG, "Cutter disc coast");
            break;
        case FORWARD:
            ESP_ERROR_CHECK(bdc_motor_forward(cutter_disc->motor));
            ESP_LOGI(TAG, "Cutter disk forward");
            break;
        case REVERSE:
            ESP_ERROR_CHECK(bdc_motor_reverse(cutter_disc->motor));
            ESP_LOGI(TAG, "Cutter disc reverse");
            break;
        default:
            ESP_LOGI(TAG, "Cutter disc unknown state");
            break;
        }
    }
    // Check whether the state for the seed dispenser has changed
    if (last_seed_dispenser_state != g_seed_dispenser_state)
    {
        last_seed_dispenser_state = g_seed_dispenser_state;

        switch (g_seed_dispenser_state)
        {
        case BREAK:
            ESP_ERROR_CHECK(bdc_motor_brake(seed_dispenser->motor));
            ESP_LOGI(TAG, "Seed dispenser break");
            break;
        case COAST:
            // TODO: Reset or not the desired speeds?
            ESP_ERROR_CHECK(bdc_motor_coast(seed_dispenser->motor));
            ESP_LOGI(TAG, "Seed dispenser coast");
            break;
        case FORWARD:
            ESP_ERROR_CHECK(bdc_motor_forward(seed_dispenser->motor));
            ESP_LOGI(TAG, "Seed dispenser forward");
            break;
        case REVERSE:
            ESP_ERROR_CHECK(bdc_motor_reverse(seed_dispenser->motor));
            ESP_LOGI(TAG, "Seed dispenser reverse");
            break;
        default:
            ESP_LOGI(TAG, "Seed dispenser unknown state");
            break;
        }
    }
    // Calculate speed error
    float cutter_disc_error = cutter_disc->desired_speed - cutter_disc_real_pulses;
    float seed_dispenser_error = seed_dispenser->desired_speed - seed_dispenser_real_pulses;

    float seed_dispenser_new_speed = 0.0f;
    float cutter_disc_new_speed = 0.0f;

    // If the vehicle is in break or coast state, the speed PID should not interfere
    if (last_cutter_disc_state != BREAK && last_cutter_disc_state != COAST)
    {
        pid_compute(cutter_disc->pid_ctrl, cutter_disc_error, &cutter_disc_new_speed);
        bdc_motor_set_speed(cutter_disc->motor, (uint32_t)cutter_disc_new_speed);
    }
    if (last_seed_dispenser_state != BREAK && last_seed_dispenser_state != COAST)
    {
        pid_compute(seed_dispenser->pid_ctrl, seed_dispenser_error, &seed_dispenser_new_speed);
        bdc_motor_set_speed(seed_dispenser->motor, (uint32_t)seed_dispenser_new_speed);
    }

    // Save information: cutter - left
    pair_data->motor_left_error = cutter_disc_error;
    pair_data->motor_right_error = seed_dispenser_error;
    pair_data->motor_left_real_pulses = cutter_disc_real_pulses;
    pair_data->motor_right_real_pulses = seed_dispenser_real_pulses;
    pair_data->motor_left_desired_speed = cutter_disc->desired_speed;
    pair_data->motor_right_desired_speed = seed_dispenser->desired_speed;
}

static void seed_planter_control_task(void *pvParameters)
{
    seed_planter_handle = (motor_pair_handle_t *)malloc(sizeof(motor_pair_handle_t));

    motor_config_t cutter_disc_config = {
        .motor_encodera_gpio_num = SEED_PLANTER_CUTTER_DISC_ENCODER_A,
        .motor_encoderb_gpio_num = SEED_PLANTER_CUTTER_DISC_ENCODER_B,
        .motor_pwma_gpio_num = SEED_PLANTER_CUTTER_DISC_PWMA,
        .motor_pwmb_gpio_num = SEED_PLANTER_CUTTER_DISC_PWMB,
        .pwm_freq_hz = SEED_PLANTER_MOTORS_PWM_FREQ,
        .motor_id = "cutter_disc",
        .pid_config = {
            .kp = SEED_PLANTER_CUTTER_DISC_KP,
            .kd = SEED_PLANTER_CUTTER_DISC_KD,
            .ki = SEED_PLANTER_CUTTER_DISC_KI,
        },
    };
    motor_config_t seed_dispenser_config = {
        .motor_encodera_gpio_num = SEED_PLANTER_SEED_DISPENSER_ENCODER_A,
        .motor_encoderb_gpio_num = SEED_PLANTER_SEED_DISPENSER_ENCODER_B,
        .motor_pwma_gpio_num = SEED_PLANTER_SEED_DISPENSER_PWMA,
        .motor_pwmb_gpio_num = SEED_PLANTER_SEED_DISPENSER_PWMB,
        .pwm_freq_hz = SEED_PLANTER_MOTORS_PWM_FREQ,
        .motor_id = "seed_dispenser",
        .pid_config = {
            .kp = SEED_PLANTER_SEED_DISPENSER_KP,
            .kd = SEED_PLANTER_SEED_DISPENSER_KD,
            .ki = SEED_PLANTER_SEED_DISPENSER_KI,
        },
    };

    // TODO: Check why there is a pwm_freq in both config structs
    motor_pair_bdc_config_t seed_planter_bdc_conf = {
        .bdc_encoder_pcnt_high_limit = SEED_PLANTER_BDC_ENCODER_PCNT_HIGH_LIMIT,
        .bdc_encoder_pcnt_low_limit = SEED_PLANTER_BDC_ENCODER_PCNT_LOW_LIMIT,
        .mcpwm_group = SEED_PLANTER_MCPWM_GROUP,
        .pwm_freq_hz = SEED_PLANTER_MOTORS_PWM_FREQ,
        .pid_loop_period = SEED_PLANTER_PID_LOOP_PERIDO_MS,
    };

    motor_pair_config_t seed_planter_config = {
        .motor_left_config = cutter_disc_config,
        .motor_right_config = seed_dispenser_config,
        .bdc_config = seed_planter_bdc_conf,
    };

    ESP_ERROR_CHECK(motor_pair_init(&seed_planter_config, seed_planter_handle));

    // Setting up timer
    // TODO: Should this be global?
    motor_pair_data_t seed_planter_data;
    const esp_timer_create_args_t seed_planter_timer_args = {
        .callback = seed_planter_pid_loop_cb,
        .arg = &seed_planter_data,
        .name = "seed_planter_loop",
    };

    esp_timer_handle_t seed_planter_pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&seed_planter_timer_args, &seed_planter_pid_loop_timer));

    ESP_LOGI(TAG, "Starting seed planter motor speed loop");
    ESP_ERROR_CHECK(esp_timer_start_periodic(seed_planter_pid_loop_timer, seed_planter_bdc_conf.pid_loop_period * 1000));

    // Setting up queue
    seed_planter_queue_handle = xQueueCreate(4, sizeof(motor_pair_data_t));

    for (;;)
    {
        if (xQueueSend(seed_planter_queue_handle, &seed_planter_data, portMAX_DELAY) != pdPASS)
        {
            ESP_LOGE(TAG, "Error sending data to queue");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void seed_planter_control_start_task(void)
{
    ESP_LOGI(TAG, "Initializing seed_planter task");

    xTaskCreatePinnedToCore(&seed_planter_control_start_task,
                            "seed_planter_control",
                            SEED_PLANTER_STACK_SIZE,
                            NULL,
                            SEED_PLANTER_TASK_PRIORITY,
                            NULL,
                            SEED_PLANTER_CORE_ID);
}
