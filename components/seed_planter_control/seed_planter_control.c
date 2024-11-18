#include "stdio.h"
#include "math.h"
#include "string.h"
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
static seed_planter_state_e g_seed_planter_current_state = STOPPED;
static motor_pair_data_t seed_planter_data;

static bool g_soft_start_seed_planter_active = false;
static float g_soft_start_cutter_disc_ts = 0.0f;
static float g_soft_start_seed_dispenser_ts = 0.0f;
static int g_soft_start_tf = 0;

static void seed_planter_pid_loop_cb(void *args)
{
    static int cutter_disc_last_pulse_count = 0;
    static int seed_dispenser_last_pulse_count = 0;
    static seed_planter_state_e last_seed_planter_state = STOPPED;
    static int soft_start_counter = 0;

    motor_control_context_t *cutter_disc = &seed_planter_handle->motor_left_ctx;
    motor_control_context_t *seed_dispenser = &seed_planter_handle->motor_right_ctx;

    // Calculate current speed
    int cutter_disc_cur_pulse_count = 0;
    int seed_dispenser_cur_pulse_count = 0;
    pcnt_unit_get_count(cutter_disc->pcnt_encoder, &cutter_disc_cur_pulse_count);
    pcnt_unit_get_count(seed_dispenser->pcnt_encoder, &seed_dispenser_cur_pulse_count);

    // The sign of the speed doesn't matter, as the forward and reverse of the motor will control the direction of the spin
    int cutter_disc_real_pulses = cutter_disc_cur_pulse_count - cutter_disc_last_pulse_count;
    int seed_dispenser_real_pulses = seed_dispenser_cur_pulse_count - seed_dispenser_last_pulse_count;

    int cutter_disc_abs_pulses = abs(cutter_disc_real_pulses);
    int seed_dispenser_abs_pulses = abs(seed_dispenser_real_pulses);

    // Save real pulse count
    seed_planter_data.motor_left_current_speed = (cutter_disc_real_pulses) * CUTTER_DISC_PULSES2REV;
    seed_planter_data.motor_right_current_speed = (seed_dispenser_real_pulses) * SEED_DISPENSER_PULSES2REV;

    seed_dispenser_last_pulse_count = seed_dispenser_cur_pulse_count;
    cutter_disc_last_pulse_count = cutter_disc_cur_pulse_count;


    // Check whether the state for the seed dispenser has changed
    if (last_seed_planter_state != g_seed_planter_current_state)
    {
        last_seed_planter_state = g_seed_planter_current_state;
        seed_planter_data.state = g_seed_planter_current_state;

        switch (g_seed_planter_current_state)
        {
        case SP_STOPPED:
            ESP_ERROR_CHECK(bdc_motor_brake(seed_dispenser->motor));
            ESP_ERROR_CHECK(bdc_motor_brake(cutter_disc->motor));
            ESP_LOGI(TAG, "SEED PLANTER: STOPPED");
            break;
        case SP_BRAKE:
            ESP_ERROR_CHECK(bdc_motor_brake(seed_dispenser->motor));
            ESP_ERROR_CHECK(bdc_motor_brake(cutter_disc->motor));
            ESP_LOGI(TAG, "SEED PLANTER: BREAK");
            break;
        case SP_STARTING:
            ESP_ERROR_CHECK(bdc_motor_forward(seed_dispenser->motor));
            ESP_ERROR_CHECK(bdc_motor_forward(cutter_disc->motor));
            ESP_LOGI(TAG, "SEED PLANTER: STARTING");
            break;
        case SP_FORWARD:
            ESP_ERROR_CHECK(bdc_motor_forward(seed_dispenser->motor));
            ESP_ERROR_CHECK(bdc_motor_forward(cutter_disc->motor));
            ESP_LOGI(TAG, "SEED PLANTER: FORWARD");
            break;
        case SP_REVERSE:
            ESP_ERROR_CHECK(bdc_motor_reverse(seed_dispenser->motor));
            ESP_ERROR_CHECK(bdc_motor_reverse(cutter_disc->motor));
            ESP_LOGI(TAG, "SEED PLANTER: REVERSE");
            break;
        default:
            ESP_LOGE(TAG, "SEED PLANTER: UNKNOWN STATE");
            break;
        }
    }

    if (g_soft_start_seed_planter_active)
    {
        // During the soft start the queue will be ignored
        float cutter_disc_point = 0.0f;
        float seed_dispenser_point = 0.0f;

        calculate_lspb_speed_point(g_soft_start_tf, soft_start_counter, g_soft_start_cutter_disc_ts, &cutter_disc_point);
        calculate_lspb_speed_point(g_soft_start_tf, soft_start_counter, g_soft_start_seed_dispenser_ts, &seed_dispenser_point);

        ESP_ERROR_CHECK(seed_planter_set_speed(cutter_disc_point, seed_dispenser_point));

        // ESP_LOGI(TAG, "soft start point: %f", point);
        soft_start_counter++;
        if (soft_start_counter == g_soft_start_tf)
        {
            g_soft_start_seed_planter_active = false;
            seed_planter_set_state(SP_FORWARD);
        }
    }
    else
    {
        // Check wheater there is another speed value in the queue
        if (!is_queue_empty(seed_planter_handle->motor_left_ctx.speed_queue) && !is_queue_empty(seed_planter_handle->motor_right_ctx.speed_queue))
        {
            float cutter_disc_speed, seed_dispenser_speed;
            ESP_ERROR_CHECK(dequeue(seed_planter_handle->motor_left_ctx.speed_queue, &cutter_disc_speed));
            ESP_ERROR_CHECK(dequeue(seed_planter_handle->motor_right_ctx.speed_queue, &seed_dispenser_speed));
            seed_planter_set_speed(cutter_disc_speed, seed_dispenser_speed);
        }
    }

    if (last_seed_planter_state != SP_BRAKE && last_seed_planter_state != SP_STOPPED)
    {
        // Calculate speed error
        float cutter_disc_error = cutter_disc->desired_speed - cutter_disc_abs_pulses;
        float seed_dispenser_error = seed_dispenser->desired_speed - seed_dispenser_abs_pulses;

        // Calculate the new speed
        float seed_dispenser_new_speed = 0.0f;
        float cutter_disc_new_speed = 0.0f;
        pid_compute(cutter_disc->pid_ctrl, cutter_disc_error, &cutter_disc_new_speed);
        pid_compute(seed_dispenser->pid_ctrl, seed_dispenser_error, &seed_dispenser_new_speed);
        ESP_ERROR_CHECK(bdc_motor_set_speed(cutter_disc->motor, (uint32_t)cutter_disc_new_speed));
        ESP_ERROR_CHECK(bdc_motor_set_speed(seed_dispenser->motor, (uint32_t)seed_dispenser_new_speed));
    }

    // Save information: cutter - left
    if (g_seed_planter_current_state == SP_REVERSE)
    {
        seed_planter_data.motor_left_desired_speed = (-1.00f) * seed_planter_handle->motor_left_ctx.desired_speed * CUTTER_DISC_PULSES2REV;
        seed_planter_data.motor_right_desired_speed = (-1.00f) * seed_planter_handle->motor_right_ctx.desired_speed * SEED_DISPENSER_PULSES2REV;
    }
    else 
    {
        seed_planter_data.motor_left_desired_speed = seed_planter_handle->motor_left_ctx.desired_speed * CUTTER_DISC_PULSES2REV;
        seed_planter_data.motor_right_desired_speed = seed_planter_handle->motor_right_ctx.desired_speed * SEED_DISPENSER_PULSES2REV;
    }
    
}

esp_err_t seed_planter_set_state(seed_planter_state_e state)
{
    g_seed_planter_current_state = state;

    return ESP_OK;
}

esp_err_t seed_planter_set_speed(float cutter_disc_speed, float seed_planter_speed)
{
    if (cutter_disc_speed <= CUTTER_DISC_MAX_SPEED_REVS && seed_planter_speed <= SEED_DISPENSER_MAX_SPEED_REVS)
    {
        int cutter_disc_pulses = (int)(cutter_disc_speed * CUTTER_DISC_REV2PULSES);
        int seed_planter_pulses = (int)(seed_planter_speed * SEED_DISPENSER_REV2PULSES);
        ESP_ERROR_CHECK(motor_pair_set_speed(cutter_disc_pulses, seed_planter_pulses, seed_planter_handle));
    }
    else
    {
        ESP_LOGE(TAG, "Speed set was out of bounds!");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void seed_planter_control_task(void *pvParameters)
{
    seed_planter_handle = (motor_pair_handle_t *)malloc(sizeof(motor_pair_handle_t));

    strcpy(seed_planter_handle->id, "seed_planter_pair");

    motor_config_t cutter_disc_config = {
        .motor_encodera_gpio_num = CUTTER_DISC_ENCODER_A,
        .motor_encoderb_gpio_num = CUTTER_DISC_ENCODER_B,
        .motor_pwma_gpio_num = CUTTER_DISC_PWMA,
        .motor_pwmb_gpio_num = CUTTER_DISC_PWMB,
        .pwm_freq_hz = SEED_PLANTER_MOTORS_PWM_FREQ,
        .motor_id = "cutter_disc",
        .pid_config = {
            .kp = CUTTER_DISC_KP,
            .kd = CUTTER_DISC_KD,
            .ki = CUTTER_DISC_KI,
        },
    };
    motor_config_t seed_dispenser_config = {
        .motor_encodera_gpio_num = SEED_DISPENSER_ENCODER_A,
        .motor_encoderb_gpio_num = SEED_DISPENSER_ENCODER_B,
        .motor_pwma_gpio_num = SEED_DISPENSER_PWMA,
        .motor_pwmb_gpio_num = SEED_DISPENSER_PWMB,
        .pwm_freq_hz = SEED_PLANTER_MOTORS_PWM_FREQ,
        .motor_id = "seed_dispenser",
        .pid_config = {
            .kp = SEED_DISPENSER_KP,
            .kd = SEED_DISPENSER_KD,
            .ki = SEED_DISPENSER_KI,
        },
    };

    // TODO: Check why there is a pwm_freq in both config structs
    motor_pair_bdc_config_t seed_planter_bdc_conf = {
        .bdc_encoder_pcnt_high_limit = SEED_PLANTER_BDC_ENCODER_PCNT_HIGH_LIMIT,
        .bdc_encoder_pcnt_low_limit = SEED_PLANTER_BDC_ENCODER_PCNT_LOW_LIMIT,
        .mcpwm_group = SEED_PLANTER_MCPWM_GROUP,
        .pwm_freq_hz = SEED_PLANTER_MOTORS_PWM_FREQ,
        .pid_loop_period = SEED_PLANTER_PID_LOOP_PERIDO_MS,
        .bdc_mcpwm_timer_resolution_hz = SEED_PLANTER_MCPWM_TIMER_RESOLUTION_HZ,
    };

    motor_pair_config_t seed_planter_config = {
        .motor_left_config = cutter_disc_config,
        .motor_right_config = seed_dispenser_config,
        .bdc_config = seed_planter_bdc_conf,
    };

    ESP_ERROR_CHECK(motor_pair_init(&seed_planter_config, seed_planter_handle));

    // Setting up timer
    const esp_timer_create_args_t seed_planter_timer_args = {
        .callback = seed_planter_pid_loop_cb,
        .arg = NULL,
        .name = "seed_planter_loop",
    };

    esp_timer_handle_t seed_planter_pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&seed_planter_timer_args, &seed_planter_pid_loop_timer));

    // Enable motors
    ESP_ERROR_CHECK(motor_pair_enable_motors(seed_planter_handle));

    // Initialize seed_planter_data
    seed_planter_data = (motor_pair_data_t){
        .state = STOPPED,
        .motor_left_desired_speed = 0.0f,
        .motor_left_current_speed = 0.0f,
        .motor_right_desired_speed = 0.0f,
        .motor_right_current_speed = 0.0f,
    };

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
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t seed_planter_soft_start(float seed_dispenser_ts, float cutter_disc_ts, int tf)
{
    if (seed_planter_handle == NULL)
        return ESP_FAIL;

    ESP_LOGI(TAG, "Starting soft start");

    seed_planter_set_state(SP_STARTING);
    g_soft_start_seed_planter_active = true;
    g_soft_start_seed_dispenser_ts = seed_dispenser_ts;
    g_soft_start_cutter_disc_ts = cutter_disc_ts;
    g_soft_start_tf = tf;

    return ESP_OK;
}

QueueHandle_t seed_planter_get_queue_handle(void)
{
    return seed_planter_queue_handle;
}

void seed_planter_control_start_task(void)
{
    ESP_LOGI(TAG, "Initializing seed_planter task");

    xTaskCreatePinnedToCore(&seed_planter_control_task,
                            "seed_planter_control",
                            SEED_PLANTER_STACK_SIZE,
                            NULL,
                            SEED_PLANTER_TASK_PRIORITY,
                            NULL,
                            SEED_PLANTER_CORE_ID);
}
