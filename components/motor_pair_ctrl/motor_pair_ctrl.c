/**
 * @file traction_control.c
 * @author Adrian Pulido
 * @brief
 * @version 0.1
 * @date 2024-09-14
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <stdio.h>
#include "math.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "math.h"
#include "string.h"

#include "motor_pair_ctrl.h"

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

#define SPEED_QUEUE_SIZE 10

static const char TAG[] = "motor_pair";

esp_err_t calculate_lspb_speed_point(const int tf, int t, const float qf, float *pvPoint)
{
    const float q0 = 0.0f;
    const float t0 = 0.0f;
    const float V = (4 * (qf - q0)) / (3 * (tf - t0));
    const float tb = (q0 - qf + V * tf) / (V);

    float point = 0.0f;

    if (t >= t0 && t < tb)
    {
        point = q0 + V / (2 * tb) * pow(t, 2);
    }
    else if (t >= tb && t < tf - tb)
    {
        point = (qf + q0 - V * tf) / 2 + V * t;
    }
    else if (t >= tf - tb)
    {
        if(t == tf)
            point = qf;
        else
            point = qf - (V * pow(tf, 2)) / (2 * tb) + (V * tf) / tb * t - V / (2 * tb) * pow(t, 2);
    }
    else
    {
        ESP_LOGE(TAG, "lspb curve time value out of bounds!");
        return ESP_FAIL;
    }

    *pvPoint = point;

    return ESP_OK;
}

esp_err_t motor_pair_set_speed(int motor_left_speed, int motor_right_speed, motor_pair_handle_t *motor_pair)
{
    motor_pair->motor_left_ctx.desired_speed = motor_left_speed;
    motor_pair->motor_right_ctx.desired_speed = motor_right_speed;

    return ESP_OK;
}

esp_err_t motor_pair_add_speed_to_queue(float motor_left_speed, float motor_right_speed, motor_pair_handle_t *motor_pair)
{
    ESP_ERROR_CHECK(enqueue(motor_pair->motor_left_ctx.speed_queue, motor_left_speed));
    
    ESP_ERROR_CHECK(enqueue(motor_pair->motor_right_ctx.speed_queue, motor_right_speed));

    return ESP_OK;
}

esp_err_t motor_pair_enable_motors(motor_pair_handle_t *pvHandle)
{

    ESP_LOGI(TAG, "Starting encoder unit: %s", pvHandle->motor_left_ctx.motor_id);
    ESP_ERROR_CHECK(pcnt_unit_enable(pvHandle->motor_left_ctx.pcnt_encoder));
    ESP_LOGI(TAG, "Starting encoder unit: %s", pvHandle->motor_right_ctx.motor_id);
    ESP_ERROR_CHECK(pcnt_unit_enable(pvHandle->motor_right_ctx.pcnt_encoder));

    ESP_ERROR_CHECK(pcnt_unit_clear_count(pvHandle->motor_left_ctx.pcnt_encoder));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pvHandle->motor_right_ctx.pcnt_encoder));

    ESP_ERROR_CHECK(pcnt_unit_start(pvHandle->motor_left_ctx.pcnt_encoder));
    ESP_ERROR_CHECK(pcnt_unit_start(pvHandle->motor_right_ctx.pcnt_encoder));

    ESP_LOGI(TAG, "Enabling motor: %s", pvHandle->motor_left_ctx.motor_id);
    ESP_ERROR_CHECK(bdc_motor_enable(pvHandle->motor_left_ctx.motor));
    ESP_LOGI(TAG, "Enabling motor: %s", pvHandle->motor_right_ctx.motor_id);
    ESP_ERROR_CHECK(bdc_motor_enable(pvHandle->motor_right_ctx.motor));

    ESP_ERROR_CHECK(bdc_motor_brake(pvHandle->motor_left_ctx.motor));
    ESP_ERROR_CHECK(bdc_motor_brake(pvHandle->motor_right_ctx.motor));

    return ESP_OK;
}

esp_err_t motor_pair_init_individual_motor(motor_config_t *motor_config, motor_pair_bdc_config_t *pair_config, motor_control_context_t *pvMotor)
{
    strcpy(pvMotor->motor_id, motor_config->motor_id);
    ESP_LOGI(TAG, "Initializing motor: %s", pvMotor->motor_id);

    pvMotor->pcnt_encoder = NULL;
    bdc_motor_config_t bdc_motor_config = {
        .pwm_freq_hz = motor_config->pwm_freq_hz,
        .pwma_gpio_num = motor_config->motor_pwma_gpio_num,
        .pwmb_gpio_num = motor_config->motor_pwmb_gpio_num,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = pair_config->mcpwm_group,
        .resolution_hz = pair_config->bdc_mcpwm_timer_resolution_hz,
    };

    bdc_motor_handle_t bdc_motor = NULL;

    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&bdc_motor_config, &mcpwm_config, &bdc_motor));

    pvMotor->motor = bdc_motor;

    // Setting up motor encoders
    pcnt_unit_config_t motor_pcnt_config = {
        .high_limit = pair_config->bdc_encoder_pcnt_high_limit,
        .low_limit = pair_config->bdc_encoder_pcnt_low_limit,
        .flags.accum_count = true,
    };

    pcnt_unit_handle_t motor_pcnt_unit = NULL;

    ESP_ERROR_CHECK(pcnt_new_unit(&motor_pcnt_config, &motor_pcnt_unit));

    pcnt_glitch_filter_config_t pcnt_filter_config = {
        .max_glitch_ns = 1000,
    };

    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(motor_pcnt_unit, &pcnt_filter_config));

    pcnt_chan_config_t pcnt_chan_a_config = {
        .edge_gpio_num = motor_config->motor_encodera_gpio_num,
        .level_gpio_num = motor_config->motor_encoderb_gpio_num,
    };
    pcnt_chan_config_t pcnt_chan_b_config = {
        .edge_gpio_num = motor_config->motor_encoderb_gpio_num,
        .level_gpio_num = motor_config->motor_encodera_gpio_num,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(motor_pcnt_unit, &pcnt_chan_a_config, &pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(motor_pcnt_unit, &pcnt_chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(motor_pcnt_unit, pair_config->bdc_encoder_pcnt_high_limit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(motor_pcnt_unit, pair_config->bdc_encoder_pcnt_low_limit));

    pvMotor->pcnt_encoder = motor_pcnt_unit;

    /* PID control init*/
    ESP_LOGI(TAG, "Create PID control block");
    pid_ctrl_parameter_t motor_pid_runtime_param = {
        .kp = motor_config->pid_config.kp,
        .ki = motor_config->pid_config.ki,
        .kd = motor_config->pid_config.kd,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output = pair_config->bdc_mcpwm_timer_resolution_hz / motor_config->pwm_freq_hz,
        .min_output = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };

    pid_ctrl_block_handle_t motor_pid_ctrl = NULL;
    pid_ctrl_config_t motor_pid_config = {
        .init_param = motor_pid_runtime_param,
    };

    ESP_ERROR_CHECK(pid_new_control_block(&motor_pid_config, &motor_pid_ctrl));

    pvMotor->pid_ctrl = motor_pid_ctrl;

    // Motor queue setup
    pvMotor->speed_queue = create_queue(SPEED_QUEUE_SIZE);

    return ESP_OK;
}

esp_err_t motor_pair_init(motor_pair_config_t *config, motor_pair_handle_t *pvHandle)
{
    ESP_LOGI(TAG, "Initializing motor pair %s", pvHandle->id);

    ESP_ERROR_CHECK(motor_pair_init_individual_motor(&config->motor_left_config, &config->bdc_config, &pvHandle->motor_left_ctx));
    ESP_ERROR_CHECK(motor_pair_init_individual_motor(&config->motor_right_config, &config->bdc_config, &pvHandle->motor_right_ctx));

    return ESP_OK;
}
