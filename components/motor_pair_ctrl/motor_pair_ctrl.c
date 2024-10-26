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
#include "esp_timer.h"

#include "motor_pair_ctrl.h"

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

static const char TAG[] = "motor_pair";

esp_err_t motor_pair_init_individual_motor(motor_config_t *motor_config,
                                           motor_pair_bdc_config_t *pair_config,
                                           motor_control_context_t *pvMotor)
{
    ESP_LOGI(TAG, "Initializing motor");

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

    ESP_LOGI(TAG, "Starting encoder unit: %s", motor_config->motor_id);

    ESP_ERROR_CHECK(pcnt_unit_enable(pvMotor->pcnt_encoder));

    ESP_ERROR_CHECK(pcnt_unit_clear_count(pvMotor->pcnt_encoder));

    ESP_ERROR_CHECK(pcnt_unit_start(pvMotor->pcnt_encoder));

    ESP_LOGI(TAG, "Enabling motor: %s", motor_config->motor_id);
    ESP_ERROR_CHECK(bdc_motor_enable(pvMotor->motor));
    ESP_ERROR_CHECK(bdc_motor_brake(pvMotor->motor));

    return ESP_OK;
}

/**
 * @brief Set motors desiered speed. Speed can be positive or negative.
 * @param motor_left_speed Speed must be in pulses
 * @param motor_right_speed 
 * @return esp_err_t 
 */
esp_err_t motor_pair_set_speed(int *motor_left_speed, int *motor_right_speed, motor_pair_handle_t *motor_pair)
{
    if (motor_left_speed == NULL || motor_right_speed == NULL)
    {
        ESP_LOGE(TAG, "Speed provided for the motors is NULL");
        return ESP_FAIL;
    }

    motor_pair->motor_left_ctx.desired_speed = *motor_left_speed;
    motor_pair->motor_right_ctx.desired_speed = *motor_right_speed;

    return ESP_OK;
}

/**
 * @brief Motor pair initialization
 *
 * @param config
 * @param pid_config
 * @param pvHandle
 * @return esp_err_t
 */
esp_err_t motor_pair_init(motor_pair_config_t *config,
                          motor_pair_handle_t *pvHandle)
{
    ESP_LOGI(TAG, "Initializing motor pair");

    // TODO: SPLIT individual initialization into a new function starting here

    motor_control_context_t *motor_left = &pvHandle->motor_left_ctx;
    motor_control_context_t *motor_right = &pvHandle->motor_right_ctx;

    motor_pair_init_individual_motor(&config->motor_left_config, &config->bdc_config, motor_left);
    motor_pair_init_individual_motor(&config->motor_right_config, &config->bdc_config, motor_right);

    return ESP_OK;
}
