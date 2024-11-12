/**
 * @file motor_pair_ctrl.h
 * @author adrian pulido
 * @brief
 * @version 0.1
 * @date 2024-11-12
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef MOTOR_PAIR_CTRL_H
#define MOTOR_PAIR_CTRL_H

#include <stdint.h>
#include "queue.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
} pid_config_t;

typedef struct
{
    uint32_t motor_pwma_gpio_num;
    uint32_t motor_pwmb_gpio_num;
    uint32_t motor_encodera_gpio_num;
    uint32_t motor_encoderb_gpio_num;
    uint32_t pwm_freq_hz;
    pid_config_t pid_config;
    char motor_id[20];
} motor_config_t;

typedef struct
{
    uint32_t pwm_freq_hz; /*PWM frequency for both motors */
    uint32_t bdc_mcpwm_timer_resolution_hz;
    uint8_t mcpwm_group;
    uint32_t bdc_encoder_pcnt_high_limit;
    uint32_t bdc_encoder_pcnt_low_limit;
    uint32_t pid_loop_period;
} motor_pair_bdc_config_t;

typedef struct
{
    motor_config_t motor_left_config;
    motor_config_t motor_right_config;
    motor_pair_bdc_config_t bdc_config;
} motor_pair_config_t;

typedef struct
{
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    char motor_id[20];
    int report_pulses;
    queue_t *speed_queue;
    int desired_speed; // In pulses
} motor_control_context_t;

typedef struct
{
    motor_control_context_t motor_left_ctx;
    motor_control_context_t motor_right_ctx;
    char id[30];
} motor_pair_handle_t;

typedef enum
{
    BRAKE, // Stop motor in a break way (Slow decay)
    COAST, // Stop motor in a coast way (aka Fast Decay)
    STARTING,
    FORWARD,
    REVERSE,
    TURN_LEFT_FORWARD, // All turns are on its axis
    TURN_RIGHT_FORWARD,
    TURN_LEFT_REVERSE,
    TURN_RIGHT_REVERSE,
} motor_pair_state_e;

typedef struct
{
    float motor_left_real_pulses;
    float motor_right_real_pulses;
    float motor_left_desired_speed;
    float motor_right_desired_speed;
    float motor_left_error;
    float motor_right_error;
    motor_pair_state_e state;
} motor_pair_data_t;

/**
 * @brief
 *
 * @param tf final time, in samples @10ms
 * @param t current time
 * @param qf final value in rev/s
 * @param pvPoint
 * @return esp_err_t
 */
esp_err_t calculate_lspb_speed_point(const int tf, int t, const float qf, float *pvPoint);

/**
 * @brief Set motors desiered speed. Speed can be positive or negative.
 * @param motor_left_speed Speed must be in pulses
 * @param motor_right_speed
 * @return esp_err_t
 */
esp_err_t motor_pair_set_speed(int motor_left_speed, int motor_right_speed, motor_pair_handle_t *motor_pair);

/**
 * @brief Speed have to be in rev/s
 *
 * @param motor_left_speed
 * @param motor_right_speed
 * @param motor_pair
 * @return esp_err_t
 */
esp_err_t motor_pair_add_speed_to_queue(float motor_left_speed, float motor_right_speed, motor_pair_handle_t *motor_pair);

/**
 * @brief Enable motor pair: bdc and pcnt units
 *
 * @param motor_pair
 * @return esp_err_t
 */
esp_err_t motor_pair_enable_motors(motor_pair_handle_t *motor_pair);

/**
 * @brief Initialize motor pair unit
 *
 * @param config
 * @param pvHandle
 * @return esp_err_t
 */
esp_err_t motor_pair_init(motor_pair_config_t *config, motor_pair_handle_t *pvHandle);

#endif