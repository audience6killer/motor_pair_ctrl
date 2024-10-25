#ifndef MOTOR_PAIR_CTRL_H
#define MOTOR_PAIR_CTRL_H

#include <stdint.h>
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"

typedef struct 
{
    uint32_t motor_pwma_gpio_num;
    uint32_t motor_pwmb_gpio_num;
    uint32_t motor_encodera_gpio_num;
    uint32_t motor_encoderb_gpio_num;
    uint32_t pwm_freq_hz;
} motor_config_t;

typedef struct
{
    uint32_t motor_left_pwma_gpio_num;      /*Motor 1 PWM A gpio number */
    uint32_t motor_left_pwmb_gpio_num;      /*Motor 1 PWM B gpio number */
    uint32_t motor_right_pwma_gpio_num;     /*Motor 2 PWM A gpio number */
    uint32_t motor_right_pwmb_gpio_num;     /*Motor 2 PWM B gpio number */
    uint32_t motor_left_encodera_gpio_num;  /*Motor 1 Encoder A gpio number*/
    uint32_t motor_left_encoderb_gpio_num;  /*Motor 1 Encoder B gpio number*/
    uint32_t motor_right_encodera_gpio_num; /*Motor 2 Encoder A gpio number*/
    uint32_t motor_right_encoderb_gpio_num; /*Motor 2 Encoder B gpio number*/
    uint32_t pwm_freq_hz;                   /*PWM frequency for both motors */
    uint32_t bdc_mcpwm_timer_resolution_hz;
    uint8_t mcpwm_group;
    uint32_t bdc_encoder_pcnt_high_limit;
    uint32_t bdc_encoder_pcnt_low_limit;
    char motor_left_id[20];
    char motor_right_id[20];
} motor_pair_config_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
} pid_config_t;

typedef struct
{
    pid_config_t motor_1;
    pid_config_t motor_2;
} motor_pair_pid_config_t;

typedef enum
{
    BREAK, // Stop motor in a break way (Slow decay)
    COAST, // Stop motor in a coast way (aka Fast Decay)
    FORWARD,
    REVERSE,
    TURN_LEFT_FORWARD, // All turns are on its axis
    TURN_RIGHT_FORWARD,
    TURN_LEFT_REVERSE,
    TURN_RIGHT_REVERSE,
} motor_state_e;

typedef struct
{
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    char motor_id[20];
    int report_pulses;
    int desired_speed; // In pulses
} motor_control_context_t;

typedef struct
{
    motor_control_context_t motor_left_ctx;
    motor_control_context_t motor_right_ctx;
    char id[20];
    motor_state_e traction_state;
    float mov_speed; // Should be always positive
} motor_pair_handle_t;


esp_err_t motor_pair_init(motor_pair_config_t *config,
                          motor_pair_pid_config_t *pid_config,
                          motor_pair_handle_t *pvHandle);


#endif