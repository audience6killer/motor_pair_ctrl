/**
 * @file traction_control.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-11-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef TRACTION_CONTROL_H
#define TRACTION_CONTROL_H

#include "motor_pair_ctrl.h"

/**
 * @brief Set traction direction
 * 
 * @param state 
 * @return esp_err_t 
 */
esp_err_t traction_control_set_direction(const motor_pair_state_e state);

/**
 * @brief The speed should be positive
 * 
 * @param motor_left_speed in rev/s 
 * @param motor_right_speed in rev/s
 * @return esp_err_t 
 */
esp_err_t traction_control_set_speed(float motor_left_speed, float motor_right_speed);

// TODO
// esp_err_t traction_control_speed_controlled_direction(float motor_left_speed, float motor_right_speed);

/**
 * @brief Soft start for motors using a LSPB speed trajectory
 * 
 * @param target_speed, in rev/s, will be applied to both wheels
 * @param tf final time, in number of samples, each one at @10ms
 * @return esp_err_t 
 */
esp_err_t traction_control_soft_start(float target_speed, int tf);

/**
 * @brief Traction control task start
 *
 */
void traction_control_start_task(void);

#endif