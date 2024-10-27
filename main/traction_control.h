#ifndef TRACTION_CONTROL_H
#define TRACTION_CONTROL_H

#include "motor_pair_ctrl.h"

/**
 * @brief Set traction direction
 * 
 * @param state 
 * @return esp_err_t 
 */
esp_err_t traction_set_direction(motor_pair_state_e *state);

/**
 * @brief 
 * 
 * @param motor_left_speed 
 * @param motor_right_speed 
 * @return esp_err_t 
 */
esp_err_t traction_set_speed(float *motor_left_speed, float *motor_right_speed);

/**
 * @brief Traction control task start
 *
 */
void traction_control_start_task(void);

#endif