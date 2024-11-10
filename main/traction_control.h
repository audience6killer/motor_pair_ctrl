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
 * @brief 
 * 
 * @param motor_left_speed in rev/s 
 * @param motor_right_speed in rev/s
 * @return esp_err_t 
 */
esp_err_t traction_control_set_speed(float motor_left_speed, float motor_right_speed);

/**
 * @brief 
 * 
 * @param target_speed 
 * @return esp_err_t 
 */
esp_err_t traction_control_smooth_start(float target_speed);


esp_err_t traction_control_soft_start(float target_speed, int tf);

/**
 * @brief Traction control task start
 *
 */
void traction_control_start_task(void);

#endif