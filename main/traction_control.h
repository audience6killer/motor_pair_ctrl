#ifndef TRACTION_CONTROL_H
#define TRACTION_CONTROL_H

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
} traction_state_e;

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