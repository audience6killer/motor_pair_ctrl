/**
 * @file tract_ctrl.h
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-11-12
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef tract_ctrl_H
#define tract_ctrl_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "motor_pair_ctrl.h"

typedef enum
{
    TRACT_CTRL_CMD_SET_SPEED,
    TRACT_CTRL_CMD_STOP,
    TRACT_CTRL_CMD_START,
} tract_ctrl_cmd_e;

typedef struct
{
    tract_ctrl_cmd_e cmd;
    float *motor_left_speed;
    float *motor_right_speed;
} tract_ctrl_cmd_t;

/**
 * @brief Get current state of the task 
 * 
 * @return motor_pair_state_e 
 */
motor_pair_state_e tract_ctrl_get_current_state(void);

/**
 * @brief Get the data queue for the traction control
 *
 * @return QueueHandle_t
 */
esp_err_t tract_ctrl_get_data_queue(QueueHandle_t *queue);

/**
 * @brief Get the command queue for the traction control
 *
 * @param queue
 * @return esp_err_t
 */
esp_err_t tract_ctrl_get_cmd_queue(QueueHandle_t *queue);

/**
 * @brief Starts task for traction control
 *
 */
void tract_ctrl_start_task(void);

#endif