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
#include "esp_event.h"
#include "motor_pair_ctrl.h"

ESP_EVENT_DECLARE_BASE(TRACT_EVENT_BASE);

typedef enum
{
    TRACT_CTRL_CMD_SET_SPEED,
    TRACT_CTRL_CMD_STOP,
    TRACT_CTRL_CMD_START,
} tract_ctrl_cmd_e;

typedef struct
{
    tract_ctrl_cmd_e cmd;
    float motor_left_speed;
    float motor_right_speed;
} tract_ctrl_cmd_t;

/**
 * @brief Get the data queue for the traction control
 *
 * @return QueueHandle_t
 */
esp_err_t tract_ctrl_get_data_queue(QueueHandle_t *queue);

/**
 * @brief Get event loop reference
 * 
 * @param handle 
 * @return esp_err_t 
 */
esp_err_t tract_ctrl_get_event_loop_handle(esp_event_loop_handle_t *handle);

/**
 * @brief Starts task for traction control
 *
 */
void tract_ctrl_start_task(TaskHandle_t *parent);

#endif