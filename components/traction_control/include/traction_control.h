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
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "motor_pair_ctrl.h"

ESP_EVENT_DECLARE_BASE(TRACT_EVENT_BASE);

extern EventGroupHandle_t tract_ctrl_event_group_handle; 

typedef enum {
    TRACT_EVENT_BIT_STOPPED = BIT0,
    TRACT_EVENT_BIT_READY = BIT1,
    TRACT_EVENT_BIT_STARTED = BIT2,
    TRACT_EVENT_BIT_ERROR = BIT3,
} tract_ctrl_event_e;

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
 * @brief Set motors speed
 * 
 * @param mleft_speed In rev/s
 * @param mright_speed In rev/s
 * @return esp_err_t 
 */
esp_err_t tract_ctrl_set_speed(float mleft_speed, float mright_speed);

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
void tract_ctrl_start_task(void);

#endif