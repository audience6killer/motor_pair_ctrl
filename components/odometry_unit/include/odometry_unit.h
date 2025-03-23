
#ifndef ODOMETRY_UNIT_H
#define ODOMETRY_UNIT_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    ODO_STOPPED,
    ODO_RUNNING,
    ODO_ERROR,
} odometry_state_e;

typedef struct 
{
    float past_value;
    float cur_value;
    float diff_value;
} differentiator_t;

typedef struct {
    differentiator_t x;
    differentiator_t y;
    differentiator_t phi_l;
    differentiator_t phi_r;
    differentiator_t theta;
    odometry_state_e odometry_state;
} odometry_data_t;

typedef enum {
    ODO_CMD_START,
    ODO_CMD_STOP
} odometry_cmd_e;

/**
 * @brief Get odometry data queue
 * 
 * @param queue 
 * @return esp_err_t 
 */
esp_err_t odometry_get_data_queue(QueueHandle_t *queue);

/**
 * @brief Get odometry command queue 
 * 
 * @param queue 
 * @return esp_err_t 
 */
esp_err_t odometry_get_cmd_queue(QueueHandle_t *queue);

/**
 * @brief Odometry start task 
 * 
 */
void odometry_start_task(void);

#endif