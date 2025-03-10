
#ifndef ODOMETRY_UNIT_H
#define ODOMETRY_UNIT_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    ODO_STOPPED,
    ODO_READING
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

/**
 * @brief 
 * 
 * @return QueueHandle_t 
 */
QueueHandle_t odometry_unit_get_queue_handle(void);

/**
 * @brief Odometry start task 
 * 
 */
void odometry_unit_start_task(void);

#endif