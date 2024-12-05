
#ifndef ODOMETRY_UNIT_H
#define ODOMETRY_UNIT_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    ODO_STOPPED,
    ODO_READING
} odometry_state_e;

typedef struct {
    float x;
    float y;
    float phi_l;
    float phi_r;
    float theta; // In degrees
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