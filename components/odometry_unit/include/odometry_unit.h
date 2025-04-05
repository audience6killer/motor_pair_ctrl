
#ifndef ODOMETRY_UNIT_H
#define ODOMETRY_UNIT_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_event.h"

ESP_EVENT_DECLARE_BASE(ODOMETRY_EVENT_BASE);

typedef enum {
    ODO_STOPPED = 0,
    ODO_READY,
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
    ODOMETRY_START_EVENT = 0,
    ODOMETRY_STOP_EVENT,
} odometry_event_e;


static inline const char *odometry_state_to_name(odometry_state_e state)
{
    static const char *states[] = {"ODO_STOPPED", "ODO_READY", "ODO_RUNNING", "ODO_ERROR"};

    return states[state];
}

/**
 * @brief Get odometry data queue
 * 
 * @param queue 
 * @return esp_err_t 
 */
esp_err_t odometry_get_data_queue(QueueHandle_t *queue);

/**
 * @brief 
 * 
 * @param handle 
 * @return esp_err_t 
 */
esp_err_t odometry_get_event_loop(esp_event_loop_handle_t *handle);

/**
 * @brief Odometry start task 
 * 
 */
void odometry_start_task(TaskHandle_t parent);

#endif