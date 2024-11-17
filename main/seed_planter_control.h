#ifndef SEED_PLANTER_CONTROL_H
#define SEED_PLANTER_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum
{
    SP_STOPPED,
    SP_BRAKE,
    SP_STARTING,
    SP_FORWARD,
    SP_REVERSE,
} seed_planter_state_e;


esp_err_t seed_planter_set_state(seed_planter_state_e state);

esp_err_t seed_planter_set_speed(float cutter_disc_speed, float seed_planter_speed);

/**
 * @brief Start planter soft start 
 * 
 * @param seed_dispenser_ts 
 * @param cutter_disc_ts 
 * @param tf 
 * @return esp_err_t 
 */
esp_err_t seed_planter_soft_start(float seed_dispenser_ts, float cutter_disc_ts, int tf);

/**
 * @brief Get QueueHandle 
 * 
 * @return QueueHandle_t 
 */
QueueHandle_t seed_planter_get_queue_handle(void);

/**
 * @brief Start task 
 * 
 */
void seed_planter_control_start_task(void);

#endif