#ifndef     KALMAN_FILTER_H
#define     KALMAN_FILTER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_event.h"

ESP_EVENT_DECLARE_BASE(KALMAN_EVENT_BASE);

typedef enum {
    KALMAN_STARTED = 0,
    KALMAN_STOPPED,
    KALMAN_READY,
} kalman_state_e;

typedef struct
{
    float x;
    float y;
    float z;
    float theta;
    float x_p;
    float y_p;
    float z_p;
    float theta_p;
    kalman_state_e state;
} kalman_info_t;

typedef enum {
    KALMAN_START_EVENT = 0,
    KALMAN_STOP_EVENT,
} kalman_events_e; 

static inline const char* kalman_state_to_name(kalman_state_e state)
{
    static const char *states[] = {"KF_STARTED", "KF_STOPPED", "KF_READY"}; 
    
    return states[state];
}

esp_err_t kalman_initialize_info(kalman_info_t *data);

esp_err_t kalman_get_data_queue(QueueHandle_t *handle);

esp_err_t kalman_get_event_loop(esp_event_loop_handle_t *handle);

void kalman_start_task(void);

#endif // KALMAN_FILTER_H