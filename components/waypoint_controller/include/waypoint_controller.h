#ifndef NAVIGATION_CONTROL_H
#define NAVIGATION_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum {
        WP_STOPPED = 0,
        WP_INIT,
        WP_READY,
        WP_NAVIGATING,
        WP_WAITING,
        WP_TRJ_FINISHED,
    } waypoint_ctrl_state_e;

    esp_err_t waypoint_ctrl_get_queue_handle(QueueHandle_t *handle);

    esp_err_t waypoint_ctrl_add_point(float x, float y, float theta);

    esp_err_t waypoint_ctrl_start_trajectory(void);

    void waypoint_ctrl_start_task(void);

#ifdef __cplusplus
}
#endif
#endif