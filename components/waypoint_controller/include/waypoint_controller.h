#ifndef NAVIGATION_CONTROL_H
#define NAVIGATION_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C"
{
#endif

    ESP_EVENT_DECLARE_BASE(WAYPOINT_EVENT_BASE);

    typedef enum
    {
        WP_START_WHILE_EMPTY = 0,
        WP_START_WHILE_MOVING,
    } waypoint_error_e;

    typedef enum
    {
        WP_STOPPED = 0,
        WP_READY,
        WP_NAVIGATING,
        WP_NEXT_POINT,
        WP_WAITING,
        WP_TRJ_FINISHED,
        WP_ERROR,
    } waypoint_ctrl_state_e;

    typedef enum
    {
        WP_STOP_EVENT = 0, // Stop trajectory
        WP_START_EVENT,    // Start trajectory
        WP_ADD_POINT_EVENT,
    } waypoint_event_t;

    static inline const char *waypoint_ctrl_state_2_string(waypoint_ctrl_state_e state)
    {
        static const char *strings[] = {
            "WP_STOPPED",
            "WP_READY",
            "WP_NAVIGATING",
            "WP_NEXT_POINT",
            "WP_WAITING",
            "WP_TRJ_FINISHED",
            "WP_ERROR",
        };

        return strings[state];
    }

    esp_err_t waypoint_ctrl_get_queue_handle(QueueHandle_t *handle);

    esp_err_t waypoint_ctrl_get_event_loop(esp_event_loop_handle_t *handle);

    void waypoint_ctrl_start_task(TaskHandle_t parent);

#ifdef __cplusplus
}
#endif
#endif