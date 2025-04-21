#ifndef NAVIGATION_CONTROL_H
#define NAVIGATION_CONTROL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "diff_drive_ctrl.h"

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
    } waypoint_state_e;

    typedef enum {
        WP_CMD_START_TRAJ = 0,
        WP_CMD_STOP_TRAJ,
        WP_CMD_RECEIVE_POINT,
    } waypoint_cmd_e;

    typedef struct {
        waypoint_cmd_e cmd;
        navigation_point_t *point;
    } waypoint_cmd_t;

    static inline const char* waypoint_state_to_string(waypoint_state_e state)
    {
        static const char* states[] = {"WP_STOPPED", "WP_INIT", "WP_READY", "WP_NAVIGATING", "WP_WAITING", "WP_TRJ_FINISHED"};

        return states[state];
    }

    esp_err_t waypoint_get_state_queue_handle(QueueHandle_t *handle);

    esp_err_t waypoint_get_cmd_queue_handle(QueueHandle_t *handle);

    void waypoint_start_task(void);

#ifdef __cplusplus
}
#endif
#endif