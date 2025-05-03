#ifndef WAYPOINT_CTRL_H
#define WAYPOINT_CTRL_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "diff_drive_ctrl.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        WP_STOPPED = BIT0,
        WP_INIT = BIT1,
        WP_READY = BIT2,
        WP_NAVIGATING = BIT3,
        WP_WAITING = BIT4,
        WP_TRJ_FINISHED = BIT5,
        WP_POINT_ADDED = BIT6,
        WP_ERROR = BIT7,
    } waypoint_state_e;

    typedef enum
    {
        WP_ERROR_EMPTY_NAV_POINTS = BIT0,
        WP_ERROR_CANNOT_START_TRACT = BIT1,
        WP_ERROR_CANNOT_SEND_FPOINT = BIT2,
        WP_ERROR_CANNOT_ADD_POINT = BIT3,
        WP_ERROR_CANNOT_STOP_TRACT = BIT4,
    } waypoint_error_e;

    typedef enum
    {
        WP_CMD_START_TRAJ = 0,
        WP_CMD_STOP_TRAJ,
        WP_CMD_RECEIVE_POINT,
    } waypoint_cmd_e;

    typedef struct
    {
        waypoint_cmd_e cmd;
        navigation_point_t *point;
    } waypoint_cmd_t;

    static inline const char *waypoint_state_to_string(waypoint_state_e state)
    {
        static const char *states[] = {
            "WP_STOPPED",
            "WP_INIT",
            "WP_READY",
            "WP_NAVIGATING",
            "WP_WAITING",
            "WP_TRJ_FINISHED",
            "WP_POINT_ADDED",
            "WP_ERROR"};

        // Ensure the state is within bounds
        if (state >= WP_STOPPED && state <= WP_ERROR)
        {
            return states[__builtin_ctz(state)]; // Use __builtin_ctz to map BITx to array index
        }
        return "UNKNOWN_STATE";
    }

    esp_err_t waypoint_get_event_group(EventGroupHandle_t *handle);

    esp_err_t waypoint_get_error_group(EventGroupHandle_t *handle);

    esp_err_t waypoint_get_state_queue_handle(QueueHandle_t *handle);

    esp_err_t waypoint_get_cmd_queue_handle(QueueHandle_t *handle);

    void waypoint_start_task(void);

#ifdef __cplusplus
}
#endif
#endif