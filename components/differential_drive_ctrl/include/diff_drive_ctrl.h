#ifndef NAVIGATION_UNIT_H
#define NAVIGATION_UNIT_H

#include "pid_ctrl.h"
#include "kalman_filter.h"
#include "esp_event.h"

ESP_EVENT_DECLARE_BASE(DIFF_DRIVE_EVENT_BASE);

typedef struct
{
    pid_ctrl_block_handle_t position_pid_ctrl;
    pid_ctrl_block_handle_t orientation_pid_ctrl;
    float vel_l;                // rad/s -> rev/s
    float vel_r;
} diff_drive_handle_t;

typedef enum {
    DD_STOPPED,
    DD_READY,
    DD_STARTED,
    DD_POINT_REACHED,
    DD_ORIENTING,
    DD_NAVIGATING
} diff_drive_state_e;

typedef struct {
    float err_x;
    float err_y;
    float err_theta;
    float err_dist;
    float err_ori;
    
    diff_drive_state_e state;
} diff_drive_state_t;

typedef struct
{
    float x;
    float y;
    float theta;

    diff_drive_state_t state;
} navigation_point_t;

typedef enum {
    DIFF_DRIVE_STOP_EVENT,
    DIFF_DRIVE_START_EVENT,
    DIFF_DRIVE_RECEIVE_POINT_EVENT,
} diff_drive_event_e;

static inline const char* diff_drive_state_2_string(diff_drive_state_e state)
{
    static const char *strings[] = {"Stopped", "Ready", "Point Reached", "Orienting", "Navigating"};

    return strings[state];
}

esp_err_t diff_drive_get_queue_handle(QueueHandle_t *queue);

void diff_drive_task_start(TaskHandle_t parent);

#endif
