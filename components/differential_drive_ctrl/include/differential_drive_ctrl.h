#ifndef NAVIGATION_UNIT_H
#define NAVIGATION_UNIT_H

#include "pid_ctrl.h"
#include "kalman_filter.h"

typedef struct
{
    pid_ctrl_block_handle_t position_pid_ctrl;
    pid_ctrl_block_handle_t orientation_pid_ctrl;
    float vel_l; // rad/s -> rev/s
    float vel_r;
} diff_drive_ctrl_handle_t;


typedef enum {
    POINT_REACHED,
    NAVIGATING
} diff_drive_state_t;

QueueHandle_t diff_drive_get_queue_handle(void);

esp_err_t diff_drive_set_navigation_point(navigation_point_t point);

void diff_drive_ctrl_task_start(void);

#endif
