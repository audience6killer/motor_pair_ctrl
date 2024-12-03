#ifndef NAVIGATION_UNIT_H
#define NAVIGATION_UNIT_H

#include "pid_ctrl.h"

typedef struct
{
    pid_ctrl_block_handle_t position_pid_ctrl;
    pid_ctrl_block_handle_t orientation_pid_ctrl;
    float vel_l; // rad/s -> rev/s
    float vel_r;
} diff_drive_ctrl_handle_t;

typedef struct
{
    float x;
    float y;
    float theta;
} navigation_point_t;

QueueHandle_t diff_drive_get_queue_handle(void);

bool diff_drive_point_reached(void);

void diff_drive_ctrl_task_start(void);

#endif
