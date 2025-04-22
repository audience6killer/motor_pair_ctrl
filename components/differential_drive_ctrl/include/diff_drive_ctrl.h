#ifndef DIFF_DRIVE_H 
#define DIFF_DRIVE_H

#include "pid_ctrl.h"
#include "kalman_filter.h"

typedef enum {
    DD_STATE_STOPPED = 0,
    DD_STATE_READY,
    DD_STATE_STARTED,
    DD_STATE_POINT_REACHED,
    DD_STATE_ORIENTING,
    DD_STATE_NAVIGATING
} diff_drive_state_e;

typedef struct {
    float err_x;
    float err_y;
    float err_theta;
    float err_dist;
    float err_ori;
} diff_drive_err_t;

typedef enum {
    DD_CMD_STOP = 0,
    DD_CMD_START,
    DD_CMD_RECEIVE_POINT,
} diff_drive_cmd_e;

typedef struct
{
    float x;
    float y;
    float theta;
} navigation_point_t;

typedef struct {
    diff_drive_cmd_e cmd;

    navigation_point_t *point; 
} diff_drive_cmd_t;

esp_err_t diff_drive_get_cmd_queue_handle(QueueHandle_t *queue);

esp_err_t diff_drive_get_state_queue_handle(QueueHandle_t *queue);

void diff_drive_ctrl_task_start(void);

#endif
