#ifndef NAVIGATION_UNIT_H
#define NAVIGATION_UNIT_H

#include "pid_ctrl.h"
#include "kalman_filter.h"

typedef struct
{
    pid_ctrl_block_handle_t position_pid_ctrl;
    pid_ctrl_block_handle_t orientation_pid_ctrl;
    float vel_l;                // rad/s -> rev/s
    float vel_r;
} diff_drive_ctrl_handle_t;

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
    
    diff_drive_state_e state;
} diff_drive_state_t;

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

    //diff_drive_state_t state;
} navigation_point_t;

typedef struct {
    diff_drive_cmd_e cmd;

    navigation_point_t *point; 
} diff_drive_cmd_t;

esp_err_t diff_drive_get_cmd_queue_handle(QueueHandle_t *queue);

esp_err_t diff_drive_get_data_queue_handle(QueueHandle_t *queue);

void diff_drive_ctrl_task_start(void);

#endif
