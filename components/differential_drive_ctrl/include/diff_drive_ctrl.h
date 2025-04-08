#ifndef NAVIGATION_UNIT_H
#define NAVIGATION_UNIT_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "pid_ctrl.h"
#include "kalman_filter.h"
#include "esp_event.h"

ESP_EVENT_DECLARE_BASE(DIFF_DRIVE_EVENT_BASE);

extern EventGroupHandle_t diff_drive_event_group_handle;

typedef struct
{
    pid_ctrl_block_handle_t position_pid_ctrl;
    pid_ctrl_block_handle_t orientation_pid_ctrl;
    float vel_l;                // rad/s -> rev/s
    float vel_r;
} diff_drive_handle_t;

typedef enum {
    DD_STOPPED = BIT0,
    DD_READY = BIT1,
    DD_STARTED = BIT2,
    DD_POINT_REACHED = BIT3,
    DD_ORIENTING = BIT4,
    DD_NAVIGATING = BIT5
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

    //diff_drive_state_t state;
} navigation_point_t;

typedef enum {
    DIFF_DRIVE_STOP_EVENT,
    DIFF_DRIVE_START_EVENT,
    DIFF_DRIVE_RECEIVE_POINT_EVENT,
} diff_drive_event_e;

static inline const char* diff_drive_state_2_string(diff_drive_state_e state)
{
    static const char *strings[] = {"Stopped", "Ready", "Started", "Point Reached", "Orienting", "Navigating"};

    return strings[state];
}

esp_err_t diff_drive_get_event_loop(esp_event_loop_handle_t *handle);

esp_err_t diff_drive_get_queue_handle(QueueHandle_t *queue);

void diff_drive_task_start(void);

#endif
