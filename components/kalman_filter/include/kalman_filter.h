#ifndef     KALMAN_FILTER_H
#define     KALMAN_FILTER_H

typedef struct
{
    float x;
    float y;
    float z;
    float theta;
    float x_p;
    float y_p;
    float z_p;
    float theta_p;
} kalman_info_t;

typedef enum
{
    KALMAN_STATE_STOPPED = 0,
    KALMAN_STATE_READY,
    KALMAN_STATE_STARTED,
} kalman_state_e;

typedef enum 
{
    KALMAN_CMD_START = 0,
    KALMAN_CMD_STOP,
} kalman_cmd_e;

esp_err_t kalman_initialize_info(kalman_info_t *data);

esp_err_t kalman_get_data_queue(QueueHandle_t *handle);

esp_err_t kalman_get_cmd_queue(QueueHandle_t *handle);

void kalman_filter_start_task(void);

#endif