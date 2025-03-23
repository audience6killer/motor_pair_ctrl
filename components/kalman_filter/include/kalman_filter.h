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

esp_err_t kalman_initialize_info(kalman_info_t *data);

esp_err_t kalman_fiter_get_data_queue(QueueHandle_t *handle);

esp_err_t kalman_fiter_get_cmd_queue(QueueHandle_t *handle);

void kalman_filter_start_task(void);

#endif