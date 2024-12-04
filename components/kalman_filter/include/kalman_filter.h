#ifndef     KALMAN_FILTER_H
#define     KALMAN_FILTER_H

typedef struct
{
    float x;
    float y;
    float theta;
} navigation_point_t;

QueueHandle_t kalman_fiter_get_queue(void);

void kalman_filter_start_task(void);

#endif