#ifndef DATA_CENTER_H
#define DATA_CENTER_H

typedef struct 
{
    // TODO: Maybe in NED and geodesic coordinates
    // Position
    float x;
    float y;
    float z;
    // Speed
    float x_p;
    float y_p;
    float z_p;
    // Orientation
    float theta;
    float phi;
    float gamma;
    float theta_p;
} date_frame_t;

typedef enum {
    NAV_POINT = 0,
    START_TRAJECTORY,
    STOP_TRAJECTORY,

} inst_code_e;


/**
 * @brief Data center task initialization. 
 * 
 */
void data_center_task_start(void);


#endif