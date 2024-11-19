
#ifndef ODOMETRY_UNIT_H
#define ODOMETRY_UNIT_H

typedef struct odometry_robot_pose
{
    float x;
    float y;
    float phi_l;
    float phi_r;
    float theta; // In degrees
} odometry_robot_pose_t;

/**
 * @brief 
 * 
 * @return QueueHandle_t 
 */
QueueHandle_t odometry_unit_get_queue_handle(void);

/**
 * @brief Odometry start task 
 * 
 */
void odometry_unit_start_task(void);

#endif