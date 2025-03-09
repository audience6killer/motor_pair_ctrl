#ifndef DIFF_DRIVE_TASK_COMMON_H
#define DIFF_DRIVE_TASK_COMMON_H


#define DIFF_DRIVE_STACK_SIZE          4096
#define DIFF_DRIVE_CORE_ID             0
#define DIFF_DRIVE_TASK_PRIORITY       10 

#define DIFF_DRIVE_POS_KP              0.8f
#define DIFF_DRIVE_POS_KD              0.2f

#define DIFF_DRIVE_ORI_KP              0.8f
#define DIFF_DRIVE_ORI_KD              0.2f

// Velocidad lineal de crucero = 0.5m/s = 0.388rev/s = 2.437rad/s
#define V_COMM                         2.437f 
#define V_MAX                          4 * M_PI    // 2 rev/s: Motor DER
#define V_MAX_RADS                     4 * M_PI    // M_der MAX = 2 rev/s
#define WHEEL_RADIUS                   0.2046f
#define DISTANCE_TH                    0.30f        // 5 cm

#define ORIENTATION_TH                 0.087266 // 5°
#define RADS2REVS(b) (b * 0.1592f)

#endif