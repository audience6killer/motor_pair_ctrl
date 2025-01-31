#ifndef DIFF_DRIVE_TASK_COMMON_H
#define DIFF_DRIVE_TASK_COMMON_H

#define DIFF_DRIVE_STACK_SIZE          4096
#define DIFF_DRIVE_CORE_ID             0
#define DIFF_DRIVE_TASK_PRIORITY       10 

#define DIFF_DRIVE_POS_KP              0.4f
#define DIFF_DRIVE_POS_KD              0.2f

#define DIFF_DRIVE_ORI_KP              0.4f
#define DIFF_DRIVE_ORI_KD              0.2f

#define V_COMM                         8.00f
#define V_MAX                          2.00f    // 2 rev/s: Motor DER
#define V_MAX_RADS                     12.466f    // M_der MAX = 2 rev/s
#define WHEEL_RADIUS                   0.033f
#define DISTANCE_TH                    0.05f        // 5 cm
#define ORIENTATION_TH                 0.174532 // 10°
#define RADS2REVS(b) (b * 0.1592f)

#endif