#ifndef DIFF_DRIVE_TASK_COMMON_H
#define DIFF_DRIVE_TASK_COMMON_H

#define DIFF_DRIVE_STACK_SIZE          4096
#define DIFF_DRIVE_CORE_ID             0
#define DIFF_DRIVE_TASK_PRIORITY       9

#define DIFF_DRIVE_POS_KP              0.4f
#define DIFF_DRIVE_POS_KD              0.2f

#define DIFF_DRIVE_ORI_KP              0.4f
#define DIFF_DRIVE_ORI_KD              0.2f

#define X_D                            4.00f
#define Y_D                            4.00f
#define THETA_D                        M_PI
#define V_COMM                         1.00f
#define V_MAX                          2.00f
#define WHEEL_RADIUS                   0.033f
#define DISTANCE_TH                    0.05f        // 5 cm
#define ORIENTATION_TH                 0.0872665 // 5°
#define RADS2REVS(b) (b * 0.1592f)

#endif