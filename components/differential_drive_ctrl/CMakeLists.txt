idf_component_register(SRCS "differential_drive_ctrl.c"
                    INCLUDE_DIRS "include"
                    REQUIRES pid_ctrl kalman_filter traction_control)
