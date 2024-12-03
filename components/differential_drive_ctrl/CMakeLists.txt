idf_component_register(SRCS "differential_drive_ctrl.c"
                    INCLUDE_DIRS "include"
                    REQUIRES odometry_unit pid_ctrl)
