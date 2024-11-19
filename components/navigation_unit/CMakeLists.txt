idf_component_register(SRCS "navigation_unit.c"
                    INCLUDE_DIRS "include"
                    REQUIRES odometry_unit pid_ctrl)
