idf_component_register(SRCS "navigation_control.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES differential_drive_ctrl odometry_unit)
