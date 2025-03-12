idf_component_register(SRCS "waypoint_controller.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES differential_drive_ctrl esp_timer)
