idf_component_register(SRCS "traction_control.c"
                    INCLUDE_DIRS "include"
                    REQUIRES motor_pair_ctrl esp_timer)

