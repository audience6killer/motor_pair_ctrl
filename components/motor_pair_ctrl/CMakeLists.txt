idf_component_register(SRCS "motor_pair_ctrl.c"
                    INCLUDE_DIRS "include"
                    REQUIRES queue esp_timer driver bdc_motor pid_ctrl)
