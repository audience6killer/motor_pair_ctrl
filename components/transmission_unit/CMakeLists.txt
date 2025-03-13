idf_component_register(SRCS "data_center.cpp" "lora_rf_unit.c"
                    INCLUDE_DIRS "include"
                    REQUIRES kalman_filter traction_control state_machine)
