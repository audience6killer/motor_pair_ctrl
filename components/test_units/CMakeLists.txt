idf_component_register(SRCS "test_diff_drive.c" "test_data_center.cpp" "test_waypoint_follower.cpp" "test_traction_control.c"
                    INCLUDE_DIRS "include"
                    REQUIRES motor_pair_ctrl seed_planter_control traction_control waypoint_controller arduino-esp32 transmission_unit differential_drive_ctrl)
