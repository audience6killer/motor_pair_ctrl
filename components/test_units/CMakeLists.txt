idf_component_register(SRCS "test_waypoint_follower.c" "test_seed_planter.c" "test_traction_control.c"
                    INCLUDE_DIRS "include"
                    REQUIRES motor_pair_ctrl seed_planter_control traction_control waypoint_controller)
