#include "traction_control.h"
//#include "test_traction_control.h"
#include "odometry_unit.h"
#include "differential_drive_ctrl.h"
#include "waypoint_controller.h"
//#include "seed_planter_control.h"
//#include "test_seed_planter.h"


void app_main(void)
{
    traction_control_start_task();
    //test_traction_control_task_begin();
    odometry_unit_start_task();
    diff_drive_ctrl_task_start();
    waypoint_controller_start_task();
    //seed_planter_control_start_task();
    //test_seed_planter_start_task();
}