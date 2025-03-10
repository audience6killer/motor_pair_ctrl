extern "C"
{
#include "traction_control.h"
// #include "test_traction_control.h"
#include "kalman_filter.h"
#include "odometry_unit.h"
#include "differential_drive_ctrl.h"
#include "waypoint_controller.h"
    // #include "seed_planter_control.h"
    // #include "test_seed_planter.h"
}

#include "test_waypoint_follower.h"
#include "Arduino.h"

extern "C" void app_main(void)
{
    initArduino();

    Serial.begin(115200);

    traction_control_start_task();
    odometry_unit_start_task();
    kalman_filter_start_task();
    diff_drive_ctrl_task_start();
    waypoint_controller_start_task();

    test_waypoint_follower_task_start();
    // seed_planter_control_start_task();
    // test_seed_planter_start_task();

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}