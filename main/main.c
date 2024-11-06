#include <stdio.h>

#include "traction_control.h"
#include "seed_planter_control.h"

void app_main(void)
{
    traction_control_start_task();
    seed_planter_control_start_task();
}