#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "traction_control.h"

void app_main(void)
{
    traction_control_start_task();
}