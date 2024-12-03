#ifndef NAVIGATION_CONTROL_H
#define NAVIGATION_CONTROL_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "differential_drive_ctrl.h"

    esp_err_t navigation_add_nav_point(navigation_point_t point);
    void navigation_control_start_task(void);

#ifdef __cplusplus
}
#endif
#endif