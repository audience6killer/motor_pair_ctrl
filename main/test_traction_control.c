#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "test_traction_control.h"
#include "traction_control.h"

static const char TAG[] = "test_traction_control";

static void test_traction_control_task(void *pvParameters)
{
    const float target_speed = 1.688f;
    // traction_control_smooth_start(target_speed);
    traction_control_set_direction(FORWARD);
    traction_control_set_speed(target_speed, target_speed);
    for(;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    } 
}

esp_err_t test_traction_control_task_begin(void)
{
    ESP_LOGI(TAG, "Creating task");
    xTaskCreatePinnedToCore(&test_traction_control_task, 
                            "test_traction_control",
                            4096,
                            NULL,
                            10,
                            NULL,
                            0);

    return ESP_OK;
}

