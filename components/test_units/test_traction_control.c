#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "test_traction_control.h"
#include "traction_control.h"

#ifndef SERIAL_DEBUG_ENABLE
#define SERIAL_DEBUG_ENABLE true
#endif

static const char TAG[] = "test_traction_control";

static motor_pair_data_t traction_data;

//void test_speed_curve(int t, float *motor_left_speed, float *motor_right_speed)
//{
//}

// Desired speed: 1.688rev/s @0.35m/s
static void test_traction_control_task(void *pvParameters)
{
    //const float target_speed = 0.850f;
    const float target_speed = 1.688f;
    const int tf = 500;
    traction_control_soft_start(target_speed, tf);

    // traction_control_set_direction(REVERSE);
    // traction_control_set_speed(target_speed, target_speed);

    QueueHandle_t traction_queue_handle = traction_control_get_queue_handle();
    for (;;)
    {

        if (xQueueReceive(traction_queue_handle, &traction_data, portMAX_DELAY) == pdPASS)
        {
#if SERIAL_DEBUG_ENABLE
            // printf()
            printf("/*left_desired_speed,%f,speed_left,%f,right_des_speed,%f,speed_right,%f,state,%d*/\r\n", traction_data.motor_left_desired_speed, traction_data.motor_left_real_pulses, traction_data.motor_right_desired_speed, traction_data.motor_right_real_pulses, traction_data.state);
#endif
            if (traction_data.state != STARTING && traction_data.state != STOPPED)
            {
                ESP_ERROR_CHECK(traction_control_speed_controlled_direction(-1.00f, -1.00f));
            }
        }
        else
        {
            ESP_LOGE(TAG, "Error receiving queue");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
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
