#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "test_traction_control.h"
#include "traction_control.h"

#define TEST_TRACTION_PRIORITY      3

static const char TAG[] = "test_traction_control";



// Desired speed: 1.688rev/s @0.35m/s
static void test_traction_control_task(void *pvParameters)
{
    //const float target_speed = 0.850f;
    const float target_speed = 2.0f;
    const int tf = 500;

    // traction_control_set_direction(REVERSE);
    //traction_control_set_speed(target_speed, target_speed);
    //traction_control_speed_controlled_direction(target_speed, target_speed);

    //QueueHandle_t traction_queue_handle = traction_control_get_queue_handle();
    for (;;)
    {

        //if (xQueueReceive(traction_queue_handle, &traction_data, portMAX_DELAY) == pdPASS)
        //{
        //    // printf()
        //    printf("/*left_setpoint,%d,speed_left,%d,right_setpoint,%d,speed_right,%d,state,%d*/\r\n", traction_data.mleft_set_point, traction_data.mleft_real_pulses, traction_data.mright_real_pulses, traction_data.mright_real_pulses, traction_data.state);
        //    if (traction_data.state != STARTING && traction_data.state != STOPPED)
        //    {
        //        //ESP_ERROR_CHECK(traction_control_speed_controlled_direction(-1.00f, -1.00f));
        //    }
        //}
        //else
        //{
        //    ESP_LOGE(TAG, "Error receiving queue");
        //}
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

esp_err_t test_traction_control_task_begin(void)
{
    ESP_LOGI(TAG, "Creating task");
    xTaskCreatePinnedToCore(&test_traction_control_task, "test_traction_control", 4096, NULL, TEST_TRACTION_PRIORITY, NULL, 0);

    return ESP_OK;
}
