
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "test_seed_planter.h"
#include "motor_pair_ctrl.h"
#include "seed_planter_control.h"

#define SERIAL_DEBUG_ENABLE true

static const char TAG[] = "test_seed_planter";

static void test_seed_planter_task(void *pvParameters)
{
    QueueHandle_t seed_planter_handle = seed_planter_get_queue_handle();

    motor_pair_data_t seed_planter_data;
    
    const float target_speed = 1.688f;
    const int tf = 500;
    seed_planter_soft_start(target_speed, target_speed, tf);

    for (;;)
    {
        if (xQueueReceive(seed_planter_handle, &seed_planter_data, portMAX_DELAY) == pdPASS)
        {

#if SERIAL_DEBUG_ENABLE
            // printf()
            printf("/*cutter_setpoint,%d,speed_cutter,%d,dispenser_setpoint,%d,speed_dispenser,%d,state,%d*/\r\n", seed_planter_data.mleft_set_point, seed_planter_data.mleft_pulses, seed_planter_data.mright_set_point, seed_planter_data.mright_pulses, seed_planter_data.state);
#endif
        }
        else
        {
            ESP_LOGE(TAG, "Failed to receive queue");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void test_seed_planter_start_task(void)
{
    ESP_LOGI(TAG, "Iniatilazing seed_planter_test task");

    xTaskCreatePinnedToCore(&test_seed_planter_task, "test_seed_planter_task", 4096, NULL, 10, NULL, 0);
}