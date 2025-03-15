extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"
#include <cstring>

#include "data_center.h"
#include "test_data_center.h"
#include "test_data_center_task_common.h"
}

const char TAG[] = "TEST_DATA_CENTER";


void test_data_center_task(void *args)
{
    ESP_LOGI(TAG, "Test data center task started");
    
    // All units tasks must be created in the main.cpp file
    QueueHandle_t data_center_queue = NULL;
    char data[200];
    memset(data, 0, 200);

    ESP_ERROR_CHECK( data_center_get_queue_handle(&data_center_queue) );

    ESP_ERROR_CHECK( data_center_start_receiving() );


    for (;;)
    {
        if(xQueueReceive(data_center_queue, data, pdMS_TO_TICKS(100)) == pdPASS)
        {
            ESP_LOGI(TAG, "Data received: %s", data);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void test_data_center_start_task(void)
{
    ESP_LOGI("TEST_DATA_CENTER", "Test data center task started");
    xTaskCreatePinnedToCore(test_data_center_task, "test_data_center_task", TEST_DATA_CENTER_TASK_STACK_SIZE, NULL, TEST_DATA_CENTER_TASK_PRIORITY, NULL, TEST_DATA_CENTER_TASK_CORE_ID);
}




