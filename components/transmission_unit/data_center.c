
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"

#include "data_center.h"
#include "data_center_task_common.h"

const char TAG[] = "data_center";

static void data_center_task(void *args)
{

}

void data_center_task_start(void)
{
    ESP_LOGI(TAG, "Data center task started");

    xTaskCreatePinnedToCore(data_center_task, "data_center_task", DATA_CENTER_TASK_STACK_SIZE, NULL, DATA_CENTER_TASK_PRIORITY, NULL, DATA_CENTER_TASK_CORE_ID);
}