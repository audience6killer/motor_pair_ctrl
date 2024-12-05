
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "test_waypoint_follower.h"
#include "waypoint_controller.h"

#define CORE_ID 0
#define STACK_SIZE 4096
#define PRIORITY 2

static const char TAG[] = "test_waypoint_task";

static void test_waypoint_task(void *pvParemeters)
{
    ESP_LOGI(TAG, "Adding point to follower");
    ESP_ERROR_CHECK(waypoint_controller_add_point(2.0f, 2.0f, 3.141516f));
    ESP_ERROR_CHECK(waypoint_controller_add_point(4.0f, 4.0f, 3.141516f));

    ESP_ERROR_CHECK(waypoint_controller_start_trajectory());

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void test_waypoint_follower_task_start(void)
{

    ESP_LOGI(TAG, "Iniatilazing waypoint task");

    xTaskCreatePinnedToCore(&test_waypoint_task, "test_waypoint", STACK_SIZE, NULL, PRIORITY, NULL, CORE_ID);
}
