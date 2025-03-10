
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "waypoint_controller.h"
}

#include "Arduino.h"

#include "test_waypoint_follower.h"

#define CORE_ID 0
#define STACK_SIZE 4096
#define PRIORITY 2

static const char TAG[] = "test_waypoint_task";

void start_test_waypoint_follower(void)
{
    ESP_LOGI(TAG, "Starting test_waypoin_task");

    ESP_ERROR_CHECK(waypoint_controller_start_trajectory());
}

static void test_waypoint_task(void *pvParemeters)
{
    ESP_LOGI(TAG, "Configuring waypoints for test");

    ESP_ERROR_CHECK(waypoint_controller_add_point(0.0f, 0.0f, 3.1516f));
    ESP_ERROR_CHECK(waypoint_controller_add_point(10.0f, 0.0f, 0.0f));

    ESP_LOGI(TAG, "Waiting for start signal");

    for (;;)
    {
        if(Serial.available())
        {
            char c = Serial.read();
            if(c == 's')
            {
                start_test_waypoint_follower();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void test_waypoint_follower_task_start(void)
{

    ESP_LOGI(TAG, "Iniatilazing waypoint task");

    xTaskCreatePinnedToCore(&test_waypoint_task, "test_waypoint", STACK_SIZE, NULL, PRIORITY, NULL, CORE_ID);
}
