
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_event.h"
#include "esp_log.h"

#include "diff_drive_ctrl.h"

#include "test_diff_drive.h"

static const char *TAG = "test_diff_drive";

static void test_diff_drive_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing test_diff_drive");

    // Wait fot diff drive task to finish initializing
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Iniitializing...");

    QueueHandle_t diff_drive_data_queue = NULL;

    ESP_ERROR_CHECK(diff_drive_get_queue_handle(&diff_drive_data_queue));
    diff_drive_state_t diff_drive_state;

    const char *state = "NULL";
    for (;;)
    {
        if (xQueueReceive(diff_drive_data_queue, &diff_drive_state, portMAX_DELAY) == pdPASS)
        {
#if true
            const char *state_str = diff_drive_state_2_string(diff_drive_state.state);
            printf("errx,%.4f,erry,%.4f,errtheta,%.4f,errdist,%.4f,errori,%.4f,diffstate,%s\r\n", diff_drive_state.err_x, diff_drive_state.err_y, diff_drive_state.err_theta,diff_drive_state.err_dist, diff_drive_state.err_ori, state_str);
#endif
        }
        else
        {
            ESP_LOGE(TAG, "Error receiving diff drive data");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void test_diff_drive_task_start(void)
{
    ESP_LOGI(TAG, "Starting test diff drive");

    TaskHandle_t parent = NULL;

    xTaskCreatePinnedToCore(&test_diff_drive_task, "test_diff_drive", 4096, NULL, 10, &parent, 0);

    diff_drive_task_start(parent);
}