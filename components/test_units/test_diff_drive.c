
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "traction_control.h"
#include "diff_drive_ctrl.h"
#include "kalman_filter.h"

#include "test_diff_drive.h"

static const char *TAG = "test_diff_drive";

static QueueHandle_t g_diff_drive_queue = NULL;
static QueueHandle_t g_tract_data_queue = NULL;
static QueueHandle_t g_kalman_data_queue = NULL;
static esp_event_loop_handle_t g_diff_drive_event_loop;
static esp_timer_handle_t g_timer_handle = NULL;

static void test_diff_simulate_event(void *args)
{
    ESP_LOGI(TAG, "Sending command to event loop");
    const navigation_point_t point = (const navigation_point_t){
        .theta = 0.0f,
        .x = 10.0f,
        .y = 77.0f,
    };

    ESP_ERROR_CHECK(esp_event_post_to(g_diff_drive_event_loop, DIFF_DRIVE_EVENT_BASE, DIFF_DRIVE_RECEIVE_POINT_EVENT, &point, sizeof(point), pdMS_TO_TICKS(100)));
}

void test_kalman_receive_data(void)
{
    static kalman_info_t kalman_data;

    if(xQueueReceive(g_kalman_data_queue, &kalman_data, 0) == pdPASS)
    {
#if true
        const char *state = kalman_state_to_name(kalman_data.state);
        printf("/*KALMAN: x,%.4f,y,%.4f,z,%.4f,theta,%.4f,x_p,%.4f,y_p,%.4f,z_p,%.4f,thetap,%.4f,state,%s*/\r\n", kalman_data.x, kalman_data.y, kalman_data.z, kalman_data.theta, kalman_data.x_p, kalman_data.y_p, kalman_data.z_p, kalman_data.theta_p, state);
#endif
    }
}

void test_tract_receive_data(void)
{
    static motor_pair_data_t tract_data;
    if (xQueueReceive(g_tract_data_queue, &tract_data, portMAX_DELAY) == pdPASS)
    {
#if true 
        const char *tract_state_str = motor_pair_state_2_string(tract_data.state);
        printf("/*TRACT: mleft_pulses,%d,mright_pulses,%d,mleft_set_point,%d,mright_set_point,%d,state,%s*/\r\n", tract_data.mleft_pulses, tract_data.mright_pulses, tract_data.mleft_set_point, tract_data.mright_set_point, tract_state_str);
#endif
    }
}

void test_diff_drive_recive_data(void)
{
    static diff_drive_state_t diff_drive_state;
    if (xQueueReceive(g_diff_drive_queue, &diff_drive_state, 0) == pdPASS)
    {
#if true
        const char *state_str = diff_drive_state_2_string(diff_drive_state.state);
        printf("/*DIFF_DRIVE: errx,%.4f,erry,%.4f,errtheta,%.4f,errdist,%.4f,errori,%.4f,state,%s*/\r\n", diff_drive_state.err_x, diff_drive_state.err_y, diff_drive_state.err_theta, diff_drive_state.err_dist, diff_drive_state.err_ori, state_str);

        if (diff_drive_state.state == DD_READY)
        {
            ESP_LOGI(TAG, "Command timer started");
            ESP_ERROR_CHECK(esp_timer_start_once(g_timer_handle, 1e6));
        }
#endif
    }
}

static void test_diff_drive_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initializing test_diff_drive");

    // Wait fot diff drive task to finish initializing
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Iniitializing...");

    ESP_ERROR_CHECK(diff_drive_get_queue_handle(&g_diff_drive_queue));
    ESP_ERROR_CHECK(tract_ctrl_get_data_queue(&g_tract_data_queue));
    ESP_ERROR_CHECK(kalman_get_data_queue(&g_kalman_data_queue));

    ESP_ERROR_CHECK(diff_drive_get_event_loop(&g_diff_drive_event_loop));

    /* Simulate the waypoints being sended to the diff drive */
    esp_timer_create_args_t timer_args = {
        .callback = test_diff_simulate_event,
        .arg = NULL,
        .name = "diff_drive_pid_loop",
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &g_timer_handle));

    for (;;)
    {
        test_diff_drive_recive_data();
        test_tract_receive_data();
        test_kalman_receive_data();

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void test_diff_drive_task_start(void)
{
    ESP_LOGI(TAG, "Starting test diff drive");

    TaskHandle_t parent = NULL;

    xTaskCreatePinnedToCore(&test_diff_drive_task, "test_diff_drive", 4096, NULL, 10, &parent, 0);

    diff_drive_task_start(parent);
}