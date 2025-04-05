
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_event.h"

#include "traction_control.h"
#include "kalman_filter.h"
#include "diff_drive_ctrl.h"
#include "waypoint_controller.h"
}

#include "Arduino.h"

#include "test_waypoint_follower.h"

#define CORE_ID 0
#define STACK_SIZE 4096
#define PRIORITY 2

static const char TAG[] = "test_waypoint_task";
static TaskHandle_t g_test_task_pv = NULL;
static esp_event_loop_handle_t g_waypoint_event_handle = NULL;
static QueueHandle_t g_waypoint_queue_handle = NULL;
static QueueHandle_t g_kalman_data_queue = NULL;
static QueueHandle_t g_tract_data_queue = NULL;
static QueueHandle_t g_diff_drive_queue = NULL;
static kalman_state_e g_kalman_last_state = KALMAN_STOPPED;

void test_kalman_receive_data(void)
{
    static kalman_info_t kalman_data;

    if (xQueueReceive(g_kalman_data_queue, &kalman_data, 0) == pdPASS)
    {
#if true
        if (kalman_data.state != g_kalman_last_state)
        {
            g_kalman_last_state = kalman_data.state;

            const char *state = kalman_state_to_name(kalman_data.state);
            printf("/*KALMAN: x,%.4f,y,%.4f,z,%.4f,theta,%.4f,x_p,%.4f,y_p,%.4f,z_p,%.4f,thetap,%.4f,state,%s*/\r\n", kalman_data.x, kalman_data.y, kalman_data.z, kalman_data.theta, kalman_data.x_p, kalman_data.y_p, kalman_data.z_p, kalman_data.theta_p, state);
        }
#endif
    }
}

void test_tract_receive_data(void)
{
    static motor_pair_data_t tract_data;
    if (xQueueReceive(g_tract_data_queue, &tract_data, pdMS_TO_TICKS(5)) == pdTRUE)
    {
#if true
        const char *tract_state_str = motor_pair_state_2_string(tract_data.state);
        // ESP_LOGI(TAG, "WHOOSJd");
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

        // if (diff_drive_state.state == DD_READY)
        //{
        //     ESP_LOGI(TAG, "Command timer started");
        // }
#endif
    }
}

void test_waypoint_receive_data(void)
{
    static waypoint_ctrl_state_e waypoint_state;
    if (xQueueReceive(g_waypoint_queue_handle, &waypoint_state, 0) == pdPASS)
    {
        const char *state_str = waypoint_ctrl_state_2_string(waypoint_state);
        printf("/*WAYPOINT: state,%s*/\r\n", state_str);
    }
}

static void test_waypoint_task(void *pvParemeters)
{
    ESP_LOGI(TAG, "Initializing waypoint test. Waiting for waypoint task to finish");

    /* Wait for waypoint to end initializing*/
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "Waypoint finished initializing");

    esp_event_loop_handle_t kalman_event_loop = NULL;
    ESP_ERROR_CHECK(kalman_get_event_loop(&kalman_event_loop));

    waypoint_ctrl_get_event_loop(&g_waypoint_event_handle);
    waypoint_ctrl_get_queue_handle(&g_waypoint_queue_handle);

    while (diff_drive_get_queue_handle(&g_diff_drive_queue))
    {
        ESP_LOGW(TAG, "Retrying getting diff drive queue...");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    while (tract_ctrl_get_data_queue(&g_tract_data_queue))
    {
        ESP_LOGW(TAG, "Retrying getting tract queue...");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    while (kalman_get_data_queue(&g_kalman_data_queue))
    {
        ESP_LOGW(TAG, "Retrying getting kalman queue...");
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // ESP_ERROR_CHECK(waypoint_controller_add_point(0.0f, 0.0f, 3.1516f));

    ESP_LOGI(TAG, "Waiting for start signal");

    // ESP_ERROR_CHECK(esp_event_post_to(g_waypoint_event_handle, WAYPOINT_EVENT_BASE, WP_START_EVENT, NULL, 0, portMAX_DELAY));

    for (;;)
    {
        if (Serial.available())
        {
            char c = Serial.read();
            if (c == 's')
            {
                ESP_LOGI(TAG, "Starting waypoint follower");

                navigation_point_t point = {
                    .x = 10.0f,
                    .y = 100.0f,
                    .theta = 30.0f,
                };
                ESP_ERROR_CHECK(esp_event_post_to(g_waypoint_event_handle, WAYPOINT_EVENT_BASE, WP_ADD_POINT_EVENT, &point, sizeof(point), pdMS_TO_TICKS(10)));
                ESP_ERROR_CHECK(esp_event_post_to(g_waypoint_event_handle, WAYPOINT_EVENT_BASE, WP_START_EVENT, NULL, 0, pdMS_TO_TICKS(10)));

                /* Start kalman data sending */
                ESP_ERROR_CHECK(esp_event_post_to(kalman_event_loop, KALMAN_EVENT_BASE, KALMAN_START_EVENT, NULL, 0, pdMS_TO_TICKS(10)));
                // start_test_waypoint_follower();
            }
        }

        test_diff_drive_recive_data();
        test_tract_receive_data();
        test_kalman_receive_data();
        test_waypoint_receive_data();

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void test_waypoint_follower_task_start(void)
{

    ESP_LOGI(TAG, "Starting waypoint task");

    xTaskCreatePinnedToCore(&test_waypoint_task, "test_waypoint", STACK_SIZE, NULL, PRIORITY, &g_test_task_pv, CORE_ID);

    waypoint_ctrl_start_task(g_test_task_pv);
}
