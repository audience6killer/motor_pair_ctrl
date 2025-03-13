
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"

#include "kalman_filter.h"
#include "traction_control.h"
#include "lora_rf_task_common.h"
#include "lora_rf_unit.h"
}

#include "data_center.h"
#include "data_center_task_common.h"

const char TAG[] = "data_center";

static QueueHandle_t g_data_center_queue_handle = NULL;
static kalman_info_t kalman_data;
static motor_pair_data_t traction_data;
static QueueHandle_t g_lora_data2send_queue = NULL;
static QueueHandle_t g_lora_received_queue = NULL;
static esp_timer_handle_t g_data_center_timer = NULL;
static data_center_status_e g_data_center_status = STOPPED_TX;

static void data_center_send_data(void *args)
{
    char frame[RF_DATA_LENGTH];

    for (size_t i = 0; i < RF_DATA_LENGTH; i++)
    {
        frame[i] = 0;
    }

    sprintf(frame, "x,%.4f,y,%.4f,theta,%.4f,x_p,%.4f,y_p,%.4f,z_p,%.4f,theta_p,%.4f,mleft_real_pulses,%d,mright_real_pulses,%d,mleft_set_point:%d, mright_set_point,%d",
            kalman_data.x, kalman_data.y, kalman_data.theta, kalman_data.x_p, kalman_data.y_p, kalman_data.z_p, kalman_data.theta_p, traction_data.mleft_real_pulses, traction_data.mright_real_pulses, traction_data.mleft_set_point, traction_data.mright_set_point);

    if (xQueueSend(g_lora_data2send_queue, frame, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending data to queue");
    }
}

esp_err_t data_center_start_sending(void)
{
    ESP_LOGI(TAG, "Data center start sending");

    if(g_data_center_status == SENDING)
    {
        ESP_LOGW(TAG, "Data center is already sending data");
        return ESP_OK;
    }

    ESP_ERROR_CHECK(esp_timer_start_periodic(g_data_center_timer, DATA_CENTER_SEND_PERIOD_MS * 1000));

    g_data_center_status = SENDING;
    return ESP_OK;
}

esp_err_t data_center_stop_sending(void)
{
    ESP_LOGI(TAG, "Data center stop sending");

    if(g_data_center_status == STOPPED_TX)
    {
        ESP_LOGW(TAG, "Data center is already stopped");
        return ESP_OK;
    }

    ESP_ERROR_CHECK(esp_timer_stop(g_data_center_timer));

    g_data_center_status = STOPPED_TX;

    return ESP_OK;
}

static void data_center_task(void *args)
{
    ESP_LOGI(TAG, "Iniatiliazing data center task");

    // Data sources
    QueueHandle_t traction_control_queue_pv = traction_control_get_queue_handle();
    QueueHandle_t kalman_filter_queue_pv = kalman_fiter_get_queue();

    // LoRa queues
    ESP_ERROR_CHECK(lora_get_queue_data2send(&g_lora_data2send_queue));
    ESP_ERROR_CHECK(lora_get_queue_data_received(&g_lora_received_queue));

    kalman_initialize_info(&kalman_data);
    motor_pair_init_data(&traction_data);

    esp_timer_create_args_t data_center_timer_args = {
        .callback = data_center_send_data,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "data_center_timer",
        .skip_unhandled_events = false};

    ESP_ERROR_CHECK(esp_timer_create(&data_center_timer_args, &g_data_center_timer));

    for (;;)
    {
        if(g_data_center_status == SENDING)
        {
            int traction_code = xQueueReceive(traction_control_queue_pv, &traction_data, portMAX_DELAY);
            int kalman_code = xQueueReceive(kalman_filter_queue_pv, &kalman_data, portMAX_DELAY);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void data_center_task_start(void)
{
    ESP_LOGI(TAG, "Data center task started");

    xTaskCreatePinnedToCore(data_center_task, "data_center_task", DATA_CENTER_TASK_STACK_SIZE, NULL, DATA_CENTER_TASK_PRIORITY, NULL, DATA_CENTER_TASK_CORE_ID);
}