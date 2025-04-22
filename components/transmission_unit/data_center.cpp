
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
#include <cstring>

typedef enum
{
    KALMAN_AND_TRACTION_DATA,
    KALMAN_DATA,
    TRACTION_DATA,
    NONE,
} data_center_sources_e;

const char TAG[] = "data_center";

static QueueHandle_t g_data_center_queue_handle = NULL;
static kalman_info_t kalman_data;
static motor_pair_data_t traction_data;
static QueueHandle_t g_lora_data2send_queue = NULL;
static QueueHandle_t g_lora_received_queue = NULL;
static esp_timer_handle_t g_data_center_timer = NULL;
static data_center_send_status_e g_data_center_send_status = STOPPED_TX;
static data_center_reception_e g_data_center_reception_status = STOPPED_RX;
static data_center_sources_e g_data_center_sources = NONE;
static data_center_msg_t g_data_center_msg;

esp_err_t data_center_get_queue_handle(QueueHandle_t *queue)
{
    ESP_RETURN_ON_FALSE(queue != NULL, ESP_ERR_INVALID_STATE, TAG, "Queue is NULL");

    *queue = g_data_center_queue_handle;

    return ESP_OK;
}

static void data_center_send_data(void *args)
{
    char frame[RF_DATA_LENGTH];
    memset(frame, 0, RF_DATA_LENGTH);

    char code[4];
    memset(code, 0, 4);

    switch (g_data_center_sources)
    {
    case KALMAN_AND_TRACTION_DATA:
        strcpy(code, "KT");
        break;

    case KALMAN_DATA:
        strcpy(code, "K");
        break;

    case TRACTION_DATA:
        strcpy(code, "T");
        break;

    case NONE:
    default:
        strcpy(code, "N");
        break;
    }

    switch (g_data_center_sources)
    {
    case KALMAN_AND_TRACTION_DATA:
        sprintf(frame, "/*%s,x,%.4f,y,%.4f,theta,%.4f,x_p,%.4f,y_p,%.4f,z_p,%.4f,theta_p,%.4f,mleft_real_pulses,%d,mright_real_pulses,%d,mleft_set_point,%d,mright_set_point,%d*/",
                code, kalman_data.x, kalman_data.y, kalman_data.theta, kalman_data.x_p, kalman_data.y_p, kalman_data.z_p, kalman_data.theta_p,
                traction_data.mleft_pulses, traction_data.mright_pulses, traction_data.mleft_set_point, traction_data.mright_set_point);
        break;

    case KALMAN_DATA:
        sprintf(frame, "/*code,%s,x,%.4f,y,%.4f,theta,%.4f,x_p,%.4f,y_p,%.4f,z_p,%.4f,theta_p,%.4f*/",
                code, kalman_data.x, kalman_data.y, kalman_data.theta, kalman_data.x_p, kalman_data.y_p, kalman_data.z_p, kalman_data.theta_p);
        break;

    case TRACTION_DATA:
        sprintf(frame, "/*code,%s,mleft_real_pulses,%d,mright_real_pulses,%d,mleft_set_point,%d,mright_set_point,%d*/",
                code, traction_data.mleft_pulses, traction_data.mright_pulses, traction_data.mleft_set_point, traction_data.mright_set_point);
        break;

    case NONE:
    default:
        // Handle the case where no data is available
        sprintf(frame, "/*code,%s*/", code);
        break;
    }

    if (xQueueSend(g_lora_data2send_queue, frame, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending data to queue");
    }
}

esp_err_t data_center_start_receiving(void)
{
    ESP_LOGI(TAG, "Data center start receiving");

    if (g_data_center_reception_status == RECEIVING)
    {
        ESP_LOGW(TAG, "Data center is already receiving data");
        return ESP_OK;
    }

    g_data_center_reception_status = RECEIVING;

    return ESP_OK;
}

esp_err_t data_center_stop_receiving(void)
{
    ESP_LOGI(TAG, "Data center stop receiving");

    if (g_data_center_reception_status == STOPPED_RX)
    {
        ESP_LOGW(TAG, "Data center is already stopped receiving data");
        return ESP_OK;
    }

    g_data_center_reception_status = STOPPED_RX;
    return ESP_OK;
}

esp_err_t data_center_start_sending(void)
{
    ESP_LOGI(TAG, "Data center start sending");

    if (g_data_center_send_status == SENDING)
    {
        ESP_LOGW(TAG, "Data center is already sending data");
        return ESP_OK;
    }

    ESP_ERROR_CHECK(esp_timer_start_periodic(g_data_center_timer, DATA_CENTER_SEND_PERIOD_MS * 1000));

    g_data_center_send_status = SENDING;
    return ESP_OK;
}

esp_err_t data_center_stop_sending(void)
{
    ESP_LOGI(TAG, "Data center stop sending");

    if (g_data_center_send_status == STOPPED_TX)
    {
        ESP_LOGW(TAG, "Data center is already stopped");
        return ESP_OK;
    }

    ESP_ERROR_CHECK(esp_timer_stop(g_data_center_timer));

    g_data_center_send_status = STOPPED_TX;

    return ESP_OK;
}

esp_err_t data_center_send2queue(data_center_msg_t *msg)
{
    if (g_data_center_queue_handle == NULL)
    {
        ESP_LOGE(TAG, "Data center queue not initialized");
        return ESP_FAIL;
    }

    if (xQueueSend(g_data_center_queue_handle, msg, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending data to queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief The data received have the format: / * code,(char[3]),(args) * /
 * depending on the code there will be different args or none.
 * Examples: / *NVP,15.00,17.00* / -> Navigation point, x = 15.00, y = 17.00
 *
 * @param data
 * @return esp_err_t
 */
esp_err_t data_center_parse_data(char *data)
{
    char code[4];

    // Check if the data is valid
    if (strstr(data, "/*") == NULL || strstr(data, "*/") == NULL)
    {
        ESP_LOGE(TAG, "Invalid data format");
        return ESP_FAIL;
    }

    // Extract the code
    sscanf(data, "/*%3s", code);

    // Check the code
    if (strcmp(code, "NVP") == 0)
    {
        float x, y;
        sscanf(data, "/*%*s,%f,%f*/", &x, &y);
        ESP_LOGI(TAG, "Navigation point received: x = %.2f, y = %.2f", x, y);
    }
    else if (strcmp(code, "STN") == 0)
    {
        ESP_LOGI(TAG, "Start navigation received");
    }
    else if (strcmp(code, "SPN") == 0)
    {
        ESP_LOGI(TAG, "Stop navigation received");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid code");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void data_center_recolect_data(void *args)
{
    
}

static void data_center_receive_task(void *args)
{
    ESP_LOGI(TAG, "Iniatiliazing data center receiving task");
    ESP_ERROR_CHECK(lora_get_queue_data_received(&g_lora_received_queue));

    char received_data[RF_DATA_LENGTH];
    memset(received_data, 0, RF_DATA_LENGTH);

    g_data_center_msg.code = EMPTY;
    memset(g_data_center_msg.agrs, 0, sizeof(g_data_center_msg.agrs));

    for (;;)
    {
        if (xQueueReceive(g_lora_received_queue, received_data, pdMS_TO_TICKS(100)) == pdPASS)
        {
            ESP_ERROR_CHECK(data_center_parse_data(received_data));
            ESP_ERROR_CHECK(data_center_send2queue(&g_data_center_msg));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void data_center_send_task(void *args)
{
    ESP_LOGI(TAG, "Iniatiliazing sendig task");

    esp_timer_create_args_t data_center_timer_args = {
        .callback = data_center_recolect_data,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_ISR,
        .name = "data_center_timer",
        .skip_unhandled_events = false
    };

    ESP_ERROR_CHECK(esp_timer_create(&data_center_timer_args, &g_data_center_timer));

    ESP_LOGI(TAG, "Starting sending periodic timer");
    ESP_ERROR_CHECK(esp_timer_start_periodic(g_data_center_timer, DATA_CENTER_SEND_PERIOD_MS));

    for (;;)
    {
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void data_center_task_start(void)
{
    ESP_LOGI(TAG, "Data center task started");

    g_data_center_queue_handle = xQueueCreate(5, sizeof(data_center_msg_t));

    xTaskCreatePinnedToCore(data_center_send_task, "dc_send_task", DATA_CENTER_TASK_STACK_SIZE, NULL, DATA_CENTER_TASK_PRIORITY, NULL, DATA_CENTER_TASK_CORE_ID);

    xTaskCreatePinnedToCore(data_center_receive_task, "dc_receive_task", DATA_CENTER_TASK_STACK_SIZE, NULL, DATA_CENTER_TASK_PRIORITY, NULL, DATA_CENTER_TASK_CORE_ID);
}