
extern "C"
{
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/queue.h"
    #include "esp_log.h"
    #include "esp_check.h"

    #include "data_center.h"
    #include "waypoint_controller.h"

    #include "state_machine.h"
    #include "state_machine_task_common.h"
}

static const char TAG[] = "state_machine";
static QueueHandle_t g_state_machine_queue_handle = NULL;
static waypoint_state_e g_waypoint_state = WP_STOPPED;

esp_err_t state_machine_send2queue(state_machine_msg_t *cmd)
{
    if (g_state_machine_queue_handle == NULL)
    {
        ESP_LOGE(TAG, "Queue handle is NULL");
        return ESP_FAIL;
    }

    if (xQueueSend(g_state_machine_queue_handle, &cmd, pdMS_TO_TICKS(100)) != pdPASS)
    {
        ESP_LOGE(TAG, "Error sending data to queue");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t state_machine_cmd_stop_nav(void)
{
    state_machine_msg_t cmd;
    cmd.code = CMD_STOP;

    return state_machine_send2queue(&cmd);
}

esp_err_t state_machine_cmd_start_nav(void)
{
    state_machine_msg_t cmd;
    cmd.code = CMD_START;

    return state_machine_send2queue(&cmd);
}

esp_err_t state_machine_cmd_navpoint(float x, float y, float theta)
{
    state_machine_msg_t cmd;
    cmd.code = CMD_WAYPOINT;
    cmd.agrs[0] = x;
    cmd.agrs[1] = y;
    cmd.agrs[2] = theta;

    return state_machine_send2queue(&cmd);
}

esp_err_t state_machine_cmd_echo(void)
{
    state_machine_msg_t cmd;
    cmd.code = CMD_ECHO;

    return state_machine_send2queue(&cmd);
}

static void state_machine_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Initilizing State machine task started");

    g_state_machine_queue_handle = xQueueCreate(10, sizeof(state_machine_msg_t));
    state_machine_msg_t command;

    // Data center queue
    QueueHandle_t data_center_queue = NULL;
    ESP_ERROR_CHECK( data_center_get_queue_handle(&data_center_queue) );
    data_center_msg_t data_center_msg;

    for(;;)
    {
        if(xQueueReceive(data_center_queue, &data_center_msg, pdMS_TO_TICKS(100)) == pdPASS)
        {
            switch (data_center_msg.code)
            {
            case EMPTY:
                ESP_LOGW(TAG, "Empty message received");
                break;
            case ECHO:
                ESP_LOGI(TAG, "Echo message received");
                ESP_ERROR_CHECK(state_machine_cmd_echo());
                break;
            case NAV_POINT:
                ESP_LOGI(TAG, "Navigation point message received");
                ESP_ERROR_CHECK(state_machine_cmd_navpoint(data_center_msg.agrs[0], data_center_msg.agrs[1], data_center_msg.agrs[2]));
                break;
            case START_NAV:
                ESP_LOGI(TAG, "Start navigation message received");
                ESP_ERROR_CHECK(state_machine_cmd_start_nav());
                break;
            case STOP_NAV:
                ESP_LOGI(TAG, "Stop navigation message received");
                ESP_ERROR_CHECK(state_machine_cmd_stop_nav());
                break;
            default:
                ESP_LOGE(TAG, "Invalid message received");
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


void state_machine_task_start(void)
{
    ESP_LOGI(TAG, "State machine task started");

    xTaskCreatePinnedToCore(state_machine_task, "state_machine_task", STATE_MACHINE_TASK_STACK_SIZE, NULL, STATE_MACHINE_TASK_PRIORITY, NULL, STATE_MACHINE_TASK_CORE_ID);
}