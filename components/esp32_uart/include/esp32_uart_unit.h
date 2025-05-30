#ifndef ESP32_UART_UNIT_H
#define ESP32_UART_UNIT_H

typedef enum 
{
    ESP32_UART_CMD_EMPTY = 0, 
    ESP32_UART_CMD_STOP_CUTTER,         // SPC
    ESP32_UART_CMD_START_CUTTER,        // STC
    ESP32_UART_CMD_TEMPERATURE,         // TEM
    ESP32_UART_CMD_HUMEDITY,            // HUM
    ESP32_UART_CMD_RPMS,                // RPM
    ESP32_UART_CMD_MOTOR_LINEAR_UP,     // IUP
    ESP32_UART_CMD_MOTOR_LINEAR_DW,     // IDW
    ESP32_UART_CMD_MOTOR_LINEAR_TR,     // ITR
    ESP32_UART_CMD_MOTOR_LINEAR_ER,     // ERM
} esp32_uart_cmd_e;

typedef struct
{
    esp32_uart_cmd_e code;
    float arg;
} esp32_uart_cmd_t;

typedef enum
{
    NONE = 0, 
    CUTTER_DISK_EVENT_ERROR,
} cutter_disk_event_e; 

typedef enum
{
    CUTTER_DISK_GENERIC_ERROR = 0, 
} cutter_disk_error_e; 

typedef struct 
{
    cutter_disk_event_e event;
    cutter_disk_error_e error;
} cutter_disk_event_t;

esp_err_t esp32_uart_get_transmit_data_queue(QueueHandle_t *handle);

esp_err_t esp32_uart_get_received_data_queue(QueueHandle_t *handle);

/**
 * @brief Starts the RF communication task. 
 * 
 * @return 
 */
void esp32_uart_task_start(void);

#endif
