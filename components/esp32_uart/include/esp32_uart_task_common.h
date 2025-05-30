#ifndef ESP32_UART_TASK_COMMON_H
#define ESP32_UART_TASK_COMMON_H

// Task configuration
#define ESP32_UART_TASK_CORE_ID                 0
#define ESP32_UART_TASK_RECEIVE_PRIORITY        4
#define ESP32_UART_TASK_TRANSFER_PRIORITY       2
#define ESP32_UART_TASK_STACK_SIZE              1024 * 2

// UART configuration
#define ESP32_UART_BUFFER_SIZE                  256
#define ESP32_UART_QUEUE_SIZE                   5
#define ESP32_UART_PORT                         UART_NUM_1
#define ESP32_UART_TX_PIN                       GPIO_NUM_18
#define ESP32_UART_RX_PIN                       GPIO_NUM_19
#define ESP32_UART_RTS_PIN                      UART_PIN_NO_CHANGE
#define ESP32_UART_CTS_PIN                      UART_PIN_NO_CHANGE
#define ESP32_UART_BAUDRATE_RF                  115200
#define ESP32_UART_DATA_BITS                    UART_DATA_8_BITS
#define ESP32_DATA_LENGTH                       200     // In bytes

#define WAIT_QUEUE_SEND_ESP32                   100




#endif