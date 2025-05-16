#ifndef COM_RF_TASK_COMMON_H
#define COM_RF_TASK_COMMON_H

// Task configuration
#define RF_TASK_CORE_ID             0
#define RF_TASK_RECEIVE_PRIORITY    4
#define RF_TASK_TRANSFER_PRIORITY   2
#define RF_TASK_STACK_SIZE          1024 * 6

// UART configuration
#define RF_UART_BUFFER_SIZE         256
#define RF_UART_QUEUE_SIZE          5
#define RF_UART_PORT                UART_NUM_1
#define RF_UART_TX_PIN              GPIO_NUM_17
#define RF_UART_RX_PIN              GPIO_NUM_16
#define RF_UART_RTS_PIN             UART_PIN_NO_CHANGE
#define RF_UART_CTS_PIN             UART_PIN_NO_CHANGE
#define RF_UART_BAUDRATE_RF         115200
#define RF_UART_DATA_BITS           UART_DATA_8_BITS
#define RF_DATA_LENGTH              200     // In bytes
#define RF_AUX_PIN                  GPIO_NUM_33

#define WAIT_QUEUE_SEND_RF          100




#endif