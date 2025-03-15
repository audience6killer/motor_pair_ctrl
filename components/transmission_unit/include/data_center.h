#ifndef DATA_CENTER_H
#define DATA_CENTER_H

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum {
    STOPPED_TX = 0,
    SENDING 
} data_center_send_status_e;

typedef enum {
    STOPPED_RX = 0,
    RECEIVING
} data_center_reception_e;

typedef enum {
    EMPTY = 0,
    ECHO,       // Test communication
    NAV_POINT,      // Send Navigation point
    START_NAV,      // Start navigation
    STOP_NAV,       // Stop navigation
} data_center_rx_codes_e;   // Data center reception codes

typedef struct {
    data_center_rx_codes_e code;
    float agrs[3];
} data_center_msg_t;

 // Task state
typedef enum {
    INIT = 0,
    READY,
} data_center_state_e; 

/**
 * @brief Get the data center queue handle object. 
 * 
 * @param queue 
 * @return esp_err_t 
 */
esp_err_t data_center_get_queue_handle(QueueHandle_t *queue);

/**
 * @brief Start receiving data from the RF module. 
 * 
 * @return esp_err_t 
 */
esp_err_t data_center_start_receiving(void);

/**
 * @brief Stop receiving data from the RF module. 
 * 
 * @return esp_err_t 
 */
esp_err_t data_center_stop_receiving(void);

/**
 * @brief Start sending data to the RF module.
 * 
 * @return esp_err_t 
 */
esp_err_t data_center_start_sending(void);

/**
 * @brief Stop sending data to the RF module.
 * 
 * @return esp_err_t 
 */
esp_err_t data_center_stop_sending(void);

/**
 * @brief Data center task initialization. 
 * 
 */
void data_center_task_start(void);

#ifdef __cplusplus
}   // extern "C"
#endif

#endif