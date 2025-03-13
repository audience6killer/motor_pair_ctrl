#ifndef DATA_CENTER_H
#define DATA_CENTER_H

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum {
    STOPPED_TX = 0,
    SENDING 
} data_center_status_e;

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