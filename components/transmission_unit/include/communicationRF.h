#ifndef COMMUNICATIONRF_H
#define COMMUNICATIONRF_H

/**
 * @brief Get the handle object 
 * 
 * @return QueueHandle_t 
 */
esp_err_t lora_get_queue_data2send(QueueHandle_t *handle);

esp_err_t lora_get_queue_data_received(QueueHandle_t *handle);

/**
 * @brief Starts the RF communication task. 
 * 
 * @return 
 */
void lora_task_start(void);

#endif
