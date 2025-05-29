#ifndef COMMUNICATIONRF_H
#define COMMUNICATIONRF_H

#include "state_machine.h"

/**
 * @brief Get the handle object
 *
 * @return QueueHandle_t
 */

// When lora is busy the pin is 0
bool lora_is_available();

esp_err_t lora_get_received_data_queue(QueueHandle_t *queue);

esp_err_t lora_get_transmit_data_queue(QueueHandle_t *queue);

esp_err_t lora_get_queue_data2send(QueueHandle_t *handle);

esp_err_t lora_get_queue_data_received(QueueHandle_t *handle);

/**
 * @brief Starts the RF communication task.
 *
 * @return
 */
void lora_task_start(void);

#endif
