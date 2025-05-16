#ifndef COMMUNICATIONRF_H
#define COMMUNICATIONRF_H

#include "state_machine.h"

typedef struct {
    state_machine_cmd_e code;
    float args[3];
} data_center_msg_t;

/**
 * @brief Get the handle object 
 * 
 * @return QueueHandle_t 
 */

esp_err_t data_center_get_queue_handle(QueueHandle_t *queue);

esp_err_t lora_get_queue_data2send(QueueHandle_t *handle);

esp_err_t lora_get_queue_data_received(QueueHandle_t *handle);

/**
 * @brief Starts the RF communication task. 
 * 
 * @return 
 */
void lora_task_start(void);

#endif
