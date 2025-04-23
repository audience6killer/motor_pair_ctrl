#ifndef FAKE_DATA_CENTER_H
#define FAKE_DATA_CENTER_H


#include "state_machine.h"

typedef struct {
    state_machine_cmd_e code;
    float args[3];
} data_center_msg_t;

esp_err_t data_center_get_queue_handle(QueueHandle_t *queue);

void fake_data_center_task_start(void);


#endif // FAKE_DATA_CENTER_H