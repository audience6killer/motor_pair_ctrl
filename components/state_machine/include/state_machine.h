#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Get queue handle 
     * 
     * @param handle 
     * @return esp_err_t 
     */
    esp_err_t state_machine_get_queue_handle(QueueHandle_t *handle);

    /**
     * @brief Start the state machine task 
     * 
     */
    void state_machine_task_start(void);

#ifdef __cplusplus
}
#endif

#endif