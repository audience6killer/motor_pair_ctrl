#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum {
        CMD_START = 0,
        CMD_STOP,
        CMD_RESET,
        CMD_PAUSE,
        CMD_RESUME,
        CMD_WAYPOINT,
        CMD_ECHO,
    } state_machine_cmd_e; // Commands sended from the state machine

    typedef struct {
        state_machine_cmd_e code;
        float agrs[3];
    } state_machine_msg_t;  

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