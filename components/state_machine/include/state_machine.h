#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum {
        SM_CMD_EMPTY = 0, 
        SM_CMD_STOP_NAV,        // SPN
        SM_CMD_START_NAV,       // STN
        SM_CMD_PAUSE_NAV,       // PSN
        SM_CMD_RESUME_NAV,      // RMN
        SM_CMD_ADD_WAYPOINT,    // NVP
        SM_CMD_RESET,           // RST
        SM_CMD_ECHO,            // ECH
    } state_machine_cmd_e; // Commands sended from the state machine

    typedef struct {
        state_machine_cmd_e code;
        float args[3];
    } state_machine_msg_t;  

    /**
     * @brief Start the state machine task 
     * 
     */
    void state_machine_task_start(void);

#ifdef __cplusplus
}
#endif

#endif