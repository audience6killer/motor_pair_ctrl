#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        SM_CMD_EMPTY = 0,
        SM_CMD_STOP_NAV,     // SPN
        SM_CMD_START_NAV,    // STN
        SM_CMD_PAUSE_NAV,    // PSN
        SM_CMD_RESUME_NAV,   // RMN
        SM_CMD_ADD_WAYPOINT, // NVP
        SM_CMD_RESET,        // RST
        SM_CMD_ECHO,         // ECH
        SM_CMD_ECHO_ESP32,   // ECE
    } state_machine_cmd_e;   // Commands sended from the state machine

    typedef enum
    {
        SM_STATE_IDLE = 0,
        SM_STATE_STARTED,
        SM_STATE_STOPPED,
        SM_STATE_WAYPOINT_ADDED,
        SM_STATE_ERROR,
    } state_machine_state_e;

    typedef struct
    {
        state_machine_cmd_e code;
        float args[3];
    } state_machine_msg_t;

    static inline const char *state_machine_state_to_string(state_machine_state_e state)
    {
        static const char *states[] = {
            "SM_STATE_IDLE",
            "SM_STATE_STARTED",
            "SM_STATE_STOPPED",
            "SM_STATE_WAYPOINT_ADDED",
            "SM_STATE_ERROR"
        };

        // Ensure the state is within bounds
        if (state >= SM_STATE_IDLE && state <= SM_STATE_WAYPOINT_ADDED)
        {
            return states[__builtin_ctz(state)]; // Use __builtin_ctz to map BITx to array index
        }
        return "UNKNOWN_STATE";
    }

    const char* state_machine_get_state_string(void);

    const char* state_machine_get_error_string(void);

    /**
     * @brief Start the state machine task
     *
     */
    void state_machine_task_start(void);

#ifdef __cplusplus
}
#endif

#endif