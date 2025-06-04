#ifndef ESP32_UART_UNIT_H
#define ESP32_UART_UNIT_H

typedef enum 
{
    SOWER_CMD_EMPTY = 0,           /**< No command. */ 
    SOWER_CMD_STOP_CUTTER,         /**< Stop the cutter. */
    SOWER_CMD_START_CUTTER,        /**< Start the cutter. */
    SOWER_CMD_DISTANCE,            /**< Send the distance measure. */
    SOWER_CMD_RPMS,                /**< Send RPMs measure. */
    SOWER_CMD_LINEAR_MOTOR_UP,     /**< Put the linear motors in up state. */
    SOWER_CMD_LINEAR_MOTOR_DOWN,     /**< Put the linear motors in down state. */
    SOWER_CMD_START_DISPENSER,     /**< Start the dispenser. */
    SOWER_CMD_STOP_DISPENSER,      /**< Stop the dispenser. */
    SOWER_CMD_ECHO_SOWER,
    SOWER_CMD_ERROR,              /**< Command no identified. */
} sower_cmd_e;

/**
 * @brief Structure for received commands
 * 
 * This structure have the commands and value if it's necessary for ESP32.
 */
typedef struct
{
    sower_cmd_e code; /**< Command.*/
    float arg;              /**< Arguments. */
} sower_cmd_t;


/** 
 * @brief enumeration for ESP32 states.
 */
typedef enum 
{
    SOWER_EVENT_EMPTY_CMD = 0,      /**< Empty cmd */
    SOWER_EVENT_ERROR,          /**< Error CMD. */
    SOWER_EVENT_CUTTER_STARTED,     /**< Cutter started. */
    SOWER_EVENT_CUTTER_STOPPED,     /**< Cutter stopped. */
    SOWER_EVENT_CUTTER_UP,          /**< Linear motors set in up position. */
    SOWER_EVENT_CUTTER_DOWN,        /**< Linear motors set in down position. */
    SOWER_EVENT_SEED_DISTANCE,      /**< Distance measure. */
    SOWER_EVENT_DISPENSER_STARTED,  /**< Dispenser started. */
    SOWER_EVENT_DISPENSER_STOPPED,  /**< Dispenser stopped. */
    SOWER_EVENT_TEMPERATURE_MEASURE, /**< Temperature measure */
    SOWER_EVENT_HUMEDITY_MEASURE,   /**< Humedity measure */
    SOWER_EVENT_CUTTER_RPM_MEASURE, /**< Cutter rpm measure */
    SOWER_EVENT_ECHO_MSG,
} sower_events_e;

/**
 * @brief enumeration for ESP32 errors.
 */
typedef enum
{
    SOWER_ERROR_NONE = 0,               /**< No error.*/
    SOWER_ERROR_CUTTER_DONT_START,
    SOWER_ERROR_CUTTER_DONT_STOP,
    SOWER_ERROR_LMOTOR_DONT_RISE,
    SOWER_ERROR_LMOTOR_DONT_DESCEND,
    SOWER_ERROR_UNKNOWN_CMD,
    SOWER_ERROR_DISPENCER_ERROR,        /**< Error in dispenser.*/
    SOWER_ERROR_SEED_DISTANCE_ERROR,    /**< Error in distance measure.*/
    SOWER_ERROR_HUMEDITY_ERROR,         /**< Error in humedity measure. */
    SOWER_ERROR_TEMPERATURE_ERROR,      /**< Error in temperature measure. */

} sower_error_e;


/**
 * @brief Structure that contains information related to events and errors.
 */
typedef struct 
{
    sower_events_e event;     /**< ESP32 event. */ 
    sower_error_e error;          /**< Error code. */
    float arg;                          /**< Arg info. */
} sower_event_t; 

/**
 * @brief Get the string name of a sower_events_e value.
 *
 * @param event The sower_events_e value.
 * @return const char* The name of the event.
 */
static inline const char* sower_event_name(sower_events_e event)
{
    switch (event)
    {
        case SOWER_EVENT_EMPTY_CMD:          return "SOWER_EVENT_EMPTY_CMD";
        case SOWER_EVENT_ERROR:              return "SOWER_EVENT_ERROR";
        case SOWER_EVENT_CUTTER_STARTED:     return "SOWER_EVENT_CUTTER_STARTED";
        case SOWER_EVENT_CUTTER_STOPPED:     return "SOWER_EVENT_CUTTER_STOPPED";
        case SOWER_EVENT_CUTTER_UP:          return "SOWER_EVENT_CUTTER_UP";
        case SOWER_EVENT_CUTTER_DOWN:        return "SOWER_EVENT_CUTTER_DOWN";
        case SOWER_EVENT_SEED_DISTANCE:      return "SOWER_EVENT_SEED_DISTANCE";
        case SOWER_EVENT_DISPENSER_STARTED:  return "SOWER_EVENT_DISPENSER_STARTED";
        case SOWER_EVENT_DISPENSER_STOPPED:  return "SOWER_EVENT_DISPENSER_STOPPED";
        case SOWER_EVENT_TEMPERATURE_MEASURE:return "SOWER_EVENT_TEMPERATURE_MEASURE";
        case SOWER_EVENT_HUMEDITY_MEASURE:   return "SOWER_EVENT_HUMEDITY_MEASURE";
        case SOWER_EVENT_CUTTER_RPM_MEASURE: return "SOWER_EVENT_CUTTER_RPM_MEASURE";
        case SOWER_EVENT_ECHO_MSG:           return "SOWER_EVENT_ECHO_MSG";
        default:                             return "UNKNOWN_EVENT";
    }
}

esp_err_t esp32_uart_handshake(void);

esp_err_t esp32_uart_get_transmit_data_queue(QueueHandle_t *handle);

esp_err_t esp32_uart_get_received_data_queue(QueueHandle_t *handle);

/**
 * @brief Starts the RF communication task. 
 * 
 * @return 
 */
void esp32_uart_task_start(void);

#endif
