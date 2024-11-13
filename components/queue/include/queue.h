/**
 * @file queue.h
 * @author Adrian Pulido
 * @brief 
 * @version 0.1
 * @date 2024-11-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef QUEUE_F_H
#define QUEUE_F_H
#include "stdbool.h"

const static int MAX_QUEUE_CAPACITY = 1000;

// TODO refactor to include esp_err_t

typedef struct 
{
    int head, tail, capacity, size;
    float*data;
} queue_t;

/**
 * @brief Empty queue. But space is not released
 * 
 * @param queue 
 * @return int 
 */
esp_err_t empty_queue(queue_t* queue);

/**
 * @brief Check whether the queue is empty or not
 * 
 * @param queue 
 * @return 
 */
bool is_queue_empty(queue_t* queue);

/**
 * @brief Is queue full 
 * 
 * @param queue 
 * @return 
 */
bool is_queue_full(queue_t* queue);

/**
 * @brief Queue current size 
 * 
 * @param queue 
 * @return int 
 */
int queue_size(queue_t* queue);

/**
 * @brief Add value to queue
 * 
 * @param queue 
 * @param data 
 * @return int 1 success, -1 fail
 */
esp_err_t enqueue(queue_t *queue, float data); 

/**
 * @brief Dequeue last value and return it
 * 
 * @param queue 
 * @return float 
 */
esp_err_t dequeue(queue_t *queue, float *value);

/**
 * @brief Get front value, but not dequeue it
 * 
 * @param queue 
 * @return float 
 */
esp_err_t front(queue_t *queue, float *value);

/**
 * @brief Get last value, but not dequeue it
 * 
 * @param queue 
 * @return float 
 */
esp_err_t back(queue_t *queue, float *value);

/**
 * @brief Release queue memory
 * 
 * @param queue 
 */
esp_err_t destroy_queue(queue_t *queue);

/**
 * @brief Print entire queue 
 * 
 * @param queue 
 */
void queue_print(queue_t *queue);

/**
 * @brief Create a queue object
 * 
 * @param capacity 
 * @return queue_t* 
 */
queue_t* create_queue(int capacity);


#endif