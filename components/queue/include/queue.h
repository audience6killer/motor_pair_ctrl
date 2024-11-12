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

const static int MAX_QUEUE_CAPACITY = 1000;

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
int empty_queue(queue_t* queue);

/**
 * @brief Check whether the queue is empty or not
 * 
 * @param queue 
 * @return int, 1 is empty
 */
int is_queue_empty(queue_t* queue);

/**
 * @brief Is queue full 
 * 
 * @param queue 
 * @return int 1 is full 
 */
int is_queue_full(queue_t* queue);

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
int enqueue(queue_t *queue, float data); 

/**
 * @brief Dequeue last value and return it
 * 
 * @param queue 
 * @return float 
 */
float dequeue(queue_t *queue);

/**
 * @brief Get front value, but not dequeue it
 * 
 * @param queue 
 * @return float 
 */
float front(queue_t *queue);

/**
 * @brief Get last value, but not dequeue it
 * 
 * @param queue 
 * @return float 
 */
float back(queue_t *queue);

/**
 * @brief Release queue memory
 * 
 * @param queue 
 */
void destroy_queue(queue_t *queue);

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