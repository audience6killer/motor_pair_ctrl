#include <stdlib.h>
#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "queue.h"

static const char TAG[] = "QUEUE";

queue_t* create_queue(int capacity) {
    if (capacity > MAX_QUEUE_CAPACITY) {
        return NULL;
    }

    queue_t *queue = (queue_t *)malloc(sizeof(queue_t));
    if (!queue) return NULL;

    queue->data = (float *)malloc(capacity * sizeof(float));
    if (!queue->data) {
        free(queue);
        return NULL;
    }

    queue->capacity = capacity;
    queue->head = 0;
    queue->tail = 0;
    queue->size = 0; // New: tracks the number of elements in the queue

    return queue;
}

esp_err_t empty_queue(queue_t* queue)
{
    if(queue != NULL)
    {
        float value;
        for (size_t i = 0; i < queue->size; i++)
        {
            dequeue(queue, &value);
        }
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}

bool is_queue_empty(queue_t *queue) {
    return queue->size == 0;
}

bool is_queue_full(queue_t *queue) {
    return queue->size == queue->capacity;
}

int queue_size(queue_t *queue)
{
    return queue->size;
}

esp_err_t enqueue(queue_t *queue, float data) {
    if (is_queue_full(queue)) {
        ESP_LOGE(TAG, "Queue is full");
        queue_print(queue);
        return ESP_FAIL;
    }

    //printf("Added value to queue: %f\n", data);
    queue->data[queue->tail] = data;
    queue->tail = (queue->tail + 1) % queue->capacity;
    queue->size++;

    return ESP_OK;
}

esp_err_t dequeue(queue_t *queue, float *value) {
    if (is_queue_empty(queue)) {
        ESP_LOGE(TAG, "Queue is empty");
        return ESP_FAIL;
    }

    // float dequeued_value = queue->data[queue->head];
    *(value) = queue->data[queue->head];
    queue->head = (queue->head + 1) % queue->capacity;
    queue->size--;

    return ESP_OK;
}

esp_err_t front(queue_t *queue, float *value) {
    if (is_queue_empty(queue)) {
        return ESP_FAIL;
    } else {
        *(value) = queue->data[queue->head];
        return queue->data[queue->head];
    }
}

esp_err_t back(queue_t *queue, float *value)
{
    if (is_queue_empty(queue)) {
        return ESP_FAIL;
    } else {
        *(value) = queue->data[queue->tail];
        return ESP_OK; 
    }
}
void queue_print(queue_t *queue)
{
  if (queue->size != 0) 
  {
    for (int i = 0; i < queue->size; i++) 
    {
      printf("%d, %f\n", i, (float)queue->data[i]);
    }
  }
}

esp_err_t destroy_queue(queue_t *queue) {
    free(queue->data);
    free(queue);
    return ESP_OK;
}
