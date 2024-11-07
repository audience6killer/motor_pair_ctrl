#include <stdlib.h>
#include "stdio.h"
#include "queue.h"

queue_t* create_queue(int capacity) {
    if (capacity > MAX_QUEUE_CAPACITY) {
        return NULL;
    }

    queue_t *queue = (queue_t *)malloc(sizeof(queue_t));
    if (!queue) return NULL;

    queue->data = (int *)malloc(capacity * sizeof(int));
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

int empty_queue(queue_t* queue)
{
    if(queue != NULL)
    {
        for (size_t i = 0; i < queue->size; i++)
        {
            dequeue(queue);
        }
        return 1;
    }
    else
    {
        return -1;
    }
}

int is_queue_empty(queue_t *queue) {
    return queue->size == 0;
}

int is_queue_full(queue_t *queue) {
    return queue->size == queue->capacity;
}

int queue_size(queue_t *queue)
{
    return queue->size;
}

int enqueue(queue_t *queue, int data) {
    if (is_queue_full(queue)) {
        return -1;
    }

    printf("Added value to queue: %d\n", data);
    queue->data[queue->tail] = data;
    queue->tail = (queue->tail + 1) % queue->capacity;
    queue->size++;

    return 1;
}

int dequeue(queue_t *queue) {
    if (is_queue_empty(queue)) {
        return -1;
    }

    int dequeued_value = queue->data[queue->head];
    queue->head = (queue->head + 1) % queue->capacity;
    queue->size--;

    return dequeued_value;
}

int front(queue_t *queue) {
    if (is_queue_empty(queue)) {
        return -1;
    } else {
        return queue->data[queue->head];
    }
}

int back(queue_t *queue)
{
    if (is_queue_empty(queue)) {
        return -1;
    } else {
        return queue->data[queue->tail];
    }
}

void destroy_queue(queue_t *queue) {
    free(queue->data);
    free(queue);
}
