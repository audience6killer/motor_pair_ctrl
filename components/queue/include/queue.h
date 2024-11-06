#ifndef QUEUE_F_H
#define QUEUE_F_H

const static int MAX_QUEUE_CAPACITY = 1000;

typedef struct 
{
    int head, tail, capacity, size;
    int *data;
} queue_t;

int is_empty(queue_t* queue);

int is_full(queue_t* queue);

int queue_size(queue_t* queue);

void enqueue(queue_t *queue, int data); 

int dequeue(queue_t *queue);

int front(queue_t *queue);

int back(queue_t *queue);

void destroy_queue(queue_t *queue);

queue_t* create_queue(int capacity);


#endif