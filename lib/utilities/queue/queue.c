/**
 * @file queue.c
 * @author luis (luishenrique.8804@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "queue.h"

void initQueue(Queue *q) {
    q->front = 0;
    q->rear = -1;
}

uint8_t isQueueFull(Queue *q) {
    return ((q->rear + 1) % QUEUE_SIZE == q->front && (!isQueueEmpty(q)));
}

uint8_t isQueueEmpty(Queue *q) {
    return (q->rear == -1);
}

uint8_t enqueue(Queue *q, uint8_t item) {
    if (isQueueFull(q)) {
        return 0; // Queue is full
    }
    q->rear = (q->rear + 1) % QUEUE_SIZE;
    q->items[q->rear] = item;
    return 1; // Enqueue successful
}

uint8_t dequeue(Queue *q, uint8_t *item) {
    if (isQueueEmpty(q)) {
        return 0; // Queue is empty
    }
    *item = q->items[q->front];
    if (q->front == q->rear) {
        // The queue had one item, now it will be empty
        q->front = 0;
        q->rear = -1;
    } else {
        q->front = (q->front + 1) % QUEUE_SIZE;
    }
    return 1; // Dequeue successful
}
