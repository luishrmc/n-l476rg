/**
 * @file queue.h
 * @author luis (luishenrique.8804@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-29
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef QUEUE_H
#define QUEUE_H

#define QUEUE_SIZE 32
#include "stdint.h"

typedef struct
{
    uint8_t items[QUEUE_SIZE];
    int8_t front;
    int8_t rear;
} Queue;

void initQueue(Queue *q);
uint8_t isQueueFull(Queue *q);
uint8_t isQueueEmpty(Queue *q);
uint8_t enqueue(Queue *q, uint8_t item);
uint8_t dequeue(Queue *q, uint8_t *item);

#endif // QUEUE_H
