#ifndef QUEUE_H
#define QUEUE_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    // Define the structure of the data to be stored in the queue
    // For example, you can define the data type here
    int data; // Example data field
} QueueItem_t;

typedef struct {
    QueueItem_t *items; // Pointer to the array of queue items
    size_t head;        // Index of the head of the queue
    size_t tail;        // Index of the tail of the queue
    size_t maxSize;     // Maximum size of the queue
    size_t currentSize; // Current size of the queue
} QueueHandle_t;

// Function prototypes
QueueHandle_t* createQueue(size_t maxSize);
void deleteQueue(QueueHandle_t* queue);
int enqueue(QueueHandle_t* queue, QueueItem_t item);
int dequeue(QueueHandle_t* queue, QueueItem_t* item);
int isQueueFull(QueueHandle_t* queue);
int isQueueEmpty(QueueHandle_t* queue);

#endif // QUEUE_H