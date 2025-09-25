#include "queue.h"
#include "FreeRTOS.h"
#include "task.h"

QueueHandle_t xQueueCreate(uint32_t uxQueueLength, uint32_t uxItemSize) {
    // Create a new queue
    return xQueueCreate(uxQueueLength, uxItemSize);
}

BaseType_t xQueueSend(QueueHandle_t xQueue, const void *pvItemToQueue, TickType_t xTicksToWait) {
    // Send an item to the queue
    return xQueueSend(xQueue, pvItemToQueue, xTicksToWait);
}

BaseType_t xQueueReceive(QueueHandle_t xQueue, void *pvBuffer, TickType_t xTicksToWait) {
    // Receive an item from the queue
    return xQueueReceive(xQueue, pvBuffer, xTicksToWait);
}

void vQueueDelete(QueueHandle_t xQueue) {
    // Delete the queue
    vQueueDelete(xQueue);
}