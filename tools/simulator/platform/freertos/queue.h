#pragma once

// FreeInk simulator — FreeRTOS queue API. Fixed-capacity, item-copying,
// blocking with simulated-clock timeouts, like the real thing.

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size);
QueueHandle_t xQueueCreateStatic(UBaseType_t length, UBaseType_t item_size, uint8_t* storage, StaticQueue_t* buffer);
void vQueueDelete(QueueHandle_t q);
BaseType_t xQueueSend(QueueHandle_t q, const void* item, TickType_t ticks_to_wait);
BaseType_t xQueueSendToBack(QueueHandle_t q, const void* item, TickType_t ticks_to_wait);
BaseType_t xQueueSendToFront(QueueHandle_t q, const void* item, TickType_t ticks_to_wait);
BaseType_t xQueueSendFromISR(QueueHandle_t q, const void* item, BaseType_t* woken);
BaseType_t xQueueReceive(QueueHandle_t q, void* out, TickType_t ticks_to_wait);
BaseType_t xQueuePeek(QueueHandle_t q, void* out, TickType_t ticks_to_wait);
BaseType_t xQueueReset(QueueHandle_t q);
UBaseType_t uxQueueMessagesWaiting(QueueHandle_t q);
UBaseType_t uxQueueSpacesAvailable(QueueHandle_t q);

#ifdef __cplusplus
}
#endif
