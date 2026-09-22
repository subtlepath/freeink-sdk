#pragma once

// FreeInk simulator — FreeRTOS semaphore/mutex API.
//
// xSemaphoreGiveFromISR() is callable from the daemon's virtual-interrupt
// thread, which is how EpdBus's BUSY-edge refresh wait wakes: the panel model
// raises the completion edge, the virtual ISR runs, and the firmware's
// waitRefreshComplete() returns — the same handshake as the glass.

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

SemaphoreHandle_t xSemaphoreCreateBinary(void);
SemaphoreHandle_t xSemaphoreCreateMutex(void);
SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void);
SemaphoreHandle_t xSemaphoreCreateCounting(UBaseType_t max_count, UBaseType_t initial_count);
SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t* buffer);
SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t* buffer);
void vSemaphoreDelete(SemaphoreHandle_t s);
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t ticks_to_wait);
BaseType_t xSemaphoreGive(SemaphoreHandle_t s);
BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t s, TickType_t ticks_to_wait);
BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t s);
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t s, BaseType_t* woken);
BaseType_t xSemaphoreTakeFromISR(SemaphoreHandle_t s, BaseType_t* woken);

#ifdef __cplusplus
}
#endif
