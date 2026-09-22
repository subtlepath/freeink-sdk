#pragma once
// FreeInk simulator — FreeRTOS software timers, driven by the simulated clock.
#include "FreeRTOS.h"

typedef void* TimerHandle_t;
typedef void (*TimerCallbackFunction_t)(TimerHandle_t);

#ifdef __cplusplus
extern "C" {
#endif
TimerHandle_t xTimerCreate(const char* name, TickType_t period, BaseType_t auto_reload, void* id,
                           TimerCallbackFunction_t cb);
BaseType_t xTimerStart(TimerHandle_t t, TickType_t block);
BaseType_t xTimerStop(TimerHandle_t t, TickType_t block);
BaseType_t xTimerDelete(TimerHandle_t t, TickType_t block);
void* pvTimerGetTimerID(TimerHandle_t t);
#ifdef __cplusplus
}
#endif
