#pragma once

// FreeInk simulator — FreeRTOS task API on host threads.

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

BaseType_t xTaskCreate(TaskFunction_t fn, const char* name, uint32_t stack_depth, void* arg, UBaseType_t priority,
                       TaskHandle_t* out);
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char* name, uint32_t stack_depth, void* arg,
                                   UBaseType_t priority, TaskHandle_t* out, BaseType_t core);
TaskHandle_t xTaskCreateStatic(TaskFunction_t fn, const char* name, uint32_t stack_depth, void* arg,
                               UBaseType_t priority, StackType_t* stack, StaticTask_t* tcb);
TaskHandle_t xTaskCreateStaticPinnedToCore(TaskFunction_t fn, const char* name, uint32_t stack_depth, void* arg,
                                           UBaseType_t priority, StackType_t* stack, StaticTask_t* tcb,
                                           BaseType_t core);
// A task deleting itself (handle == NULL) never returns.
void vTaskDelete(TaskHandle_t handle);
void vTaskDelay(TickType_t ticks);
void vTaskDelayUntil(TickType_t* previous_wake, TickType_t increment);
TickType_t xTaskGetTickCount(void);
TaskHandle_t xTaskGetCurrentTaskHandle(void);
const char* pcTaskGetName(TaskHandle_t handle);
UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t handle);
UBaseType_t uxTaskPriorityGet(TaskHandle_t handle);
void vTaskPrioritySet(TaskHandle_t handle, UBaseType_t priority);
void vTaskSuspend(TaskHandle_t handle);
void vTaskResume(TaskHandle_t handle);

BaseType_t xTaskNotifyGive(TaskHandle_t handle);
typedef enum { eNoAction, eSetBits, eIncrement, eSetValueWithOverwrite, eSetValueWithoutOverwrite } eNotifyAction;
BaseType_t xTaskNotify(TaskHandle_t handle, uint32_t value, eNotifyAction action);
void vTaskNotifyGiveFromISR(TaskHandle_t handle, BaseType_t* woken);
uint32_t ulTaskNotifyTake(BaseType_t clear_on_exit, TickType_t ticks_to_wait);

#ifdef __cplusplus
}
#endif
