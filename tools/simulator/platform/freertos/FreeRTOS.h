#pragma once

// FreeInk simulator — FreeRTOS shim on host threads.
//
// Tasks are std::threads, queues and semaphores are condition-variable backed,
// and every timed wait is measured on the *simulated* clock, not wall time — so
// a paused simulator really pauses a vTaskDelay(), and a stepped run advances
// firmware tasks by exactly the budget it was given. Without that, pausing the
// machine would leave background tasks running and the screen would change
// under a capture.
//
// Critical sections are a single recursive global mutex. That is coarser than
// a real port's per-spinlock granularity, but it preserves the property SDK
// code relies on — mutual exclusion against the task and ISR touching the same
// state — and cannot deadlock against itself.

#include <cstddef>
#include <cstdint>
#include <freeink_sim_abi.h>

#include "../esp_err.h"

typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef uint32_t TickType_t;
typedef void* TaskHandle_t;
typedef void* QueueHandle_t;
typedef void* SemaphoreHandle_t;
typedef void* EventGroupHandle_t;
typedef void (*TaskFunction_t)(void*);

// Opaque static-allocation carriers. Real FreeRTOS places the TCB and stack in
// caller memory; here the handle owns its own storage, so these only need to be
// large enough for MemoryManager's sizeof() checks to be sane.
typedef struct {
  void* opaque[24];
} StaticTask_t;
typedef struct {
  void* opaque[8];
} StaticQueue_t;
typedef StaticQueue_t StaticSemaphore_t;
typedef uint8_t StackType_t;

#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define pdFAIL 0
#define errQUEUE_FULL 0

#define configTICK_RATE_HZ 1000
#define portTICK_PERIOD_MS 1
#define portMAX_DELAY ((TickType_t)0xFFFFFFFFUL)
#define configMINIMAL_STACK_SIZE 2048
#define tskIDLE_PRIORITY 0
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define pdTICKS_TO_MS(t) ((uint32_t)(t))

#ifdef __cplusplus
extern "C" {
#endif

// Critical sections / spinlocks.
typedef struct {
  uint32_t owner;
  uint32_t count;
} portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED {0, 0}
#define portMUX_FREE_VAL 0

void fsim_rtos_enter_critical(portMUX_TYPE* mux);
void fsim_rtos_exit_critical(portMUX_TYPE* mux);

#define portENTER_CRITICAL(mux) fsim_rtos_enter_critical(mux)
#define portEXIT_CRITICAL(mux) fsim_rtos_exit_critical(mux)
#define portENTER_CRITICAL_ISR(mux) fsim_rtos_enter_critical(mux)
#define portEXIT_CRITICAL_ISR(mux) fsim_rtos_exit_critical(mux)
#define portENTER_CRITICAL_SAFE(mux) fsim_rtos_enter_critical(mux)
#define portEXIT_CRITICAL_SAFE(mux) fsim_rtos_exit_critical(mux)
#define taskENTER_CRITICAL(mux) fsim_rtos_enter_critical(mux)
#define taskEXIT_CRITICAL(mux) fsim_rtos_exit_critical(mux)

#define portYIELD_FROM_ISR(...) ((void)0)
#define taskYIELD() fsim_yield()
#define portYIELD() fsim_yield()

uint32_t xPortGetFreeHeapSize(void);
uint32_t xPortGetMinimumEverFreeHeapSize(void);
BaseType_t xPortInIsrContext(void);

#ifdef __cplusplus
}
#endif
