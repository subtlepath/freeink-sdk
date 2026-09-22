#pragma once
// FreeInk simulator — FreeRTOS event groups.
#include "FreeRTOS.h"

typedef uint32_t EventBits_t;

#ifdef __cplusplus
extern "C" {
#endif
EventGroupHandle_t xEventGroupCreate(void);
void vEventGroupDelete(EventGroupHandle_t g);
EventBits_t xEventGroupSetBits(EventGroupHandle_t g, EventBits_t bits);
EventBits_t xEventGroupClearBits(EventGroupHandle_t g, EventBits_t bits);
EventBits_t xEventGroupGetBits(EventGroupHandle_t g);
EventBits_t xEventGroupWaitBits(EventGroupHandle_t g, EventBits_t bits, BaseType_t clear_on_exit,
                                BaseType_t wait_for_all, TickType_t ticks_to_wait);
#ifdef __cplusplus
}
#endif
