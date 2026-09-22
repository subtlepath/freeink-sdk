#pragma once

// FreeInk simulator — esp_system shim.

#include <cstddef>
#include <cstdint>
#include <freeink_sim_abi.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif
// Tears the bundle down and re-runs setup(), so firmware reboot paths are
// really exercised (NVS survives, RAM does not).
void esp_restart(void) __attribute__((noreturn));
uint32_t esp_get_free_heap_size(void);
uint32_t esp_get_minimum_free_heap_size(void);
const char* esp_get_idf_version(void);

typedef enum {
  ESP_RST_UNKNOWN = 0,
  ESP_RST_POWERON,
  ESP_RST_EXT,
  ESP_RST_SW,
  ESP_RST_PANIC,
  ESP_RST_INT_WDT,
  ESP_RST_TASK_WDT,
  ESP_RST_WDT,
  ESP_RST_DEEPSLEEP,
  ESP_RST_BROWNOUT,
  ESP_RST_SDIO,
} esp_reset_reason_t;
esp_reset_reason_t esp_reset_reason(void);
#ifdef __cplusplus
}
#endif
