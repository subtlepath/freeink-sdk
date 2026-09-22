#pragma once

// FreeInk simulator — task watchdog shim.
//
// The daemon runs a real watchdog against the simulated clock: a firmware loop
// that stops feeding it for longer than the configured timeout is reported as a
// WDT trip rather than silently hanging the simulator, which is the failure
// mode worth catching in an unattended test run.

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t esp_task_wdt_reset(void);
esp_err_t esp_task_wdt_add(void* handle);
esp_err_t esp_task_wdt_delete(void* handle);
esp_err_t esp_task_wdt_status(void* handle);
esp_err_t esp_task_wdt_init(uint32_t timeout_s, bool panic);
esp_err_t esp_task_wdt_deinit(void);
#ifdef __cplusplus
}
#endif
