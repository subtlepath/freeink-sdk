#pragma once

// FreeInk simulator — esp_private/esp_sleep_internal shim. The SDK calls
// esp_sleep_sub_mode_config() to pick a deep-sleep sub-mode; the model records
// the choice (reported by `freeink-sim status`) and otherwise ignores it.

#include "../esp_err.h"

typedef enum {
  ESP_SLEEP_RTC_USE_RC_FAST_MODE = 0,
  ESP_SLEEP_DIG_USE_XTAL_MODE,
  ESP_SLEEP_LP_USE_XTAL_MODE,
  ESP_SLEEP_ULTRA_LOW_MODE,
  ESP_SLEEP_RTC_FAST_USE_XTAL_MODE,
  ESP_SLEEP_VDDSDIO_USE_XTAL_MODE,
  ESP_SLEEP_MODE_MAX,
} esp_sleep_sub_mode_t;

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t esp_sleep_sub_mode_config(esp_sleep_sub_mode_t mode, bool activate);
#ifdef __cplusplus
}
#endif
