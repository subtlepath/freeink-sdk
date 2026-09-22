#pragma once

// FreeInk simulator — esp_sleep shim.
//
// Deep sleep is modelled honestly: the call does not return. The daemon parks
// the firmware thread, marks the machine asleep (visible in `freeink-sim
// status`), and on a wake source — timer expiry, or a configured GPIO reaching
// its wake level when the CLI presses that button — tears the bundle down and
// re-enters setup() with the wake cause set. That is what the device does, and
// it is the behaviour reader firmware most often gets wrong.

#include <cstdint>
#include <freeink_sim_abi.h>

#include "esp_err.h"

typedef enum {
  ESP_SLEEP_WAKEUP_UNDEFINED = 0,
  ESP_SLEEP_WAKEUP_ALL,
  ESP_SLEEP_WAKEUP_EXT0,
  ESP_SLEEP_WAKEUP_EXT1,
  ESP_SLEEP_WAKEUP_TIMER,
  ESP_SLEEP_WAKEUP_TOUCHPAD,
  ESP_SLEEP_WAKEUP_ULP,
  ESP_SLEEP_WAKEUP_GPIO,
  ESP_SLEEP_WAKEUP_UART,
} esp_sleep_wakeup_cause_t;

typedef enum { ESP_EXT1_WAKEUP_ALL_LOW = 0, ESP_EXT1_WAKEUP_ANY_HIGH = 1, ESP_EXT1_WAKEUP_ANY_LOW = 2 } esp_sleep_ext1_wakeup_mode_t;
typedef enum { ESP_GPIO_WAKEUP_GPIO_LOW = 0, ESP_GPIO_WAKEUP_GPIO_HIGH = 1 } esp_deepsleep_gpio_wake_up_mode_t;
typedef enum { ESP_SLEEP_PD_DOMAIN_RTC_PERIPH = 0, ESP_SLEEP_PD_DOMAIN_RTC_SLOW_MEM, ESP_SLEEP_PD_DOMAIN_RTC_FAST_MEM, ESP_SLEEP_PD_DOMAIN_VDDSDIO, ESP_SLEEP_PD_DOMAIN_MAX } esp_sleep_pd_domain_t;
typedef enum { ESP_PD_OPTION_OFF = 0, ESP_PD_OPTION_ON, ESP_PD_OPTION_AUTO } esp_sleep_pd_option_t;

#ifdef __cplusplus
extern "C" {
#endif
void esp_deep_sleep_start(void) __attribute__((noreturn));
esp_err_t esp_light_sleep_start(void);
void esp_deep_sleep(uint64_t time_in_us) __attribute__((noreturn));

esp_err_t esp_sleep_enable_timer_wakeup(uint64_t time_in_us);
esp_err_t esp_sleep_enable_ext0_wakeup(int gpio_num, int level);
esp_err_t esp_sleep_enable_ext1_wakeup(uint64_t mask, esp_sleep_ext1_wakeup_mode_t mode);
esp_err_t esp_sleep_enable_ext1_wakeup_io(uint64_t mask, esp_sleep_ext1_wakeup_mode_t mode);
esp_err_t esp_deep_sleep_enable_gpio_wakeup(uint64_t mask, esp_deepsleep_gpio_wake_up_mode_t mode);
esp_err_t esp_sleep_enable_gpio_wakeup(void);
esp_err_t esp_sleep_disable_wakeup_source(int source);
esp_err_t esp_sleep_pd_config(esp_sleep_pd_domain_t domain, esp_sleep_pd_option_t option);
esp_sleep_wakeup_cause_t esp_sleep_get_wakeup_cause(void);
uint64_t esp_sleep_get_gpio_wakeup_status(void);
uint64_t esp_sleep_get_ext1_wakeup_status(void);
esp_err_t esp_sleep_config_gpio_isolate(void);
esp_err_t gpio_deep_sleep_hold_en(void);
esp_err_t gpio_deep_sleep_hold_dis(void);
#ifdef __cplusplus
}
#endif
