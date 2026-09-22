#pragma once

// FreeInk simulator — driver/gpio shim over the same GPIO matrix the Arduino
// shim drives, so a pin configured through ESP-IDF and read back through
// digitalRead() agrees. gpio_hold_en/dis are modelled for real: a held pad
// ignores writes until released, which is the latch behaviour EpdBus and
// PowerManager work around on wake.

#include <cstdint>
#include <freeink_sim_abi.h>

#include "../esp_err.h"

typedef int gpio_num_t;
#define GPIO_NUM_NC (-1)

typedef enum {
  GPIO_MODE_DISABLE = 0,
  GPIO_MODE_INPUT = 1,
  GPIO_MODE_OUTPUT = 2,
  GPIO_MODE_OUTPUT_OD = 6,
  GPIO_MODE_INPUT_OUTPUT_OD = 7,
  GPIO_MODE_INPUT_OUTPUT = 3,
} gpio_mode_t;

typedef enum { GPIO_PULLUP_DISABLE = 0, GPIO_PULLUP_ENABLE = 1 } gpio_pullup_t;
typedef enum { GPIO_PULLDOWN_DISABLE = 0, GPIO_PULLDOWN_ENABLE = 1 } gpio_pulldown_t;
typedef enum {
  GPIO_INTR_DISABLE = 0,
  GPIO_INTR_POSEDGE = 1,
  GPIO_INTR_NEGEDGE = 2,
  GPIO_INTR_ANYEDGE = 3,
  GPIO_INTR_LOW_LEVEL = 4,
  GPIO_INTR_HIGH_LEVEL = 5,
} gpio_int_type_t;

typedef struct {
  uint64_t pin_bit_mask;
  gpio_mode_t mode;
  gpio_pullup_t pull_up_en;
  gpio_pulldown_t pull_down_en;
  gpio_int_type_t intr_type;
} gpio_config_t;

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t gpio_config(const gpio_config_t* cfg);
esp_err_t gpio_reset_pin(gpio_num_t pin);
esp_err_t gpio_set_direction(gpio_num_t pin, gpio_mode_t mode);
esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level);
int gpio_get_level(gpio_num_t pin);
esp_err_t gpio_set_pull_mode(gpio_num_t pin, int pull);
esp_err_t gpio_pullup_en(gpio_num_t pin);
esp_err_t gpio_pullup_dis(gpio_num_t pin);
esp_err_t gpio_pulldown_en(gpio_num_t pin);
esp_err_t gpio_pulldown_dis(gpio_num_t pin);
esp_err_t gpio_hold_en(gpio_num_t pin);
esp_err_t gpio_hold_dis(gpio_num_t pin);
esp_err_t gpio_sleep_sel_dis(gpio_num_t pin);
esp_err_t gpio_install_isr_service(int flags);
void gpio_uninstall_isr_service(void);
esp_err_t gpio_isr_handler_add(gpio_num_t pin, void (*isr)(void*), void* arg);
esp_err_t gpio_isr_handler_remove(gpio_num_t pin);
esp_err_t gpio_set_intr_type(gpio_num_t pin, gpio_int_type_t type);
esp_err_t gpio_intr_enable(gpio_num_t pin);
esp_err_t gpio_intr_disable(gpio_num_t pin);
esp_err_t gpio_wakeup_enable(gpio_num_t pin, gpio_int_type_t type);
esp_err_t gpio_wakeup_disable(gpio_num_t pin);
#ifdef __cplusplus
}
#endif
