#pragma once
// FreeInk simulator — rtc_io shim. RTC-domain pad control maps onto the same
// virtual pins; the hold calls share the GPIO matrix's latch model.
#include "gpio.h"

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t rtc_gpio_init(gpio_num_t pin);
esp_err_t rtc_gpio_deinit(gpio_num_t pin);
esp_err_t rtc_gpio_set_direction(gpio_num_t pin, int mode);
esp_err_t rtc_gpio_set_level(gpio_num_t pin, uint32_t level);
int rtc_gpio_get_level(gpio_num_t pin);
esp_err_t rtc_gpio_hold_en(gpio_num_t pin);
esp_err_t rtc_gpio_hold_dis(gpio_num_t pin);
esp_err_t rtc_gpio_pullup_en(gpio_num_t pin);
esp_err_t rtc_gpio_pulldown_dis(gpio_num_t pin);
esp_err_t rtc_gpio_isolate(gpio_num_t pin);
bool rtc_gpio_is_valid_gpio(gpio_num_t pin);
#ifdef __cplusplus
}
#endif
