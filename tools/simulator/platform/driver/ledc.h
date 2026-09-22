#pragma once
// FreeInk simulator — LEDC shim. Duty writes land in the daemon's PWM model,
// which is what makes frontlight and buzzer state observable from the CLI.
#include <cstdint>
#include <freeink_sim_abi.h>

#include "../esp_err.h"

typedef enum { LEDC_LOW_SPEED_MODE = 0, LEDC_HIGH_SPEED_MODE = 1, LEDC_SPEED_MODE_MAX } ledc_mode_t;
typedef int ledc_channel_t;
typedef int ledc_timer_t;
typedef int ledc_timer_bit_t;
typedef enum { LEDC_AUTO_CLK = 0 } ledc_clk_cfg_t;
typedef enum { LEDC_INTR_DISABLE = 0, LEDC_INTR_FADE_END } ledc_intr_type_t;

typedef struct {
  ledc_mode_t speed_mode;
  ledc_timer_bit_t duty_resolution;
  ledc_timer_t timer_num;
  uint32_t freq_hz;
  ledc_clk_cfg_t clk_cfg;
} ledc_timer_config_t;

typedef struct {
  int gpio_num;
  ledc_mode_t speed_mode;
  ledc_channel_t channel;
  ledc_intr_type_t intr_type;
  ledc_timer_t timer_sel;
  uint32_t duty;
  int hpoint;
} ledc_channel_config_t;

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t ledc_timer_config(const ledc_timer_config_t* cfg);
esp_err_t ledc_channel_config(const ledc_channel_config_t* cfg);
esp_err_t ledc_set_duty(ledc_mode_t mode, ledc_channel_t ch, uint32_t duty);
esp_err_t ledc_update_duty(ledc_mode_t mode, ledc_channel_t ch);
uint32_t ledc_get_duty(ledc_mode_t mode, ledc_channel_t ch);
esp_err_t ledc_stop(ledc_mode_t mode, ledc_channel_t ch, uint32_t idle_level);
esp_err_t ledc_set_freq(ledc_mode_t mode, ledc_timer_t timer, uint32_t freq_hz);
#ifdef __cplusplus
}
#endif
