#pragma once
// FreeInk simulator — I2S standard (Philips) mode shim.
#include "i2s_common.h"

typedef struct {
  i2s_clock_src_t clk_src;
  uint32_t sample_rate_hz;
  i2s_mclk_multiple_t mclk_multiple;
} i2s_std_clk_config_t;

typedef struct {
  i2s_data_bit_width_t data_bit_width;
  i2s_slot_bit_width_t slot_bit_width;
  i2s_slot_mode_t slot_mode;
  uint32_t slot_mask;
  uint32_t ws_width;
  bool ws_pol;
  bool bit_shift;
  bool left_align;
  bool big_endian;
  bool bit_order_lsb;
} i2s_std_slot_config_t;

typedef struct {
  int mclk;
  int bclk;
  int ws;
  int dout;
  int din;
  struct {
    uint32_t mclk_inv : 1;
    uint32_t bclk_inv : 1;
    uint32_t ws_inv : 1;
  } invert_flags;
} i2s_std_gpio_config_t;

typedef struct {
  i2s_std_clk_config_t clk_cfg;
  i2s_std_slot_config_t slot_cfg;
  i2s_std_gpio_config_t gpio_cfg;
} i2s_std_config_t;

#define I2S_STD_CLK_DEFAULT_CONFIG(rate) \
  { .clk_src = I2S_CLK_SRC_DEFAULT, .sample_rate_hz = (rate), .mclk_multiple = I2S_MCLK_MULTIPLE_256 }

#define I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bits, mode)                                                       \
  { .data_bit_width = (bits), .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO, .slot_mode = (mode), .slot_mask = 3, \
    .ws_width = (bits), .ws_pol = false, .bit_shift = true, .left_align = false, .big_endian = false,         \
    .bit_order_lsb = false }

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t i2s_channel_init_std_mode(i2s_chan_handle_t handle, const i2s_std_config_t* cfg);
esp_err_t i2s_channel_reconfig_std_clock(i2s_chan_handle_t handle, const i2s_std_clk_config_t* cfg);
esp_err_t i2s_channel_reconfig_std_slot(i2s_chan_handle_t handle, const i2s_std_slot_config_t* cfg);
#ifdef __cplusplus
}
#endif
