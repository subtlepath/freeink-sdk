#pragma once
// FreeInk simulator — I2S PDM receive shim (the Paper Mono microphone path).
#include "i2s_common.h"

typedef struct {
  i2s_clock_src_t clk_src;
  uint32_t sample_rate_hz;
  uint32_t dn_sample_mode;
  uint32_t bclk_div;
} i2s_pdm_rx_clk_config_t;

typedef struct {
  i2s_data_bit_width_t data_bit_width;
  i2s_slot_bit_width_t slot_bit_width;
  i2s_slot_mode_t slot_mode;
  uint32_t slot_mask;
} i2s_pdm_rx_slot_config_t;

typedef struct {
  int clk;
  int din;
  struct {
    uint32_t clk_inv : 1;
  } invert_flags;
} i2s_pdm_rx_gpio_config_t;

typedef struct {
  i2s_pdm_rx_clk_config_t clk_cfg;
  i2s_pdm_rx_slot_config_t slot_cfg;
  i2s_pdm_rx_gpio_config_t gpio_cfg;
} i2s_pdm_rx_config_t;

#define I2S_PDM_DSR_8S 0

#define I2S_PDM_RX_CLK_DEFAULT_CONFIG(rate) \
  { .clk_src = I2S_CLK_SRC_DEFAULT, .sample_rate_hz = (rate), .dn_sample_mode = I2S_PDM_DSR_8S, .bclk_div = 8 }

#define I2S_PDM_RX_SLOT_DEFAULT_CONFIG(bits, mode) \
  { .data_bit_width = (bits), .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO, .slot_mode = (mode), .slot_mask = 1 }

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t i2s_channel_init_pdm_rx_mode(i2s_chan_handle_t handle, const i2s_pdm_rx_config_t* cfg);
#ifdef __cplusplus
}
#endif
