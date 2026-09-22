#pragma once

// FreeInk simulator — I2S channel shim (shared by the std and PDM headers).
//
// Channels are records; writes go to the daemon's audio sink (a WAV file the
// CLI can fetch, so a firmware's tone or clip is inspectable), and reads are
// served from a queued capture file or silence. The default-config macros
// mirror ESP-IDF's so the SDK's designated initializers compile unchanged.

#include <cstddef>
#include <cstdint>
#include <freeink_sim_abi.h>

#include "../esp_err.h"

typedef struct i2s_chan_obj_t* i2s_chan_handle_t;

typedef enum { I2S_NUM_0 = 0, I2S_NUM_1 = 1, I2S_NUM_AUTO = -1 } i2s_port_t;
typedef enum { I2S_ROLE_MASTER = 0, I2S_ROLE_SLAVE = 1 } i2s_role_t;
typedef enum {
  I2S_DATA_BIT_WIDTH_8BIT = 8,
  I2S_DATA_BIT_WIDTH_16BIT = 16,
  I2S_DATA_BIT_WIDTH_24BIT = 24,
  I2S_DATA_BIT_WIDTH_32BIT = 32,
} i2s_data_bit_width_t;
typedef enum { I2S_SLOT_MODE_MONO = 1, I2S_SLOT_MODE_STEREO = 2 } i2s_slot_mode_t;
typedef enum { I2S_SLOT_BIT_WIDTH_AUTO = 0, I2S_SLOT_BIT_WIDTH_16BIT = 16, I2S_SLOT_BIT_WIDTH_32BIT = 32 } i2s_slot_bit_width_t;
typedef enum { I2S_CLK_SRC_DEFAULT = 0 } i2s_clock_src_t;
typedef enum {
  I2S_MCLK_MULTIPLE_128 = 128,
  I2S_MCLK_MULTIPLE_256 = 256,
  I2S_MCLK_MULTIPLE_384 = 384,
  I2S_MCLK_MULTIPLE_512 = 512,
} i2s_mclk_multiple_t;

#define I2S_GPIO_UNUSED (-1)

typedef struct {
  i2s_port_t id;
  i2s_role_t role;
  uint32_t dma_desc_num;
  uint32_t dma_frame_num;
  bool auto_clear;
  bool auto_clear_before_cb;
  int intr_priority;
} i2s_chan_config_t;

#define I2S_CHANNEL_DEFAULT_CONFIG(i2s_num, i2s_role) \
  { .id = (i2s_num), .role = (i2s_role), .dma_desc_num = 6, .dma_frame_num = 240, .auto_clear = false, \
    .auto_clear_before_cb = false, .intr_priority = 0 }

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t i2s_new_channel(const i2s_chan_config_t* cfg, i2s_chan_handle_t* tx, i2s_chan_handle_t* rx);
esp_err_t i2s_del_channel(i2s_chan_handle_t handle);
esp_err_t i2s_channel_enable(i2s_chan_handle_t handle);
esp_err_t i2s_channel_disable(i2s_chan_handle_t handle);
esp_err_t i2s_channel_write(i2s_chan_handle_t handle, const void* src, size_t size, size_t* written, uint32_t timeout_ms);
esp_err_t i2s_channel_read(i2s_chan_handle_t handle, void* dst, size_t size, size_t* read, uint32_t timeout_ms);
#ifdef __cplusplus
}
#endif
