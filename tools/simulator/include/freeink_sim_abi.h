#pragma once

// FreeInk simulator — ABI between the simulator daemon and a firmware bundle.
//
// A firmware bundle is a shared library holding the consumer firmware, the
// FreeInk SDK, and the host platform shims in tools/simulator/platform. The
// shims never touch real hardware: every hardware access below is a call into
// the daemon, which owns the virtual machine state (GPIO matrix, SPI/I2C buses,
// virtual panel controller, storage, power rails).
//
// The bundle is linked with undefined `fsim_*` symbols and resolves them from
// the daemon at dlopen() time (the daemon exports its dynamic symbol table:
// -rdynamic on Linux, -Wl,-export_dynamic on macOS). That keeps the shims free
// of vtable plumbing and lets lldb step straight from firmware into the model.
//
// Bundles export the entry points at the bottom of this header. Everything else
// here is provided by the daemon.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ABI revision. The daemon refuses a bundle built against a different major.
#define FREEINK_SIM_ABI_MAJOR 1
#define FREEINK_SIM_ABI_MINOR 2

// ── Time ─────────────────────────────────────────────────────────────────────
// The simulated clock. It advances with wall time while running, is frozen
// while the machine is paused, and can be stepped by a fixed budget, so a test
// can drive N simulated milliseconds without racing the host.
uint64_t fsim_micros(void);
// Blocks the calling firmware thread until the simulated clock has advanced by
// `us`. Returns early (nonzero) if the machine is being torn down.
int fsim_delay_us(uint64_t us);
// Cooperative yield: lets the daemon service commands and run the panel model.
void fsim_yield(void);
// Nonzero once the daemon wants the firmware to unwind — a reload, a restart, a
// deep-sleep wake, or daemon shutdown. The bundle's entry loop polls it and
// returns, so the firmware thread really exits instead of the daemon hanging on
// a join that can never complete.
int fsim_should_exit(void);

// ── GPIO ─────────────────────────────────────────────────────────────────────
// Pin modes mirror the Arduino constants the SDK passes.
#define FSIM_PIN_INPUT 0x01
#define FSIM_PIN_OUTPUT 0x02
#define FSIM_PIN_PULLUP 0x04
#define FSIM_PIN_PULLDOWN 0x08
#define FSIM_PIN_OPEN_DRAIN 0x10

void fsim_pin_mode(int pin, uint32_t mode);
void fsim_gpio_write(int pin, int level);
int fsim_gpio_read(int pin);
// Deep-sleep pad hold. The model honours it: a held pin ignores writes until
// released, which is the behaviour EpdBus::begin() works around with
// gpio_hold_dis().
void fsim_gpio_hold(int pin, int enable);
void fsim_gpio_attach_interrupt(int pin, void (*isr)(void), int mode);
void fsim_gpio_detach_interrupt(int pin);

// ── ADC ──────────────────────────────────────────────────────────────────────
// Raw 12-bit counts and calibrated millivolts. The model derives both from the
// pin's virtual analog source (resistor ladder, battery divider, or an explicit
// value set over the control socket).
int fsim_adc_read(int pin);
int fsim_adc_read_mv(int pin);
void fsim_adc_set_attenuation(int pin, int atten);

// ── SPI ──────────────────────────────────────────────────────────────────────
// `bus` is the HSPI/VSPI-style index; FreeInk only ever uses bus 0 for the
// panel. Transfers are full duplex: `rx` may be NULL for write-only traffic.
void fsim_spi_begin(int bus, int sclk, int miso, int mosi, int cs);
void fsim_spi_end(int bus);
void fsim_spi_settings(int bus, uint32_t hz, uint8_t bit_order, uint8_t mode);
void fsim_spi_transfer(int bus, const uint8_t* tx, uint8_t* rx, size_t len);

// ── I2C ──────────────────────────────────────────────────────────────────────
// One combined write-then-read transaction, the shape both the Arduino Wire
// shim and the ESP-IDF i2c_master shim reduce to. Returns 0 on ACK, nonzero
// when no virtual device answers at `addr` (which is how probe code discovers
// that a controller is absent).
void fsim_i2c_begin(int bus, int sda, int scl, uint32_t hz);
int fsim_i2c_xfer(int bus, uint8_t addr, const uint8_t* tx, size_t tx_len, uint8_t* rx, size_t rx_len);

// ── LEDC (PWM: frontlight, buzzer, backlight) ────────────────────────────────
void fsim_ledc_setup(int channel, uint32_t freq_hz, uint8_t resolution_bits);
void fsim_ledc_attach(int pin, int channel);
void fsim_ledc_write(int pin_or_channel, uint32_t duty, int by_pin);

// ── I2S (speaker out / PDM microphone in) ────────────────────────────────────
void fsim_audio_open(int bus, uint32_t sample_rate, uint8_t bits, uint8_t channels);
void fsim_audio_write(int bus, const void* samples, size_t bytes);
void fsim_audio_close(int bus);
// Returns bytes actually produced; the daemon serves a queued capture file or
// silence.
size_t fsim_mic_read(int bus, void* samples, size_t bytes);
void fsim_mic_open(int bus, uint32_t sample_rate, uint8_t bits);
void fsim_mic_close(int bus);

// ── NVS / Preferences ────────────────────────────────────────────────────────
// Key/value blobs namespaced exactly as Preferences does. Persisted by the
// daemon so a reboot keeps settings, and inspectable over the control socket.
int fsim_nvs_get(const char* ns, const char* key, void* out, size_t cap, size_t* out_len);
int fsim_nvs_set(const char* ns, const char* key, const void* data, size_t len);
int fsim_nvs_erase(const char* ns, const char* key);
int fsim_nvs_clear(const char* ns);

// ── Storage (SD card) ────────────────────────────────────────────────────────
// The virtual card is a host directory. The SdFat shim maps firmware paths into
// it, so firmware file I/O is real file I/O against a directory the CLI mounts.
// Absent card => fsim_sd_present() == 0 and every open fails, which is what
// firmware sees when no card is inserted.
int fsim_sd_present(void);
// Resolves a firmware-visible path ("/books/x.epub") to a host path inside the
// mounted directory. Refuses traversal outside the mount. Returns 0 on success.
int fsim_sd_resolve(const char* fw_path, char* out, size_t cap);
uint64_t fsim_sd_capacity_bytes(void);

// ── Power / system ───────────────────────────────────────────────────────────
#define FSIM_POWER_RESTART 0
#define FSIM_POWER_DEEP_SLEEP 1
#define FSIM_POWER_LIGHT_SLEEP 2
#define FSIM_POWER_PANIC 3
// Reports a power transition. Deep sleep parks the firmware thread until a
// configured wake source fires; restart tears the bundle down and re-runs
// setup(). Never returns for restart/deep-sleep.
void fsim_power_event(int kind, uint64_t sleep_us);
void fsim_sleep_enable_wakeup(int pin, int level);
// Wake cause for the boot the firmware is currently in.
int fsim_wake_cause(void);

uint32_t fsim_random(void);
void fsim_log(const char* text, size_t len);
// Structured trace channel for the SDK's own instrumentation, surfaced in the
// CLI as `freeink-sim log --channel <name>`.
void fsim_trace(const char* channel, const char* text, size_t len);

// ── Wi-Fi / network ──────────────────────────────────────────────────────────
#define FSIM_WIFI_IDLE 0
#define FSIM_WIFI_CONNECTING 1
#define FSIM_WIFI_CONNECTED 2
#define FSIM_WIFI_FAILED 3
int fsim_wifi_begin(const char* ssid, const char* pass);
int fsim_wifi_status(void);
void fsim_wifi_disconnect(void);
int fsim_wifi_scan(char* out_json, size_t cap);
uint32_t fsim_wifi_local_ip(void);
// TCP through the host, gated by the daemon's network policy (off by default:
// a simulated device should not reach the internet unless the operator says
// so). Returns a socket handle or -1.
int fsim_net_connect(const char* host, uint16_t port);
int fsim_net_write(int sock, const void* data, size_t len);
int fsim_net_read(int sock, void* data, size_t len);
void fsim_net_close(int sock);

// ── USB mass storage ─────────────────────────────────────────────────────────
void fsim_usb_msc_begin(uint32_t block_count, uint16_t block_size);
void fsim_usb_msc_end(void);
int fsim_usb_connected(void);

// ── Panel hand-off for external-bus drivers ──────────────────────────────────
// Drivers that own their bus (M5GFX/LovyanGFX/IT8951 backends) never reach the
// virtual SPI controller, so they push finished frames straight to the panel
// model instead. `plane` is 1bpp MSB-first, `gray` optional second plane.
void fsim_panel_push_frame(const uint8_t* plane, size_t len, int width, int height, int mode);

// ── Board description ────────────────────────────────────────────────────────
// The daemon needs the pinout to route traffic: which pin is the panel's D/C,
// which GPIOs are buttons, which I2C address the digitizer answers on. Rather
// than keep a second copy of BoardConfig in the daemon — guaranteed to drift —
// the bundle reports the active profile at attach time, straight out of
// BoardConfig::ACTIVE. One source of truth, and a board added to the SDK is
// simulatable with no daemon change.

#define FSIM_PANEL_SSD1677 0
#define FSIM_PANEL_UC8253_X3 1
#define FSIM_PANEL_UC8179 2
#define FSIM_PANEL_UC8279 3
#define FSIM_PANEL_ED2208 4
#define FSIM_PANEL_IT8951 5
#define FSIM_PANEL_EXTERNAL 6  // driver owns its bus; frames arrive via push_frame

#define FSIM_INPUT_ADC_LADDER 0
#define FSIM_INPUT_DIGITAL 1
#define FSIM_INPUT_ONEPAGE_LADDER 2

#define FSIM_TOUCH_NONE 0
#define FSIM_TOUCH_CHSC6X 1
#define FSIM_TOUCH_GT911 2
#define FSIM_TOUCH_FT5X06 3
#define FSIM_TOUCH_FT6336U 4
#define FSIM_TOUCH_GSLX680 5

typedef struct {
  const char* board_name;

  uint16_t panel_width;
  uint16_t panel_height;
  uint8_t panel_controller;  // FSIM_PANEL_*
  int8_t epd_sclk, epd_mosi, epd_cs, epd_dc, epd_rst, epd_busy, epd_power_enable;
  uint8_t epd_mirror_x, epd_mirror_y;
  // The glass scans its gates in reverse of RAM row order, so controller RAM
  // row 0 is the bottom of the screen. Each affected driver compensates (the
  // SSD1677 path programs a descending Y window; the X3 UltraChip paths map
  // logical y to (height-1-y) gate coordinates), and the model has to apply the
  // physical reversal those compensations assume — otherwise every capture
  // comes out upside down. It is a property of the panel and its driver, not of
  // the controller family: the X4's UC8179/UC8279 panels are NOT reversed while
  // the X3's UC8279d is, which is why the bundle reports it instead of the
  // daemon inferring it from the controller id.
  uint8_t epd_gates_reversed;

  uint8_t input_style;  // FSIM_INPUT_*
  // Indexed by the SDK's button constants: BACK, CONFIRM, LEFT, RIGHT, UP,
  // DOWN, POWER. -1 where the board has no such key.
  int8_t buttons[7];
  uint8_t power_active_high;
  int8_t adc_ladder_pin;

  uint8_t touch_controller;  // FSIM_TOUCH_*
  int8_t touch_sda, touch_scl, touch_irq, touch_rst;
  uint8_t touch_addr, touch_addr_alt;
  uint16_t touch_raw_max_x, touch_raw_max_y;
  uint8_t touch_swap_xy, touch_flip_x, touch_flip_y;
  // The level the digitizer's interrupt line takes while a finger is down.
  // Firmware that waits on that edge rather than polling sees nothing at all
  // without it — the point is in the controller's registers and nothing has
  // told the firmware to go and read them.
  uint8_t touch_irq_active_low;
  // The rail the digitizer hangs off, where it has one of its own. A
  // controller whose rail is off is silent — it does not NACK politely, it is
  // not powered — and firmware that has not switched it on discovers nothing
  // at that address. Modelling that is what makes the firmware's own power-up
  // sequence something the simulator can be wrong about.
  int8_t touch_power_enable;
  uint8_t touch_power_active_high;

  int8_t battery_adc;
  float battery_divider;
  int8_t battery_charge_status;
  uint8_t battery_charge_active_high;
  int8_t sensors_sda, sensors_scl;
  uint8_t gauge_addr, rtc_addr, imu_addr;

  int8_t frontlight_pin;
  // A frontlight driven by an I2C controller rather than a PWM pin (the EEGO
  // A4's LM3630A). Zero where the board has none — and then nothing answers at
  // that address, which is what lets a firmware's probe discover it is absent.
  uint8_t frontlight_i2c_addr;
  int8_t sd_power_enable;
  uint8_t sd_power_active_high;
  // How the card is wired. A bundle never needs this — its file I/O goes
  // through the host — but an emulated image talks to a card controller, and
  // the simulator has to know which controller and which pins to put the
  // virtual card behind. bus_width 0 means the card is on SPI with the pins
  // above it; 1 or 4 means native SDMMC on the pins below.
  int8_t sd_sclk, sd_miso, sd_mosi, sd_cs;
  uint8_t sd_bus_width;
  int8_t sdmmc_clk, sdmmc_cmd, sdmmc_d0, sdmmc_d1, sdmmc_d2, sdmmc_d3;
} fsim_board_desc;

// Called once by the bundle's entry stub before setup(), from BoardConfig::ACTIVE.
void fsim_describe_board(const fsim_board_desc* desc);

// ── Bundle entry points (exported by the firmware bundle) ────────────────────
typedef struct {
  uint16_t abi_major;
  uint16_t abi_minor;
  const char* firmware_name;
  const char* board;       // BoardConfig profile name compiled in, e.g. "xteink_x3"
  uint16_t panel_width;    // native panel geometry, for the daemon's window
  uint16_t panel_height;
  const char* build_flags;  // the -D set the bundle was built with, for `sim info`
} freeink_sim_bundle_info;

#ifdef __cplusplus
}
#endif
