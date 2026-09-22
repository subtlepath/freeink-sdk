// FreeInk simulator — platform shim implementation.
//
// Compiled into every firmware bundle. Everything here is glue: it owns the
// Arduino globals (Serial, SPI, Wire), implements the ESP-IDF entry points the
// headers declare, and runs a FreeRTOS-on-pthreads port whose timed waits are
// measured on the *simulated* clock rather than wall time. Hardware state lives
// in the daemon, reached through the fsim_* ABI.

#include <Arduino.h>
#include <Preferences.h>
#include <SPI.h>
#include <SdFat.h>
#include <USB.h>
#include <USBMSC.h>
#include <WiFi.h>
#include <Wire.h>
#include <base64.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/i2s_pdm.h>
#include <driver/i2s_std.h>
#include <driver/ledc.h>
#include <driver/rtc_io.h>
#include <esp_adc_cal.h>
#include <esp_heap_caps.h>
#include <esp_now.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_rom_crc.h>
#include <esp_rom_sys.h>
#include <esp_private/esp_sleep_internal.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <nvs.h>
#include <soc/gpio_struct.h>
#include <spi_flash_mmap.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

// ── Arduino globals ──────────────────────────────────────────────────────────
SimSerial Serial;
SimSerial Serial0;
SimSerial USBSerial;
SimSPI SPI(0);
SimWire Wire(0);
SimWire Wire1(1);
SimUSB USB;
SimWiFiClass WiFi;
extern "C" gpio_dev_t GPIO;
gpio_dev_t GPIO = {};

// ── Simulated-clock waiting ──────────────────────────────────────────────────
// Every blocking primitive below funnels through this. It never sleeps on wall
// time: it slices the wait through fsim_delay_us(), so a paused or stepped
// machine freezes firmware tasks along with everything else.
namespace {

constexpr uint64_t kWaitSliceUs = 200;

// Waits until `pred` is true or the simulated deadline passes. Returns whether
// the predicate was satisfied.
template <typename Pred>
bool waitSimulated(std::unique_lock<std::mutex>& lock, TickType_t ticks_to_wait, Pred pred) {
  if (pred()) return true;
  if (ticks_to_wait == 0) return false;
  const bool forever = ticks_to_wait == portMAX_DELAY;
  const uint64_t deadline = fsim_micros() + static_cast<uint64_t>(ticks_to_wait) * 1000ULL;
  while (!pred()) {
    if (!forever && fsim_micros() >= deadline) return pred();
    lock.unlock();
    fsim_delay_us(kWaitSliceUs);
    lock.lock();
  }
  return true;
}

std::recursive_mutex& criticalMutex() {
  static std::recursive_mutex m;
  return m;
}

}  // namespace

extern "C" void fsim_rtos_enter_critical(portMUX_TYPE*) { criticalMutex().lock(); }
extern "C" void fsim_rtos_exit_critical(portMUX_TYPE*) { criticalMutex().unlock(); }

// ── Heap ─────────────────────────────────────────────────────────────────────
// One host heap, with per-capability accounting so `freeink-sim mem` can report
// how much the firmware asked for as PSRAM vs internal — the split that decides
// whether a build fits a real device.
namespace {
std::atomic<size_t> g_psramBytes{0};
std::atomic<size_t> g_internalBytes{0};
std::mutex g_allocMutex;
std::map<void*, std::pair<size_t, uint32_t>> g_allocs;

void accountAlloc(void* p, size_t size, uint32_t caps) {
  if (!p) return;
  std::lock_guard<std::mutex> lock(g_allocMutex);
  g_allocs[p] = {size, caps};
  if (caps & MALLOC_CAP_SPIRAM) {
    g_psramBytes += size;
  } else {
    g_internalBytes += size;
  }
}

void accountFree(void* p) {
  if (!p) return;
  std::lock_guard<std::mutex> lock(g_allocMutex);
  auto it = g_allocs.find(p);
  if (it == g_allocs.end()) return;
  if (it->second.second & MALLOC_CAP_SPIRAM) {
    g_psramBytes -= it->second.first;
  } else {
    g_internalBytes -= it->second.first;
  }
  g_allocs.erase(it);
}
}  // namespace

extern "C" void* heap_caps_malloc(size_t size, uint32_t caps) {
  void* p = malloc(size);
  accountAlloc(p, size, caps);
  return p;
}
extern "C" void* heap_caps_calloc(size_t n, size_t size, uint32_t caps) {
  void* p = calloc(n, size);
  accountAlloc(p, n * size, caps);
  return p;
}
extern "C" void* heap_caps_realloc(void* ptr, size_t size, uint32_t caps) {
  accountFree(ptr);
  void* p = realloc(ptr, size);
  accountAlloc(p, size, caps);
  return p;
}
extern "C" void* heap_caps_aligned_alloc(size_t alignment, size_t size, uint32_t caps) {
  void* p = nullptr;
  if (alignment < sizeof(void*)) alignment = sizeof(void*);
  // posix_memalign requires a power-of-two multiple of sizeof(void*).
  if (posix_memalign(&p, alignment, size) != 0) return nullptr;
  accountAlloc(p, size, caps);
  return p;
}
extern "C" void heap_caps_free(void* ptr) {
  accountFree(ptr);
  free(ptr);
}
// The host heap has no fixed ceiling; report the device-plausible budgets the
// daemon was configured with so firmware low-memory branches stay reachable.
extern "C" size_t heap_caps_get_total_size(uint32_t caps) {
  return (caps & MALLOC_CAP_SPIRAM) ? 8u * 1024 * 1024 : 320u * 1024;
}
extern "C" size_t heap_caps_get_free_size(uint32_t caps) {
  const size_t used = (caps & MALLOC_CAP_SPIRAM) ? g_psramBytes.load() : g_internalBytes.load();
  const size_t total = heap_caps_get_total_size(caps);
  return used >= total ? 0 : total - used;
}
extern "C" size_t heap_caps_get_largest_free_block(uint32_t caps) { return heap_caps_get_free_size(caps); }
extern "C" size_t heap_caps_get_minimum_free_size(uint32_t caps) { return heap_caps_get_free_size(caps); }

extern "C" uint32_t xPortGetFreeHeapSize(void) {
  return static_cast<uint32_t>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}
extern "C" uint32_t xPortGetMinimumEverFreeHeapSize(void) { return xPortGetFreeHeapSize(); }
extern "C" BaseType_t xPortInIsrContext(void) { return pdFALSE; }
extern "C" uint32_t esp_get_free_heap_size(void) { return xPortGetFreeHeapSize(); }
extern "C" uint32_t esp_get_minimum_free_heap_size(void) { return xPortGetFreeHeapSize(); }

// ── ROM helpers ──────────────────────────────────────────────────────────────
extern "C" int esp_rom_printf(const char* fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  const int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n > 0) fsim_log(buf, static_cast<size_t>(n) < sizeof(buf) ? static_cast<size_t>(n) : sizeof(buf) - 1);
  return n;
}
extern "C" void esp_rom_delay_us(uint32_t us) { fsim_delay_us(us); }
extern "C" void esp_rom_install_channel_putc(int, void (*)(char)) {}

extern "C" uint32_t esp_rom_crc32_le(uint32_t crc, const uint8_t* buf, uint32_t len) {
  crc = ~crc;
  for (uint32_t i = 0; i < len; ++i) {
    crc ^= buf[i];
    for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xEDB88320u & (~(crc & 1) + 1));
  }
  return ~crc;
}

extern "C" const char* esp_get_idf_version(void) { return "v5.3.0-freeink-sim"; }

// ── System / reset ───────────────────────────────────────────────────────────
extern "C" void esp_restart(void) {
  fsim_power_event(FSIM_POWER_RESTART, 0);
  // fsim_power_event does not return for a restart; keep the compiler happy.
  _exit(0);
}
extern "C" esp_reset_reason_t esp_reset_reason(void) {
  return fsim_wake_cause() == ESP_SLEEP_WAKEUP_UNDEFINED ? ESP_RST_POWERON : ESP_RST_DEEPSLEEP;
}

// ── Sleep ────────────────────────────────────────────────────────────────────
namespace {
uint64_t g_sleepTimerUs = 0;
}

extern "C" void esp_deep_sleep_start(void) {
  fsim_power_event(FSIM_POWER_DEEP_SLEEP, g_sleepTimerUs);
  _exit(0);
}
extern "C" void esp_deep_sleep(uint64_t time_in_us) {
  g_sleepTimerUs = time_in_us;
  esp_deep_sleep_start();
}
extern "C" esp_err_t esp_light_sleep_start(void) {
  fsim_power_event(FSIM_POWER_LIGHT_SLEEP, g_sleepTimerUs);
  return ESP_OK;
}
extern "C" esp_err_t esp_sleep_enable_timer_wakeup(uint64_t time_in_us) {
  g_sleepTimerUs = time_in_us;
  return ESP_OK;
}
extern "C" esp_err_t esp_sleep_enable_ext0_wakeup(int gpio_num, int level) {
  fsim_sleep_enable_wakeup(gpio_num, level);
  return ESP_OK;
}
extern "C" esp_err_t esp_sleep_enable_ext1_wakeup(uint64_t mask, esp_sleep_ext1_wakeup_mode_t mode) {
  for (int pin = 0; pin < 64; ++pin) {
    if (mask & (1ULL << pin)) fsim_sleep_enable_wakeup(pin, mode == ESP_EXT1_WAKEUP_ANY_HIGH ? 1 : 0);
  }
  return ESP_OK;
}
extern "C" esp_err_t esp_sleep_enable_ext1_wakeup_io(uint64_t mask, esp_sleep_ext1_wakeup_mode_t mode) {
  return esp_sleep_enable_ext1_wakeup(mask, mode);
}
extern "C" esp_err_t esp_deep_sleep_enable_gpio_wakeup(uint64_t mask, esp_deepsleep_gpio_wake_up_mode_t mode) {
  for (int pin = 0; pin < 64; ++pin) {
    if (mask & (1ULL << pin)) fsim_sleep_enable_wakeup(pin, mode == ESP_GPIO_WAKEUP_GPIO_HIGH ? 1 : 0);
  }
  return ESP_OK;
}
extern "C" esp_err_t esp_sleep_enable_gpio_wakeup(void) { return ESP_OK; }
extern "C" esp_err_t esp_sleep_disable_wakeup_source(int) { return ESP_OK; }
extern "C" esp_err_t esp_sleep_pd_config(esp_sleep_pd_domain_t, esp_sleep_pd_option_t) { return ESP_OK; }
extern "C" esp_sleep_wakeup_cause_t esp_sleep_get_wakeup_cause(void) {
  return static_cast<esp_sleep_wakeup_cause_t>(fsim_wake_cause());
}
extern "C" uint64_t esp_sleep_get_gpio_wakeup_status(void) { return 0; }
extern "C" uint64_t esp_sleep_get_ext1_wakeup_status(void) { return 0; }
extern "C" esp_err_t esp_sleep_config_gpio_isolate(void) { return ESP_OK; }
extern "C" esp_err_t gpio_deep_sleep_hold_en(void) { return ESP_OK; }
extern "C" esp_err_t gpio_deep_sleep_hold_dis(void) { return ESP_OK; }
extern "C" esp_err_t esp_sleep_sub_mode_config(esp_sleep_sub_mode_t, bool) { return ESP_OK; }

// ── Watchdog ─────────────────────────────────────────────────────────────────
extern "C" esp_err_t esp_task_wdt_reset(void) {
  fsim_trace("wdt", "feed", 4);
  return ESP_OK;
}
extern "C" esp_err_t esp_task_wdt_add(void*) { return ESP_OK; }
extern "C" esp_err_t esp_task_wdt_delete(void*) { return ESP_OK; }
extern "C" esp_err_t esp_task_wdt_status(void*) { return ESP_OK; }
extern "C" esp_err_t esp_task_wdt_init(uint32_t, bool) { return ESP_OK; }
extern "C" esp_err_t esp_task_wdt_deinit(void) { return ESP_OK; }

// ── GPIO (ESP-IDF surface over the same matrix as the Arduino shim) ──────────
extern "C" esp_err_t gpio_config(const gpio_config_t* cfg) {
  if (!cfg) return ESP_ERR_INVALID_ARG;
  uint32_t mode = 0;
  if (cfg->mode & GPIO_MODE_INPUT) mode |= FSIM_PIN_INPUT;
  if (cfg->mode & GPIO_MODE_OUTPUT) mode |= FSIM_PIN_OUTPUT;
  if (cfg->mode == GPIO_MODE_OUTPUT_OD || cfg->mode == GPIO_MODE_INPUT_OUTPUT_OD) mode |= FSIM_PIN_OPEN_DRAIN;
  if (cfg->pull_up_en == GPIO_PULLUP_ENABLE) mode |= FSIM_PIN_PULLUP;
  if (cfg->pull_down_en == GPIO_PULLDOWN_ENABLE) mode |= FSIM_PIN_PULLDOWN;
  for (int pin = 0; pin < 64; ++pin) {
    if (cfg->pin_bit_mask & (1ULL << pin)) fsim_pin_mode(pin, mode);
  }
  return ESP_OK;
}
extern "C" esp_err_t gpio_reset_pin(gpio_num_t pin) {
  fsim_gpio_hold(pin, 0);
  fsim_pin_mode(pin, FSIM_PIN_INPUT);
  return ESP_OK;
}
extern "C" esp_err_t gpio_set_direction(gpio_num_t pin, gpio_mode_t mode) {
  uint32_t m = 0;
  if (mode & GPIO_MODE_INPUT) m |= FSIM_PIN_INPUT;
  if (mode & GPIO_MODE_OUTPUT) m |= FSIM_PIN_OUTPUT;
  fsim_pin_mode(pin, m ? m : FSIM_PIN_INPUT);
  return ESP_OK;
}
extern "C" esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level) {
  fsim_gpio_write(pin, level ? 1 : 0);
  return ESP_OK;
}
extern "C" int gpio_get_level(gpio_num_t pin) { return fsim_gpio_read(pin); }
extern "C" esp_err_t gpio_set_pull_mode(gpio_num_t pin, int pull) {
  fsim_pin_mode(pin, FSIM_PIN_INPUT | (pull == 0 ? FSIM_PIN_PULLUP : FSIM_PIN_PULLDOWN));
  return ESP_OK;
}
extern "C" esp_err_t gpio_pullup_en(gpio_num_t pin) {
  fsim_pin_mode(pin, FSIM_PIN_INPUT | FSIM_PIN_PULLUP);
  return ESP_OK;
}
extern "C" esp_err_t gpio_pullup_dis(gpio_num_t pin) {
  fsim_pin_mode(pin, FSIM_PIN_INPUT);
  return ESP_OK;
}
extern "C" esp_err_t gpio_pulldown_en(gpio_num_t pin) {
  fsim_pin_mode(pin, FSIM_PIN_INPUT | FSIM_PIN_PULLDOWN);
  return ESP_OK;
}
extern "C" esp_err_t gpio_pulldown_dis(gpio_num_t pin) {
  fsim_pin_mode(pin, FSIM_PIN_INPUT);
  return ESP_OK;
}
extern "C" esp_err_t gpio_hold_en(gpio_num_t pin) {
  fsim_gpio_hold(pin, 1);
  return ESP_OK;
}
extern "C" esp_err_t gpio_hold_dis(gpio_num_t pin) {
  fsim_gpio_hold(pin, 0);
  return ESP_OK;
}
extern "C" esp_err_t gpio_sleep_sel_dis(gpio_num_t) { return ESP_OK; }
extern "C" esp_err_t gpio_install_isr_service(int) { return ESP_OK; }
extern "C" void gpio_uninstall_isr_service(void) {}

namespace {
// gpio_isr_handler_add takes an argument-carrying ISR; the ABI's attach point
// takes a bare one. Trampoline through a per-pin record.
struct IsrRecord {
  void (*fn)(void*) = nullptr;
  void* arg = nullptr;
};
IsrRecord g_isrs[64];

template <int Pin>
void isrTrampoline() {
  if (g_isrs[Pin].fn) g_isrs[Pin].fn(g_isrs[Pin].arg);
}

using Trampoline = void (*)();

// A table of 64 distinct thunks, one per pin, so each pin's ISR keeps its own
// argument without the daemon needing to pass one.
template <int... Pins>
constexpr auto makeTrampolines(std::integer_sequence<int, Pins...>) {
  return std::array<Trampoline, sizeof...(Pins)>{&isrTrampoline<Pins>...};
}
const auto g_trampolines = makeTrampolines(std::make_integer_sequence<int, 64>{});
}  // namespace

extern "C" esp_err_t gpio_isr_handler_add(gpio_num_t pin, void (*isr)(void*), void* arg) {
  if (pin < 0 || pin >= 64) return ESP_ERR_INVALID_ARG;
  g_isrs[pin] = {isr, arg};
  fsim_gpio_attach_interrupt(pin, g_trampolines[pin], CHANGE);
  return ESP_OK;
}
extern "C" esp_err_t gpio_isr_handler_remove(gpio_num_t pin) {
  if (pin < 0 || pin >= 64) return ESP_ERR_INVALID_ARG;
  g_isrs[pin] = {};
  fsim_gpio_detach_interrupt(pin);
  return ESP_OK;
}
extern "C" esp_err_t gpio_set_intr_type(gpio_num_t, gpio_int_type_t) { return ESP_OK; }
extern "C" esp_err_t gpio_intr_enable(gpio_num_t) { return ESP_OK; }
extern "C" esp_err_t gpio_intr_disable(gpio_num_t) { return ESP_OK; }
extern "C" esp_err_t gpio_wakeup_enable(gpio_num_t pin, gpio_int_type_t type) {
  fsim_sleep_enable_wakeup(pin, type == GPIO_INTR_HIGH_LEVEL || type == GPIO_INTR_POSEDGE ? 1 : 0);
  return ESP_OK;
}
extern "C" esp_err_t gpio_wakeup_disable(gpio_num_t) { return ESP_OK; }

// RTC-domain pads are the same virtual pins.
extern "C" esp_err_t rtc_gpio_init(gpio_num_t) { return ESP_OK; }
extern "C" esp_err_t rtc_gpio_deinit(gpio_num_t) { return ESP_OK; }
extern "C" esp_err_t rtc_gpio_set_direction(gpio_num_t pin, int mode) {
  return gpio_set_direction(pin, static_cast<gpio_mode_t>(mode));
}
extern "C" esp_err_t rtc_gpio_set_level(gpio_num_t pin, uint32_t level) { return gpio_set_level(pin, level); }
extern "C" int rtc_gpio_get_level(gpio_num_t pin) { return fsim_gpio_read(pin); }
extern "C" esp_err_t rtc_gpio_hold_en(gpio_num_t pin) { return gpio_hold_en(pin); }
extern "C" esp_err_t rtc_gpio_hold_dis(gpio_num_t pin) { return gpio_hold_dis(pin); }
extern "C" esp_err_t rtc_gpio_pullup_en(gpio_num_t pin) { return gpio_pullup_en(pin); }
extern "C" esp_err_t rtc_gpio_pulldown_dis(gpio_num_t pin) { return gpio_pulldown_dis(pin); }
extern "C" esp_err_t rtc_gpio_isolate(gpio_num_t) { return ESP_OK; }
extern "C" bool rtc_gpio_is_valid_gpio(gpio_num_t pin) { return pin >= 0 && pin < 64; }

// ── LEDC ─────────────────────────────────────────────────────────────────────
namespace {
uint32_t g_ledcDuty[16] = {};
int g_ledcPin[16] = {};
}  // namespace

extern "C" esp_err_t ledc_timer_config(const ledc_timer_config_t* cfg) {
  if (cfg) fsim_ledc_setup(cfg->timer_num, cfg->freq_hz, static_cast<uint8_t>(cfg->duty_resolution));
  return ESP_OK;
}
extern "C" esp_err_t ledc_channel_config(const ledc_channel_config_t* cfg) {
  if (!cfg) return ESP_ERR_INVALID_ARG;
  if (cfg->channel >= 0 && cfg->channel < 16) g_ledcPin[cfg->channel] = cfg->gpio_num;
  fsim_ledc_attach(cfg->gpio_num, cfg->channel);
  fsim_ledc_write(cfg->gpio_num, cfg->duty, 1);
  return ESP_OK;
}
extern "C" esp_err_t ledc_set_duty(ledc_mode_t, ledc_channel_t ch, uint32_t duty) {
  if (ch >= 0 && ch < 16) g_ledcDuty[ch] = duty;
  return ESP_OK;
}
extern "C" esp_err_t ledc_update_duty(ledc_mode_t, ledc_channel_t ch) {
  if (ch >= 0 && ch < 16) fsim_ledc_write(ch, g_ledcDuty[ch], 0);
  return ESP_OK;
}
extern "C" uint32_t ledc_get_duty(ledc_mode_t, ledc_channel_t ch) {
  return (ch >= 0 && ch < 16) ? g_ledcDuty[ch] : 0;
}
extern "C" esp_err_t ledc_stop(ledc_mode_t, ledc_channel_t ch, uint32_t idle_level) {
  fsim_ledc_write(ch, idle_level ? 0xFFFFFFFFu : 0, 0);
  return ESP_OK;
}
extern "C" esp_err_t ledc_set_freq(ledc_mode_t, ledc_timer_t timer, uint32_t freq_hz) {
  fsim_ledc_setup(timer, freq_hz, 8);
  return ESP_OK;
}

// ── I2C (ESP-IDF v5 master API) ──────────────────────────────────────────────
struct i2c_master_bus_t {
  int port;
};
struct i2c_master_dev_t {
  int port;
  uint16_t addr;
};

extern "C" esp_err_t i2c_new_master_bus(const i2c_master_bus_config_t* cfg, i2c_master_bus_handle_t* out) {
  if (!cfg || !out) return ESP_ERR_INVALID_ARG;
  fsim_i2c_begin(cfg->i2c_port, cfg->sda_io_num, cfg->scl_io_num, 400000);
  *out = new i2c_master_bus_t{cfg->i2c_port};
  return ESP_OK;
}
extern "C" esp_err_t i2c_del_master_bus(i2c_master_bus_handle_t bus) {
  delete bus;
  return ESP_OK;
}
extern "C" esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus, const i2c_device_config_t* cfg,
                                               i2c_master_dev_handle_t* out) {
  if (!bus || !cfg || !out) return ESP_ERR_INVALID_ARG;
  *out = new i2c_master_dev_t{bus->port, cfg->device_address};
  return ESP_OK;
}
extern "C" esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t dev) {
  delete dev;
  return ESP_OK;
}
extern "C" esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus, uint16_t address, int) {
  if (!bus) return ESP_ERR_INVALID_ARG;
  return fsim_i2c_xfer(bus->port, static_cast<uint8_t>(address), nullptr, 0, nullptr, 0) == 0 ? ESP_OK
                                                                                              : ESP_ERR_NOT_FOUND;
}
extern "C" esp_err_t i2c_master_transmit(i2c_master_dev_handle_t dev, const uint8_t* data, size_t len, int) {
  if (!dev) return ESP_ERR_INVALID_ARG;
  return fsim_i2c_xfer(dev->port, static_cast<uint8_t>(dev->addr), data, len, nullptr, 0) == 0 ? ESP_OK : ESP_FAIL;
}
extern "C" esp_err_t i2c_master_receive(i2c_master_dev_handle_t dev, uint8_t* data, size_t len, int) {
  if (!dev) return ESP_ERR_INVALID_ARG;
  return fsim_i2c_xfer(dev->port, static_cast<uint8_t>(dev->addr), nullptr, 0, data, len) == 0 ? ESP_OK : ESP_FAIL;
}
extern "C" esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t dev, const uint8_t* tx, size_t tx_len,
                                                 uint8_t* rx, size_t rx_len, int) {
  if (!dev) return ESP_ERR_INVALID_ARG;
  return fsim_i2c_xfer(dev->port, static_cast<uint8_t>(dev->addr), tx, tx_len, rx, rx_len) == 0 ? ESP_OK : ESP_FAIL;
}

// ── I2S ──────────────────────────────────────────────────────────────────────
struct i2s_chan_obj_t {
  int port;
  bool rx;
  bool enabled;
  uint32_t sample_rate;
  uint8_t bits;
  uint8_t channels;
};

extern "C" esp_err_t i2s_new_channel(const i2s_chan_config_t* cfg, i2s_chan_handle_t* tx, i2s_chan_handle_t* rx) {
  const int port = cfg ? static_cast<int>(cfg->id) : 0;
  if (tx) *tx = new i2s_chan_obj_t{port, false, false, 44100, 16, 2};
  if (rx) *rx = new i2s_chan_obj_t{port, true, false, 16000, 16, 1};
  return ESP_OK;
}
extern "C" esp_err_t i2s_del_channel(i2s_chan_handle_t h) {
  if (!h) return ESP_ERR_INVALID_ARG;
  if (h->rx) {
    fsim_mic_close(h->port);
  } else {
    fsim_audio_close(h->port);
  }
  delete h;
  return ESP_OK;
}
extern "C" esp_err_t i2s_channel_enable(i2s_chan_handle_t h) {
  if (!h) return ESP_ERR_INVALID_ARG;
  h->enabled = true;
  if (h->rx) {
    fsim_mic_open(h->port, h->sample_rate, h->bits);
  } else {
    fsim_audio_open(h->port, h->sample_rate, h->bits, h->channels);
  }
  return ESP_OK;
}
extern "C" esp_err_t i2s_channel_disable(i2s_chan_handle_t h) {
  if (!h) return ESP_ERR_INVALID_ARG;
  h->enabled = false;
  if (h->rx) {
    fsim_mic_close(h->port);
  } else {
    fsim_audio_close(h->port);
  }
  return ESP_OK;
}
extern "C" esp_err_t i2s_channel_write(i2s_chan_handle_t h, const void* src, size_t size, size_t* written,
                                       uint32_t) {
  if (!h || !h->enabled) return ESP_ERR_INVALID_STATE;
  fsim_audio_write(h->port, src, size);
  if (written) *written = size;
  // Writing N frames costs N/rate seconds of simulated time, so a firmware that
  // paces playback by the blocking write keeps the timing it has on device.
  const uint32_t frameBytes = static_cast<uint32_t>(h->bits / 8) * h->channels;
  if (frameBytes && h->sample_rate) {
    fsim_delay_us((static_cast<uint64_t>(size) / frameBytes) * 1000000ULL / h->sample_rate);
  }
  return ESP_OK;
}
extern "C" esp_err_t i2s_channel_read(i2s_chan_handle_t h, void* dst, size_t size, size_t* read, uint32_t) {
  if (!h || !h->enabled) return ESP_ERR_INVALID_STATE;
  const size_t n = fsim_mic_read(h->port, dst, size);
  if (read) *read = n;
  const uint32_t frameBytes = static_cast<uint32_t>(h->bits / 8);
  if (frameBytes && h->sample_rate) {
    fsim_delay_us((static_cast<uint64_t>(n) / frameBytes) * 1000000ULL / h->sample_rate);
  }
  return ESP_OK;
}
extern "C" esp_err_t i2s_channel_init_std_mode(i2s_chan_handle_t h, const i2s_std_config_t* cfg) {
  if (!h || !cfg) return ESP_ERR_INVALID_ARG;
  h->sample_rate = cfg->clk_cfg.sample_rate_hz;
  h->bits = static_cast<uint8_t>(cfg->slot_cfg.data_bit_width);
  h->channels = static_cast<uint8_t>(cfg->slot_cfg.slot_mode);
  return ESP_OK;
}
extern "C" esp_err_t i2s_channel_reconfig_std_clock(i2s_chan_handle_t h, const i2s_std_clk_config_t* cfg) {
  if (!h || !cfg) return ESP_ERR_INVALID_ARG;
  h->sample_rate = cfg->sample_rate_hz;
  return ESP_OK;
}
extern "C" esp_err_t i2s_channel_reconfig_std_slot(i2s_chan_handle_t h, const i2s_std_slot_config_t* cfg) {
  if (!h || !cfg) return ESP_ERR_INVALID_ARG;
  h->bits = static_cast<uint8_t>(cfg->data_bit_width);
  h->channels = static_cast<uint8_t>(cfg->slot_mode);
  return ESP_OK;
}
extern "C" esp_err_t i2s_channel_init_pdm_rx_mode(i2s_chan_handle_t h, const i2s_pdm_rx_config_t* cfg) {
  if (!h || !cfg) return ESP_ERR_INVALID_ARG;
  h->sample_rate = cfg->clk_cfg.sample_rate_hz;
  h->bits = static_cast<uint8_t>(cfg->slot_cfg.data_bit_width);
  h->channels = 1;
  return ESP_OK;
}

// ── ADC calibration (legacy API) ─────────────────────────────────────────────
extern "C" esp_adc_cal_value_t esp_adc_cal_characterize(adc_unit_t, adc_atten_t, adc_bits_width_t, uint32_t,
                                                        esp_adc_cal_characteristics_t* chars) {
  if (chars) *chars = {};
  return ESP_ADC_CAL_VAL_EFUSE_TP;
}
extern "C" uint32_t esp_adc_cal_raw_to_voltage(uint32_t raw, const esp_adc_cal_characteristics_t*) {
  // The 11 dB / 12-bit scaling the SDK's battery divider math assumes.
  return raw * 3300u / 4095u;
}

// ── NVS (raw C API over the daemon's key/value store) ────────────────────────
namespace {
struct NvsHandle {
  std::string ns;
};
std::map<nvs_handle_t, NvsHandle> g_nvsHandles;
nvs_handle_t g_nextNvs = 1;
std::mutex g_nvsMutex;

const char* nvsNamespace(nvs_handle_t h) {
  std::lock_guard<std::mutex> lock(g_nvsMutex);
  auto it = g_nvsHandles.find(h);
  return it == g_nvsHandles.end() ? "" : it->second.ns.c_str();
}
}  // namespace

extern "C" esp_err_t nvs_flash_init(void) { return ESP_OK; }
extern "C" esp_err_t nvs_flash_erase(void) { return fsim_nvs_clear(nullptr) == 0 ? ESP_OK : ESP_FAIL; }
extern "C" esp_err_t nvs_open(const char* name, nvs_open_mode_t, nvs_handle_t* out) {
  if (!out) return ESP_ERR_INVALID_ARG;
  std::lock_guard<std::mutex> lock(g_nvsMutex);
  const nvs_handle_t h = g_nextNvs++;
  g_nvsHandles[h] = NvsHandle{name ? name : ""};
  *out = h;
  return ESP_OK;
}
extern "C" void nvs_close(nvs_handle_t h) {
  std::lock_guard<std::mutex> lock(g_nvsMutex);
  g_nvsHandles.erase(h);
}
extern "C" esp_err_t nvs_commit(nvs_handle_t) { return ESP_OK; }

namespace {
esp_err_t nvsGetFixed(nvs_handle_t h, const char* key, void* out, size_t want) {
  size_t len = 0;
  if (fsim_nvs_get(nvsNamespace(h), key, out, want, &len) != 0) return ESP_ERR_NVS_NOT_FOUND;
  return len == want ? ESP_OK : ESP_ERR_NVS_NOT_FOUND;
}
esp_err_t nvsSetFixed(nvs_handle_t h, const char* key, const void* v, size_t len) {
  return fsim_nvs_set(nvsNamespace(h), key, v, len) == 0 ? ESP_OK : ESP_FAIL;
}
}  // namespace

extern "C" esp_err_t nvs_get_u8(nvs_handle_t h, const char* k, uint8_t* o) { return nvsGetFixed(h, k, o, 1); }
extern "C" esp_err_t nvs_set_u8(nvs_handle_t h, const char* k, uint8_t v) { return nvsSetFixed(h, k, &v, 1); }
extern "C" esp_err_t nvs_get_u16(nvs_handle_t h, const char* k, uint16_t* o) { return nvsGetFixed(h, k, o, 2); }
extern "C" esp_err_t nvs_set_u16(nvs_handle_t h, const char* k, uint16_t v) { return nvsSetFixed(h, k, &v, 2); }
extern "C" esp_err_t nvs_get_u32(nvs_handle_t h, const char* k, uint32_t* o) { return nvsGetFixed(h, k, o, 4); }
extern "C" esp_err_t nvs_set_u32(nvs_handle_t h, const char* k, uint32_t v) { return nvsSetFixed(h, k, &v, 4); }
extern "C" esp_err_t nvs_get_i32(nvs_handle_t h, const char* k, int32_t* o) { return nvsGetFixed(h, k, o, 4); }
extern "C" esp_err_t nvs_set_i32(nvs_handle_t h, const char* k, int32_t v) { return nvsSetFixed(h, k, &v, 4); }
extern "C" esp_err_t nvs_get_str(nvs_handle_t h, const char* k, char* out, size_t* length) {
  if (!length) return ESP_ERR_INVALID_ARG;
  size_t len = 0;
  if (fsim_nvs_get(nvsNamespace(h), k, out, out ? *length : 0, &len) != 0) return ESP_ERR_NVS_NOT_FOUND;
  *length = len;
  return ESP_OK;
}
extern "C" esp_err_t nvs_set_str(nvs_handle_t h, const char* k, const char* v) {
  return nvsSetFixed(h, k, v, v ? strlen(v) + 1 : 0);
}
extern "C" esp_err_t nvs_get_blob(nvs_handle_t h, const char* k, void* out, size_t* length) {
  return nvs_get_str(h, k, static_cast<char*>(out), length);
}
extern "C" esp_err_t nvs_set_blob(nvs_handle_t h, const char* k, const void* v, size_t length) {
  return nvsSetFixed(h, k, v, length);
}
extern "C" esp_err_t nvs_erase_key(nvs_handle_t h, const char* k) {
  return fsim_nvs_erase(nvsNamespace(h), k) == 0 ? ESP_OK : ESP_ERR_NVS_NOT_FOUND;
}
extern "C" esp_err_t nvs_erase_all(nvs_handle_t h) {
  return fsim_nvs_clear(nvsNamespace(h)) == 0 ? ESP_OK : ESP_FAIL;
}

// ── Wi-Fi / ESP-NOW / network ────────────────────────────────────────────────
extern "C" esp_err_t esp_wifi_set_ps(wifi_ps_type_t) { return ESP_OK; }
extern "C" esp_err_t esp_wifi_set_channel(uint8_t, wifi_second_chan_t) { return ESP_OK; }
extern "C" esp_err_t esp_wifi_get_mac(wifi_interface_t ifx, uint8_t mac[6]) {
  return esp_read_mac(mac, ifx == WIFI_IF_AP ? ESP_MAC_WIFI_SOFTAP : ESP_MAC_WIFI_STA);
}
extern "C" esp_err_t esp_wifi_start(void) { return ESP_OK; }
extern "C" esp_err_t esp_wifi_stop(void) { return ESP_OK; }

namespace {
esp_now_recv_cb_t g_espNowRecv = nullptr;
esp_now_send_cb_t g_espNowSend = nullptr;
std::vector<std::array<uint8_t, ESP_NOW_ETH_ALEN>> g_espNowPeers;
}  // namespace

extern "C" esp_err_t esp_now_init(void) { return ESP_OK; }
extern "C" esp_err_t esp_now_deinit(void) {
  g_espNowPeers.clear();
  g_espNowRecv = nullptr;
  g_espNowSend = nullptr;
  return ESP_OK;
}
extern "C" esp_err_t esp_now_add_peer(const esp_now_peer_info_t* peer) {
  if (!peer) return ESP_ERR_INVALID_ARG;
  std::array<uint8_t, ESP_NOW_ETH_ALEN> a{};
  memcpy(a.data(), peer->peer_addr, ESP_NOW_ETH_ALEN);
  g_espNowPeers.push_back(a);
  return ESP_OK;
}
extern "C" esp_err_t esp_now_del_peer(const uint8_t* addr) {
  if (!addr) return ESP_ERR_INVALID_ARG;
  for (auto it = g_espNowPeers.begin(); it != g_espNowPeers.end(); ++it) {
    if (memcmp(it->data(), addr, ESP_NOW_ETH_ALEN) == 0) {
      g_espNowPeers.erase(it);
      return ESP_OK;
    }
  }
  return ESP_ERR_NOT_FOUND;
}
extern "C" bool esp_now_is_peer_exist(const uint8_t* addr) {
  if (!addr) return false;
  for (const auto& p : g_espNowPeers) {
    if (memcmp(p.data(), addr, ESP_NOW_ETH_ALEN) == 0) return true;
  }
  return false;
}
extern "C" esp_err_t esp_now_send(const uint8_t* addr, const uint8_t* data, size_t len) {
  // Frames are handed to the daemon, which relays them to any peer instance
  // sharing the channel. With no peer attached the send simply succeeds.
  fsim_trace("espnow", reinterpret_cast<const char*>(data), len);
  if (g_espNowSend && addr) g_espNowSend(addr, 0);
  return ESP_OK;
}
extern "C" esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t cb) {
  g_espNowRecv = cb;
  return ESP_OK;
}
extern "C" esp_err_t esp_now_unregister_recv_cb(void) {
  g_espNowRecv = nullptr;
  return ESP_OK;
}
extern "C" esp_err_t esp_now_register_send_cb(esp_now_send_cb_t cb) {
  g_espNowSend = cb;
  return ESP_OK;
}
extern "C" esp_err_t esp_now_unregister_send_cb(void) {
  g_espNowSend = nullptr;
  return ESP_OK;
}

int16_t SimWiFiClass::scanNetworks(bool, bool) {
  char json[4096];
  const int n = fsim_wifi_scan(json, sizeof(json));
  _scan.clear();
  if (n <= 0) return 0;
  // The daemon returns one "ssid\trssi\tenc" record per line — enough structure
  // for a scan list without pulling a JSON parser into the bundle.
  const char* p = json;
  while (*p) {
    const char* eol = strchr(p, '\n');
    const size_t lineLen = eol ? static_cast<size_t>(eol - p) : strlen(p);
    std::string line(p, lineLen);
    const size_t t1 = line.find('\t');
    const size_t t2 = t1 == std::string::npos ? std::string::npos : line.find('\t', t1 + 1);
    if (t1 != std::string::npos) {
      ScanEntry e;
      e.ssid = line.substr(0, t1);
      e.rssi = atoi(line.c_str() + t1 + 1);
      e.enc = t2 == std::string::npos ? 0 : static_cast<uint8_t>(atoi(line.c_str() + t2 + 1));
      _scan.push_back(e);
    }
    if (!eol) break;
    p = eol + 1;
  }
  return static_cast<int16_t>(_scan.size());
}
String SimWiFiClass::SSID(uint8_t i) { return i < _scan.size() ? String(_scan[i].ssid) : String(); }
int32_t SimWiFiClass::RSSI(uint8_t i) { return i < _scan.size() ? _scan[i].rssi : -100; }
uint8_t SimWiFiClass::encryptionType(uint8_t i) { return i < _scan.size() ? _scan[i].enc : 0; }
void SimWiFiClass::scanDelete() { _scan.clear(); }

// ── base64 ───────────────────────────────────────────────────────────────────
String base64::encode(const uint8_t* data, size_t length) {
  static const char* kAlphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve(((length + 2) / 3) * 4);
  for (size_t i = 0; i < length; i += 3) {
    const uint32_t a = data[i];
    const uint32_t b = i + 1 < length ? data[i + 1] : 0;
    const uint32_t c = i + 2 < length ? data[i + 2] : 0;
    const uint32_t triple = (a << 16) | (b << 8) | c;
    out += kAlphabet[(triple >> 18) & 0x3F];
    out += kAlphabet[(triple >> 12) & 0x3F];
    out += i + 1 < length ? kAlphabet[(triple >> 6) & 0x3F] : '=';
    out += i + 2 < length ? kAlphabet[triple & 0x3F] : '=';
  }
  return String(out);
}

// ── USB mass storage ─────────────────────────────────────────────────────────
namespace {
msc_read_cb g_mscRead = nullptr;
msc_write_cb g_mscWrite = nullptr;
msc_start_stop_cb g_mscStartStop = nullptr;
}  // namespace

bool USBMSC::begin(uint32_t block_count, uint16_t block_size) {
  fsim_usb_msc_begin(block_count, block_size);
  return true;
}
void USBMSC::end() { fsim_usb_msc_end(); }
void USBMSC::onRead(msc_read_cb cb) { g_mscRead = cb; }
void USBMSC::onWrite(msc_write_cb cb) { g_mscWrite = cb; }
void USBMSC::onStartStop(msc_start_stop_cb cb) { g_mscStartStop = cb; }

// Reached from the daemon when the CLI drives the MSC endpoint, so the
// firmware's own sector callbacks run.
extern "C" int32_t fsim_bundle_msc_read(uint32_t lba, uint32_t offset, void* buf, uint32_t size) {
  return g_mscRead ? g_mscRead(lba, offset, buf, size) : -1;
}
extern "C" int32_t fsim_bundle_msc_write(uint32_t lba, uint32_t offset, uint8_t* buf, uint32_t size) {
  return g_mscWrite ? g_mscWrite(lba, offset, buf, size) : -1;
}
extern "C" int fsim_bundle_msc_start_stop(uint8_t power, int start, int eject) {
  return g_mscStartStop ? (g_mscStartStop(power, start != 0, eject != 0) ? 1 : 0) : 0;
}
