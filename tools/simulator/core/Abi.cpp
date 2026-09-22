// FreeInk simulator — daemon side of the fsim_* ABI.
//
// These are the symbols a firmware bundle resolves at dlopen() time. Each one
// is a thin adapter onto the Machine; the interesting behaviour lives there and
// in the panel model. Two things are worth pointing out:
//
//   * fsim_spi_transfer reads the D/C pin out of the GPIO matrix for every
//     byte. That is how command/data framing stays honest: a driver that
//     forgets to lower D/C before a command byte writes pixels instead, here as
//     on glass.
//   * fsim_gpio_read special-cases the BUSY pin, presenting the panel's state
//     with the controller family's polarity, so EpdBus's waits behave as they
//     do on device.

#include "I2cDevices.h"
#include "Machine.h"
#include "Panel.h"

#include <cstring>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

using freeink::sim::Gpio;
using freeink::sim::Machine;
using freeink::sim::PowerState;

namespace {

Machine& machine() { return Machine::instance(); }

// A firmware thread calling into the model is "running" as far as the virtual
// clock is concerned; it stops being so only while it is blocked in a wait.
struct RunningScope {
  RunningScope() { machine().clock().enterRunning(); }
  ~RunningScope() { machine().clock().exitRunning(); }
};

// SPI bus state the panel decode needs.
struct SpiState {
  int sclk = -1;
  int miso = -1;
  int mosi = -1;
  int cs = -1;
  uint32_t hz = 1000000;
};
SpiState g_spi[2];

std::vector<int> g_netSockets;

}  // namespace

extern "C" {

// ── Time ─────────────────────────────────────────────────────────────────────
uint64_t fsim_micros(void) { return machine().clock().nowUs(); }

int fsim_delay_us(uint64_t us) {
  if (machine().tearingDown()) return 1;
  return machine().clock().sleepUs(us);
}

int fsim_should_exit(void) { return machine().tearingDown() ? 1 : 0; }

void fsim_yield(void) {
  // A cooperative point: give the clock a chance to advance in virtual mode and
  // let control commands land between firmware loop iterations.
  machine().clock().sleepUs(0);
}

// ── GPIO ─────────────────────────────────────────────────────────────────────
void fsim_pin_mode(int pin, uint32_t mode) {
  RunningScope scope;
  std::lock_guard<std::mutex> lock(machine().mutex());
  machine().gpio().setMode(pin, mode);
}

void fsim_gpio_write(int pin, int level) {
  RunningScope scope;
  int before = 0;
  int after = 0;
  {
    std::lock_guard<std::mutex> lock(machine().mutex());
    Gpio& gpio = machine().gpio();
    before = gpio.read(pin);
    gpio.write(pin, level);
    after = gpio.read(pin);
  }

  // The panel's reset line is wired for real: a LOW pulse resets the
  // controller, discarding any half-programmed window or standing command.
  const auto& board = machine().board();
  if (machine().boardKnown() && board.epd_rst >= 0 && pin == board.epd_rst && before == 1 && after == 0) {
    machine().panel().hardwareReset();
  }

  // Fire any armed ISR outside the lock so the handler can take it.
  if (before != after) machine().gpio().fireInterrupts(pin, before, after);
}

int fsim_gpio_read(int pin) {
  RunningScope scope;
  machine().noteGpioRead(pin);
  std::lock_guard<std::mutex> lock(machine().mutex());
  return machine().gpio().read(pin);
}

void fsim_gpio_hold(int pin, int enable) {
  std::lock_guard<std::mutex> lock(machine().mutex());
  machine().gpio().setHold(pin, enable != 0);
}

void fsim_gpio_attach_interrupt(int pin, void (*isr)(void), int mode) {
  std::lock_guard<std::mutex> lock(machine().mutex());
  machine().gpio().attachInterrupt(pin, isr, mode);
}

void fsim_gpio_detach_interrupt(int pin) {
  std::lock_guard<std::mutex> lock(machine().mutex());
  machine().gpio().detachInterrupt(pin);
}

// ── ADC ──────────────────────────────────────────────────────────────────────
int fsim_adc_read(int pin) {
  RunningScope scope;
  machine().noteAdcRead(pin);
  std::lock_guard<std::mutex> lock(machine().mutex());
  return machine().adc().readRaw(pin);
}

int fsim_adc_read_mv(int pin) {
  RunningScope scope;
  machine().noteAdcRead(pin);
  std::lock_guard<std::mutex> lock(machine().mutex());
  return machine().adc().readMillivolts(pin);
}

void fsim_adc_set_attenuation(int, int) {}

// ── SPI ──────────────────────────────────────────────────────────────────────
void fsim_spi_begin(int bus, int sclk, int miso, int mosi, int cs) {
  if (bus < 0 || bus > 1) return;
  g_spi[bus] = {sclk, miso, mosi, cs, g_spi[bus].hz};
}

void fsim_spi_end(int) {}

void fsim_spi_settings(int bus, uint32_t hz, uint8_t, uint8_t) {
  if (bus < 0 || bus > 1) return;
  g_spi[bus].hz = hz;
}

void fsim_spi_transfer(int bus, const uint8_t* tx, uint8_t* rx, size_t len) {
  RunningScope scope;
  if (bus < 0 || bus > 1 || len == 0) return;

  const auto& board = machine().board();
  if (!machine().boardKnown() || board.epd_dc < 0) {
    if (rx) memset(rx, 0xFF, len);
    return;
  }

  // Only traffic while the panel's chip select is asserted reaches it. A driver
  // that streams a frame without lowering CS paints nothing, here as on glass.
  bool selected = true;
  bool isData = true;
  {
    std::lock_guard<std::mutex> lock(machine().mutex());
    Gpio& gpio = machine().gpio();
    if (board.epd_cs >= 0) selected = gpio.read(board.epd_cs) == 0;
    isData = gpio.read(board.epd_dc) != 0;
  }
  if (!selected) {
    if (rx) memset(rx, 0xFF, len);
    return;
  }

  auto& panel = machine().panel();
  for (size_t i = 0; i < len; ++i) {
    if (tx) panel.spiByte(tx[i], isData);
    if (rx) rx[i] = panel.spiReadByte();
  }

  // Charge the transfer its wire time so a slow bus really is slow: streaming
  // 48 KB at 10 MHz costs ~38 ms of simulated time, which is what makes a
  // firmware's refresh budget measurable here.
  const uint32_t hz = g_spi[bus].hz ? g_spi[bus].hz : 10000000u;
  const uint64_t us = (static_cast<uint64_t>(len) * 8ULL * 1000000ULL) / hz;
  if (us > 0) machine().clock().sleepUs(us);
}

// ── I2C ──────────────────────────────────────────────────────────────────────
void fsim_i2c_begin(int bus, int sda, int scl, uint32_t hz) { machine().i2c().begin(bus, sda, scl, hz); }

int fsim_i2c_xfer(int bus, uint8_t addr, const uint8_t* tx, size_t tx_len, uint8_t* rx, size_t rx_len) {
  RunningScope scope;
  return machine().i2c().transfer(bus, addr, tx, tx_len, rx, rx_len) ? 0 : 1;
}

// ── LEDC ─────────────────────────────────────────────────────────────────────
void fsim_ledc_setup(int channel, uint32_t freq_hz, uint8_t resolution_bits) {
  std::lock_guard<std::mutex> lock(machine().mutex());
  machine().peripherals().pwmFreq[channel] = freq_hz;
  machine().peripherals().pwmBits[channel] = resolution_bits;
}

void fsim_ledc_attach(int pin, int channel) {
  std::lock_guard<std::mutex> lock(machine().mutex());
  machine().peripherals().pwmChannelPin[channel] = pin;
}

void fsim_ledc_write(int pin_or_channel, uint32_t duty, int by_pin) {
  std::lock_guard<std::mutex> lock(machine().mutex());
  auto& p = machine().peripherals();
  int pin = pin_or_channel;
  if (!by_pin) {
    auto it = p.pwmChannelPin.find(pin_or_channel);
    if (it != p.pwmChannelPin.end()) pin = it->second;
  }
  p.pwmDuty[pin] = duty;
}

// ── Audio ────────────────────────────────────────────────────────────────────
void fsim_audio_open(int, uint32_t sample_rate, uint8_t bits, uint8_t channels) {
  std::lock_guard<std::mutex> lock(machine().mutex());
  auto& p = machine().peripherals();
  p.audioOpen = true;
  p.audioSampleRate = sample_rate;
  p.audioBits = bits;
  p.audioChannels = channels;
  p.audioCapture.clear();
}

void fsim_audio_write(int, const void* samples, size_t bytes) {
  if (!samples || bytes == 0) return;
  std::lock_guard<std::mutex> lock(machine().mutex());
  auto& capture = machine().peripherals().audioCapture;
  // Bounded: a long playback should not grow the daemon without limit. 16 MB
  // is about three minutes of 44.1 kHz stereo, past which the oldest goes.
  constexpr size_t kMaxCapture = 16u * 1024 * 1024;
  const uint8_t* data = static_cast<const uint8_t*>(samples);
  capture.insert(capture.end(), data, data + bytes);
  if (capture.size() > kMaxCapture) {
    capture.erase(capture.begin(), capture.begin() + (capture.size() - kMaxCapture));
  }
}

void fsim_audio_close(int) {
  std::lock_guard<std::mutex> lock(machine().mutex());
  machine().peripherals().audioOpen = false;
}

void fsim_mic_open(int, uint32_t, uint8_t) {}
void fsim_mic_close(int) {}

size_t fsim_mic_read(int, void* samples, size_t bytes) {
  if (!samples || bytes == 0) return 0;
  std::lock_guard<std::mutex> lock(machine().mutex());
  auto& p = machine().peripherals();
  uint8_t* out = static_cast<uint8_t*>(samples);
  if (p.micQueue.empty()) {
    // No clip queued: hand back silence rather than blocking, so a firmware
    // that always records gets a well-defined, quiet input.
    memset(out, 0, bytes);
    return bytes;
  }
  const size_t available = p.micQueue.size() - p.micPos;
  const size_t n = bytes < available ? bytes : available;
  memcpy(out, p.micQueue.data() + p.micPos, n);
  p.micPos += n;
  if (n < bytes) memset(out + n, 0, bytes - n);
  if (p.micPos >= p.micQueue.size()) {
    p.micQueue.clear();
    p.micPos = 0;
  }
  return bytes;
}

// ── NVS ──────────────────────────────────────────────────────────────────────
int fsim_nvs_get(const char* ns, const char* key, void* out, size_t cap, size_t* out_len) {
  if (!key) return 1;
  std::string value;
  if (!machine().storage().nvsGet(ns ? ns : "", key, &value)) return 1;
  if (out_len) *out_len = value.size();
  if (out && cap > 0) {
    const size_t n = value.size() < cap ? value.size() : cap;
    memcpy(out, value.data(), n);
  }
  return 0;
}

int fsim_nvs_set(const char* ns, const char* key, const void* data, size_t len) {
  if (!key) return 1;
  machine().storage().nvsSet(ns ? ns : "", key,
                             std::string(static_cast<const char*>(data), data ? len : 0));
  return 0;
}

int fsim_nvs_erase(const char* ns, const char* key) {
  return key && machine().storage().nvsErase(ns ? ns : "", key) ? 0 : 1;
}

int fsim_nvs_clear(const char* ns) {
  machine().storage().nvsClear(ns ? ns : "");
  return 0;
}

// ── Storage ──────────────────────────────────────────────────────────────────
int fsim_sd_present(void) { return machine().storage().cardPresent() ? 1 : 0; }

int fsim_sd_resolve(const char* fw_path, char* out, size_t cap) {
  if (!fw_path || !out || cap == 0) return 1;
  std::string resolved;
  if (!machine().storage().resolve(fw_path, &resolved)) return 1;
  if (resolved.size() + 1 > cap) return 1;
  memcpy(out, resolved.c_str(), resolved.size() + 1);
  return 0;
}

uint64_t fsim_sd_capacity_bytes(void) { return machine().storage().cardCapacity(); }

// ── Power ────────────────────────────────────────────────────────────────────
void fsim_power_event(int kind, uint64_t sleep_us) {
  switch (kind) {
    case FSIM_POWER_RESTART:
      machine().log().append("sim", "[sim] firmware requested restart\n", 34);
      machine().requestRestart();
      break;
    case FSIM_POWER_DEEP_SLEEP:
      machine().log().append("sim", "[sim] firmware entered deep sleep\n", 34);
      machine().enterDeepSleep(sleep_us);
      break;
    case FSIM_POWER_LIGHT_SLEEP:
      machine().enterLightSleep(sleep_us);
      return;  // light sleep returns to the caller
    default:
      machine().requestRestart();
      break;
  }
  // Deep sleep and restart never return: park until the daemon tears the
  // bundle down and re-enters setup().
  while (!machine().tearingDown()) machine().clock().sleepUs(10000);
  // The firmware thread unwinds through the bundle's teardown path.
  pthread_exit(nullptr);
}

void fsim_sleep_enable_wakeup(int pin, int level) { machine().setSleepWakePin(pin, level); }

int fsim_wake_cause(void) { return machine().wakeCause(); }

uint32_t fsim_random(void) { return machine().nextRandom(); }

void fsim_log(const char* text, size_t len) {
  if (text && len > 0) machine().log().append("serial", text, len);
}

void fsim_trace(const char* channel, const char* text, size_t len) {
  if (text && len > 0) machine().log().append(channel ? channel : "trace", text, len);
}

// ── Board description ────────────────────────────────────────────────────────
void fsim_describe_board(const fsim_board_desc* desc) {
  if (desc) machine().describeBoard(*desc);
}

// ── Wi-Fi / network ──────────────────────────────────────────────────────────
int fsim_wifi_begin(const char* ssid, const char* pass) {
  std::lock_guard<std::mutex> lock(machine().mutex());
  auto& p = machine().peripherals();
  p.wifiSsid = ssid ? ssid : "";
  // Association takes time, and whether it succeeds is the operator's call:
  // the network must be in the virtual list and the password must match.
  bool found = false;
  for (const auto& network : p.networks) {
    if (network.ssid != p.wifiSsid) continue;
    found = network.password.empty() || network.password == (pass ? pass : "");
    break;
  }
  p.wifiState = found ? FSIM_WIFI_CONNECTING : FSIM_WIFI_FAILED;
  p.wifiJoinCompleteUs = machine().clock().nowUs() + 1500000ULL;  // ~1.5 s to associate
  p.wifiLocalIp = found ? 0x0A00020Fu : 0u;                       // 15.2.0.10 little-endian
  return found ? 0 : 1;
}

int fsim_wifi_status(void) {
  std::lock_guard<std::mutex> lock(machine().mutex());
  auto& p = machine().peripherals();
  if (p.wifiState == FSIM_WIFI_CONNECTING && machine().clock().nowUs() >= p.wifiJoinCompleteUs) {
    p.wifiState = FSIM_WIFI_CONNECTED;
  }
  return p.wifiState;
}

void fsim_wifi_disconnect(void) {
  std::lock_guard<std::mutex> lock(machine().mutex());
  auto& p = machine().peripherals();
  p.wifiState = FSIM_WIFI_IDLE;
  p.wifiLocalIp = 0;
}

int fsim_wifi_scan(char* out_json, size_t cap) {
  std::lock_guard<std::mutex> lock(machine().mutex());
  std::string out;
  for (const auto& network : machine().peripherals().networks) {
    out += network.ssid + "\t" + std::to_string(network.rssi) + "\t" + std::to_string(network.enc) + "\n";
  }
  if (!out_json || out.size() + 1 > cap) return 0;
  memcpy(out_json, out.c_str(), out.size() + 1);
  return static_cast<int>(machine().peripherals().networks.size());
}

uint32_t fsim_wifi_local_ip(void) {
  std::lock_guard<std::mutex> lock(machine().mutex());
  return machine().peripherals().wifiLocalIp;
}

int fsim_net_connect(const char* host, uint16_t port) {
  // Off unless the operator enabled it: a simulated device should not reach the
  // internet by accident just because the firmware asked.
  {
    std::lock_guard<std::mutex> lock(machine().mutex());
    if (!machine().peripherals().networkEnabled) {
      machine().log().append("sim", "[sim] outbound connection refused (networking disabled)\n", 55);
      return -1;
    }
    if (machine().peripherals().wifiState != FSIM_WIFI_CONNECTED) return -1;
  }
  if (!host) return -1;

  struct addrinfo hints {};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  struct addrinfo* result = nullptr;
  if (::getaddrinfo(host, std::to_string(port).c_str(), &hints, &result) != 0) return -1;

  int sock = -1;
  for (struct addrinfo* it = result; it; it = it->ai_next) {
    sock = ::socket(it->ai_family, it->ai_socktype, it->ai_protocol);
    if (sock < 0) continue;
    if (::connect(sock, it->ai_addr, it->ai_addrlen) == 0) break;
    ::close(sock);
    sock = -1;
  }
  ::freeaddrinfo(result);
  return sock;
}

int fsim_net_write(int sock, const void* data, size_t len) {
  if (sock < 0) return -1;
  return static_cast<int>(::send(sock, data, len, 0));
}

int fsim_net_read(int sock, void* data, size_t len) {
  if (sock < 0) return -1;
  return static_cast<int>(::recv(sock, data, len, 0));
}

void fsim_net_close(int sock) {
  if (sock >= 0) ::close(sock);
}

// ── USB ──────────────────────────────────────────────────────────────────────
void fsim_usb_msc_begin(uint32_t block_count, uint16_t block_size) {
  std::lock_guard<std::mutex> lock(machine().mutex());
  auto& p = machine().peripherals();
  p.mscActive = true;
  p.mscBlockCount = block_count;
  p.mscBlockSize = block_size;
}

void fsim_usb_msc_end(void) {
  std::lock_guard<std::mutex> lock(machine().mutex());
  machine().peripherals().mscActive = false;
}

int fsim_usb_connected(void) {
  std::lock_guard<std::mutex> lock(machine().mutex());
  return machine().peripherals().usbConnected ? 1 : 0;
}

// ── External-bus panel hand-off ──────────────────────────────────────────────
void fsim_panel_push_frame(const uint8_t* plane, size_t len, int width, int height, int mode) {
  machine().panel().pushFrame(plane, len, width, height, mode);
}

}  // extern "C"
