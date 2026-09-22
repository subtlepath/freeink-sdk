#pragma once

// FreeInk simulator — Arduino core shim.
//
// Every entry point here forwards to the simulator daemon through the fsim_*
// ABI, so SDK code that calls digitalWrite()/analogRead()/millis() runs
// unmodified against the virtual machine. Nothing in this header knows about
// any particular board: the pin numbers the SDK passes are the same ones the
// daemon's GPIO matrix models.

#include <freeink_sim_abi.h>

#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "WString.h"

// ── Pin modes and levels ─────────────────────────────────────────────────────
#define INPUT FSIM_PIN_INPUT
#define OUTPUT FSIM_PIN_OUTPUT
#define INPUT_PULLUP (FSIM_PIN_INPUT | FSIM_PIN_PULLUP)
#define INPUT_PULLDOWN (FSIM_PIN_INPUT | FSIM_PIN_PULLDOWN)
#define OUTPUT_OPEN_DRAIN (FSIM_PIN_OUTPUT | FSIM_PIN_OPEN_DRAIN)

#define LOW 0
#define HIGH 1

#define RISING 1
#define FALLING 2
#define CHANGE 3
#define ONLOW 4
#define ONHIGH 5

#define MSBFIRST 1
#define LSBFIRST 0

// ── Attributes the SDK uses for placement; meaningless on the host ───────────
#define IRAM_ATTR
#define DRAM_ATTR
#define PROGMEM
#define PGM_P const char*
#define pgm_read_byte(p) (*reinterpret_cast<const uint8_t*>(p))
#define pgm_read_word(p) (*reinterpret_cast<const uint16_t*>(p))
#define pgm_read_dword(p) (*reinterpret_cast<const uint32_t*>(p))
#define pgm_read_ptr(p) (*reinterpret_cast<void* const*>(p))
#define memcpy_P memcpy
#define strncpy_P strncpy
#define strlen_P strlen
#define snprintf_P snprintf
#define F(s) (s)
#define PSTR(s) (s)

// ── ADC attenuation (accepted and recorded; the model works in millivolts) ───
typedef enum { ADC_0db = 0, ADC_2_5db = 1, ADC_6db = 2, ADC_11db = 3, ADC_ATTENDB_MAX } adc_attenuation_t;

// ── Time ─────────────────────────────────────────────────────────────────────
inline unsigned long millis() { return static_cast<unsigned long>(fsim_micros() / 1000ULL); }
inline unsigned long micros() { return static_cast<unsigned long>(fsim_micros()); }
inline void delay(unsigned long ms) { fsim_delay_us(static_cast<uint64_t>(ms) * 1000ULL); }
inline void delayMicroseconds(unsigned int us) { fsim_delay_us(us); }
inline void yield() { fsim_yield(); }

// ── GPIO ─────────────────────────────────────────────────────────────────────
inline void pinMode(int pin, uint32_t mode) { fsim_pin_mode(pin, mode); }
inline void digitalWrite(int pin, int level) { fsim_gpio_write(pin, level); }
inline int digitalRead(int pin) { return fsim_gpio_read(pin); }
inline void attachInterrupt(int pin, void (*isr)(void), int mode) { fsim_gpio_attach_interrupt(pin, isr, mode); }
inline void detachInterrupt(int pin) { fsim_gpio_detach_interrupt(pin); }
inline int digitalPinToInterrupt(int pin) { return pin; }

// ── ADC / DAC ────────────────────────────────────────────────────────────────
inline int analogRead(int pin) { return fsim_adc_read(pin); }
inline uint32_t analogReadMilliVolts(int pin) { return static_cast<uint32_t>(fsim_adc_read_mv(pin)); }
inline void analogSetAttenuation(adc_attenuation_t atten) { fsim_adc_set_attenuation(-1, atten); }
inline void analogSetPinAttenuation(int pin, adc_attenuation_t atten) { fsim_adc_set_attenuation(pin, atten); }
inline void analogReadResolution(int) {}
inline void dacWrite(int, uint8_t) {}

// ── LEDC PWM ─────────────────────────────────────────────────────────────────
inline void ledcSetup(int channel, uint32_t freq, uint8_t bits) { fsim_ledc_setup(channel, freq, bits); }
inline void ledcAttachPin(int pin, int channel) { fsim_ledc_attach(pin, channel); }
inline bool ledcAttach(int pin, uint32_t freq, uint8_t bits) {
  fsim_ledc_setup(pin, freq, bits);
  fsim_ledc_attach(pin, pin);
  return true;
}
inline bool ledcAttachChannel(int pin, uint32_t freq, uint8_t bits, int channel) {
  fsim_ledc_setup(channel, freq, bits);
  fsim_ledc_attach(pin, channel);
  return true;
}
inline void ledcWrite(int pin_or_channel, uint32_t duty) { fsim_ledc_write(pin_or_channel, duty, 1); }
inline void ledcWriteChannel(int channel, uint32_t duty) { fsim_ledc_write(channel, duty, 0); }
inline bool ledcDetach(int pin) {
  fsim_ledc_attach(pin, -1);
  return true;
}
inline void ledcDetachPin(int pin) { ledcDetach(pin); }

// ── Math helpers ─────────────────────────────────────────────────────────────
// Templates, not macros. Arduino's classic function-like min/max macros break
// every standard header that spells `std::numeric_limits<T>::max()` — the macro
// expands there with zero arguments — so the ESP32 core defines these as
// templates in C++, and so do we.
template <typename T, typename U>
constexpr auto min(const T& a, const U& b) -> decltype(a < b ? a : b) {
  return a < b ? a : b;
}
template <typename T, typename U>
constexpr auto max(const T& a, const U& b) -> decltype(a > b ? a : b) {
  return a > b ? a : b;
}
template <typename T, typename L, typename H>
constexpr T constrain(const T& x, const L& lo, const H& hi) {
  return x < static_cast<T>(lo) ? static_cast<T>(lo) : (x > static_cast<T>(hi) ? static_cast<T>(hi) : x);
}
template <typename T>
constexpr T sq(const T& x) {
  return x * x;
}
using std::abs;

#define bitRead(v, b) (((v) >> (b)) & 0x01)
#define bitSet(v, b) ((v) |= (1UL << (b)))
#define bitClear(v, b) ((v) &= ~(1UL << (b)))
#define bitWrite(v, b, x) ((x) ? bitSet(v, b) : bitClear(v, b))
#define lowByte(w) (static_cast<uint8_t>((w) & 0xFF))
#define highByte(w) (static_cast<uint8_t>(((w) >> 8) & 0xFF))

inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ── Randomness ───────────────────────────────────────────────────────────────
// Seeded and stepped by the daemon so a run is reproducible: the CLI can pin
// the seed, which matters when a test asserts on anything the firmware
// randomizes.
inline long random(long howbig) { return howbig <= 0 ? 0 : static_cast<long>(fsim_random() % static_cast<uint32_t>(howbig)); }
inline long random(long howsmall, long howbig) {
  return howbig <= howsmall ? howsmall : howsmall + random(howbig - howsmall);
}
inline void randomSeed(unsigned long) {}
inline uint32_t esp_random() { return fsim_random(); }

// ── Serial ───────────────────────────────────────────────────────────────────
// Everything written lands in the daemon's log ring, which `freeink-sim log`
// tails and `freeink-sim expect` matches against.
class SimSerial {
 public:
  void begin(unsigned long = 115200) {}
  void end() {}
  void setDebugOutput(bool) {}
  void flush() {}
  operator bool() const { return true; }
  int available() { return 0; }
  int read() { return -1; }

  size_t write(uint8_t c) { return write(&c, 1); }
  size_t write(const uint8_t* data, size_t len) {
    fsim_log(reinterpret_cast<const char*>(data), len);
    return len;
  }
  size_t write(const char* s) { return write(reinterpret_cast<const uint8_t*>(s), strlen(s)); }

  size_t print(const char* s) { return s ? write(s) : 0; }
  size_t print(const String& s) { return write(s.c_str()); }
  size_t print(char c) { return write(static_cast<uint8_t>(c)); }
  size_t print(int v) { return printf("%d", v); }
  size_t print(unsigned v) { return printf("%u", v); }
  size_t print(long v) { return printf("%ld", v); }
  size_t print(unsigned long v) { return printf("%lu", v); }
  size_t print(double v) { return printf("%f", v); }

  size_t println() { return write("\n"); }
  size_t println(const char* s) { return printf("%s\n", s ? s : ""); }
  size_t println(const String& s) { return printf("%s\n", s.c_str()); }
  size_t println(char c) { return printf("%c\n", c); }
  size_t println(int v) { return printf("%d\n", v); }
  size_t println(unsigned v) { return printf("%u\n", v); }
  size_t println(long v) { return printf("%ld\n", v); }
  size_t println(unsigned long v) { return printf("%lu\n", v); }
  size_t println(double v) { return printf("%f\n", v); }

  size_t printf(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    char stack[512];
    va_list ap;
    va_start(ap, fmt);
    const int need = vsnprintf(stack, sizeof(stack), fmt, ap);
    va_end(ap);
    if (need < 0) return 0;
    if (static_cast<size_t>(need) < sizeof(stack)) {
      fsim_log(stack, static_cast<size_t>(need));
      return static_cast<size_t>(need);
    }
    // Long line: format again into a right-sized buffer rather than truncating
    // firmware output, which a test may be matching on.
    char* heap = static_cast<char*>(malloc(static_cast<size_t>(need) + 1));
    if (!heap) return 0;
    va_start(ap, fmt);
    vsnprintf(heap, static_cast<size_t>(need) + 1, fmt, ap);
    va_end(ap);
    fsim_log(heap, static_cast<size_t>(need));
    free(heap);
    return static_cast<size_t>(need);
  }
};

extern SimSerial Serial;
extern SimSerial Serial0;
extern SimSerial USBSerial;

// ── Stream/Print bases some libraries expect ─────────────────────────────────
class Print {
 public:
  virtual ~Print() = default;
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t* buf, size_t size) {
    size_t n = 0;
    while (n < size) n += write(buf[n]);
    return n;
  }
};

class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
};
