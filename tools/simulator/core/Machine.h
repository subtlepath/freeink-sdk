#pragma once

// FreeInk simulator — the virtual machine.
//
// One Machine owns every piece of modelled hardware and every fsim_* entry
// point resolves into it. It is shared between three kinds of thread:
//
//   * the firmware thread(s), inside the dlopen'd bundle,
//   * the control thread, serving the Unix socket,
//   * the main thread, drawing the SDL window (macOS requires the window on
//     the main thread, which is why the socket server does not live there).
//
// So everything here is guarded. The coarse `mutex()` protects device state;
// the clock has its own lock because every blocking wait touches it.

#include <freeink_sim_abi.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace freeink::sim {

class Panel;
class I2cBus;
class VirtualCard;

// ── Clock ────────────────────────────────────────────────────────────────────
// Two modes, both of which stop dead when the machine is paused:
//
//   Realtime — simulated time tracks wall time (optionally scaled). What you
//     want when watching the window; a refresh takes as long as it does.
//   Virtual  — time advances only when every firmware thread is blocked in a
//     wait, and then jumps straight to the earliest deadline. A 30-second
//     sleep costs no wall time, and a run is reproducible, which is what makes
//     the simulator usable in a test loop.
//
// In realtime mode the rate is the familiar multiplier. In virtual mode it is
// ignored — virtual time runs as fast as the host can manage, which is what
// makes a 30-second sleep free — unless the clock is *paced*, and then it
// becomes a ceiling: simulated time may run at most `rate` times wall time.
//
// Pacing exists for one case, and it is not a preference. An emulated device
// image idles forward to its next timer thousands of times faster than wall
// time, so the firmware's own inactivity timeout expires while the operator is
// still reaching for the keyboard: the device goes to sleep before it can be
// used. Loading an image turns pacing on for that reason; a host-compiled
// bundle keeps its clock unpaced, and `rate 0` lifts the ceiling for a test
// that only wants to reach the end.
enum class ClockMode { Realtime, Virtual };

class Clock {
 public:
  uint64_t nowUs() const { return now_.load(std::memory_order_relaxed); }

  void setMode(ClockMode mode);
  ClockMode mode() const { return mode_; }
  // Realtime: the multiplier. Virtual: the ceiling while paced, 0 meaning none.
  void setRate(double rate);
  void setPaced(bool paced);
  bool paced() const { return paced_; }
  double rate() const { return rate_; }

  void pause();
  void resume();
  bool paused() const { return paused_.load(); }
  // Runs for `us` of simulated time, then pauses again. Returns when done.
  void step(uint64_t us);

  // Blocks the calling firmware thread until the simulated clock reaches
  // now+us. Returns nonzero if the machine is shutting down.
  int sleepUs(uint64_t us);
  // A firmware thread that is running rather than waiting. Virtual time cannot
  // advance while any of these are outstanding, which is what keeps a virtual
  // run deterministic. They nest — an ABI call made from firmware that is
  // already "running" adds a level — and a thread that goes to sleep releases
  // all of its levels at once, or virtual time could never reach zero holders.
  void enterRunning();
  void exitRunning();

  // Driven by the daemon's main loop. Returns whether simulated time actually
  // advanced, so a virtual-mode caller knows whether to keep going flat out or
  // back off — the difference between a 30-second sleep costing nothing and
  // costing thirty seconds.
  bool tick();
  void shutdown();

 private:
  void advanceTo(uint64_t target);

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::atomic<uint64_t> now_{0};
  std::atomic<bool> paused_{false};
  std::atomic<bool> shutdown_{false};
  ClockMode mode_ = ClockMode::Realtime;
  double rate_ = 1.0;
  uint64_t lastWallUs_ = 0;
  // Virtual-mode allowance: simulated microseconds earned by wall time passing
  // and spent by advancing. Accruing it rather than comparing totals is what
  // lets a long idle jump be paid for over several ticks instead of being
  // taken all at once.
  double virtualBudgetUs_ = 0.0;
  bool paced_ = false;
  uint64_t stepDeadline_ = 0;
  bool stepping_ = false;
  int running_ = 0;
  std::multimap<uint64_t, int> waiters_;  // deadline -> waiter id
  int nextWaiterId_ = 1;
};

// ── GPIO matrix ──────────────────────────────────────────────────────────────
// Models what a pin actually reads, which is the part that matters: an input
// with no driver reads its pull, a held pad ignores writes until released, and
// an external driver (a button pulling to ground, a device's open-drain IRQ)
// wins over the pull. Without the hold model, firmware that latches pins for
// deep sleep would behave differently here than on the device.
class Gpio {
 public:
  static constexpr int kPinCount = 64;

  void setMode(int pin, uint32_t mode);
  void write(int pin, int level);
  int read(int pin) const;
  void setHold(int pin, bool held);
  bool held(int pin) const;

  // External drive: a button, or a peripheral asserting an IRQ. kFloat releases
  // it back to the pull.
  static constexpr int kFloat = -1;
  void driveExternal(int pin, int level);
  int externalLevel(int pin) const;

  uint32_t mode(int pin) const;
  int outputLevel(int pin) const;

  void attachInterrupt(int pin, void (*isr)(void), int mode);
  void detachInterrupt(int pin);
  // Fires any ISR armed for the edge the pin just took. Called with the
  // machine lock released so the ISR can take it.
  void fireInterrupts(int pin, int from, int to);

 private:
  struct Pin {
    uint32_t mode = FSIM_PIN_INPUT;
    int output = 0;
    int external = kFloat;
    bool held = false;
    int heldLevel = 0;
    void (*isr)(void) = nullptr;
    int isrMode = 0;
  };
  Pin pins_[kPinCount];
};

// ── ADC ──────────────────────────────────────────────────────────────────────
// Sources a millivolt value per pin. Button ladders are computed from which
// buttons are down, so the real InputManager decode runs against real-looking
// counts rather than a synthesized button mask.
class Adc {
 public:
  void setMillivolts(int pin, int mv);
  void clearOverride(int pin);
  int readMillivolts(int pin) const;
  int readRaw(int pin) const;

  // The ladder model, fed by the button state.
  void setLadderValue(int pin, int mv) { setMillivolts(pin, mv); }

 private:
  std::map<int, int> millivolts_;
};

// ── Log ring ─────────────────────────────────────────────────────────────────
class LogRing {
 public:
  void append(const std::string& channel, const char* text, size_t len);
  // Returns lines with sequence numbers after `since`; the CLI tails by passing
  // back the last sequence it saw.
  std::vector<std::pair<uint64_t, std::string>> since(uint64_t seq, const std::string& channel = "") const;
  uint64_t lastSeq() const;
  void clear();

 private:
  struct Line {
    uint64_t seq;
    uint64_t timeUs;
    std::string channel;
    std::string text;
  };
  mutable std::mutex mutex_;
  std::deque<Line> lines_;
  std::string partial_;      // Serial output arrives unterminated
  std::string partialChan_;
  uint64_t nextSeq_ = 1;
  static constexpr size_t kMaxLines = 20000;
};

// ── Storage ──────────────────────────────────────────────────────────────────
class Storage {
 public:
  // NVS, persisted as a file so settings survive a daemon restart.
  bool nvsGet(const std::string& ns, const std::string& key, std::string* out) const;
  void nvsSet(const std::string& ns, const std::string& key, const std::string& value);
  bool nvsErase(const std::string& ns, const std::string& key);
  void nvsClear(const std::string& ns);  // empty ns clears everything
  std::map<std::string, std::string> nvsAll() const;
  void nvsLoad(const std::string& path);
  void nvsSave() const;

  // Virtual SD card, backed by a host directory.
  void mountCard(const std::string& hostDir, uint64_t capacityBytes);
  void ejectCard();
  bool cardPresent() const;
  uint64_t cardCapacity() const;
  std::string cardRoot() const;
  // Resolves a firmware path inside the mount, refusing traversal outside it.
  bool resolve(const std::string& fwPath, std::string* out) const;

  void setStateDir(const std::string& dir) { stateDir_ = dir; }
  const std::string& stateDir() const { return stateDir_; }

 private:
  mutable std::mutex mutex_;
  std::map<std::string, std::string> nvs_;  // "ns\0key" -> value
  std::string nvsPath_;
  std::string cardRoot_;
  uint64_t cardCapacity_ = 0;
  std::string stateDir_;
};

// ── Peripheral state the CLI reports and drives ──────────────────────────────
struct Peripherals {
  // PWM sinks, by pin: frontlight, buzzer, LEDs.
  std::map<int, uint32_t> pwmDuty;
  std::map<int, uint32_t> pwmFreq;
  std::map<int, uint8_t> pwmBits;
  std::map<int, int> pwmChannelPin;

  int batteryPercent = 76;
  int batteryMillivolts = 3900;
  bool charging = false;
  bool usbConnected = false;

  double imuAccel[3] = {0.0, 0.0, 1.0};
  double imuGyro[3] = {0.0, 0.0, 0.0};
  double temperatureC = 21.5;
  double humidityPct = 45.0;

  int wifiState = FSIM_WIFI_IDLE;
  uint64_t wifiJoinCompleteUs = 0;
  std::string wifiSsid;
  uint32_t wifiLocalIp = 0;
  bool networkEnabled = false;  // host TCP is off unless the operator enables it
  struct Network {
    std::string ssid;
    int32_t rssi;
    uint8_t enc;
    std::string password;  // empty accepts any
  };
  std::vector<Network> networks;

  // Audio capture, so a firmware's tone or clip can be listened to.
  std::vector<uint8_t> audioCapture;
  uint32_t audioSampleRate = 0;
  uint8_t audioBits = 16;
  uint8_t audioChannels = 2;
  bool audioOpen = false;
  std::vector<uint8_t> micQueue;
  size_t micPos = 0;

  bool mscActive = false;
  uint32_t mscBlockCount = 0;
  uint16_t mscBlockSize = 512;
};

// ── Power ────────────────────────────────────────────────────────────────────
enum class PowerState { Running, DeepSleep, LightSleep, Off };

// ── The machine ──────────────────────────────────────────────────────────────
class Machine {
 public:
  static Machine& instance();

  std::mutex& mutex() { return mutex_; }
  Clock& clock() { return clock_; }
  Gpio& gpio() { return gpio_; }
  Adc& adc() { return adc_; }
  LogRing& log() { return log_; }
  Storage& storage() { return storage_; }
  Peripherals& peripherals() { return peripherals_; }
  Panel& panel();
  I2cBus& i2c();
  // The card as blocks. A bundle reaches its files through Storage and never
  // touches this; an emulated image talks to a card controller and needs a
  // card with a real filesystem on it. `mountCard` sets both up from one
  // directory so the two kinds of firmware see the same content.
  VirtualCard& card();
  bool mountCard(const std::string& hostDir, uint64_t capacityBytes, std::string* error);
  bool mountCardImage(const std::string& imagePath, std::string* error);
  void ejectCard();

  const fsim_board_desc& board() const { return board_; }
  bool boardKnown() const { return boardKnown_; }
  void describeBoard(const fsim_board_desc& desc);
  // Fired whenever the bundle reports a (possibly new) profile, so the window
  // can resize when firmware switches between the X3 and X4 geometries.
  void setBoardChangedHook(std::function<void()> hook) { boardChanged_ = std::move(hook); }

  // ── Input ──────────────────────────────────────────────────────────────────
  // Button indices match the SDK's: BACK, CONFIRM, LEFT, RIGHT, UP, DOWN, POWER.
  static constexpr int kButtonCount = 7;
  void setButton(int index, bool pressed);
  bool button(int index) const { return buttons_[index]; }
  static const char* buttonName(int index);
  static int buttonIndexFromName(const std::string& name);

  // Drives the panel's BUSY pin from the panel model, with the controller
  // family's polarity, and fires any ISR armed on the resulting edge. The panel
  // cannot simply have gpio_read() report its state: EpdBus sleeps on a BUSY
  // edge interrupt, so BUSY has to be a pin that actually transitions.
  void setPanelBusy(bool busy);

  // How many times the firmware has sampled the input source behind a button
  // (its GPIO, or the ADC pin carrying its ladder). A scripted press holds
  // until this advances, so it is not silently swallowed by a blocking refresh
  // the way a 60 ms tap would be on hardware. The physics are unchanged — the
  // press is simply held until the device notices, as a person would.
  uint64_t inputSampleCount(int buttonIndex) const;
  void noteGpioRead(int pin);
  void noteAdcRead(int pin);

  void setTouch(int x, int y, bool down);
  void clearTouch();
  struct TouchPoint {
    int x = 0;
    int y = 0;
    bool down = false;
  };
  TouchPoint touch() const { return touch_; }

  // ── Power ──────────────────────────────────────────────────────────────────
  PowerState powerState() const { return powerState_; }
  void requestRestart();
  void enterDeepSleep(uint64_t sleepUs);
  void enterLightSleep(uint64_t sleepUs);
  void setSleepWakePin(int pin, int level);
  int wakeCause() const { return wakeCause_; }
  // Set when a deep sleep should end: the timer expired or a wake pin reached
  // its level. Wakes the firmware thread by tearing the bundle down.
  bool shouldWake() const;
  void wakeNow(int cause);

  // Asks the firmware thread to unwind so the daemon can reload the bundle.
  void requestTeardown() { teardown_ = true; }
  bool tearingDown() const { return teardown_.load(); }
  void clearTeardown() { teardown_ = false; }

  uint32_t nextRandom();
  void seedRandom(uint32_t seed);

  // Memory accounting reported by the bundle's heap shim.
  void setHeapUsage(size_t psram, size_t internal) {
    heapPsram_ = psram;
    heapInternal_ = internal;
  }

 private:
  Machine();

  // Recomputes the ADC ladder voltages and digital button pin levels from the
  // current button state, so the real InputManager decode path sees what the
  // hardware would present.
  void applyButtonsToPins();

  mutable std::mutex mutex_;
  Clock clock_;
  Gpio gpio_;
  Adc adc_;
  LogRing log_;
  Storage storage_;
  Peripherals peripherals_;
  std::unique_ptr<Panel> panel_;
  std::unique_ptr<I2cBus> i2c_;
  std::unique_ptr<VirtualCard> card_;

  fsim_board_desc board_{};
  bool boardKnown_ = false;
  std::function<void()> boardChanged_;

  bool buttons_[kButtonCount] = {};
  TouchPoint touch_;
  std::map<int, uint64_t> gpioReads_;
  std::map<int, uint64_t> adcReads_;

  PowerState powerState_ = PowerState::Running;
  uint64_t sleepUntilUs_ = 0;
  int wakePin_ = -1;
  int wakeLevel_ = 0;
  int wakeCause_ = 0;
  std::atomic<bool> teardown_{false};

  uint32_t randomState_ = 0x12345678u;
  std::atomic<size_t> heapPsram_{0};
  std::atomic<size_t> heapInternal_{0};
};

}  // namespace freeink::sim
