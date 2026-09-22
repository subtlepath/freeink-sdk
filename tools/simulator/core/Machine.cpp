// FreeInk simulator — virtual machine implementation.

#include "Machine.h"

#include "I2cDevices.h"
#include "VirtualCard.h"
#include "Panel.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <fstream>
#include <sstream>
#include <thread>

#include <sys/stat.h>

namespace freeink::sim {
namespace {

uint64_t wallMicros() {
  using namespace std::chrono;
  return static_cast<uint64_t>(duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

// The X3/X4 resistor ladder, in raw 12-bit counts. These are the averaged
// per-button values recorded from real devices that InputManager's ADC_RANGES_*
// tables were derived from, so the real decode path runs against realistic
// counts instead of a synthesized button mask — a ladder threshold regression
// in the SDK would show up here.
constexpr int kLadderNoButton = 4095;
constexpr int kLadderGroup1[4] = {3512, 2694, 1493, 5};  // BACK, CONFIRM, LEFT, RIGHT
constexpr int kLadderGroup2[2] = {2242, 5};              // UP, DOWN

// OnePage's single-ladder millivolt targets, from InputManager::getState().
constexpr int kOnePageLadderMv[4] = {2592, 0, 1956, 1316};  // BACK, CONFIRM(ENTER), LEFT, RIGHT

}  // namespace

// ── Clock ────────────────────────────────────────────────────────────────────
void Clock::setMode(ClockMode mode) {
  std::lock_guard<std::mutex> lock(mutex_);
  mode_ = mode;
  lastWallUs_ = wallMicros();
}

void Clock::setRate(double rate) {
  std::lock_guard<std::mutex> lock(mutex_);
  // Zero is "as fast as the host can manage". In realtime mode that is
  // meaningless, so the rate stays 1; in virtual mode it lifts the ceiling.
  rate_ = rate < 0.0 ? 1.0 : rate;
  if (mode_ == ClockMode::Realtime && rate_ == 0.0) rate_ = 1.0;
  virtualBudgetUs_ = 0.0;
}

void Clock::setPaced(bool paced) {
  std::lock_guard<std::mutex> lock(mutex_);
  paced_ = paced;
  virtualBudgetUs_ = 0.0;
}

void Clock::pause() {
  paused_ = true;
  cv_.notify_all();
}

void Clock::resume() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    lastWallUs_ = wallMicros();
    stepping_ = false;
  }
  paused_ = false;
  cv_.notify_all();
}

void Clock::step(uint64_t us) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stepDeadline_ = now_.load() + us;
    stepping_ = true;
    lastWallUs_ = wallMicros();
  }
  paused_ = false;
  cv_.notify_all();
  // Return once the budget is spent. Polled rather than signalled because the
  // advance happens on the clock thread.
  while (!shutdown_.load()) {
    if (now_.load() >= stepDeadline_) break;
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  paused_ = true;
}

namespace {
// How many nested "running" levels the calling thread holds. Needed because a
// sleep can happen several frames deep inside firmware (an SPI transfer charges
// its own wire time, for instance), and the sleeping thread has to release
// every one of its levels — otherwise virtual time waits on a holder that is
// itself asleep, and the machine stalls.
thread_local int t_runningDepth = 0;
}  // namespace

void Clock::enterRunning() {
  std::lock_guard<std::mutex> lock(mutex_);
  ++running_;
  ++t_runningDepth;
}

void Clock::exitRunning() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_ > 0) --running_;
    if (t_runningDepth > 0) --t_runningDepth;
  }
  cv_.notify_all();
}

int Clock::sleepUs(uint64_t us) {
  const uint64_t target = now_.load() + us;
  std::unique_lock<std::mutex> lock(mutex_);
  const int id = nextWaiterId_++;
  auto it = waiters_.emplace(target, id);

  // Release this thread's whole running depth while it waits; that is what lets
  // virtual time see zero runners and jump to the earliest deadline.
  const int released = t_runningDepth;
  running_ -= released;
  if (running_ < 0) running_ = 0;
  cv_.notify_all();

  while (!shutdown_.load() && now_.load() < target) {
    cv_.wait_for(lock, std::chrono::milliseconds(5));
  }

  waiters_.erase(it);
  running_ += released;
  return shutdown_.load() ? 1 : 0;
}

void Clock::advanceTo(uint64_t target) {
  if (target <= now_.load()) return;
  now_.store(target, std::memory_order_relaxed);
  cv_.notify_all();
}

bool Clock::tick() {
  std::unique_lock<std::mutex> lock(mutex_);
  const uint64_t wall = wallMicros();
  const uint64_t wallDelta = wall > lastWallUs_ ? wall - lastWallUs_ : 0;
  lastWallUs_ = wall;

  if (paused_.load() && !stepping_) return false;

  const uint64_t before = now_.load();
  uint64_t target = before;
  if (mode_ == ClockMode::Realtime) {
    target += static_cast<uint64_t>(static_cast<double>(wallDelta) * rate_);
  } else {
    // Virtual time: only advance when no firmware thread is running, and then
    // jump straight to the earliest pending deadline. A 30-second sleep costs
    // no wall time and the run is reproducible.
    if (running_ > 0) return false;
    if (waiters_.empty()) {
      // Nothing waiting and nothing running: the firmware is between slices.
      // Nudge time forward so a polling loop still makes progress.
      target += 1000;
    } else {
      target = std::max(target, waiters_.begin()->first);
    }
    // Spend the allowance the wall has earned. Without this an emulated
    // machine runs its own clock thousands of times faster than the person
    // watching it, and every timeout the firmware has expires at once.
    if (paced_ && rate_ > 0.0) {
      virtualBudgetUs_ += static_cast<double>(wallDelta) * rate_;
      // A budget that accrues while nothing is advancing would be spent in one
      // jump the moment something did; a second of it is plenty of slack.
      virtualBudgetUs_ = std::min(virtualBudgetUs_, 1000000.0);
      const double wanted = static_cast<double>(target - before);
      if (wanted > virtualBudgetUs_) target = before + static_cast<uint64_t>(virtualBudgetUs_);
      virtualBudgetUs_ -= static_cast<double>(target - before);
    }
  }

  if (stepping_ && target > stepDeadline_) {
    target = stepDeadline_;
    stepping_ = false;
  }
  lock.unlock();
  advanceTo(target);
  return now_.load() > before;
}

void Clock::shutdown() {
  shutdown_ = true;
  cv_.notify_all();
}

// ── GPIO ─────────────────────────────────────────────────────────────────────
void Gpio::setMode(int pin, uint32_t mode) {
  if (pin < 0 || pin >= kPinCount) return;
  pins_[pin].mode = mode;
}

void Gpio::write(int pin, int level) {
  if (pin < 0 || pin >= kPinCount) return;
  // A pad held for deep sleep latches its level: writes bounce off until
  // gpio_hold_dis(). EpdBus and PowerManager both depend on this.
  if (pins_[pin].held) return;
  pins_[pin].output = level ? 1 : 0;
}

int Gpio::read(int pin) const {
  if (pin < 0 || pin >= kPinCount) return 0;
  const Pin& p = pins_[pin];
  if (p.held) return p.heldLevel;
  // An external driver (a button to ground, a device's open-drain IRQ) wins
  // over the internal pull; an output reads back what it drives.
  if (p.external != kFloat) return p.external;
  if (p.mode & FSIM_PIN_OUTPUT) return p.output;
  if (p.mode & FSIM_PIN_PULLUP) return 1;
  if (p.mode & FSIM_PIN_PULLDOWN) return 0;
  return 0;
}

void Gpio::setHold(int pin, bool held) {
  if (pin < 0 || pin >= kPinCount) return;
  if (held && !pins_[pin].held) pins_[pin].heldLevel = read(pin);
  pins_[pin].held = held;
}

bool Gpio::held(int pin) const { return pin >= 0 && pin < kPinCount && pins_[pin].held; }

void Gpio::driveExternal(int pin, int level) {
  if (pin < 0 || pin >= kPinCount) return;
  pins_[pin].external = level;
}

int Gpio::externalLevel(int pin) const {
  return (pin >= 0 && pin < kPinCount) ? pins_[pin].external : kFloat;
}

uint32_t Gpio::mode(int pin) const { return (pin >= 0 && pin < kPinCount) ? pins_[pin].mode : 0; }
int Gpio::outputLevel(int pin) const { return (pin >= 0 && pin < kPinCount) ? pins_[pin].output : 0; }

void Gpio::attachInterrupt(int pin, void (*isr)(void), int mode) {
  if (pin < 0 || pin >= kPinCount) return;
  pins_[pin].isr = isr;
  pins_[pin].isrMode = mode;
}

void Gpio::detachInterrupt(int pin) {
  if (pin < 0 || pin >= kPinCount) return;
  pins_[pin].isr = nullptr;
}

void Gpio::fireInterrupts(int pin, int from, int to) {
  if (pin < 0 || pin >= kPinCount || from == to) return;
  const Pin& p = pins_[pin];
  if (!p.isr) return;
  const bool rising = from == 0 && to == 1;
  const bool matches = p.isrMode == 3 /* CHANGE */ || (rising && p.isrMode == 1) || (!rising && p.isrMode == 2);
  if (matches) p.isr();
}

// ── ADC ──────────────────────────────────────────────────────────────────────
void Adc::setMillivolts(int pin, int mv) { millivolts_[pin] = mv; }
void Adc::clearOverride(int pin) { millivolts_.erase(pin); }

int Adc::readMillivolts(int pin) const {
  auto it = millivolts_.find(pin);
  return it == millivolts_.end() ? 0 : it->second;
}

int Adc::readRaw(int pin) const {
  // 12-bit at the 11 dB attenuation the SDK configures: 0..3300 mV over 0..4095.
  const int mv = readMillivolts(pin);
  const int raw = mv * 4095 / 3300;
  return std::clamp(raw, 0, 4095);
}

// ── Log ring ─────────────────────────────────────────────────────────────────
void LogRing::append(const std::string& channel, const char* text, size_t len) {
  std::lock_guard<std::mutex> lock(mutex_);
  // Serial output arrives in fragments with no line structure; buffer until a
  // newline so the CLI can match whole lines.
  if (channel != partialChan_ && !partial_.empty()) {
    lines_.push_back({nextSeq_++, 0, partialChan_, partial_});
    partial_.clear();
  }
  partialChan_ = channel;
  partial_.append(text, len);
  size_t nl;
  while ((nl = partial_.find('\n')) != std::string::npos) {
    std::string line = partial_.substr(0, nl);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    lines_.push_back({nextSeq_++, 0, channel, std::move(line)});
    partial_.erase(0, nl + 1);
    while (lines_.size() > kMaxLines) lines_.pop_front();
  }
}

std::vector<std::pair<uint64_t, std::string>> LogRing::since(uint64_t seq, const std::string& channel) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::pair<uint64_t, std::string>> out;
  for (const auto& line : lines_) {
    if (line.seq <= seq) continue;
    if (!channel.empty() && line.channel != channel) continue;
    out.emplace_back(line.seq, line.text);
  }
  return out;
}

uint64_t LogRing::lastSeq() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return nextSeq_ - 1;
}

void LogRing::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  lines_.clear();
  partial_.clear();
}

// ── Storage ──────────────────────────────────────────────────────────────────
namespace {
std::string nvsKeyFor(const std::string& ns, const std::string& key) { return ns + "\x1f" + key; }
}  // namespace

bool Storage::nvsGet(const std::string& ns, const std::string& key, std::string* out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = nvs_.find(nvsKeyFor(ns, key));
  if (it == nvs_.end()) return false;
  if (out) *out = it->second;
  return true;
}

void Storage::nvsSet(const std::string& ns, const std::string& key, const std::string& value) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    nvs_[nvsKeyFor(ns, key)] = value;
  }
  nvsSave();
}

bool Storage::nvsErase(const std::string& ns, const std::string& key) {
  bool erased;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    erased = nvs_.erase(nvsKeyFor(ns, key)) > 0;
  }
  if (erased) nvsSave();
  return erased;
}

void Storage::nvsClear(const std::string& ns) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (ns.empty()) {
      nvs_.clear();
    } else {
      const std::string prefix = ns + "\x1f";
      for (auto it = nvs_.begin(); it != nvs_.end();) {
        it = it->first.rfind(prefix, 0) == 0 ? nvs_.erase(it) : std::next(it);
      }
    }
  }
  nvsSave();
}

std::map<std::string, std::string> Storage::nvsAll() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return nvs_;
}

void Storage::nvsLoad(const std::string& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  nvsPath_ = path;
  nvs_.clear();
  std::ifstream in(path, std::ios::binary);
  if (!in) return;
  // One record per line: key, then the value hex-encoded (values are blobs).
  std::string line;
  while (std::getline(in, line)) {
    const size_t tab = line.find('\t');
    if (tab == std::string::npos) continue;
    std::string key = line.substr(0, tab);
    std::string hex = line.substr(tab + 1);
    std::string value;
    for (size_t i = 0; i + 1 < hex.size(); i += 2) {
      value.push_back(static_cast<char>(std::stoi(hex.substr(i, 2), nullptr, 16)));
    }
    // The separator is stored as a literal \x1f in the key field.
    nvs_[key] = value;
  }
}

void Storage::nvsSave() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (nvsPath_.empty()) return;
  std::ofstream out(nvsPath_, std::ios::binary | std::ios::trunc);
  if (!out) return;
  static const char* kHex = "0123456789abcdef";
  for (const auto& [key, value] : nvs_) {
    out << key << '\t';
    for (unsigned char c : value) {
      out << kHex[c >> 4] << kHex[c & 0x0F];
    }
    out << '\n';
  }
}

VirtualCard& Machine::card() { return *card_; }

bool Machine::mountCard(const std::string& hostDir, uint64_t capacityBytes, std::string* error) {
  // The block image is built first: if the directory cannot be turned into a
  // filesystem the card is not inserted at all, rather than appearing to a
  // bundle and not to an image.
  if (!card_->mountDirectory(hostDir, capacityBytes, error)) return false;
  storage_.mountCard(hostDir, card_->capacityBytes());
  return true;
}

bool Machine::mountCardImage(const std::string& imagePath, std::string* error) {
  if (!card_->mountImage(imagePath, error)) return false;
  // A raw image has no host directory behind it, so a bundle's file I/O has
  // nothing to resolve into; only an emulated image can read this card.
  storage_.ejectCard();
  return true;
}

void Machine::ejectCard() {
  card_->unmount();
  storage_.ejectCard();
}

void Storage::mountCard(const std::string& hostDir, uint64_t capacityBytes) {
  std::lock_guard<std::mutex> lock(mutex_);
  cardRoot_ = hostDir;
  cardCapacity_ = capacityBytes;
  while (cardRoot_.size() > 1 && cardRoot_.back() == '/') cardRoot_.pop_back();
}

void Storage::ejectCard() {
  std::lock_guard<std::mutex> lock(mutex_);
  cardRoot_.clear();
}

bool Storage::cardPresent() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return !cardRoot_.empty();
}

uint64_t Storage::cardCapacity() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cardCapacity_;
}

std::string Storage::cardRoot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cardRoot_;
}

bool Storage::resolve(const std::string& fwPath, std::string* out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  // Reserved escape: "\x01flash" addresses the virtual flash image, which lives
  // in the daemon's state directory rather than on the card.
  if (!fwPath.empty() && fwPath[0] == '\x01') {
    if (stateDir_.empty()) return false;
    if (out) *out = stateDir_ + "/" + fwPath.substr(1) + ".bin";
    return true;
  }
  if (cardRoot_.empty()) return false;

  // Normalize the firmware path, resolving "." and ".." inside the mount. A
  // path that walks above the root is refused rather than clamped, so a
  // traversal bug in firmware is visible as a failed open.
  std::vector<std::string> parts;
  std::string component;
  std::istringstream stream(fwPath);
  while (std::getline(stream, component, '/')) {
    if (component.empty() || component == ".") continue;
    if (component == "..") {
      if (parts.empty()) return false;
      parts.pop_back();
      continue;
    }
    parts.push_back(component);
  }

  std::string resolved = cardRoot_;
  for (const auto& part : parts) resolved += "/" + part;
  if (out) *out = resolved;
  return true;
}

// ── Machine ──────────────────────────────────────────────────────────────────
Machine& Machine::instance() {
  static Machine machine;
  return machine;
}

Machine::Machine() {
  panel_ = std::make_unique<Panel>(*this);
  i2c_ = std::make_unique<I2cBus>(*this);
  card_ = std::make_unique<VirtualCard>();
}

Panel& Machine::panel() { return *panel_; }
I2cBus& Machine::i2c() { return *i2c_; }

void Machine::describeBoard(const fsim_board_desc& desc) {
  bool changed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    changed = !boardKnown_ || board_.panel_width != desc.panel_width || board_.panel_height != desc.panel_height ||
              board_.panel_controller != desc.panel_controller;
    board_ = desc;
    boardKnown_ = true;
  }
  panel_->configure(desc.panel_width, desc.panel_height, desc.panel_controller, desc.epd_mirror_x != 0,
                    desc.epd_mirror_y != 0, desc.epd_gates_reversed != 0);
  i2c_->configure(desc);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    applyButtonsToPins();
  }
  if (changed && boardChanged_) boardChanged_();
}

const char* Machine::buttonName(int index) {
  static const char* kNames[kButtonCount] = {"back", "confirm", "left", "right", "up", "down", "power"};
  return (index >= 0 && index < kButtonCount) ? kNames[index] : "?";
}

int Machine::buttonIndexFromName(const std::string& name) {
  for (int i = 0; i < kButtonCount; ++i) {
    if (name == buttonName(i)) return i;
  }
  return -1;
}

void Machine::setButton(int index, bool pressed) {
  if (index < 0 || index >= kButtonCount) return;
  std::lock_guard<std::mutex> lock(mutex_);
  buttons_[index] = pressed;
  applyButtonsToPins();
}

void Machine::applyButtonsToPins() {
  if (!boardKnown_) return;

  if (board_.input_style == FSIM_INPUT_ADC_LADDER) {
    // Two ladders: GPIO1 carries BACK/CONFIRM/LEFT/RIGHT, GPIO2 carries
    // UP/DOWN. The lowest-numbered pressed button on a ladder wins, as it
    // would electrically.
    int group1 = kLadderNoButton;
    for (int i = 0; i < 4; ++i) {
      if (buttons_[i]) {
        group1 = kLadderGroup1[i];
        break;
      }
    }
    int group2 = kLadderNoButton;
    for (int i = 0; i < 2; ++i) {
      if (buttons_[4 + i]) {
        group2 = kLadderGroup2[i];
        break;
      }
    }
    adc_.setMillivolts(1, group1 * 3300 / 4095);
    adc_.setMillivolts(2, group2 * 3300 / 4095);
  } else if (board_.input_style == FSIM_INPUT_ONEPAGE_LADDER) {
    int mv = 3300;
    static const int kOrder[4] = {0, 1, 2, 3};  // BACK, CONFIRM, LEFT, RIGHT
    for (int i = 0; i < 4; ++i) {
      if (buttons_[kOrder[i]]) {
        mv = kOnePageLadderMv[i];
        break;
      }
    }
    if (board_.adc_ladder_pin >= 0) adc_.setMillivolts(board_.adc_ladder_pin, mv);
    // UP/DOWN stay discrete GPIOs on this board.
    for (int i = 4; i <= 5; ++i) {
      const int pin = board_.buttons[i];
      if (pin >= 0) gpio_.driveExternal(pin, buttons_[i] ? 0 : Gpio::kFloat);
    }
  } else {
    // Plain active-low GPIO buttons: pressed pulls the pin to ground, released
    // floats it back to the internal pull-up.
    for (int i = 0; i < kButtonCount; ++i) {
      if (i == 6) continue;  // power handled below, it has its own polarity
      const int pin = board_.buttons[i];
      if (pin < 0) continue;
      gpio_.driveExternal(pin, buttons_[i] ? 0 : Gpio::kFloat);
    }
  }

  // Power is always a discrete pin, with a per-board polarity.
  const int powerPin = board_.buttons[6];
  if (powerPin >= 0) {
    const int activeLevel = board_.power_active_high ? 1 : 0;
    gpio_.driveExternal(powerPin, buttons_[6] ? activeLevel : Gpio::kFloat);
  }

  // The charge-status line follows the modelled charger.
  if (board_.battery_charge_status >= 0) {
    const bool charging = peripherals_.charging;
    const int level = board_.battery_charge_active_high ? (charging ? 1 : 0) : (charging ? 0 : 1);
    gpio_.driveExternal(board_.battery_charge_status, level);
  }

  // The battery ADC, where the board has one, reflects the modelled pack
  // voltage through the profile's divider.
  if (board_.battery_adc >= 0 && board_.battery_divider > 0.0f) {
    adc_.setMillivolts(board_.battery_adc,
                       static_cast<int>(peripherals_.batteryMillivolts / board_.battery_divider));
  }
}

void Machine::setPanelBusy(bool busy) {
  int pin = -1;
  int before = 0;
  int after = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!boardKnown_ || board_.epd_busy < 0) return;
    pin = board_.epd_busy;
    // SSD1677 is busy while HIGH; the UltraChip parts idle HIGH and are busy
    // LOW. This is the same split EpdBus encodes as BusyPolarity.
    const bool busyIsHigh = board_.panel_controller == FSIM_PANEL_SSD1677;
    const int level = busy == busyIsHigh ? 1 : 0;
    before = gpio_.read(pin);
    gpio_.driveExternal(pin, level);
    after = gpio_.read(pin);
  }
  // Outside the lock: the ISR runs on this thread and may take locks of its own.
  if (before != after) gpio_.fireInterrupts(pin, before, after);
}

void Machine::noteGpioRead(int pin) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++gpioReads_[pin];
}

void Machine::noteAdcRead(int pin) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++adcReads_[pin];
}

uint64_t Machine::inputSampleCount(int buttonIndex) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!boardKnown_ || buttonIndex < 0 || buttonIndex >= kButtonCount) return 0;

  auto countGpio = [this](int pin) -> uint64_t {
    if (pin < 0) return 0;
    auto it = gpioReads_.find(pin);
    return it == gpioReads_.end() ? 0 : it->second;
  };
  auto countAdc = [this](int pin) -> uint64_t {
    if (pin < 0) return 0;
    auto it = adcReads_.find(pin);
    return it == adcReads_.end() ? 0 : it->second;
  };

  // Power is always a discrete pin, whatever the board's input style.
  if (buttonIndex == 6) return countGpio(board_.buttons[6]);

  if (board_.input_style == FSIM_INPUT_ADC_LADDER) {
    // GPIO1 carries BACK/CONFIRM/LEFT/RIGHT, GPIO2 carries UP/DOWN.
    return countAdc(buttonIndex < 4 ? 1 : 2);
  }
  if (board_.input_style == FSIM_INPUT_ONEPAGE_LADDER) {
    return buttonIndex < 4 ? countAdc(board_.adc_ladder_pin) : countGpio(board_.buttons[buttonIndex]);
  }
  return countGpio(board_.buttons[buttonIndex]);
}

void Machine::setTouch(int x, int y, bool down) {
  int pin = -1;
  int before = 0;
  int after = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    touch_ = {x, y, down};
    // The digitizer's interrupt line is a real pin: firmware that waits on its
    // edge rather than polling the controller is told nothing otherwise, and a
    // tap simply never happens.
    if (boardKnown_ && board_.touch_irq >= 0) {
      pin = board_.touch_irq;
      const int asserted = board_.touch_irq_active_low ? 0 : 1;
      before = gpio_.read(pin);
      gpio_.driveExternal(pin, down ? asserted : Gpio::kFloat);
      after = gpio_.read(pin);
    }
  }
  if (pin >= 0 && before != after) gpio_.fireInterrupts(pin, before, after);
}

void Machine::clearTouch() { setTouch(0, 0, false); }

void Machine::requestRestart() {
  wakeCause_ = 0;
  powerState_ = PowerState::Running;
  teardown_ = true;
}

void Machine::enterDeepSleep(uint64_t sleepUs) {
  std::lock_guard<std::mutex> lock(mutex_);
  powerState_ = PowerState::DeepSleep;
  sleepUntilUs_ = sleepUs > 0 ? clock_.nowUs() + sleepUs : 0;
}

void Machine::enterLightSleep(uint64_t sleepUs) {
  std::lock_guard<std::mutex> lock(mutex_);
  powerState_ = PowerState::LightSleep;
  sleepUntilUs_ = sleepUs > 0 ? clock_.nowUs() + sleepUs : 0;
}

void Machine::setSleepWakePin(int pin, int level) {
  std::lock_guard<std::mutex> lock(mutex_);
  wakePin_ = pin;
  wakeLevel_ = level;
}

bool Machine::shouldWake() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (powerState_ != PowerState::DeepSleep && powerState_ != PowerState::LightSleep) return false;
  if (sleepUntilUs_ != 0 && clock_.nowUs() >= sleepUntilUs_) return true;
  if (wakePin_ >= 0 && gpio_.read(wakePin_) == wakeLevel_) return true;
  return false;
}

void Machine::wakeNow(int cause) {
  std::lock_guard<std::mutex> lock(mutex_);
  wakeCause_ = cause;
  powerState_ = PowerState::Running;
  teardown_ = true;
}

uint32_t Machine::nextRandom() {
  // xorshift32: deterministic for a given seed, which is what makes a run with
  // a pinned seed reproducible.
  uint32_t x = randomState_;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  randomState_ = x;
  return x;
}

void Machine::seedRandom(uint32_t seed) { randomState_ = seed ? seed : 0x12345678u; }

}  // namespace freeink::sim
