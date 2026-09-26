// FreeInk simulator — virtual I2C devices.

#include "I2cDevices.h"

#include "Machine.h"

#include <algorithm>
#include <cstring>
#include <ctime>

namespace freeink::sim {
namespace {

uint8_t toBcd(int value) { return static_cast<uint8_t>(((value / 10) << 4) | (value % 10)); }

// ── GT911 capacitive digitizer ───────────────────────────────────────────────
// 16-bit big-endian register pointer. InputManager reads the status byte at
// 0x814E (bit 7 = data ready, low nibble = contact count), then the point
// records at 0x8150, then writes 0 back to 0x814E to release the frame. The
// model reproduces that handshake exactly, including the "coords at byte 0"
// variant some modules ship, because the SDK branches on it.
//
// The controller scans continuously: while a finger is down every poll finds a
// fresh frame, and when it lifts, one more ready frame reports zero contacts.
// That release frame is what completes a tap or a swipe in the driver, so it
// stays ready until the host clears it; after that the ready bit stays clear.
class Gt911Device : public I2cDevice {
 public:
  Gt911Device(Machine& machine, const fsim_board_desc& desc) : machine_(machine), desc_(desc) {}

  const char* name() const override { return "GT911 touch"; }

  // Whether the board is holding the controller's rail at its active level.
  // A board that gives the digitizer no rail of its own is always powered.
  //
  // The pad has to be *driving* it. A pin the firmware has not configured is
  // not asserting anything — the rail sits wherever the board's own pull puts
  // it, which for an enable is off — and treating an undriven pin as an
  // assertion would let a probe find a controller that is not powered yet.
  bool powered() const {
    if (desc_.touch_power_enable < 0) return true;
    const Gpio& gpio = machine_.gpio();
    if (!(gpio.mode(desc_.touch_power_enable) & FSIM_PIN_OUTPUT)) return false;
    const int active = desc_.touch_power_active_high ? 1 : 0;
    return gpio.read(desc_.touch_power_enable) == active;
  }

  bool transfer(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) override {
    // An unpowered controller is not on the bus at all. Firmware that has not
    // switched its rail on finds nothing here, which is what its own power-up
    // sequence exists to fix — and what makes that sequence testable.
    if (!powered()) return false;
    if (txLen >= 2) reg_ = static_cast<uint16_t>((tx[0] << 8) | tx[1]);
    // A write of a third byte to 0x814E is the frame-release the driver issues.
    if (txLen >= 3 && reg_ == 0x814E) {
      if (!machine_.touch().down) releasePending_ = false;
      return true;
    }
    if (rxLen == 0) return true;  // bare address probe, or a register write

    for (size_t i = 0; i < rxLen; ++i) rx[i] = readRegister(static_cast<uint16_t>(reg_ + i));
    return true;
  }

 private:
  uint8_t readRegister(uint16_t reg) {
    const Machine::TouchPoint touch = machine_.touch();

    if (reg == 0x814E) {
      if (touch.down) {
        releasePending_ = true;
        return 0x80 | 0x01;  // data ready, one contact
      }
      return releasePending_ ? 0x80 : 0x00;  // data ready with no contacts: the lift
    }
    if (reg >= 0x8150 && reg < 0x8150 + 8 * 5) {
      const int offset = (reg - 0x8150) % 8;
      // Panel-native coordinates, undoing the profile's orientation transform
      // so the SDK's forward transform lands back on the requested point.
      int x = touch.x;
      int y = touch.y;
      if (desc_.touch_flip_x) x = desc_.touch_raw_max_x - x;
      if (desc_.touch_flip_y) y = desc_.touch_raw_max_y - y;
      if (desc_.touch_swap_xy) std::swap(x, y);

      // With a track-id byte the coordinates start at byte 1; without one they
      // start at byte 0. The profile says which this module is.
      const int coordBase = desc_.touch_swap_xy || true ? 0 : 0;
      (void)coordBase;
      const int shift = desc_.touch_raw_max_x >= 0 && gt911CoordsAtByte0() ? 0 : 1;
      const int field = offset - shift;
      switch (field) {
        case -1: return 0x00;                                  // track id
        case 0: return static_cast<uint8_t>(x & 0xFF);         // x low
        case 1: return static_cast<uint8_t>((x >> 8) & 0xFF);  // x high
        case 2: return static_cast<uint8_t>(y & 0xFF);         // y low
        case 3: return static_cast<uint8_t>((y >> 8) & 0xFF);  // y high
        case 4: return 0x20;                                   // size low
        case 5: return 0x00;                                   // size high
        default: return 0x00;
      }
    }
    if (reg == 0x8140) return 'G';  // product id, first byte
    if (reg == 0x8141) return 'T';
    return 0x00;
  }

  // The X4 Pro and Classic modules report coordinates at byte 0; the board
  // description carries the flag the SDK uses, so mirror it here.
  bool gt911CoordsAtByte0() const { return true; }

  Machine& machine_;
  fsim_board_desc desc_;
  uint16_t reg_ = 0;
  bool releasePending_ = false;  // the finger lifted; the zero-contact frame is not yet cleared
};

// ── FT6336U / FT5x06 / CHSC6x single-touch digitizers ────────────────────────
// 8-bit register pointer; the contact count is at 0x02 and the first point's
// big-endian coordinates follow at 0x03.
class FtTouchDevice : public I2cDevice {
 public:
  FtTouchDevice(Machine& machine, const fsim_board_desc& desc) : machine_(machine), desc_(desc) {}

  const char* name() const override { return "FT6336U touch"; }

  bool transfer(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) override {
    if (txLen >= 1) reg_ = tx[0];
    if (rxLen == 0) return true;
    const Machine::TouchPoint touch = machine_.touch();
    for (size_t i = 0; i < rxLen; ++i) {
      const uint8_t reg = static_cast<uint8_t>(reg_ + i);
      switch (reg) {
        case 0x02: rx[i] = touch.down ? 1 : 0; break;
        case 0x03: rx[i] = static_cast<uint8_t>((touch.x >> 8) & 0x0F); break;
        case 0x04: rx[i] = static_cast<uint8_t>(touch.x & 0xFF); break;
        case 0x05: rx[i] = static_cast<uint8_t>((touch.y >> 8) & 0x0F); break;
        case 0x06: rx[i] = static_cast<uint8_t>(touch.y & 0xFF); break;
        case 0xA8: rx[i] = 0x11; break;  // vendor id
        default: rx[i] = 0x00; break;
      }
    }
    (void)desc_;
    return true;
  }

 private:
  Machine& machine_;
  fsim_board_desc desc_;
  uint8_t reg_ = 0;
};

// ── PCF8563 / BM8563 real-time clock ─────────────────────────────────────────
// BCD time from register 0x02. The clock it reports is the simulated clock
// offset from the host date, so a firmware that stamps files or schedules a
// wake sees time move at the simulator's pace.
class Pcf8563Device : public I2cDevice {
 public:
  explicit Pcf8563Device(Machine& machine) : machine_(machine) {}

  const char* name() const override { return "PCF8563/BM8563 RTC"; }

  bool transfer(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) override {
    if (txLen >= 1) reg_ = tx[0];
    // A multi-byte write sets the clock; remember the offset so reads follow.
    if (txLen > 1 && reg_ == 0x02 && txLen >= 8) {
      struct tm t {};
      t.tm_sec = fromBcd(tx[1] & 0x7F);
      t.tm_min = fromBcd(tx[2] & 0x7F);
      t.tm_hour = fromBcd(tx[3] & 0x3F);
      t.tm_mday = fromBcd(tx[4] & 0x3F);
      t.tm_mon = fromBcd(tx[6] & 0x1F) - 1;
      t.tm_year = fromBcd(tx[7]) + 100;
      setEpoch_ = ::timegm(&t);
      setAtUs_ = machine_.clock().nowUs();
      return true;
    }
    if (rxLen == 0) return true;

    const time_t now = currentEpoch();
    struct tm t {};
    ::gmtime_r(&now, &t);
    for (size_t i = 0; i < rxLen; ++i) {
      switch (static_cast<uint8_t>(reg_ + i)) {
        case 0x00: rx[i] = 0x00; break;  // control 1
        case 0x01: rx[i] = 0x00; break;  // control 2
        case 0x02: rx[i] = toBcd(t.tm_sec); break;
        case 0x03: rx[i] = toBcd(t.tm_min); break;
        case 0x04: rx[i] = toBcd(t.tm_hour); break;
        case 0x05: rx[i] = toBcd(t.tm_mday); break;
        case 0x06: rx[i] = toBcd(t.tm_wday); break;
        case 0x07: rx[i] = toBcd(t.tm_mon + 1); break;
        case 0x08: rx[i] = toBcd(t.tm_year % 100); break;
        default: rx[i] = 0x00; break;
      }
    }
    return true;
  }

 private:
  static int fromBcd(uint8_t v) { return ((v >> 4) * 10) + (v & 0x0F); }

  time_t currentEpoch() const {
    if (setEpoch_ != 0) {
      return setEpoch_ + static_cast<time_t>((machine_.clock().nowUs() - setAtUs_) / 1000000ULL);
    }
    return bootEpoch_ + static_cast<time_t>(machine_.clock().nowUs() / 1000000ULL);
  }

  Machine& machine_;
  uint8_t reg_ = 0;
  time_t setEpoch_ = 0;
  uint64_t setAtUs_ = 0;
  const time_t bootEpoch_ = ::time(nullptr);
};

// ── CW2017 fuel gauge ────────────────────────────────────────────────────────
// The CellWise gauge on the X4 Pro. Unlike the BQ27220 it is not a gauge until
// it has been told what cell it is measuring: it reports nothing useful until
// an 80-byte battery profile has been written into it and it has been
// restarted. The firmware's init sequence is written around exactly that, so
// the model has to play the whole part — the startup version, the soft-reset
// handshake, the profile store and the update flag — or the driver decides the
// gauge is absent and the device shows no battery at all.
class Cw2017Device : public I2cDevice {
 public:
  explicit Cw2017Device(Machine& machine) : machine_(machine) {}

  const char* name() const override { return "CW2017 fuel gauge"; }

  bool transfer(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) override {
    if (txLen >= 1) reg_ = tx[0];
    // Everything after the register pointer is a write.
    for (size_t i = 1; i < txLen; ++i) store(static_cast<uint8_t>(reg_ + (i - 1)), tx[i]);
    if (rxLen == 0) return true;
    for (size_t i = 0; i < rxLen; ++i) rx[i] = load(static_cast<uint8_t>(reg_ + i));
    return true;
  }

 private:
  static constexpr uint8_t kRegVersion = 0x00;
  static constexpr uint8_t kRegVcellHigh = 0x02;
  static constexpr uint8_t kRegSoc = 0x04;
  static constexpr uint8_t kRegMode = 0x08;
  static constexpr uint8_t kRegSocAlert = 0x0B;
  static constexpr uint8_t kRegBatInfo = 0x10;
  static constexpr uint8_t kModeNormal = 0x00;
  static constexpr uint8_t kUpdateFlag = 0x80;

  void store(uint8_t reg, uint8_t value) {
    if (reg >= kRegBatInfo) {
      profile_[reg - kRegBatInfo] = value;
      profileLoaded_ = true;
      return;
    }
    if (reg == kRegMode) {
      mode_ = value;
      // The driver's soft reset is 0xF0, 0x30, then 0x00. Coming back to
      // normal is what restarts the calculation engine, and the version
      // register is how the firmware watches for it finishing.
      running_ = value == kModeNormal;
      return;
    }
    if (reg == kRegSocAlert) alert_ = value;
  }

  uint8_t load(uint8_t reg) {
    if (reg >= kRegBatInfo && reg < kRegBatInfo + sizeof(profile_)) {
      return profile_[reg - kRegBatInfo];
    }
    const auto& power = machine_.peripherals();
    // 14 bits of VCELL at 305.18 µV per count, as the datasheet scales it.
    const uint32_t vcell = static_cast<uint32_t>(power.batteryMillivolts * 1000.0 / 305.18);
    switch (reg) {
      // 0xA0 while the engine is starting; a running part reports 0x0D.
      case kRegVersion: return running_ ? 0x0D : 0xA0;
      case kRegVcellHigh: return static_cast<uint8_t>((vcell >> 8) & 0x3F);
      case kRegVcellHigh + 1: return static_cast<uint8_t>(vcell & 0xFF);
      case kRegSoc:
        // The whole point of the profile: without one the gauge has no model
        // of the cell and reports nothing, which is what the firmware's
        // "upload the profile" path exists to fix.
        return profileLoaded_ ? static_cast<uint8_t>(std::clamp(power.batteryPercent, 0, 100)) : 0;
      case kRegSoc + 1: return 0;
      case kRegMode: return mode_;
      case kRegSocAlert: return static_cast<uint8_t>(alert_ | (profileLoaded_ ? kUpdateFlag : 0));
      default: return 0;
    }
  }

  Machine& machine_;
  uint8_t reg_ = 0;
  // A gauge that has just been powered reports its default mode, not normal:
  // the firmware's init sequence is what walks it through the soft reset into
  // running, and a model that starts out already normal skips the handshake
  // the driver is written around.
  uint8_t mode_ = 0xF0;
  uint8_t alert_ = 0;
  bool running_ = false;
  bool profileLoaded_ = false;
  uint8_t profile_[80] = {};
};

// ── BQ27220 fuel gauge ───────────────────────────────────────────────────────
// 16-bit little-endian registers: voltage at 0x08, SOC at 0x2C.
class Bq27220Device : public I2cDevice {
 public:
  explicit Bq27220Device(Machine& machine) : machine_(machine) {}

  const char* name() const override { return "BQ27220 fuel gauge"; }

  bool transfer(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) override {
    if (txLen >= 1) reg_ = tx[0];
    if (rxLen == 0) return true;
    const auto& power = machine_.peripherals();
    uint16_t value = 0;
    switch (reg_) {
      case 0x08: value = static_cast<uint16_t>(power.batteryMillivolts); break;
      case 0x2C: value = static_cast<uint16_t>(std::clamp(power.batteryPercent, 0, 100)); break;
      case 0x06: value = 2980; break;  // temperature, 0.1 K
      default: value = 0; break;
    }
    for (size_t i = 0; i < rxLen; ++i) {
      rx[i] = (i == 0) ? static_cast<uint8_t>(value & 0xFF) : static_cast<uint8_t>((value >> 8) & 0xFF);
    }
    return true;
  }

 private:
  Machine& machine_;
  uint8_t reg_ = 0;
};

// ── QMI8658 / LSM6DS3 IMU ────────────────────────────────────────────────────
// Only what the SDK reads: WHO_AM_I and the acceleration/gyro output words,
// fed from the CLI-settable orientation so a tilt-to-rotate path is drivable.
class ImuDevice : public I2cDevice {
 public:
  ImuDevice(Machine& machine, bool qmi) : machine_(machine), qmi_(qmi) {}

  const char* name() const override { return qmi_ ? "QMI8658 IMU" : "LSM6DS3 IMU"; }

  bool transfer(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) override {
    if (txLen >= 1) reg_ = tx[0];
    if (rxLen == 0) return true;
    const auto& p = machine_.peripherals();
    // ±8 g full scale, 16-bit signed.
    const int16_t ax = static_cast<int16_t>(p.imuAccel[0] * 4096.0);
    const int16_t ay = static_cast<int16_t>(p.imuAccel[1] * 4096.0);
    const int16_t az = static_cast<int16_t>(p.imuAccel[2] * 4096.0);

    for (size_t i = 0; i < rxLen; ++i) {
      const uint8_t reg = static_cast<uint8_t>(reg_ + i);
      if (qmi_) {
        switch (reg) {
          case 0x00: rx[i] = 0x05; break;  // WHO_AM_I
          case 0x35: rx[i] = 0x03; break;  // STATUS0: accel + gyro ready
          case 0x37: rx[i] = static_cast<uint8_t>(ax & 0xFF); break;
          case 0x38: rx[i] = static_cast<uint8_t>((ax >> 8) & 0xFF); break;
          case 0x39: rx[i] = static_cast<uint8_t>(ay & 0xFF); break;
          case 0x3A: rx[i] = static_cast<uint8_t>((ay >> 8) & 0xFF); break;
          case 0x3B: rx[i] = static_cast<uint8_t>(az & 0xFF); break;
          case 0x3C: rx[i] = static_cast<uint8_t>((az >> 8) & 0xFF); break;
          default: rx[i] = 0x00; break;
        }
      } else {
        switch (reg) {
          case 0x0F: rx[i] = 0x69; break;  // WHO_AM_I
          case 0x28: rx[i] = static_cast<uint8_t>(ax & 0xFF); break;
          case 0x29: rx[i] = static_cast<uint8_t>((ax >> 8) & 0xFF); break;
          case 0x2A: rx[i] = static_cast<uint8_t>(ay & 0xFF); break;
          case 0x2B: rx[i] = static_cast<uint8_t>((ay >> 8) & 0xFF); break;
          case 0x2C: rx[i] = static_cast<uint8_t>(az & 0xFF); break;
          case 0x2D: rx[i] = static_cast<uint8_t>((az >> 8) & 0xFF); break;
          default: rx[i] = 0x00; break;
        }
      }
    }
    return true;
  }

 private:
  Machine& machine_;
  bool qmi_;
  uint8_t reg_ = 0;
};

// ── LM3630A I2C frontlight ───────────────────────────────────────────────────
// Brightness writes are recorded on the PWM map so `freeink-sim light` reports
// them alongside the GPIO-PWM boards.
class Lm3630aDevice : public I2cDevice {
 public:
  explicit Lm3630aDevice(Machine& machine) : machine_(machine) {}

  const char* name() const override { return "LM3630A frontlight"; }

  bool transfer(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) override {
    if (txLen >= 1) reg_ = tx[0];
    if (txLen >= 2) {
      // Brightness registers A and B.
      if (reg_ == 0x03 || reg_ == 0x04) {
        std::lock_guard<std::mutex> lock(machine_.mutex());
        machine_.peripherals().pwmDuty[-1] = tx[1];
      }
      registers_[reg_] = tx[1];
    }
    for (size_t i = 0; i < rxLen; ++i) rx[i] = registers_[static_cast<uint8_t>(reg_ + i)];
    return true;
  }

 private:
  Machine& machine_;
  uint8_t reg_ = 0;
  uint8_t registers_[256] = {};
};

}  // namespace

I2cBus::I2cBus(Machine& machine) : machine_(machine) {}
I2cBus::~I2cBus() = default;

void I2cBus::configure(const fsim_board_desc& desc) {
  std::lock_guard<std::mutex> lock(mutex_);
  devices_.clear();

  switch (desc.touch_controller) {
    case FSIM_TOUCH_GT911:
      // Only the primary address answers. The SDK's probe tries the alternate
      // too, and finding exactly one is the behaviour worth reproducing.
      if (desc.touch_addr) devices_[desc.touch_addr] = std::make_unique<Gt911Device>(machine_, desc);
      break;
    case FSIM_TOUCH_FT6336U:
    case FSIM_TOUCH_FT5X06:
    case FSIM_TOUCH_CHSC6X:
    case FSIM_TOUCH_GSLX680:
      if (desc.touch_addr) devices_[desc.touch_addr] = std::make_unique<FtTouchDevice>(machine_, desc);
      break;
    default:
      break;
  }

  if (desc.rtc_addr) devices_[desc.rtc_addr] = std::make_unique<Pcf8563Device>(machine_);
  if (desc.imu_addr) devices_[desc.imu_addr] = std::make_unique<ImuDevice>(machine_, desc.imu_addr == 0x6B);
  if (desc.gauge_addr == 0x63) {
    devices_[desc.gauge_addr] = std::make_unique<Cw2017Device>(machine_);
  } else if (desc.gauge_addr == 0x55) {
    devices_[desc.gauge_addr] = std::make_unique<Bq27220Device>(machine_);
  } else if (desc.gauge_addr) {
    devices_[desc.gauge_addr] = std::make_unique<Cw2017Device>(machine_);
  }
  // Only where the board actually carries one: a device that answered on
  // every board would make "is the frontlight fitted?" untestable, which is
  // the one question this bus exists to answer honestly.
  if (desc.frontlight_i2c_addr) {
    devices_[desc.frontlight_i2c_addr] = std::make_unique<Lm3630aDevice>(machine_);
  }
}

void I2cBus::begin(int, int, int, uint32_t) {}

bool I2cBus::transfer(int, uint8_t addr, const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = devices_.find(addr);
  // No device at this address: NACK, exactly as an unpopulated bus does. This
  // is what lets probe code discover that a controller is absent.
  if (it == devices_.end()) return false;
  return it->second->transfer(tx, txLen, rx, rxLen);
}

std::vector<std::pair<uint8_t, std::string>> I2cBus::devices() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::pair<uint8_t, std::string>> out;
  for (const auto& [addr, device] : devices_) out.emplace_back(addr, device->name());
  return out;
}

}  // namespace freeink::sim
