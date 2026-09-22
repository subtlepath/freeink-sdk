#pragma once

// FreeInk simulator — virtual I2C devices.
//
// The real drivers run: InputManager's GT911 point-frame reader, the BM8563
// date arithmetic, the CW2017 gauge decode, the QMI8658 WHO_AM_I probe. Each
// device answers register reads the way its silicon does, and an address with
// no device behind it NACKs — which is what makes controller autodetection and
// "is this peripheral fitted?" paths testable instead of stubbed out.

#include <freeink_sim_abi.h>

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace freeink::sim {

class Machine;

class I2cDevice {
 public:
  virtual ~I2cDevice() = default;
  virtual const char* name() const = 0;
  // One combined transaction. `tx` is the write phase (usually a register
  // pointer), `rx` the read phase. Return false to NACK.
  virtual bool transfer(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen) = 0;
};

class I2cBus {
 public:
  explicit I2cBus(Machine& machine);
  ~I2cBus();

  // Rebuilds the device set from the board description: the digitizer at its
  // address, the RTC, the gauge, the IMU.
  void configure(const fsim_board_desc& desc);

  bool transfer(int bus, uint8_t addr, const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxLen);
  void begin(int bus, int sda, int scl, uint32_t hz);

  // Addresses currently answering, for `freeink-sim i2c scan`.
  std::vector<std::pair<uint8_t, std::string>> devices() const;

 private:
  Machine& machine_;
  mutable std::mutex mutex_;
  std::map<uint8_t, std::unique_ptr<I2cDevice>> devices_;
};

}  // namespace freeink::sim
