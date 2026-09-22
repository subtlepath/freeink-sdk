#pragma once

// FreeInk emulator — the SPI memory controller the flash hangs off.
//
// This is the block an app drives for every flash access that does not go
// through the cache: reading the partition table, NVS, OTA writes, `esp_flash`
// calls. It is modelled at the transaction level — the firmware composes a SPI
// transaction in the USER/USER1/USER2 registers and sets a start bit; this
// decodes it, runs the NOR command against the FlashImage, and puts the
// result back in the data registers.
//
// Modelling it at the command level rather than faking `esp_flash` means
// firmware that talks to the flash its own way, or a driver bug that sends a
// malformed transaction, behaves here the way it does on the device.

#include "Bus.h"
#include "Peripherals.h"

#include <cstdint>

namespace freeink::sim::emu {

class FlashControllerDevice : public GenericPeripheral {
 public:
  FlashControllerDevice(std::string name, uint32_t base, FlashImage& flash);

  uint32_t read(uint32_t offset) override;
  void write(uint32_t offset, uint32_t value, uint32_t mask) override;

  // The JEDEC id the chip answers with. 0xC84018 is a 16 MB GigaDevice part,
  // which is what these boards carry; a firmware that keys its timing off the
  // id sees a consistent answer. The ROM's own flash routines report the same
  // one, so a driver gets the same part whichever way it asks.
  static constexpr uint32_t kDefaultJedecId = 0xC84018;
  void setJedecId(uint32_t id) { jedecId_ = id; }

 private:
  void runUserTransaction();
  void runDedicatedCommand(uint32_t command);
  void writeDataWords(const uint8_t* data, size_t length);
  void readDataWords(uint8_t* data, size_t length);

  FlashImage& flash_;
  uint32_t jedecId_ = kDefaultJedecId;
  uint32_t status_ = 0;
  bool writeEnabled_ = false;
  uint32_t words_[16] = {};
};

}  // namespace freeink::sim::emu
