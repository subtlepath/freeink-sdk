#pragma once

// FreeInk simulator — the board an emulated image runs on.
//
// This is the piece that turns the emulator from a chip into a device. The
// emulated SoC's pin-facing peripherals — GPIO, I2C, SPI, the SAR ADC — reach
// the outside world through emu::BoardBridge, and this implements it against
// the *same* virtual machine a host-compiled bundle drives.
//
// That sharing is the point. The GT911 that answers an emulated image is the
// GT911 that answers a bundle; the panel an emulated image paints is the panel
// `freeink-sim capture` screenshots; a button pressed over the control socket
// arrives as the same voltage on the same ladder. Nothing here is a second
// implementation of anything.
//
// What it adds on top of the bundle path is the part a bundle gets for free:
// an image has no board profile, so it is told which board it is on, and this
// class is where that profile decides where a byte on SPI2 actually goes.

#include "../emu/Board.h"
#include "SdNativeCard.h"
#include "SdSpiCard.h"

#include <freeink_sim_abi.h>

#include <cstdint>
#include <functional>
#include <string>
#include <utility>

namespace freeink::sim {

class Machine;

class EmuBoard : public emu::BoardBridge {
 public:
  // `chip` is the SoC name ("esp32c3", "esp32s3") — it decides which pad each
  // ADC channel belongs to, which is a fact about the silicon rather than the
  // board.
  EmuBoard(Machine& machine, const std::string& chip);

  void pinPull(int pin, bool pullUp, bool pullDown) override;
  void pinDirection(int pin, bool output) override;
  void pinWrite(int pin, int level) override;
  int pinRead(int pin, bool observed) override;

  bool i2cTransfer(int bus, uint8_t addr, const uint8_t* tx, size_t txLen, uint8_t* rx,
                   size_t rxLen) override;
  void i2cConfigured(int bus, int sda, int scl, uint32_t hz) override;

  void spiTransfer(int bus, const uint8_t* tx, uint8_t* rx, size_t len) override;

  bool sdCommand(int index, uint32_t argument, bool longResponse, uint32_t* response) override;
  bool sdReadBlocks(uint32_t lba, uint32_t blocks, uint8_t* out) override;
  size_t sdCardData(uint8_t* out, size_t max) override;
  bool sdWriteBlocks(uint32_t lba, uint32_t blocks, const uint8_t* in) override;
  bool sdPresent() override;

  uint32_t adcSample(int unit, int channel) override;

  // Every bus transaction, as one line, for the operator watching a firmware
  // bring its peripherals up. An image has no logging of its own to turn on,
  // so this is the only way to see it talk to a chip that is not answering.
  void setTrace(std::function<void(const std::string&)> sink) { trace_ = std::move(sink); }

 private:
  // Which pad an ADC channel is wired to on this chip, or -1.
  int adcPin(int unit, int channel) const;
  // Which device on the bus is selected, judged the way a scope would: by the
  // level of the chip selects the board declares.
  bool selected(int csPin, bool defaultWhenUnset) const;
  void refreshMode(int pin);

  Machine& machine_;
  std::string chip_;
  std::function<void(const std::string&)> trace_;
  // The card answers on the same bus as the panel, behind its own chip select.
  // It is a member rather than a device on a list because it is the only thing
  // on these boards that shares the panel's bus.
  SdSpiCard sdCard_;
  // The same card on the other kind of host. Which one a board uses is a
  // board fact, so both exist and only one ever sees traffic.
  SdNativeCard sdNative_;
  struct PinState {
    bool output = false;
    bool pullUp = false;
    bool pullDown = false;
  };
  PinState pins_[64];
};

}  // namespace freeink::sim
