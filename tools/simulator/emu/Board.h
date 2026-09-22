#pragma once

// FreeInk emulator — the seam between the chip and the board around it.
//
// Everything else in emu/ models silicon: the cores, the buses, the peripheral
// blocks. None of it knows what is soldered to a pin. But a firmware image is
// not interesting until it can read a button, get an answer from the fuel
// gauge and paint the glass — and those are board facts, not chip facts.
//
// So the chip's pin-facing peripherals (GPIO, I2C, SPI, the ADC) talk to this
// interface, and the simulator daemon implements it against the same virtual
// devices a firmware *bundle* drives: the same panel model, the same I2C
// devices, the same GPIO matrix. That is what makes `capture` show what an
// emulated vendor image painted rather than only what a bundle painted.
//
// The default implementation is inert on purpose. `freeink-emu` runs without a
// board, and a firmware that probes a bus with nothing on it should see an
// empty bus rather than a fabricated answer.

#include <cstddef>
#include <cstdint>

namespace freeink::sim::emu {

class BoardBridge {
 public:
  virtual ~BoardBridge() = default;

  // ── GPIO ───────────────────────────────────────────────────────────────────
  // The chip owns direction and the output register; the board owns what an
  // input actually reads, because that is decided by what is wired to the pad:
  // a button pulling to ground, an open-drain IRQ, the panel's BUSY line.
  // IO_MUX owns the pad's pulls; the GPIO block owns its direction. They are
  // separate registers on the chip and arrive here separately, because a
  // firmware sets them at different moments and the level a pad reads in
  // between depends on which it has set so far.
  virtual void pinPull(int /*pin*/, bool /*pullUp*/, bool /*pullDown*/) {}
  virtual void pinDirection(int /*pin*/, bool /*output*/) {}
  virtual void pinWrite(int /*pin*/, int /*level*/) {}
  // The level an input pad settles at, or -1 when nothing external is driving
  // it and the chip's own pull decides. `observed` marks a read the firmware
  // actually made, as opposed to the machine's own sweep for interrupt edges —
  // scripted input holds a key down until the firmware has genuinely sampled
  // it, and a sweep is not a sample.
  virtual int pinRead(int /*pin*/, bool /*observed*/) { return -1; }

  // ── I2C ────────────────────────────────────────────────────────────────────
  // One combined write-then-read transaction, which is the shape every command
  // list the drivers build reduces to. False means nothing acknowledged the
  // address — the answer a probe for an absent peripheral must get.
  virtual bool i2cTransfer(int /*bus*/, uint8_t /*addr*/, const uint8_t* /*tx*/, size_t /*txLen*/,
                           uint8_t* /*rx*/, size_t /*rxLen*/) {
    return false;
  }
  virtual void i2cConfigured(int /*bus*/, int /*sda*/, int /*scl*/, uint32_t /*hz*/) {}

  // ── SPI ────────────────────────────────────────────────────────────────────
  // Full duplex, one chip-select assertion. `rx` may be null for write-only
  // traffic. The bridge decides which device is selected from the pins the
  // GPIO matrix routed the bus to and the CS level at the time.
  // Which device on the bus is selected is decided by the bridge, from the CS
  // pins the board profile declares and the levels the GPIO model is holding
  // them at. The chip cannot answer that question — it does not know what is
  // on the other end of the wire.
  virtual void spiTransfer(int /*bus*/, const uint8_t* /*tx*/, uint8_t* /*rx*/, size_t /*len*/) {}

  // ── SD, natively ───────────────────────────────────────────────────────────
  // A card on an SDMMC host rather than on SPI. The protocol is the same
  // commands, but the host controller exchanges them as fields rather than as
  // a byte stream, so the split is at the command rather than at the byte.
  //
  // `response` is four words, as the controller's RESP0..RESP3 hold them.
  // Returning false is "no card answered", which is what a host times out on.
  virtual bool sdCommand(int /*index*/, uint32_t /*argument*/, bool /*longResponse*/,
                         uint32_t* /*response*/) {
    return false;
  }
  virtual bool sdReadBlocks(uint32_t /*lba*/, uint32_t /*blocks*/, uint8_t* /*out*/) { return false; }
  // Some commands answer with a block of the card's own data rather than with
  // anything off the medium: the configuration register, the card's status,
  // the result of a function switch. Returns how many bytes it produced.
  virtual size_t sdCardData(uint8_t* /*out*/, size_t /*max*/) { return 0; }
  virtual bool sdWriteBlocks(uint32_t /*lba*/, uint32_t /*blocks*/, const uint8_t* /*in*/) {
    return false;
  }
  // Whether a card is in the slot at all, for the host's card-detect line.
  virtual bool sdPresent() { return false; }

  // ── ADC ────────────────────────────────────────────────────────────────────
  // A raw conversion result for a unit (1 or 2) and channel. Full scale is the
  // honest answer for a pin with nothing on it: an unconnected input with the
  // chip's pull-up reads high.
  virtual uint32_t adcSample(int /*unit*/, int /*channel*/) { return 4095; }
};

}  // namespace freeink::sim::emu
