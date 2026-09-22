#pragma once

// FreeInk emulator — the peripheral blocks that reach the outside world.
//
// Peripherals.h holds the blocks a firmware needs to *boot*: timers, the
// interrupt matrix, the console. These are the ones it needs to be a *device*:
// the pads, the two buses everything hangs off, and the DMA the bigger of
// those two transfers pull through.
//
// Each of them is a faithful model of the register block and nothing more —
// what is on the other end of the wire arrives through the BoardBridge, so the
// same GT911, the same fuel gauge and the same virtual panel answer an
// emulated image and a host-compiled bundle alike.

#include "Board.h"
#include "Peripherals.h"

#include <array>
#include <cstdint>
#include <vector>

namespace freeink::sim::emu {

// ── GPIO ─────────────────────────────────────────────────────────────────────
// Direction, the output register and its atomic set/clear aliases, the input
// register, the per-pin configuration in IO_MUX, and the pin-change interrupt.
//
// The part that has to be right is what a *read* returns. A pad the firmware
// configured as an input reads whatever is driving it — a button pulling it to
// ground, a controller's open-drain IRQ, the panel's BUSY line — and its own
// pull only when nothing is. Firmware that waits on BUSY, debounces a key or
// probes for a chip is doing nothing but reading this register, so a GPIO block
// that answers zero is a firmware that waits forever for a pin that never moves.
class GpioDevice : public GenericPeripheral {
 public:
  GpioDevice(const SocDesc& soc, BoardBridge* board, InterruptMatrixDevice* matrix);

  uint32_t read(uint32_t offset) override;
  void write(uint32_t offset, uint32_t value, uint32_t mask) override;

  // The level the chip itself is putting on a pad, or -1 when it is not
  // driving one. The SPI model asks this to find which chip select is
  // asserted, exactly as a scope on the board would.
  int drivenLevel(int pin) const;
  bool outputEnabled(int pin) const;
  // Re-reads the input pins and raises the GPIO interrupt for any that changed
  // in a way a pin's configured edge/level trigger asks for. Called once per
  // machine tick, because an input can change without the firmware touching a
  // register — that is the whole point of an interrupt.
  void pollInputs();

 private:
  static constexpr int kMaxPins = 64;

  int inputLevel(int pin, bool observed);
  // The pending status, narrowed to the pads whose interrupt was enabled for
  // one destination — which is the form the driver's ISR reads it in.
  uint32_t maskedStatus(int word, uint32_t enableBit) const;
  void pushOutput(int pin);
  void noteEdge(int pin, int from, int to);

  const SocDesc& soc_;
  BoardBridge* board_;
  InterruptMatrixDevice* matrix_;
  uint32_t out_[2] = {0, 0};
  uint32_t enable_[2] = {0, 0};
  uint32_t status_[2] = {0, 0};
  std::array<uint32_t, kMaxPins> pinConf_{};
  std::array<int, kMaxPins> lastInput_{};
};

// The IO_MUX block, which owns each pad's pull-up, pull-down and input enable.
// It is a plain register file, but the GPIO model has to read it — a pin's
// resting level is decided here, not in the GPIO block — so the two are wired
// together rather than each keeping its own idea of the pad.
class IoMuxDevice : public GenericPeripheral {
 public:
  IoMuxDevice(const SocDesc& soc, BoardBridge* board);

  void write(uint32_t offset, uint32_t value, uint32_t mask) override;

  bool pullUp(int pin) const;
  bool pullDown(int pin) const;
  bool inputEnabled(int pin) const;

 private:
  uint32_t padConf(int pin) const;

  const SocDesc& soc_;
  BoardBridge* board_;
};

// ── I2C master ───────────────────────────────────────────────────────────────
// The ESP32 I2C master is programmed as a little command list: RESTART, WRITE
// n, READ n, STOP, each in its own register, with the bytes flowing through a
// 32-byte FIFO. Writing TRANS_START runs the list.
//
// That is what this model decodes. It walks the list, takes the address out of
// the first byte of the first write, hands the whole combined transaction to
// the board, and raises the interrupt the driver is blocked on — or the NACK
// interrupt, when nothing answers. An address with no device behind it is the
// answer a probe needs, which is what makes "is this peripheral fitted?" a
// real question here.
class I2cMasterDevice : public GenericPeripheral {
 public:
  I2cMasterDevice(std::string name, uint32_t base, int bus, BoardBridge* board,
                  InterruptMatrixDevice* matrix, int interruptSource);

  uint32_t read(uint32_t offset) override;
  void write(uint32_t offset, uint32_t value, uint32_t mask) override;

 private:
  void runCommandList();
  void raise(uint32_t bits);
  void refreshInterrupt();

  int bus_;
  BoardBridge* board_;
  InterruptMatrixDevice* matrix_;
  int source_;
  std::vector<uint8_t> txFifo_;
  std::vector<uint8_t> rxFifo_;
  size_t rxRead_ = 0;
  // Bytes the board handed back that the command list has not drained yet. A
  // driver splitting a long read across segments (the END command) continues
  // the same bus transaction, so the second segment must get the rest of the
  // same answer rather than a fresh one.
  std::vector<uint8_t> pendingRead_;
  uint8_t address_ = 0;
  std::vector<uint8_t> writePhase_;
  uint32_t intRaw_ = 0;
  uint32_t intEna_ = 0;
};

// ── GDMA ─────────────────────────────────────────────────────────────────────
// Anything longer than the SPI block's 64-byte register file goes through DMA,
// and a panel frame is three orders of magnitude longer than that — so without
// this, an emulated image can configure the panel perfectly and never send it a
// pixel.
//
// The model is the descriptor chain: a linked list in RAM, each entry naming a
// buffer, how much of it is valid, and whether it is the last. Transmit walks
// it and gathers; receive walks it and scatters, writing back the lengths and
// releasing ownership the way the hardware does.
class GdmaDevice : public GenericPeripheral {
 public:
  GdmaDevice(const SocDesc& soc, Bus& bus, InterruptMatrixDevice* matrix);

  uint32_t read(uint32_t offset) override;
  void write(uint32_t offset, uint32_t value, uint32_t mask) override;

  // The channel currently bound to a peripheral, or -1. `peripheral` is the
  // GDMA_*_PERI_SEL value — 0 is SPI2, 1 is SPI3 on the parts here.
  int channelForTx(int peripheral) const;
  int channelForRx(int peripheral) const;

  // Walks the transmit descriptor chain, returning the bytes it holds.
  std::vector<uint8_t> gather(int channel, size_t limit);
  // Walks the receive chain, writing bytes into its buffers and reporting how
  // many it could place.
  size_t scatter(int channel, const uint8_t* data, size_t length);
  // Raises the "last descriptor consumed" interrupt the driver waits on.
  void signalTxEof(int channel);
  void signalRxEof(int channel);

 private:
  struct Channel {
    uint32_t inLink = 0;
    uint32_t outLink = 0;
    uint32_t inPeri = 0x3F;
    uint32_t outPeri = 0x3F;
    uint32_t intRaw = 0;
    uint32_t intEna = 0;
    uint32_t inEofDesc = 0;
    uint32_t outEofDesc = 0;
  };

  // Descriptor addresses are held as 20 bits; the hardware supplies the
  // internal-RAM base the rest of the way.
  uint32_t descriptorAddress(uint32_t link) const;
  void refreshInterrupt(int channel);

  const SocDesc& soc_;
  Bus& bus_;
  InterruptMatrixDevice* matrix_;
  std::vector<Channel> channels_;
};

// ── SPI master ───────────────────────────────────────────────────────────────
// A transaction here is not a byte stream the driver hands over; it is a set of
// phases the driver composes in the USER registers — command, address, dummy
// cycles, then data out and data in — with the length in bits and the payload
// either in the register file or on the far end of a DMA chain. Writing the USR
// bit runs it.
//
// So a driver that programs the wrong length, forgets to enable the MOSI phase
// or starts a transfer before its DMA link is set sends the wrong bytes here,
// as it would on the board.
class SpiMasterDevice : public GenericPeripheral {
 public:
  SpiMasterDevice(std::string name, uint32_t base, int bus, int dmaPeripheral, BoardBridge* board,
                  GdmaDevice* dma, InterruptMatrixDevice* matrix, int interruptSource);

  uint32_t read(uint32_t offset) override;
  void write(uint32_t offset, uint32_t value, uint32_t mask) override;

 private:
  void runTransaction();

  int bus_;
  int dmaPeripheral_;
  BoardBridge* board_;
  GdmaDevice* dma_;
  InterruptMatrixDevice* matrix_;
  int source_;
  uint32_t intRaw_ = 0;
  uint32_t intEna_ = 0;
};

// ── SDMMC host ───────────────────────────────────────────────────────────────
// The S3's card slot is not on SPI: it has a dedicated host controller, and a
// firmware that uses it never touches the SPI block at all. So without this a
// board whose card is native has no storage, and its firmware stops at the
// screen that says so.
//
// The controller is the usual DesignWare mobile-storage design: a command
// register that fires when its top bit is written, four response registers,
// and an internal DMA that moves the data through a chain of descriptors in
// RAM. The model runs a command to completion the moment it is started —
// there is no card latency to spend — and raises the two interrupts the
// driver waits on: the command finished, and the data finished.
class SdmmcHostDevice : public GenericPeripheral {
 public:
  SdmmcHostDevice(uint32_t base, Bus& bus, BoardBridge* board, InterruptMatrixDevice* matrix,
                  int interruptSource);

  uint32_t read(uint32_t offset) override;
  void write(uint32_t offset, uint32_t value, uint32_t mask) override;

 private:
  void runCommand(uint32_t command);
  // Moves `length` bytes between the card and the descriptor chain the driver
  // built. Returns how many it could place.
  size_t transferThroughDma(bool toCard, uint32_t lba, uint32_t length);
  void raise(uint32_t bits);

  Bus& bus_;
  BoardBridge* board_;
  InterruptMatrixDevice* matrix_;
  int source_;
  uint32_t intRaw_ = 0;
  uint32_t intMask_ = 0;
  uint32_t dmaStatus_ = 0;
  uint32_t dmaMask_ = 0;
  uint32_t response_[4] = {0, 0, 0, 0};
  uint32_t descriptorBase_ = 0;
  // The address the next read or write starts at, as the last command set it.
  uint32_t blockAddress_ = 0;
  bool reading_ = false;
};

}  // namespace freeink::sim::emu
