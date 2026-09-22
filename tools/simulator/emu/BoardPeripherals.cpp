// FreeInk emulator — the peripheral blocks that reach the outside world.

#include "BoardPeripherals.h"

#include <algorithm>
#include <cstring>

namespace freeink::sim::emu {
namespace {

// ── GPIO register map ────────────────────────────────────────────────────────
// Identical on the C3 and the S3; the S3 simply has the second word of each
// pair populated because it has more than 32 pads.
constexpr uint32_t kGpioOut = 0x0004;
constexpr uint32_t kGpioOutW1ts = 0x0008;
constexpr uint32_t kGpioOutW1tc = 0x000C;
constexpr uint32_t kGpioOut1 = 0x0010;
constexpr uint32_t kGpioOut1W1ts = 0x0014;
constexpr uint32_t kGpioOut1W1tc = 0x0018;
constexpr uint32_t kGpioEnable = 0x0020;
constexpr uint32_t kGpioEnableW1ts = 0x0024;
constexpr uint32_t kGpioEnableW1tc = 0x0028;
constexpr uint32_t kGpioEnable1 = 0x002C;
constexpr uint32_t kGpioEnable1W1ts = 0x0030;
constexpr uint32_t kGpioEnable1W1tc = 0x0034;
constexpr uint32_t kGpioStrap = 0x0038;
constexpr uint32_t kGpioIn = 0x003C;
constexpr uint32_t kGpioIn1 = 0x0040;
constexpr uint32_t kGpioStatus = 0x0044;
constexpr uint32_t kGpioStatusW1ts = 0x0048;
constexpr uint32_t kGpioStatusW1tc = 0x004C;
constexpr uint32_t kGpioStatus1 = 0x0050;
constexpr uint32_t kGpioStatus1W1ts = 0x0054;
constexpr uint32_t kGpioStatus1W1tc = 0x0058;
// The interrupt status, masked by which CPU each pad's interrupt was enabled
// for. This is the register the driver's own ISR reads — not GPIO_STATUS — so
// a model that answers zero here is one where the handler finds nothing to do,
// returns, and is entered again immediately, forever.
constexpr uint32_t kGpioPcpuInt = 0x005C;
constexpr uint32_t kGpioPcpuNmiInt = 0x0060;
constexpr uint32_t kGpioCpuSdioInt = 0x0064;
constexpr uint32_t kGpioPcpuInt1 = 0x0068;
constexpr uint32_t kGpioPcpuNmiInt1 = 0x006C;
constexpr uint32_t kGpioCpuSdioInt1 = 0x0070;
constexpr uint32_t kGpioPin0 = 0x0074;
// GPIO_PINn_REG runs from 0x74 up to the chip's pad count — 22 pads on the C3,
// 48 on the S3 — and GPIO_FUNCn_IN_SEL_CFG starts at 0x154. Reading past the
// pads would take the signal-routing registers for pad configuration, and a
// routing word with the right bits set would arm an interrupt on a pad that
// does not exist.
constexpr int kGpioPinRegisters = 48;

// GPIO_PINn_REG: the trigger the pad's interrupt uses, and which CPUs it is
// delivered to.
constexpr uint32_t kPinIntTypeShift = 7;
constexpr uint32_t kPinIntEnaShift = 13;
// Within that field: which CPU the pad's interrupt is delivered to.
constexpr uint32_t kPinIntEnaCpu = 1u << 0;
constexpr uint32_t kPinIntEnaCpuNmi = 1u << 1;
constexpr uint32_t kPinIntEnaSdio = 1u << 4;

// IO_MUX: one register per pad, starting one word in (the block's own
// configuration register is at offset zero).
constexpr uint32_t kIoMuxPad0 = 0x0004;
constexpr uint32_t kIoMuxFunWpd = 1u << 7;
constexpr uint32_t kIoMuxFunWpu = 1u << 8;
constexpr uint32_t kIoMuxFunIe = 1u << 9;

// ── I2C register map ─────────────────────────────────────────────────────────
constexpr uint32_t kI2cCtr = 0x0004;
constexpr uint32_t kI2cSr = 0x0008;
constexpr uint32_t kI2cFifoSt = 0x0014;
constexpr uint32_t kI2cFifoConf = 0x0018;
constexpr uint32_t kI2cData = 0x001C;
constexpr uint32_t kI2cIntRaw = 0x0020;
constexpr uint32_t kI2cIntClr = 0x0024;
constexpr uint32_t kI2cIntEna = 0x0028;
constexpr uint32_t kI2cIntStatus = 0x002C;
constexpr uint32_t kI2cComd0 = 0x0058;
constexpr int kI2cCommands = 8;

constexpr uint32_t kI2cCtrTransStart = 1u << 5;
constexpr uint32_t kI2cFifoConfRxRst = 1u << 12;
constexpr uint32_t kI2cFifoConfTxRst = 1u << 13;

// I2C_INT_RAW bits.
constexpr uint32_t kI2cIntEndDetect = 1u << 3;
constexpr uint32_t kI2cIntByteTransDone = 1u << 4;
constexpr uint32_t kI2cIntTransComplete = 1u << 7;
constexpr uint32_t kI2cIntNack = 1u << 10;

// Command opcodes, as the C3 and S3 number them (the original ESP32 numbers
// RESTART and STOP differently, which is why these are not shared constants).
constexpr uint32_t kCmdWrite = 1;
constexpr uint32_t kCmdStop = 2;
constexpr uint32_t kCmdRead = 3;
constexpr uint32_t kCmdEnd = 4;
constexpr uint32_t kCmdRestart = 6;

// ── SPI register map ─────────────────────────────────────────────────────────
constexpr uint32_t kSpiCmd = 0x0000;
constexpr uint32_t kSpiAddr = 0x0004;
constexpr uint32_t kSpiUser = 0x0010;
constexpr uint32_t kSpiUser1 = 0x0014;
constexpr uint32_t kSpiUser2 = 0x0018;
constexpr uint32_t kSpiMsDlen = 0x001C;
constexpr uint32_t kSpiDmaConf = 0x0030;
constexpr uint32_t kSpiDmaIntEna = 0x0034;
constexpr uint32_t kSpiDmaIntClr = 0x0038;
constexpr uint32_t kSpiDmaIntRaw = 0x003C;
constexpr uint32_t kSpiDmaIntSt = 0x0040;
constexpr uint32_t kSpiW0 = 0x0098;
constexpr int kSpiWordCount = 18;

constexpr uint32_t kSpiCmdUsr = 1u << 24;
// Software sets this to ask the block to latch the configuration it has just
// written, and then spins until the hardware clears it. It takes a couple of
// bus clocks on the chip and no time at all here — but a bit that never clears
// is a driver that never sends its first byte.
constexpr uint32_t kSpiCmdUpdate = 1u << 23;
constexpr uint32_t kSpiUserCommand = 1u << 31;
constexpr uint32_t kSpiUserAddr = 1u << 30;
constexpr uint32_t kSpiUserMiso = 1u << 28;
constexpr uint32_t kSpiUserMosi = 1u << 27;
constexpr uint32_t kSpiIntTransDone = 1u << 12;
constexpr uint32_t kSpiDmaConfTxEna = 1u << 21;
constexpr uint32_t kSpiDmaConfRxEna = 1u << 20;

// ── GDMA register map ────────────────────────────────────────────────────────
constexpr uint32_t kGdmaIntRawCh0 = 0x0000;
constexpr uint32_t kGdmaIntStCh0 = 0x0004;
constexpr uint32_t kGdmaIntEnaCh0 = 0x0008;
constexpr uint32_t kGdmaIntClrCh0 = 0x000C;
constexpr uint32_t kGdmaIntStride = 0x0010;
constexpr uint32_t kGdmaChannelBase = 0x0070;
constexpr uint32_t kGdmaChannelStride = 0x00C0;
constexpr uint32_t kGdmaInLink = 0x0010;      // within the channel block
constexpr uint32_t kGdmaInSucEofDesc = 0x0018;
constexpr uint32_t kGdmaInPeriSel = 0x0030;
constexpr uint32_t kGdmaOutLink = 0x0070;
constexpr uint32_t kGdmaOutEofDesc = 0x0078;
constexpr uint32_t kGdmaOutPeriSel = 0x0090;
constexpr uint32_t kGdmaLinkAddrMask = 0x000FFFFF;
constexpr uint32_t kGdmaDescriptorBase = 0x3FC00000;

// GDMA_INT_RAW bits, per channel.
constexpr uint32_t kGdmaIntInDone = 1u << 0;
constexpr uint32_t kGdmaIntInSucEof = 1u << 1;
constexpr uint32_t kGdmaIntOutDone = 1u << 3;
constexpr uint32_t kGdmaIntOutEof = 1u << 4;

// A DMA descriptor: size, how much of it is valid, the end-of-list flag and
// who owns it, then the buffer and the next entry.
struct DmaDescriptor {
  uint32_t flags;
  uint32_t buffer;
  uint32_t next;

  uint32_t size() const { return flags & 0xFFF; }
  uint32_t length() const { return (flags >> 12) & 0xFFF; }
  bool eof() const { return (flags & (1u << 30)) != 0; }
  bool ownedByDma() const { return (flags & (1u << 31)) != 0; }
};

}  // namespace

// ── GPIO ─────────────────────────────────────────────────────────────────────

GpioDevice::GpioDevice(const SocDesc& soc, BoardBridge* board, InterruptMatrixDevice* matrix)
    : GenericPeripheral("GPIO", soc.gpio, 0x1000), soc_(soc), board_(board), matrix_(matrix) {
  lastInput_.fill(-1);
}

bool GpioDevice::outputEnabled(int pin) const {
  if (pin < 0 || pin >= kMaxPins) return false;
  return (enable_[pin / 32] >> (pin % 32)) & 1;
}

int GpioDevice::drivenLevel(int pin) const {
  if (!outputEnabled(pin)) return -1;
  return (out_[pin / 32] >> (pin % 32)) & 1;
}

uint32_t GpioDevice::maskedStatus(int word, uint32_t enableBit) const {
  uint32_t out = 0;
  for (int index = 0; index < 32; ++index) {
    const int pin = word * 32 + index;
    if (pin >= kMaxPins) break;
    if (!(status_[word] & (1u << index))) continue;
    if (((pinConf_[pin] >> kPinIntEnaShift) & 0x1F) & enableBit) out |= 1u << index;
  }
  return out;
}

int GpioDevice::inputLevel(int pin, bool observed) {
  // What the board is doing to the pad wins: an external driver — a button
  // pulling to ground, a controller's open-drain IRQ, the panel's BUSY —
  // overrides a pull, exactly as it does on copper.
  if (board_) {
    const int external = board_->pinRead(pin, observed);
    if (external >= 0) return external;
  }
  const int driven = drivenLevel(pin);
  if (driven >= 0) return driven;
  // Nothing is driving it, so the pad sits at its pull. Without a bridge to
  // ask, an undriven input reads high, which is what an unconnected pin with
  // the reset-default pull-up does.
  return 1;
}

void GpioDevice::pushOutput(int pin) {
  if (!board_) return;
  const int level = drivenLevel(pin);
  if (level >= 0) board_->pinWrite(pin, level);
}

void GpioDevice::noteEdge(int pin, int from, int to) {
  if (from == to || from < 0) return;
  const uint32_t conf = pinConf_[pin];
  const uint32_t type = (conf >> kPinIntTypeShift) & 0x7;
  const uint32_t enable = (conf >> kPinIntEnaShift) & 0x1F;
  if (!enable) return;

  bool fire = false;
  switch (type) {
    case 1: fire = to == 1; break;              // rising
    case 2: fire = to == 0; break;              // falling
    case 3: fire = true; break;                 // either edge
    case 4: fire = to == 0; break;              // low level
    case 5: fire = to == 1; break;              // high level
    default: break;
  }
  if (!fire) return;
  status_[pin / 32] |= 1u << (pin % 32);
  if (matrix_) matrix_->setPeripheralInterrupt(static_cast<int>(soc_.gpioIntSource), true);
}

void GpioDevice::pollInputs() {
  for (int pin = 0; pin < kMaxPins; ++pin) {
    if (outputEnabled(pin)) continue;
    const int level = inputLevel(pin, false);
    const int previous = lastInput_[pin];
    lastInput_[pin] = level;
    noteEdge(pin, previous, level);
  }
}

uint32_t GpioDevice::read(uint32_t offset) {
  switch (offset) {
    case kGpioOut: return out_[0];
    case kGpioOut1: return out_[1];
    case kGpioEnable: return enable_[0];
    case kGpioEnable1: return enable_[1];
    case kGpioStatus: return status_[0];
    case kGpioStatus1: return status_[1];
    case kGpioPcpuInt: return maskedStatus(0, kPinIntEnaCpu);
    case kGpioPcpuInt1: return maskedStatus(1, kPinIntEnaCpu);
    case kGpioPcpuNmiInt: return maskedStatus(0, kPinIntEnaCpuNmi);
    case kGpioPcpuNmiInt1: return maskedStatus(1, kPinIntEnaCpuNmi);
    case kGpioCpuSdioInt: return maskedStatus(0, kPinIntEnaSdio);
    case kGpioCpuSdioInt1: return maskedStatus(1, kPinIntEnaSdio);
    // The strapping pins as a normal boot leaves them: booting from flash with
    // the download pins released.
    case kGpioStrap: return 0x08;
    case kGpioIn:
    case kGpioIn1: {
      const int first = offset == kGpioIn ? 0 : 32;
      uint32_t value = 0;
      for (int index = 0; index < 32; ++index) {
        const int pin = first + index;
        if (pin >= kMaxPins) break;
        const int level = inputLevel(pin, true);
        lastInput_[pin] = level;
        if (level) value |= 1u << index;
      }
      return value;
    }
    default: break;
  }
  return GenericPeripheral::read(offset);
}

void GpioDevice::write(uint32_t offset, uint32_t value, uint32_t mask) {
  GenericPeripheral::write(offset, value, mask);
  const uint32_t masked = value & mask;

  auto applyWord = [&](int word, uint32_t before) {
    for (int index = 0; index < 32; ++index) {
      const int pin = word * 32 + index;
      if (pin >= kMaxPins) break;
      const uint32_t bit = 1u << index;
      if (((before ^ out_[word]) & bit) || ((enable_[word] & bit) && !(before & bit))) pushOutput(pin);
    }
  };
  auto applyDirection = [&](int word, uint32_t before) {
    if (!board_) return;
    for (int index = 0; index < 32; ++index) {
      const int pin = word * 32 + index;
      if (pin >= kMaxPins) break;
      const uint32_t bit = 1u << index;
      if ((before ^ enable_[word]) & bit) board_->pinDirection(pin, (enable_[word] & bit) != 0);
    }
  };

  switch (offset) {
    case kGpioOut:
    case kGpioOut1: {
      const int word = offset == kGpioOut ? 0 : 1;
      const uint32_t before = out_[word];
      out_[word] = (out_[word] & ~mask) | masked;
      applyWord(word, before);
      return;
    }
    case kGpioOutW1ts:
    case kGpioOut1W1ts: {
      const int word = offset == kGpioOutW1ts ? 0 : 1;
      const uint32_t before = out_[word];
      out_[word] |= masked;
      applyWord(word, before);
      return;
    }
    case kGpioOutW1tc:
    case kGpioOut1W1tc: {
      const int word = offset == kGpioOutW1tc ? 0 : 1;
      const uint32_t before = out_[word];
      out_[word] &= ~masked;
      applyWord(word, before);
      return;
    }
    case kGpioEnable:
    case kGpioEnable1: {
      const int word = offset == kGpioEnable ? 0 : 1;
      const uint32_t before = enable_[word];
      enable_[word] = (enable_[word] & ~mask) | masked;
      applyDirection(word, before);
      applyWord(word, out_[word]);
      return;
    }
    case kGpioEnableW1ts:
    case kGpioEnable1W1ts: {
      const int word = offset == kGpioEnableW1ts ? 0 : 1;
      const uint32_t before = enable_[word];
      enable_[word] |= masked;
      applyDirection(word, before);
      applyWord(word, out_[word]);
      return;
    }
    case kGpioEnableW1tc:
    case kGpioEnable1W1tc: {
      const int word = offset == kGpioEnableW1tc ? 0 : 1;
      const uint32_t before = enable_[word];
      enable_[word] &= ~masked;
      applyDirection(word, before);
      return;
    }
    case kGpioStatus:
    case kGpioStatus1: {
      const int word = offset == kGpioStatus ? 0 : 1;
      status_[word] = (status_[word] & ~mask) | masked;
      break;
    }
    case kGpioStatusW1ts:
    case kGpioStatus1W1ts: {
      status_[offset == kGpioStatusW1ts ? 0 : 1] |= masked;
      break;
    }
    case kGpioStatusW1tc:
    case kGpioStatus1W1tc: {
      status_[offset == kGpioStatusW1tc ? 0 : 1] &= ~masked;
      break;
    }
    default:
      if (offset >= kGpioPin0 && offset < kGpioPin0 + kGpioPinRegisters * 4) {
        const int pin = static_cast<int>((offset - kGpioPin0) / 4);
        pinConf_[pin] = (pinConf_[pin] & ~mask) | masked;
      }
      return;
  }

  if (matrix_ && !status_[0] && !status_[1]) {
    matrix_->setPeripheralInterrupt(static_cast<int>(soc_.gpioIntSource), false);
  }
}

// ── IO_MUX ───────────────────────────────────────────────────────────────────

IoMuxDevice::IoMuxDevice(const SocDesc& soc, BoardBridge* board)
    : GenericPeripheral("IO_MUX", soc.ioMux, 0x1000), soc_(soc), board_(board) {}

uint32_t IoMuxDevice::padConf(int pin) const {
  const auto it = registers_.find(kIoMuxPad0 + static_cast<uint32_t>(pin) * 4);
  return it == registers_.end() ? 0 : it->second;
}

bool IoMuxDevice::pullUp(int pin) const { return (padConf(pin) & kIoMuxFunWpu) != 0; }
bool IoMuxDevice::pullDown(int pin) const { return (padConf(pin) & kIoMuxFunWpd) != 0; }
bool IoMuxDevice::inputEnabled(int pin) const { return (padConf(pin) & kIoMuxFunIe) != 0; }

void IoMuxDevice::write(uint32_t offset, uint32_t value, uint32_t mask) {
  GenericPeripheral::write(offset, value, mask);
  if (offset < kIoMuxPad0) return;
  const uint32_t index = (offset - kIoMuxPad0) / 4;
  if (index >= 64) return;
  if (!board_) return;
  const uint32_t conf = padConf(static_cast<int>(index));
  board_->pinPull(static_cast<int>(index), (conf & kIoMuxFunWpu) != 0, (conf & kIoMuxFunWpd) != 0);
  (void)soc_;
}

// ── I2C master ───────────────────────────────────────────────────────────────

I2cMasterDevice::I2cMasterDevice(std::string name, uint32_t base, int bus, BoardBridge* board,
                                 InterruptMatrixDevice* matrix, int interruptSource)
    : GenericPeripheral(std::move(name), base, 0x1000),
      bus_(bus),
      board_(board),
      matrix_(matrix),
      source_(interruptSource) {}

void I2cMasterDevice::raise(uint32_t bits) {
  intRaw_ |= bits;
  refreshInterrupt();
}

void I2cMasterDevice::refreshInterrupt() {
  if (!matrix_) return;
  matrix_->setPeripheralInterrupt(source_, (intRaw_ & intEna_) != 0);
}

void I2cMasterDevice::runCommandList() {
  // Walk the list the driver built. The address rides in the first byte of the
  // first write after a start, exactly as it does on the wire.
  size_t txCursor = 0;
  size_t readWanted = 0;
  bool sawStop = false;
  bool sawEnd = false;
  bool expectAddress = false;
  bool reading = false;

  for (int index = 0; index < kI2cCommands; ++index) {
    const uint32_t offset = kI2cComd0 + static_cast<uint32_t>(index) * 4;
    const auto it = registers_.find(offset);
    if (it == registers_.end()) break;
    const uint32_t command = it->second;
    const uint32_t op = (command >> 11) & 0x7;
    const uint32_t bytes = command & 0xFF;

    if (op == kCmdRestart) {
      expectAddress = true;
      continue;
    }
    if (op == kCmdWrite) {
      for (uint32_t byte = 0; byte < bytes && txCursor < txFifo_.size(); ++byte, ++txCursor) {
        const uint8_t value = txFifo_[txCursor];
        if (expectAddress) {
          const uint8_t addr = value >> 1;
          // A repeated start onto a *different* address is a new transaction,
          // not a continuation of this one.
          if (!writePhase_.empty() && addr != address_) writePhase_.clear();
          address_ = addr;
          reading = (value & 1) != 0;
          expectAddress = false;
          continue;
        }
        writePhase_.push_back(value);
      }
      continue;
    }
    if (op == kCmdRead) {
      readWanted += bytes;
      reading = true;
      continue;
    }
    if (op == kCmdStop) {
      sawStop = true;
      break;
    }
    if (op == kCmdEnd) {
      sawEnd = true;
      break;
    }
    break;
  }
  (void)reading;

  txFifo_.erase(txFifo_.begin(), txFifo_.begin() + static_cast<long>(std::min(txCursor, txFifo_.size())));

  bool acknowledged = true;
  if (readWanted > 0) {
    if (pendingRead_.empty()) {
      std::vector<uint8_t> answer(readWanted);
      acknowledged = board_ && board_->i2cTransfer(bus_, address_, writePhase_.data(),
                                                   writePhase_.size(), answer.data(), answer.size());
      if (acknowledged) pendingRead_ = std::move(answer);
      writePhase_.clear();
    }
    const size_t take = std::min(readWanted, pendingRead_.size());
    rxFifo_.insert(rxFifo_.end(), pendingRead_.begin(), pendingRead_.begin() + static_cast<long>(take));
    pendingRead_.erase(pendingRead_.begin(), pendingRead_.begin() + static_cast<long>(take));
    // A read the board could not fill is still clocked on the wire; the bus
    // idles high, which is what the master latches.
    for (size_t index = take; index < readWanted; ++index) rxFifo_.push_back(0xFF);
  } else if (sawStop || sawEnd) {
    // Write-only, or a bare address probe. Either way the answer the firmware
    // is after is whether anything acknowledged.
    acknowledged = board_ && board_->i2cTransfer(bus_, address_, writePhase_.data(),
                                                 writePhase_.size(), nullptr, 0);
    writePhase_.clear();
  }

  if (sawStop) {
    pendingRead_.clear();
    writePhase_.clear();
  }

  // Mark every command consumed, as the hardware does.
  for (int index = 0; index < kI2cCommands; ++index) {
    const uint32_t offset = kI2cComd0 + static_cast<uint32_t>(index) * 4;
    auto it = registers_.find(offset);
    if (it != registers_.end()) it->second |= 1u << 31;
  }

  registers_[kI2cCtr] &= ~kI2cCtrTransStart;
  raise(kI2cIntByteTransDone);
  if (!acknowledged) raise(kI2cIntNack);
  if (sawEnd) raise(kI2cIntEndDetect);
  if (sawStop || !sawEnd) raise(kI2cIntTransComplete);
}

uint32_t I2cMasterDevice::read(uint32_t offset) {
  switch (offset) {
    case kI2cData: {
      if (rxRead_ < rxFifo_.size()) return rxFifo_[rxRead_++];
      return 0xFF;
    }
    case kI2cIntRaw: return intRaw_;
    case kI2cIntStatus: return intRaw_ & intEna_;
    case kI2cIntEna: return intEna_;
    case kI2cSr: {
      const uint32_t rxCount = static_cast<uint32_t>(rxFifo_.size() - rxRead_) & 0x3F;
      const uint32_t txCount = static_cast<uint32_t>(txFifo_.size()) & 0x3F;
      // Bit 0 reports the last byte's acknowledgement; the bus is never busy
      // here because a transaction completes within the write that starts it.
      return (rxCount << 8) | (txCount << 18);
    }
    case kI2cFifoSt: {
      const uint32_t rxCount = static_cast<uint32_t>(rxFifo_.size() - rxRead_) & 0x1F;
      return rxCount << 5;
    }
    default: break;
  }
  return GenericPeripheral::read(offset);
}

void I2cMasterDevice::write(uint32_t offset, uint32_t value, uint32_t mask) {
  const uint32_t masked = value & mask;
  switch (offset) {
    case kI2cData:
      txFifo_.push_back(static_cast<uint8_t>(masked & 0xFF));
      GenericPeripheral::write(offset, value, mask);
      return;
    case kI2cIntClr:
      intRaw_ &= ~masked;
      refreshInterrupt();
      return;
    case kI2cIntEna:
      intEna_ = masked;
      refreshInterrupt();
      return;
    case kI2cIntRaw:
      return;
    case kI2cFifoConf:
      if (masked & kI2cFifoConfTxRst) txFifo_.clear();
      if (masked & kI2cFifoConfRxRst) {
        rxFifo_.clear();
        rxRead_ = 0;
      }
      // The reset bits are level-held by the driver and cleared again; store
      // them with the resets stripped so a read-back does not re-trigger.
      GenericPeripheral::write(offset, value & ~(kI2cFifoConfTxRst | kI2cFifoConfRxRst), mask);
      return;
    case kI2cCtr: {
      GenericPeripheral::write(offset, value, mask);
      if (masked & kI2cCtrTransStart) {
        rxFifo_.clear();
        rxRead_ = 0;
        runCommandList();
      }
      return;
    }
    default: break;
  }
  GenericPeripheral::write(offset, value, mask);
}

// ── GDMA ─────────────────────────────────────────────────────────────────────

GdmaDevice::GdmaDevice(const SocDesc& soc, Bus& bus, InterruptMatrixDevice* matrix)
    : GenericPeripheral("GDMA", soc.gdma, 0x1000), soc_(soc), bus_(bus), matrix_(matrix) {
  channels_.resize(soc.dmaChannels ? soc.dmaChannels : 1);
}

uint32_t GdmaDevice::descriptorAddress(uint32_t link) const {
  const uint32_t addr = link & kGdmaLinkAddrMask;
  return addr ? (kGdmaDescriptorBase | addr) : 0;
}

void GdmaDevice::refreshInterrupt(int channel) {
  if (!matrix_ || channel < 0 || channel >= static_cast<int>(channels_.size())) return;
  const Channel& state = channels_[channel];
  const bool active = (state.intRaw & state.intEna) != 0;
  // The C3 gives a channel one slot for both directions; the S3 splits them,
  // and a driver may have allocated either. Raising both is harmless — an
  // unrouted source goes nowhere — and covers both parts with one rule.
  matrix_->setPeripheralInterrupt(static_cast<int>(soc_.dmaInIntSource) + channel, active);
  if (soc_.dmaOutIntSource != soc_.dmaInIntSource) {
    matrix_->setPeripheralInterrupt(static_cast<int>(soc_.dmaOutIntSource) + channel, active);
  }
}

int GdmaDevice::channelForTx(int peripheral) const {
  for (size_t index = 0; index < channels_.size(); ++index) {
    if ((channels_[index].outPeri & 0x3F) == static_cast<uint32_t>(peripheral)) return static_cast<int>(index);
  }
  return -1;
}

int GdmaDevice::channelForRx(int peripheral) const {
  for (size_t index = 0; index < channels_.size(); ++index) {
    if ((channels_[index].inPeri & 0x3F) == static_cast<uint32_t>(peripheral)) return static_cast<int>(index);
  }
  return -1;
}

std::vector<uint8_t> GdmaDevice::gather(int channel, size_t limit) {
  std::vector<uint8_t> out;
  if (channel < 0 || channel >= static_cast<int>(channels_.size())) return out;
  uint32_t address = descriptorAddress(channels_[channel].outLink);
  // A malformed chain must not spin the emulator; a real one is a handful of
  // entries per transfer.
  for (int guard = 0; address && guard < 4096 && out.size() < limit; ++guard) {
    DmaDescriptor descriptor{};
    descriptor.flags = bus_.read32(address);
    descriptor.buffer = bus_.read32(address + 4);
    descriptor.next = bus_.read32(address + 8);
    if (bus_.faulted()) break;

    const uint32_t length = std::min<uint32_t>(descriptor.length(), descriptor.size());
    const uint8_t* host = bus_.hostRead(descriptor.buffer, length);
    if (host) {
      const size_t take = std::min<size_t>(length, limit - out.size());
      out.insert(out.end(), host, host + take);
    } else {
      for (uint32_t index = 0; index < length && out.size() < limit; ++index) {
        out.push_back(bus_.read8(descriptor.buffer + index));
      }
    }
    // The hardware hands each entry back as it finishes with it.
    bus_.write32(address, descriptor.flags & ~(1u << 31));
    channels_[channel].outEofDesc = address;
    if (descriptor.eof()) break;
    address = descriptor.next;
  }
  return out;
}

size_t GdmaDevice::scatter(int channel, const uint8_t* data, size_t length) {
  if (channel < 0 || channel >= static_cast<int>(channels_.size())) return 0;
  uint32_t address = descriptorAddress(channels_[channel].inLink);
  size_t written = 0;
  for (int guard = 0; address && guard < 4096 && written < length; ++guard) {
    DmaDescriptor descriptor{};
    descriptor.flags = bus_.read32(address);
    descriptor.buffer = bus_.read32(address + 4);
    descriptor.next = bus_.read32(address + 8);
    if (bus_.faulted()) break;

    const size_t room = std::min<size_t>(descriptor.size(), length - written);
    uint8_t* host = bus_.hostWrite(descriptor.buffer, static_cast<uint32_t>(room));
    if (host) {
      std::memcpy(host, data + written, room);
    } else {
      for (size_t index = 0; index < room; ++index) bus_.write8(descriptor.buffer + index, data[written + index]);
    }
    written += room;
    // Report back how much landed, release the entry, and mark the last one.
    uint32_t flags = (descriptor.flags & ~(0xFFFu << 12)) & ~(1u << 31);
    flags |= (static_cast<uint32_t>(room) & 0xFFF) << 12;
    if (written >= length) flags |= 1u << 30;
    bus_.write32(address, flags);
    channels_[channel].inEofDesc = address;
    if (descriptor.eof()) break;
    address = descriptor.next;
  }
  return written;
}

void GdmaDevice::signalTxEof(int channel) {
  if (channel < 0 || channel >= static_cast<int>(channels_.size())) return;
  channels_[channel].intRaw |= kGdmaIntOutEof | kGdmaIntOutDone;
  refreshInterrupt(channel);
}

void GdmaDevice::signalRxEof(int channel) {
  if (channel < 0 || channel >= static_cast<int>(channels_.size())) return;
  channels_[channel].intRaw |= kGdmaIntInSucEof | kGdmaIntInDone;
  refreshInterrupt(channel);
}

uint32_t GdmaDevice::read(uint32_t offset) {
  for (size_t index = 0; index < channels_.size(); ++index) {
    const uint32_t base = kGdmaIntRawCh0 + static_cast<uint32_t>(index) * kGdmaIntStride;
    if (offset == base) return channels_[index].intRaw;
    if (offset == base + (kGdmaIntStCh0 - kGdmaIntRawCh0)) return channels_[index].intRaw & channels_[index].intEna;
    if (offset == base + (kGdmaIntEnaCh0 - kGdmaIntRawCh0)) return channels_[index].intEna;

    const uint32_t channel = kGdmaChannelBase + static_cast<uint32_t>(index) * kGdmaChannelStride;
    if (offset == channel + kGdmaInSucEofDesc) return channels_[index].inEofDesc;
    if (offset == channel + kGdmaOutEofDesc) return channels_[index].outEofDesc;
  }
  return GenericPeripheral::read(offset);
}

void GdmaDevice::write(uint32_t offset, uint32_t value, uint32_t mask) {
  GenericPeripheral::write(offset, value, mask);
  const uint32_t masked = value & mask;
  for (size_t index = 0; index < channels_.size(); ++index) {
    const uint32_t base = kGdmaIntRawCh0 + static_cast<uint32_t>(index) * kGdmaIntStride;
    if (offset == base + (kGdmaIntEnaCh0 - kGdmaIntRawCh0)) {
      channels_[index].intEna = masked;
      refreshInterrupt(static_cast<int>(index));
      return;
    }
    if (offset == base + (kGdmaIntClrCh0 - kGdmaIntRawCh0)) {
      channels_[index].intRaw &= ~masked;
      refreshInterrupt(static_cast<int>(index));
      return;
    }

    const uint32_t channel = kGdmaChannelBase + static_cast<uint32_t>(index) * kGdmaChannelStride;
    if (offset == channel + kGdmaInLink) {
      channels_[index].inLink = masked;
      return;
    }
    if (offset == channel + kGdmaOutLink) {
      channels_[index].outLink = masked;
      return;
    }
    if (offset == channel + kGdmaInPeriSel) {
      channels_[index].inPeri = masked;
      return;
    }
    if (offset == channel + kGdmaOutPeriSel) {
      channels_[index].outPeri = masked;
      return;
    }
  }
}

// ── SPI master ───────────────────────────────────────────────────────────────

SpiMasterDevice::SpiMasterDevice(std::string name, uint32_t base, int bus, int dmaPeripheral,
                                 BoardBridge* board, GdmaDevice* dma, InterruptMatrixDevice* matrix,
                                 int interruptSource)
    : GenericPeripheral(std::move(name), base, 0x1000),
      bus_(bus),
      dmaPeripheral_(dmaPeripheral),
      board_(board),
      dma_(dma),
      matrix_(matrix),
      source_(interruptSource) {}

void SpiMasterDevice::runTransaction() {
  const uint32_t user = registers_[kSpiUser];
  const uint32_t dmaConf = registers_[kSpiDmaConf];
  const uint32_t dataBits = (registers_[kSpiMsDlen] & 0x3FFFF) + 1;
  const size_t dataBytes = (dataBits + 7) / 8;

  std::vector<uint8_t> tx;

  // The command and address phases go out ahead of the data, most significant
  // byte first, which is how they appear on the wire.
  if (user & kSpiUserCommand) {
    const uint32_t user2 = registers_[kSpiUser2];
    const uint32_t bits = ((user2 >> 28) & 0xF) + 1;
    const uint32_t command = user2 & 0xFFFF;
    for (int bit = static_cast<int>((bits + 7) / 8) - 1; bit >= 0; --bit) {
      tx.push_back(static_cast<uint8_t>((command >> (bit * 8)) & 0xFF));
    }
  }
  if (user & kSpiUserAddr) {
    const uint32_t bits = ((registers_[kSpiUser1] >> 27) & 0x1F) + 1;
    const uint32_t address = registers_[kSpiAddr];
    for (int bit = static_cast<int>((bits + 7) / 8) - 1; bit >= 0; --bit) {
      tx.push_back(static_cast<uint8_t>((address >> (bit * 8)) & 0xFF));
    }
  }

  const bool useDma = (dmaConf & (kSpiDmaConfTxEna | kSpiDmaConfRxEna)) != 0;
  const int txChannel = useDma && dma_ ? dma_->channelForTx(dmaPeripheral_) : -1;
  const int rxChannel = useDma && dma_ ? dma_->channelForRx(dmaPeripheral_) : -1;

  if (user & kSpiUserMosi) {
    if ((dmaConf & kSpiDmaConfTxEna) && txChannel >= 0) {
      const std::vector<uint8_t> payload = dma_->gather(txChannel, dataBytes);
      tx.insert(tx.end(), payload.begin(), payload.end());
      // A chain shorter than the programmed length still clocks out the full
      // count; the shortfall is whatever the FIFO held, which is zero here.
      tx.resize(tx.size() + (dataBytes - std::min(dataBytes, payload.size())), 0);
    } else {
      for (size_t index = 0; index < dataBytes && index < kSpiWordCount * 4; ++index) {
        const uint32_t word = registers_[kSpiW0 + static_cast<uint32_t>(index / 4) * 4];
        tx.push_back(static_cast<uint8_t>((word >> ((index % 4) * 8)) & 0xFF));
      }
    }
  }

  const bool wantsRead = (user & kSpiUserMiso) != 0;
  std::vector<uint8_t> rx(wantsRead ? std::max(tx.size(), dataBytes) : 0, 0xFF);
  if (tx.empty() && wantsRead) tx.assign(dataBytes, 0xFF);

  if (board_ && !tx.empty()) {
    board_->spiTransfer(bus_, tx.data(), wantsRead ? rx.data() : nullptr, tx.size());
  }

  if (wantsRead) {
    // The read phase lands after whatever the command and address phases
    // consumed — those clock out while the far end is still being addressed.
    const size_t skip = tx.size() > dataBytes ? tx.size() - dataBytes : 0;
    const uint8_t* payload = rx.data() + skip;
    const size_t length = std::min(dataBytes, rx.size() - skip);
    if ((dmaConf & kSpiDmaConfRxEna) && rxChannel >= 0) {
      dma_->scatter(rxChannel, payload, length);
      dma_->signalRxEof(rxChannel);
    } else {
      for (size_t index = 0; index < length && index < kSpiWordCount * 4; ++index) {
        const uint32_t reg = kSpiW0 + static_cast<uint32_t>(index / 4) * 4;
        const uint32_t shift = (index % 4) * 8;
        registers_[reg] = (registers_[reg] & ~(0xFFu << shift)) | (static_cast<uint32_t>(payload[index]) << shift);
      }
    }
  }
  if ((dmaConf & kSpiDmaConfTxEna) && txChannel >= 0) dma_->signalTxEof(txChannel);

  registers_[kSpiCmd] &= ~kSpiCmdUsr;
  intRaw_ |= kSpiIntTransDone;
  if (matrix_) matrix_->setPeripheralInterrupt(source_, (intRaw_ & intEna_) != 0);
}

uint32_t SpiMasterDevice::read(uint32_t offset) {
  switch (offset) {
    case kSpiDmaIntRaw: return intRaw_;
    case kSpiDmaIntSt: return intRaw_ & intEna_;
    case kSpiDmaIntEna: return intEna_;
    default: break;
  }
  return GenericPeripheral::read(offset);
}

void SpiMasterDevice::write(uint32_t offset, uint32_t value, uint32_t mask) {
  const uint32_t masked = value & mask;
  switch (offset) {
    case kSpiDmaIntEna:
      intEna_ = masked;
      if (matrix_) matrix_->setPeripheralInterrupt(source_, (intRaw_ & intEna_) != 0);
      return;
    case kSpiDmaIntClr:
      intRaw_ &= ~masked;
      if (matrix_) matrix_->setPeripheralInterrupt(source_, (intRaw_ & intEna_) != 0);
      return;
    default: break;
  }
  GenericPeripheral::write(offset, value, mask);
  if (offset != kSpiCmd) return;
  if (masked & kSpiCmdUpdate) registers_[kSpiCmd] &= ~kSpiCmdUpdate;
  if (masked & kSpiCmdUsr) runTransaction();
}


// ── SDMMC host ───────────────────────────────────────────────────────────────

namespace {

// The DesignWare register map, as the S3 lays it out.
constexpr uint32_t kSdCtrl = 0x00;
constexpr uint32_t kSdClkDiv = 0x08;
constexpr uint32_t kSdClkEna = 0x10;
constexpr uint32_t kSdBlkSiz = 0x1C;
constexpr uint32_t kSdByteCnt = 0x20;
constexpr uint32_t kSdIntMask = 0x24;
constexpr uint32_t kSdCmdArg = 0x28;
constexpr uint32_t kSdCmd = 0x2C;
constexpr uint32_t kSdResp0 = 0x30;
constexpr uint32_t kSdMaskedInts = 0x40;
constexpr uint32_t kSdRawInts = 0x44;
constexpr uint32_t kSdStatus = 0x48;
constexpr uint32_t kSdCardDetect = 0x50;
constexpr uint32_t kSdWriteProtect = 0x54;
constexpr uint32_t kSdHwConfig = 0x70;
constexpr uint32_t kSdBusMode = 0x80;
constexpr uint32_t kSdPollDemand = 0x84;
constexpr uint32_t kSdDescBase = 0x88;
constexpr uint32_t kSdDmaStatus = 0x8C;
constexpr uint32_t kSdDmaIntEna = 0x90;

// CMD register.
constexpr uint32_t kSdCmdStart = 1u << 31;
constexpr uint32_t kSdCmdUpdateClock = 1u << 21;
constexpr uint32_t kSdCmdDataExpected = 1u << 9;
constexpr uint32_t kSdCmdWrite = 1u << 10;
constexpr uint32_t kSdCmdLongResponse = 1u << 7;
constexpr uint32_t kSdCmdResponseExpected = 1u << 6;
constexpr uint32_t kSdCmdIndexMask = 0x3F;

// RINTSTS.
constexpr uint32_t kSdIntCmdDone = 1u << 2;
constexpr uint32_t kSdIntDataOver = 1u << 3;
constexpr uint32_t kSdIntResponseTimeout = 1u << 8;

// IDSTS: the transmit and receive completions, and their summary bit.
constexpr uint32_t kSdDmaTx = 1u << 0;
constexpr uint32_t kSdDmaRx = 1u << 1;
constexpr uint32_t kSdDmaNormal = 1u << 8;

// One internal-DMA descriptor: flags, the two buffer sizes, and the pointers.
constexpr uint32_t kSdDescOwned = 1u << 31;
constexpr uint32_t kSdDescLast = 1u << 2;
constexpr uint32_t kSdDescChained = 1u << 4;
constexpr uint32_t kSdDescEndOfRing = 1u << 5;

}  // namespace

SdmmcHostDevice::SdmmcHostDevice(uint32_t base, Bus& bus, BoardBridge* board,
                                 InterruptMatrixDevice* matrix, int interruptSource)
    : GenericPeripheral("SDMMC", base, 0x1000),
      bus_(bus),
      board_(board),
      matrix_(matrix),
      source_(interruptSource) {}

void SdmmcHostDevice::raise(uint32_t bits) {
  intRaw_ |= bits;
  if (matrix_) matrix_->setPeripheralInterrupt(source_, (intRaw_ & intMask_) != 0);
}

size_t SdmmcHostDevice::transferThroughDma(bool toCard, uint32_t lba, uint32_t length) {
  // The payload is assembled whole and then moved, rather than descriptor by
  // descriptor: a descriptor's buffer is whatever size the driver chose, and
  // the medium is addressed in 512-byte blocks, so the two boundaries do not
  // line up and walking them together is how you write past the end of a
  // buffer that happens to be eight bytes long.
  std::vector<uint8_t> payload;

  if (!toCard) {
    // Some commands answer with a block of the card's own data — its
    // configuration register, its status, a function-switch result — and some
    // read the medium. The card offers the former; anything left is the latter.
    payload.assign(length, 0);
    const size_t fromCard = board_ ? board_->sdCardData(payload.data(), payload.size()) : 0;
    if (fromCard == 0 && board_) {
      const uint32_t blocks = (length + 511) / 512;
      std::vector<uint8_t> blockBuffer(static_cast<size_t>(blocks) * 512, 0);
      board_->sdReadBlocks(lba, blocks, blockBuffer.data());
      std::memcpy(payload.data(), blockBuffer.data(), length);
    }
  }

  uint32_t address = descriptorBase_;
  size_t moved = 0;
  for (int guard = 0; address && guard < 4096 && moved < length; ++guard) {
    const uint32_t flags = bus_.read32(address);
    const uint32_t sizes = bus_.read32(address + 4);
    const uint32_t buffer = bus_.read32(address + 8);
    const uint32_t next = bus_.read32(address + 12);
    if (bus_.faulted()) break;
    if (!(flags & kSdDescOwned)) break;

    const uint32_t room = std::min<uint32_t>(sizes & 0x1FFF, length - static_cast<uint32_t>(moved));
    if (room) {
      if (toCard) {
        payload.resize(moved + room);
        const uint8_t* host = bus_.hostRead(buffer, room);
        if (host) {
          std::memcpy(payload.data() + moved, host, room);
        } else {
          for (uint32_t index = 0; index < room; ++index) {
            payload[moved + index] = bus_.read8(buffer + index);
          }
        }
      } else {
        uint8_t* host = bus_.hostWrite(buffer, room);
        if (host) {
          std::memcpy(host, payload.data() + moved, room);
        } else {
          for (uint32_t index = 0; index < room; ++index) {
            bus_.write8(buffer + index, payload[moved + index]);
          }
        }
      }
      moved += room;
    }

    // Hand the descriptor back, as the hardware does when it is finished with it.
    bus_.write32(address, flags & ~kSdDescOwned);
    if (flags & kSdDescLast) break;
    address = (flags & (kSdDescChained | kSdDescEndOfRing)) ? next : address + 16;
  }

  if (toCard && board_ && moved) {
    // Whole blocks, padded: the medium has no finer unit than a block.
    const uint32_t blocks = static_cast<uint32_t>((moved + 511) / 512);
    payload.resize(static_cast<size_t>(blocks) * 512, 0);
    board_->sdWriteBlocks(lba, blocks, payload.data());
  }
  return moved;
}

void SdmmcHostDevice::runCommand(uint32_t command) {
  // "Update the clock registers" is not a card command at all: the driver uses
  // it to make a clock change take effect, and waits for the same done bit.
  if (command & kSdCmdUpdateClock) {
    registers_[kSdCmd] &= ~kSdCmdStart;
    raise(kSdIntCmdDone);
    return;
  }

  const int index = static_cast<int>(command & kSdCmdIndexMask);
  const uint32_t argument = registers_[kSdCmdArg];
  const bool longResponse = (command & kSdCmdLongResponse) != 0;

  uint32_t response[4] = {0, 0, 0, 0};
  const bool answered = board_ && board_->sdCommand(index, argument, longResponse, response);
  if (command & kSdCmdResponseExpected) {
    std::memcpy(response_, response, sizeof(response_));
  } else {
    std::memset(response_, 0, sizeof(response_));
  }

  registers_[kSdCmd] &= ~kSdCmdStart;
  if (!answered) {
    // Nothing in the slot, or a command this card does not implement. A
    // response timeout is what the host reports, and what the driver's own
    // "no card" path is written against.
    raise(kSdIntCmdDone | kSdIntResponseTimeout);
    return;
  }
  raise(kSdIntCmdDone);

  if (!(command & kSdCmdDataExpected)) return;

  // A data command's address is its argument: a block number on a
  // high-capacity card.
  // A byte count the driver has not set, or one that is obvious nonsense, is
  // not something to allocate against: a real host would transfer what the
  // card gave it and stop.
  constexpr uint32_t kMaxTransfer = 1u << 20;
  const uint32_t length = std::min(registers_[kSdByteCnt], kMaxTransfer);
  const bool toCard = (command & kSdCmdWrite) != 0;
  blockAddress_ = argument;
  reading_ = !toCard;
  transferThroughDma(toCard, blockAddress_, length);

  dmaStatus_ |= (toCard ? kSdDmaTx : kSdDmaRx) | kSdDmaNormal;
  raise(kSdIntDataOver);
}

uint32_t SdmmcHostDevice::read(uint32_t offset) {
  switch (offset) {
    case kSdRawInts: return intRaw_;
    case kSdMaskedInts: return intRaw_ & intMask_;
    case kSdIntMask: return intMask_;
    case kSdDmaStatus: return dmaStatus_;
    case kSdDmaIntEna: return dmaMask_;
    case kSdResp0: return response_[0];
    case kSdResp0 + 4: return response_[1];
    case kSdResp0 + 8: return response_[2];
    case kSdResp0 + 12: return response_[3];
    case kSdStatus:
      // The FIFO is empty and the card is never busy: a command run here is
      // over before the driver can look.
      return 1u << 2;  // FIFO empty
    case kSdCardDetect:
      // Active low: zero means a card is in the slot.
      return (board_ && board_->sdPresent()) ? 0u : 1u;
    case kSdWriteProtect: return 0;  // never write protected
    case kSdHwConfig:
      // One card, a 32-bit data bus, and an internal DMA — which is what the
      // driver checks before using the descriptor chain.
      return (1u << 0) | (1u << 16);
    default: break;
  }
  return GenericPeripheral::read(offset);
}

void SdmmcHostDevice::write(uint32_t offset, uint32_t value, uint32_t mask) {
  const uint32_t masked = value & mask;
  switch (offset) {
    case kSdRawInts:
      // Write-one-to-clear.
      intRaw_ &= ~masked;
      if (matrix_) matrix_->setPeripheralInterrupt(source_, (intRaw_ & intMask_) != 0);
      return;
    case kSdIntMask:
      intMask_ = masked;
      if (matrix_) matrix_->setPeripheralInterrupt(source_, (intRaw_ & intMask_) != 0);
      return;
    case kSdDmaStatus:
      dmaStatus_ &= ~masked;
      return;
    case kSdDmaIntEna:
      dmaMask_ = masked;
      return;
    case kSdDescBase:
      descriptorBase_ = masked;
      GenericPeripheral::write(offset, value, mask);
      return;
    case kSdCtrl: {
      GenericPeripheral::write(offset, value, mask);
      // The three reset bits are self-clearing: the driver sets one and spins
      // until the hardware takes it back.
      registers_[kSdCtrl] &= ~0x7u;
      return;
    }
    case kSdBusMode: {
      GenericPeripheral::write(offset, value, mask);
      registers_[kSdBusMode] &= ~1u;  // the software-reset bit, likewise
      return;
    }
    case kSdCmd: {
      GenericPeripheral::write(offset, value, mask);
      if (masked & kSdCmdStart) runCommand(registers_[kSdCmd]);
      return;
    }
    case kSdPollDemand:
      return;  // the descriptor chain is walked when the command runs
    default: break;
  }
  GenericPeripheral::write(offset, value, mask);
  (void)kSdClkDiv;
  (void)kSdClkEna;
  (void)kSdBlkSiz;
}

}  // namespace freeink::sim::emu
