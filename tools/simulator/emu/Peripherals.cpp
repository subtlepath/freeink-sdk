// FreeInk emulator — SoC peripherals.

#include "Peripherals.h"

#include "Cpu.h"

#include <utility>

namespace freeink::sim::emu {

// ── GenericPeripheral ────────────────────────────────────────────────────────

GenericPeripheral::GenericPeripheral(std::string name, uint32_t base, uint32_t size)
    : name_(std::move(name)), base_(base), size_(size) {}

uint32_t GenericPeripheral::read(uint32_t offset) {
  ++reads_[offset];
  auto fixed = fixed_.find(offset);
  if (fixed != fixed_.end()) return fixed->second;
  auto stored = registers_.find(offset);
  if (stored == registers_.end()) return 0;

  // A self-clearing bit is what a "start this operation" register looks like
  // from the firmware's side: it writes the bit, the hardware does the work
  // and clears it. Here the work is instantaneous, so the bit is already gone
  // by the first read — which is the one behaviour that lets the firmware's
  // own polling loop terminate.
  auto clearing = selfClearing_.find(offset);
  if (clearing != selfClearing_.end()) {
    const uint32_t value = stored->second;
    registers_[offset] = value & ~clearing->second;
    return value & ~clearing->second;
  }
  return stored->second;
}

void GenericPeripheral::write(uint32_t offset, uint32_t value, uint32_t mask) {
  ++writes_[offset];
  const uint32_t old = registers_.count(offset) ? registers_[offset] : 0;
  registers_[offset] = (old & ~mask) | (value & mask);
}

std::vector<RegisterAccess> GenericPeripheral::accesses() const {
  std::vector<RegisterAccess> out;
  for (const auto& entry : writes_) {
    auto value = registers_.find(entry.first);
    out.push_back({base_ + entry.first, value == registers_.end() ? 0 : value->second, true, entry.second});
  }
  for (const auto& entry : reads_) {
    auto value = registers_.find(entry.first);
    out.push_back({base_ + entry.first, value == registers_.end() ? 0 : value->second, false, entry.second});
  }
  return out;
}

// ── UART ─────────────────────────────────────────────────────────────────────

namespace {
constexpr uint32_t kUartFifo = 0x00;
constexpr uint32_t kUartIntRaw = 0x04;
constexpr uint32_t kUartIntSt = 0x08;
constexpr uint32_t kUartStatus = 0x1C;
constexpr uint32_t kUartFsmStatus = 0x88;  // UART_FSM_STATUS_REG on C3/S3
}  // namespace

UartDevice::UartDevice(std::string name, uint32_t base, std::function<void(const char*, size_t)> sink)
    : GenericPeripheral(std::move(name), base, 0x1000), sink_(std::move(sink)) {}

uint32_t UartDevice::read(uint32_t offset) {
  switch (offset) {
    case kUartStatus:
      // TXFIFO_CNT (bits 25:16) and RXFIFO_CNT (bits 9:0) both zero: nothing
      // queued to send, nothing waiting to be read.
      return 0;
    case kUartFsmStatus:
      return 0;  // both state machines idle
    case kUartIntRaw:
    case kUartIntSt:
      // TX_DONE and TXFIFO_EMPTY are always true here, since a write to the
      // FIFO has already left the machine by the time the firmware looks.
      return (1u << 1) | (1u << 14);
    case kUartFifo:
      return 0;
    default:
      return GenericPeripheral::read(offset);
  }
}

void UartDevice::write(uint32_t offset, uint32_t value, uint32_t mask) {
  if (offset == kUartFifo) {
    const char byte = static_cast<char>(value & 0xFF);
    if (sink_) sink_(&byte, 1);
    return;
  }
  GenericPeripheral::write(offset, value, mask);
}

// ── USB Serial/JTAG ──────────────────────────────────────────────────────────

namespace {
constexpr uint32_t kUsbEp1 = 0x00;      // the byte FIFO
constexpr uint32_t kUsbEp1Conf = 0x04;
constexpr uint32_t kUsbIntRaw = 0x08;
constexpr uint32_t kUsbIntSt = 0x0C;

constexpr uint32_t kUsbWrDone = 1u << 0;
constexpr uint32_t kUsbInEpDataFree = 1u << 1;
constexpr uint32_t kUsbSerialInEmpty = 1u << 2;
}  // namespace

UsbSerialJtagDevice::UsbSerialJtagDevice(uint32_t base, std::function<void(const char*, size_t)> sink)
    : GenericPeripheral("USB_SERIAL_JTAG", base, 0x1000), sink_(std::move(sink)) {}

uint32_t UsbSerialJtagDevice::read(uint32_t offset) {
  switch (offset) {
    case kUsbEp1Conf:
      // Always room to write, and nothing waiting to be read.
      return kUsbInEpDataFree;
    case kUsbIntRaw:
    case kUsbIntSt:
      // SERIAL_IN_EMPTY is what a driver waits on to know its packet went out.
      return kUsbSerialInEmpty;
    case kUsbEp1:
      return 0;
    default:
      return GenericPeripheral::read(offset);
  }
}

void UsbSerialJtagDevice::write(uint32_t offset, uint32_t value, uint32_t mask) {
  if (offset == kUsbEp1) {
    pending_ += static_cast<char>(value & 0xFF);
    // Flush on a newline as well as on the explicit packet-done bit: a
    // firmware that fills the FIFO without flushing still shows its output
    // line by line, which is what makes a hung boot readable.
    if ((value & 0xFF) == '\n' || pending_.size() >= 64) flush();
    return;
  }
  if (offset == kUsbEp1Conf) {
    if (value & mask & kUsbWrDone) flush();
    return;
  }
  GenericPeripheral::write(offset, value, mask);
}

void UsbSerialJtagDevice::flush() {
  if (pending_.empty()) return;
  if (sink_) sink_(pending_.data(), pending_.size());
  pending_.clear();
}

// ── System ───────────────────────────────────────────────────────────────────

namespace {
constexpr int kFromCpuCount = 4;
}  // namespace

SystemDevice::SystemDevice(uint32_t base, InterruptMatrixDevice* matrix, int firstFromCpuSource,
                           uint32_t fromCpuOffset)
    : GenericPeripheral("SYSTEM", base, 0x1000),
      matrix_(matrix),
      firstSource_(firstFromCpuSource),
      fromCpuOffset_(fromCpuOffset) {}

namespace {
// SYSTEM_RTC_FASTMEM_CONFIG_REG, and the two bits of it that matter: the
// firmware sets START and waits for FINISH.
constexpr uint32_t kRtcFastmemConfig = 0x48;
constexpr uint32_t kRtcMemCrcStart = 1u << 8;
constexpr uint32_t kRtcMemCrcFinish = 1u << 31;
}  // namespace

uint32_t SystemDevice::read(uint32_t offset) {
  const uint32_t value = GenericPeripheral::read(offset);
  // The checksum is over memory nothing else in the machine has touched since
  // it was asked for, so it is finished as soon as it is asked for.
  if (offset == kRtcFastmemConfig && (value & kRtcMemCrcStart)) return value | kRtcMemCrcFinish;
  return value;
}

void SystemDevice::write(uint32_t offset, uint32_t value, uint32_t mask) {
  GenericPeripheral::write(offset, value, mask);
  if (offset < fromCpuOffset_ || offset >= fromCpuOffset_ + kFromCpuCount * 4) return;
  if (!matrix_) return;
  // The register is a latch: the handler clears it by writing zero back, and
  // the interrupt stays asserted until it does.
  const int index = static_cast<int>((offset - fromCpuOffset_) / 4);
  matrix_->setPeripheralInterrupt(firstSource_ + index, (GenericPeripheral::read(offset) & 1) != 0);
}

// ── SAR ADC ──────────────────────────────────────────────────────────────────

namespace {
constexpr uint32_t kAdcOnetimeSample = 0x20;
constexpr uint32_t kAdc1DataStatus = 0x2C;
constexpr uint32_t kAdc2DataStatus = 0x30;
constexpr uint32_t kAdcIntEna = 0x40;
constexpr uint32_t kAdcIntRaw = 0x44;
constexpr uint32_t kAdcIntSt = 0x48;
constexpr uint32_t kAdcIntClr = 0x4C;

constexpr uint32_t kOnetimeStart = 1u << 29;
constexpr uint32_t kOnetimeChannelShift = 25;
constexpr uint32_t kAdc1Onetime = 1u << 31;
constexpr uint32_t kAdc2Onetime = 1u << 30;
constexpr uint32_t kAdc1Done = 1u << 31;
constexpr uint32_t kAdc2Done = 1u << 30;
}  // namespace

SarAdcDevice::SarAdcDevice(uint32_t base, Sampler sampler)
    : GenericPeripheral("APB_SARADC", base, 0x1000), sampler_(std::move(sampler)) {}

uint32_t SarAdcDevice::read(uint32_t offset) {
  switch (offset) {
    case kAdcIntRaw: return intRaw_;
    case kAdcIntSt: return intRaw_ & GenericPeripheral::read(kAdcIntEna);
    case kAdc1DataStatus: return data_[0];
    case kAdc2DataStatus: return data_[1];
    default: return GenericPeripheral::read(offset);
  }
}

void SarAdcDevice::write(uint32_t offset, uint32_t value, uint32_t mask) {
  GenericPeripheral::write(offset, value, mask);
  if (offset == kAdcIntClr) {
    intRaw_ &= ~(value & mask);
    return;
  }
  if (offset != kAdcOnetimeSample) return;

  const uint32_t control = GenericPeripheral::read(kAdcOnetimeSample);
  if (!(control & kOnetimeStart)) return;
  const int channel = static_cast<int>((control >> kOnetimeChannelShift) & 0xF);
  // Conversion takes a few microseconds on the device and no time here; the
  // done flag is set before the firmware's first poll, which is the only way
  // its wait loop can finish.
  if (control & kAdc1Onetime) {
    data_[0] = sampler_ ? sampler_(1, channel) : 0;
    intRaw_ |= kAdc1Done;
  }
  if (control & kAdc2Onetime) {
    data_[1] = sampler_ ? sampler_(2, channel) : 0;
    intRaw_ |= kAdc2Done;
  }
}

// ── System configuration, and the hardware RNG ───────────────────────────────

namespace {
constexpr uint32_t kSysconRndData = 0xB0;  // WDEV_RND_REG
}  // namespace

SysconDevice::SysconDevice(uint32_t base, uint32_t seed)
    : GenericPeripheral("APB_CTRL", base, 0x1000), state_(seed ? seed : 0x2545F491u) {}

uint32_t SysconDevice::read(uint32_t offset) {
  if (offset != kSysconRndData) return GenericPeripheral::read(offset);
  // xorshift32: cheap, passes for entropy as far as any firmware can tell, and
  // identical from run to run for the same seed.
  state_ ^= state_ << 13;
  state_ ^= state_ >> 17;
  state_ ^= state_ << 5;
  return state_;
}

// ── MMU table ────────────────────────────────────────────────────────────────

uint32_t MmuTableDevice::read(uint32_t offset) { return bus_.mmuEntry(offset / 4); }

void MmuTableDevice::write(uint32_t offset, uint32_t value, uint32_t mask) {
  const uint32_t index = offset / 4;
  const uint32_t old = bus_.mmuEntry(index);
  bus_.setMmuEntry(index, (old & ~mask) | (value & mask));
}

// ── Interrupt matrix ─────────────────────────────────────────────────────────

namespace {
// The C3's own registers, which follow its map registers. The S3 has none of
// these: its masking lives in the CPU.
constexpr uint32_t kCpuIntEnable = 0x104;
constexpr uint32_t kCpuIntType = 0x108;
constexpr uint32_t kCpuIntClear = 0x10C;
constexpr uint32_t kCpuIntEipStatus = 0x110;
constexpr uint32_t kCpuIntPriBase = 0x114;
constexpr uint32_t kCpuIntThresh = 0x194;
}  // namespace

InterruptMatrixDevice::InterruptMatrixDevice(const SocDesc& soc, Cpu& core0, Cpu* core1)
    : GenericPeripheral("interrupt matrix", soc.interrupt, 0x1000),
      soc_(soc),
      lineZeroIsRouted_(soc.arch == Arch::Xtensa) {
  cores_[0].cpu = &core0;
  cores_[1].cpu = core1;
  // On the S3 the matrix does no masking of its own, so every line is live as
  // far as this model is concerned and INTENABLE in the core decides.
  if (lineZeroIsRouted_) {
    cores_[0].enabled = 0xFFFFFFFFu;
    cores_[1].enabled = 0xFFFFFFFFu;
  }
}

std::pair<int, int> InterruptMatrixDevice::mapRegister(uint32_t offset) const {
  if (offset < soc_.matrixMapEnd) return {0, static_cast<int>(offset / 4)};
  if (soc_.matrixCore1Offset && offset >= soc_.matrixCore1Offset &&
      offset < soc_.matrixCore1Offset + soc_.matrixMapEnd) {
    return {1, static_cast<int>((offset - soc_.matrixCore1Offset) / 4)};
  }
  return {-1, 0};
}

std::vector<InterruptMatrixDevice::SourceUse> InterruptMatrixDevice::sourcesUsed() const {
  std::vector<SourceUse> out;
  for (const auto& entry : raised_) {
    int line = -1;
    for (const CoreRouting& core : cores_) {
      const auto it = core.routing.find(entry.first);
      if (it != core.routing.end()) line = it->second;
    }
    const auto active = active_.find(entry.first);
    out.push_back({entry.first, line, entry.second, active != active_.end() && active->second});
  }
  std::sort(out.begin(), out.end(),
            [](const SourceUse& a, const SourceUse& b) { return a.raised > b.raised; });
  return out;
}

void InterruptMatrixDevice::route(int cpu, int peripheral, int line) {
  if (cpu < 0 || cpu > 1) return;
  cores_[cpu].routing[peripheral] = line;
  refresh();
}

void InterruptMatrixDevice::setPeripheralInterrupt(int peripheral, bool active) {
  if (active && !active_[peripheral]) ++raised_[peripheral];
  active_[peripheral] = active;
  refresh();
}

void InterruptMatrixDevice::refresh() {
  // A CPU line is asserted while any peripheral routed to it is asserting and
  // the line is enabled. Several peripherals can share a line, which is why
  // this recomputes rather than tracking edges.
  const int lowest = lineZeroIsRouted_ ? 0 : 1;
  for (CoreRouting& core : cores_) {
    if (!core.cpu) continue;
    uint32_t pending = 0;
    for (const auto& entry : active_) {
      if (!entry.second) continue;
      auto route = core.routing.find(entry.first);
      if (route == core.routing.end() || route->second < lowest || route->second >= 32) continue;
      pending |= 1u << route->second;
    }
    for (int line = lowest; line < 32; ++line) {
      const bool assert = (pending & (1u << line)) && (core.enabled & (1u << line));
      core.cpu->setInterruptPending(line, assert);
    }
  }
}

uint32_t InterruptMatrixDevice::read(uint32_t offset) {
  if (!lineZeroIsRouted_) {
    if (offset == kCpuIntEnable) return cores_[0].enabled;
    if (offset == kCpuIntEipStatus) {
      uint32_t pending = 0;
      for (const auto& entry : active_) {
        if (!entry.second) continue;
        auto route = cores_[0].routing.find(entry.first);
        if (route != cores_[0].routing.end() && route->second > 0 && route->second < 32) {
          pending |= 1u << route->second;
        }
      }
      return pending;
    }
  }
  return GenericPeripheral::read(offset);
}

void InterruptMatrixDevice::write(uint32_t offset, uint32_t value, uint32_t mask) {
  GenericPeripheral::write(offset, value, mask);

  const std::pair<int, int> slot = mapRegister(offset);
  if (slot.first >= 0) {
    cores_[slot.first].routing[slot.second] = static_cast<int>(value & 0x1F);
    refresh();
    return;
  }
  if (lineZeroIsRouted_) return;  // the rest of this block is the C3's alone

  if (offset == kCpuIntEnable) {
    cores_[0].enabled = (cores_[0].enabled & ~mask) | (value & mask);
    refresh();
    return;
  }
  if (offset == kCpuIntThresh) {
    // The core consults this before taking an interrupt, so nesting behaves.
    cores_[0].cpu->setInterruptThreshold(value & 0xF);
    return;
  }
  if (offset >= kCpuIntPriBase && offset <= kCpuIntPriBase + 31 * 4) {
    cores_[0].cpu->setInterruptPriority(static_cast<int>((offset - kCpuIntPriBase) / 4), value & 0xF);
    return;
  }
  if (offset == kCpuIntClear) {
    for (int line = 1; line < 32; ++line) {
      if (value & (1u << line)) cores_[0].cpu->setInterruptPending(line, false);
    }
    return;
  }
  if (offset == kCpuIntType) return;
}

// ── Timer group ──────────────────────────────────────────────────────────────

namespace {
constexpr uint32_t kTimgT0Lo = 0x04;
constexpr uint32_t kTimgT0Hi = 0x08;
constexpr uint32_t kTimgT0Update = 0x0C;
constexpr uint32_t kTimgRtcCaliCfg = 0x68;
constexpr uint32_t kTimgRtcCaliCfg1 = 0x6C;
constexpr uint32_t kTimgRtcCaliCfg2 = 0x80;

constexpr uint32_t kCaliStart = 1u << 31;
constexpr uint32_t kCaliRdy = 1u << 15;
constexpr uint32_t kCaliMaxShift = 16;
constexpr uint32_t kCaliMaxMask = 0x7FFF;
constexpr uint32_t kCaliClkSelShift = 13;
constexpr uint32_t kCaliValueShift = 7;

// rtc_cal_sel_t. The frequency each selection presents to the calibrator: the
// numbers a C3 reports for its RC oscillators, the crystal for the 32 kHz one.
uint32_t calibrationSourceHz(uint32_t select) {
  switch (select) {
    case 0: return 136000;    // RTC_CAL_RTC_MUX — the RC slow oscillator
    case 1: return 8500000 / 256;  // RTC_CAL_8MD256
    case 2: return 32768;     // RTC_CAL_32K_XTAL
    case 3: return 17500000;  // RTC_CAL_RC_FAST
    default: return 136000;
  }
}
}  // namespace

TimerGroupDevice::TimerGroupDevice(std::string name, uint32_t base, std::function<uint64_t()> nowUs,
                                   uint32_t xtalHz)
    : GenericPeripheral(std::move(name), base, 0x1000), nowUs_(std::move(nowUs)), xtalHz_(xtalHz) {}

uint64_t TimerGroupDevice::counter() const {
  // The general-purpose timer counts APB/divider; IDF configures it for 1 MHz
  // and reads it as microseconds, which is what this reports.
  return nowUs_ ? nowUs_() : 0;
}

uint32_t TimerGroupDevice::read(uint32_t offset) {
  switch (offset) {
    case kTimgT0Lo: return static_cast<uint32_t>(latched_);
    case kTimgT0Hi: return static_cast<uint32_t>(latched_ >> 32);
    case kTimgRtcCaliCfg: {
      // The measurement completes immediately, so the ready bit is set by the
      // time the firmware's first poll gets here.
      const uint32_t configured = GenericPeripheral::read(offset);
      return (configured & ~kCaliStart) | kCaliRdy;
    }
    case kTimgRtcCaliCfg1:
      return calibration_ << kCaliValueShift;
    case kTimgRtcCaliCfg2:
      // Never times out: the result is always there.
      return GenericPeripheral::read(offset) & ~1u;
    default:
      return GenericPeripheral::read(offset);
  }
}

void TimerGroupDevice::write(uint32_t offset, uint32_t value, uint32_t mask) {
  GenericPeripheral::write(offset, value, mask);
  if (offset == kTimgT0Update) {
    latched_ = counter();
    return;
  }
  if (offset == kTimgRtcCaliCfg && (value & mask & kCaliStart)) {
    // Count crystal cycles over `max` cycles of the selected slow clock. This
    // is the ratio every RTC-derived timeout in the system is scaled by.
    const uint32_t configured = GenericPeripheral::read(offset);
    const uint32_t max = (configured >> kCaliMaxShift) & kCaliMaxMask;
    const uint32_t sourceHz = calibrationSourceHz((configured >> kCaliClkSelShift) & 3);
    calibration_ = sourceHz ? static_cast<uint32_t>((static_cast<uint64_t>(max) * xtalHz_) / sourceHz) : 0;
  }
}

// ── RTC controller ───────────────────────────────────────────────────────────

namespace {
// The SAR/"SENS" block shares this page, 0x800 in.
constexpr uint32_t kSensMeas1Ctrl2 = 0x800 + 0x0C;  // SENS_SAR_MEAS1_CTRL2_REG
constexpr uint32_t kSensMeas2Ctrl2 = 0x800 + 0x30;  // SENS_SAR_MEAS2_CTRL2_REG
constexpr uint32_t kSensMeasStart = 1u << 17;       // SENS_MEASn_START_SAR
constexpr uint32_t kSensMeasDone = 1u << 16;        // SENS_MEASn_DONE_SAR
constexpr uint32_t kRtcTimeUpdate = 0x0C;
constexpr uint32_t kRtcTimeLow0 = 0x10;
constexpr uint32_t kRtcTimeHigh0 = 0x14;
constexpr uint32_t kRtcResetState = 0x38;
// RTC_CNTL_STATE0_REG, and the bit that starts the sleep. The timer's target
// is programmed into SLP_TIMER0/1 first, in slow-clock ticks.
constexpr uint32_t kRtcState0 = 0x18;
constexpr uint32_t kRtcSleepEn = 1u << 31;
constexpr uint32_t kRtcSlpTimer0 = 0x00;
constexpr uint32_t kRtcSlpTimer1 = 0x04;
constexpr uint32_t kRtcSlpValEn = 1u << 16;
constexpr uint32_t kRtcTimeValid = 1u << 30;
}  // namespace

RtcCntlDevice::RtcCntlDevice(uint32_t base, std::function<uint64_t()> nowUs, Sampler sampler)
    : GenericPeripheral("RTC_CNTL", base, 0x1000), nowUs_(std::move(nowUs)), sampler_(std::move(sampler)) {}

uint32_t RtcCntlDevice::read(uint32_t offset) {
  switch (offset) {
    case kRtcTimeUpdate:
      // The value is always ready: there is no clock domain to cross here.
      return GenericPeripheral::read(offset) | kRtcTimeValid;
    case kRtcTimeLow0: return static_cast<uint32_t>(latched_);
    case kRtcTimeHigh0: return static_cast<uint32_t>(latched_ >> 32);
    case kRtcResetState:
      // POWERON_RESET for both the reset cause and the wake cause fields.
      return 0x00000001;
    default:
      return GenericPeripheral::read(offset);
  }
}

void RtcCntlDevice::write(uint32_t offset, uint32_t value, uint32_t mask) {
  GenericPeripheral::write(offset, value, mask);

  // The SAR block's two one-shot measurement registers. Starting a conversion
  // completes it: there is no conversion time to model here, and a firmware
  // polling for the result gets one on its first look rather than never.
  if (offset == kSensMeas1Ctrl2 || offset == kSensMeas2Ctrl2) {
    const bool unit2 = offset == kSensMeas2Ctrl2;
    uint32_t current = GenericPeripheral::read(offset);
    if (current & kSensMeasStart) {
      const uint32_t sample = sampler_ ? sampler_(unit2 ? 2 : 1, 0) : 4095;
      current = (current & ~0xFFFFu) | (sample & 0xFFFFu) | kSensMeasDone;
    } else {
      current &= ~kSensMeasDone;
    }
    GenericPeripheral::write(offset, current, 0xFFFFFFFFu);
    return;
  }

  if (offset == kRtcTimeUpdate) {
    const uint64_t us = nowUs_ ? nowUs_() : 0;
    latched_ = (us * slowHz_) / 1000000ULL;
  }

  if (offset == kRtcState0 && (value & mask & kRtcSleepEn) && sleep_) {
    // How long the firmware asked to sleep for, in microseconds, from the
    // wake-up timer it just programmed. Zero means it armed no timer and is
    // waiting for something else — a pad, a touch — to wake it.
    const uint32_t low = GenericPeripheral::read(kRtcSlpTimer0);
    const uint32_t high = GenericPeripheral::read(kRtcSlpTimer1) & 0xFFFF;
    const uint64_t ticks = (static_cast<uint64_t>(high) << 32) | low;
    const uint64_t requested =
        (GenericPeripheral::read(kRtcSlpTimer1) & kRtcSlpValEn) && slowHz_
            ? ticks * 1000000ULL / slowHz_
            : 0;
    sleep_(requested);
  }
}

// ── System timer ─────────────────────────────────────────────────────────────

namespace {
constexpr uint32_t kSysConf = 0x00;
constexpr uint32_t kSysUnit0Op = 0x04;
constexpr uint32_t kSysTarget0Hi = 0x1C;
constexpr uint32_t kSysTarget0Conf = 0x34;
constexpr uint32_t kSysUnit0ValueHi = 0x40;
constexpr uint32_t kSysComp0Load = 0x50;
constexpr uint32_t kSysIntEna = 0x64;
constexpr uint32_t kSysIntRaw = 0x68;
constexpr uint32_t kSysIntClr = 0x6C;
constexpr uint32_t kSysIntSt = 0x70;

constexpr uint32_t kUnitUpdate = 1u << 30;
constexpr uint32_t kUnitValueValid = 1u << 29;
constexpr uint32_t kTargetPeriodMode = 1u << 30;
constexpr uint32_t kTargetPeriodMask = 0x03FFFFFF;
}  // namespace

SystimerDevice::SystimerDevice(uint32_t base, std::function<uint64_t()> nowUs,
                               InterruptMatrixDevice* matrix, int firstInterruptSource)
    : GenericPeripheral("SYSTIMER", base, 0x1000),
      nowUs_(std::move(nowUs)),
      matrix_(matrix),
      firstSource_(firstInterruptSource) {}

uint64_t SystimerDevice::counter() const { return (nowUs_ ? nowUs_() : 0) * kTicksPerUs; }

uint32_t SystimerDevice::read(uint32_t offset) {
  if (offset == kSysUnit0Op || offset == kSysUnit0Op + 4) {
    // Whatever was asked for has already happened, so the value is valid.
    return GenericPeripheral::read(offset) | kUnitValueValid;
  }
  if (offset >= kSysUnit0ValueHi && offset < kSysUnit0ValueHi + 16) {
    const int unit = static_cast<int>((offset - kSysUnit0ValueHi) / 8);
    const bool high = ((offset - kSysUnit0ValueHi) % 8) == 0;
    const uint64_t value = unitLatched_[unit];
    return high ? static_cast<uint32_t>(value >> 32) : static_cast<uint32_t>(value);
  }
  if (offset == kSysIntRaw) return intRaw_;
  if (offset == kSysIntSt) return intRaw_ & intEnable_;
  if (offset == kSysIntEna) return intEnable_;
  return GenericPeripheral::read(offset);
}

void SystimerDevice::write(uint32_t offset, uint32_t value, uint32_t mask) {
  GenericPeripheral::write(offset, value, mask);

  if (offset == kSysUnit0Op || offset == kSysUnit0Op + 4) {
    if (value & mask & kUnitUpdate) {
      unitLatched_[(offset - kSysUnit0Op) / 4] = counter();
    }
    return;
  }
  if (offset >= kSysTarget0Hi && offset < kSysTarget0Hi + kTargets * 8) {
    const int target = static_cast<int>((offset - kSysTarget0Hi) / 8);
    const bool high = ((offset - kSysTarget0Hi) % 8) == 0;
    const uint64_t current = target_[target];
    target_[target] = high ? ((static_cast<uint64_t>(value) << 32) | (current & 0xFFFFFFFFULL))
                           : ((current & ~0xFFFFFFFFULL) | value);
    // A fresh target re-arms the comparator. The low half is written last by
    // every driver that writes both, so arming on either half is safe and
    // arming on the second is what actually matters.
    armed_[target] = true;
    return;
  }
  if (offset >= kSysTarget0Conf && offset < kSysTarget0Conf + kTargets * 4) {
    targetConf_[(offset - kSysTarget0Conf) / 4] = GenericPeripheral::read(offset);
    return;
  }
  if (offset >= kSysComp0Load && offset < kSysComp0Load + kTargets * 4) {
    // Loading a comparator hands it whatever was staged: a periodic alarm
    // restarts from now, a one-shot re-arms on its target.
    const int target = static_cast<int>((offset - kSysComp0Load) / 4);
    periodBase_[target] = counter();
    armed_[target] = true;
    return;
  }
  if (offset == kSysConf) {
    const uint32_t conf = GenericPeripheral::read(offset);
    for (int i = 0; i < kTargets; ++i) workEnabled_[i] = (conf & (1u << (24 + i))) != 0;
    return;
  }
  if (offset == kSysIntEna) {
    intEnable_ = GenericPeripheral::read(offset);
    refreshInterrupts();
    return;
  }
  if (offset == kSysIntClr) {
    intRaw_ &= ~(value & mask);
    refreshInterrupts();
    return;
  }
}

void SystimerDevice::tick(uint64_t) {
  const uint64_t now = counter();
  for (int i = 0; i < kTargets; ++i) {
    if (!workEnabled_[i]) continue;
    if (targetConf_[i] & kTargetPeriodMode) {
      // A periodic alarm schedules its next one from the alarm that fired, not
      // from when the firmware got round to acknowledging it, or the tick rate
      // drifts with interrupt latency. Ticks the firmware was too slow to take
      // are lost, exactly as they are on the chip — there is one pending bit.
      const uint32_t period = targetConf_[i] & kTargetPeriodMask;
      if (!period) continue;
      while (now >= periodBase_[i] + period) {
        periodBase_[i] += period;
        intRaw_ |= 1u << i;
      }
    } else if (armed_[i] && now >= target_[i]) {
      intRaw_ |= 1u << i;
      armed_[i] = false;
    }
  }
  refreshInterrupts();
}

void SystimerDevice::refreshInterrupts() {
  if (!matrix_) return;
  for (int i = 0; i < kTargets; ++i) {
    matrix_->setPeripheralInterrupt(firstSource_ + i, (intRaw_ & intEnable_ & (1u << i)) != 0);
  }
}

uint64_t SystimerDevice::nextDeadlineUs() const {
  const uint64_t now = counter();
  uint64_t best = 0;
  for (int i = 0; i < kTargets; ++i) {
    if (!workEnabled_[i] || !(intEnable_ & (1u << i))) continue;
    uint64_t at = 0;
    if (targetConf_[i] & kTargetPeriodMode) {
      const uint32_t period = targetConf_[i] & kTargetPeriodMask;
      if (!period) continue;
      at = periodBase_[i] + period;
    } else {
      if (!armed_[i]) continue;
      at = target_[i];
    }
    if (at <= now) return 0;  // already due
    const uint64_t inUs = (at - now) / kTicksPerUs + 1;
    if (best == 0 || inUs < best) best = inUs;
  }
  return best;
}

}  // namespace freeink::sim::emu
