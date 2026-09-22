#pragma once

// FreeInk emulator — SoC peripherals.
//
// These are the blocks an ESP-IDF app touches on its way up, modelled to the
// depth that boot and the drivers this SDK cares about actually need. Three
// kinds live here:
//
//   * Blocks with real behaviour — the UART's transmit path, the MMU table,
//     the system timer that FreeRTOS ticks from, the interrupt matrix that
//     routes a peripheral's IRQ to a CPU line.
//   * Blocks that only have to be consistent — clock and reset gating,
//     eFuse — where reading back what was written is indistinguishable from
//     the real thing to the code that pokes them.
//   * GenericPeripheral, the fallback. It does the same read-back, and it
//     *records every register touched*, which is the tool that turns "the
//     firmware hangs somewhere in early boot" into a list of registers it was
//     waiting on.

#include "Bus.h"

#include <cstdint>
#include <functional>
#include <utility>
#include <map>
#include <string>
#include <vector>

namespace freeink::sim::emu {

class Cpu;
class InterruptMatrixDevice;

// A register access, for the trace `freeink-sim emu regs` prints.
struct RegisterAccess {
  uint32_t address;
  uint32_t value;
  bool write;
  uint64_t count;
};

// Reads back what was written, and remembers what was touched.
class GenericPeripheral : public MmioDevice {
 public:
  GenericPeripheral(std::string name, uint32_t base, uint32_t size);

  const char* name() const override { return name_.c_str(); }
  uint32_t read(uint32_t offset) override;
  void write(uint32_t offset, uint32_t value, uint32_t mask) override;

  // Registers this block has seen, in address order.
  std::vector<RegisterAccess> accesses() const;
  // Some blocks have a register that must read back a fixed value (a "ready"
  // or "done" bit the firmware polls). This is how a model declares one
  // without needing a whole subclass.
  void setFixedRead(uint32_t offset, uint32_t value) { fixed_[offset] = value; }
  // A bit the firmware sets and the hardware clears when the operation
  // finishes. Declaring it here makes a polling loop terminate on the next
  // read, which is what the hardware would do faster than the firmware can
  // notice.
  void setSelfClearing(uint32_t offset, uint32_t mask) { selfClearing_[offset] = mask; }
  void setStorage(uint32_t offset, uint32_t value) { registers_[offset] = value; }

 protected:
  std::string name_;
  uint32_t base_;
  uint32_t size_;
  std::map<uint32_t, uint32_t> registers_;
  std::map<uint32_t, uint32_t> fixed_;
  std::map<uint32_t, uint32_t> selfClearing_;
  std::map<uint32_t, uint64_t> reads_;
  std::map<uint32_t, uint64_t> writes_;
};

// ── UART ─────────────────────────────────────────────────────────────────────
// The transmit path is what matters: everything an IDF app logs after the
// early ROM stage goes through the FIFO register. The status register always
// reports an empty transmit FIFO, so a driver waiting for the line to drain
// never waits — the host has already taken the bytes.
class UartDevice : public GenericPeripheral {
 public:
  UartDevice(std::string name, uint32_t base, std::function<void(const char*, size_t)> sink);

  uint32_t read(uint32_t offset) override;
  void write(uint32_t offset, uint32_t value, uint32_t mask) override;

 private:
  std::function<void(const char*, size_t)> sink_;
};

// ── USB Serial/JTAG ──────────────────────────────────────────────────────────
// On these boards this is the console: there is no UART bridge, so an app's
// stdout goes out over USB CDC. The firmware writes bytes into the endpoint
// FIFO and sets a "packet done" bit to flush it, and polls a "space available"
// bit before each write — which always reads true here, since the host has
// already taken the bytes. Without this block modelled, a boot looks silent
// while the firmware waits on a FIFO that never drains.
class UsbSerialJtagDevice : public GenericPeripheral {
 public:
  UsbSerialJtagDevice(uint32_t base, std::function<void(const char*, size_t)> sink);

  uint32_t read(uint32_t offset) override;
  void write(uint32_t offset, uint32_t value, uint32_t mask) override;

 private:
  void flush();

  std::function<void(const char*, size_t)> sink_;
  std::string pending_;
};

// ── System (clock, reset, and the cross-CPU interrupts) ──────────────────────
// Clock and reset gating here is read-back-what-you-wrote, and that is enough.
// The part with real behaviour is the "interrupt from CPU" registers: writing
// one raises a software interrupt through the matrix. FreeRTOS uses it to ask
// for a context switch, and the *first* such request is what starts the very
// first task — so without it a firmware brings its scheduler up, ticks
// happily, and never runs a line of application code.
// The other thing with behaviour is the RTC fast-memory CRC. On the way into
// deep sleep the firmware asks this block to checksum the memory that survives
// the sleep, and then *spins with interrupts off* until the "finished" bit
// appears — so a block that never sets it is a firmware that gets as far as
// going to sleep and then hangs a few instructions short of it, which looks
// from the outside like a hang with no cause at all.
class SystemDevice : public GenericPeripheral {
 public:
  SystemDevice(uint32_t base, InterruptMatrixDevice* matrix, int firstFromCpuSource,
               uint32_t fromCpuOffset);

  uint32_t read(uint32_t offset) override;
  void write(uint32_t offset, uint32_t value, uint32_t mask) override;

 private:
  InterruptMatrixDevice* matrix_;
  int firstSource_;
  uint32_t fromCpuOffset_;
};

// ── SAR ADC ──────────────────────────────────────────────────────────────────
// A one-shot conversion here completes immediately and reports whatever the
// machine says is on the pin. That matters beyond "the firmware stops
// spinning": on these boards the buttons are a resistor ladder into an ADC
// channel, so this block is the input device, and the firmware's own ladder
// decode runs against the voltages it reads here.
class SarAdcDevice : public GenericPeripheral {
 public:
  // Returns the raw conversion result for a unit (1 or 2) and channel.
  using Sampler = std::function<uint32_t(int unit, int channel)>;

  SarAdcDevice(uint32_t base, Sampler sampler);

  uint32_t read(uint32_t offset) override;
  void write(uint32_t offset, uint32_t value, uint32_t mask) override;

 private:
  Sampler sampler_;
  uint32_t intRaw_ = 0;
  uint32_t data_[2] = {0, 0};
};

// ── System configuration, and the hardware RNG ───────────────────────────────
// The random number register is the part with behaviour: every read must
// produce a different value, and firmware polls it hard — `esp_random()` and
// anything seeded from it will spin forever on a register that reads back the
// same number. The generator here is deterministic, seeded per machine, so a
// run stays reproducible while still looking random to the firmware.
class SysconDevice : public GenericPeripheral {
 public:
  SysconDevice(uint32_t base, uint32_t seed);

  uint32_t read(uint32_t offset) override;

 private:
  uint32_t state_;
};

// ── Flash MMU table ──────────────────────────────────────────────────────────
// 128 (C3) or 512 (S3) words that decide which 64 KB of flash each cached page
// reads from. Writing one here really does change what the CPU fetches, which
// is what makes esp_mmu_map() and OTA behave.
class MmuTableDevice : public MmioDevice {
 public:
  explicit MmuTableDevice(Bus& bus) : bus_(bus) {}
  const char* name() const override { return "MMU table"; }
  uint32_t read(uint32_t offset) override;
  void write(uint32_t offset, uint32_t value, uint32_t mask) override;

 private:
  Bus& bus_;
};

// ── Cache controller ─────────────────────────────────────────────────────────
// There is no cache in this model — the MMU translates straight into the flash
// — so the controller is a recording peripheral with one honest answer in it:
// the cache is idle. Firmware reconfiguring its own cache waits for that
// before and after every change, and a controller that never goes idle is a
// boot that never finishes.
class CacheControlDevice : public GenericPeripheral {
 public:
  CacheControlDevice(uint32_t base, uint32_t stateOffset, uint32_t idleValue)
      : GenericPeripheral("EXTMEM", base, 0x1000), stateOffset_(stateOffset), idleValue_(idleValue) {}

  uint32_t read(uint32_t offset) override {
    if (offset == stateOffset_) return idleValue_;
    return GenericPeripheral::read(offset);
  }

 private:
  uint32_t stateOffset_;
  uint32_t idleValue_;
};

// ── Analog register master ───────────────────────────────────────────────────
// The PLL, the regulators and the RF front end are not memory-mapped: they are
// reached through a little register-over-I2C master, which the ROM's
// rom_i2c_readReg/writeReg drive and which newer IDF pokes directly. The one
// thing that has to answer here is the status register, because clock setup
// spins on its "calibration finished" bit with interrupts off.
class AnalogMasterDevice : public GenericPeripheral {
 public:
  AnalogMasterDevice(uint32_t base, uint32_t statusOffset, uint32_t readyBit)
      : GenericPeripheral("I2C_ANA_MST", base, 0x1000),
        statusOffset_(statusOffset),
        readyBit_(readyBit) {}

  uint32_t read(uint32_t offset) override {
    const uint32_t value = GenericPeripheral::read(offset);
    return offset == statusOffset_ ? (value | readyBit_) : value;
  }

 private:
  uint32_t statusOffset_;
  uint32_t readyBit_;
};

// ── Interrupt matrix ─────────────────────────────────────────────────────────
// Every peripheral IRQ is routed to one of the CPU's interrupt lines by
// writing that line number into the peripheral's slot. The matrix also holds
// each line's priority and the threshold below which lines stay masked, both
// of which the core consults before taking an interrupt.
// A multi-core part has one of these per CPU, interleaved in the same block of
// registers, and the two halves route independently: the same peripheral can
// be wired to a different line on each core, or to only one of them. That is
// how an IDF app pins an interrupt to the core that allocated it.
//
// The two chips also mask differently, which is why the core is asked rather
// than assumed. The C3 gates lines in the matrix (its own enable, priority and
// threshold registers, all absent on the S3) and reserves line 0 to mean
// "unrouted"; the S3 gates them in the CPU's own INTENABLE, and line 0 there
// is an ordinary level-1 interrupt.
class InterruptMatrixDevice : public GenericPeripheral {
 public:
  InterruptMatrixDevice(const SocDesc& soc, Cpu& core0, Cpu* core1 = nullptr);

  uint32_t read(uint32_t offset) override;
  void write(uint32_t offset, uint32_t value, uint32_t mask) override;

  // Called by a peripheral model when its interrupt condition changes.
  void setPeripheralInterrupt(int peripheral, bool active);
  // Routing, also reachable through the ROM's intr_matrix_set().
  void route(int cpu, int peripheral, int line);

  // How many times each peripheral raised its condition, and where the
  // firmware routed it. A firmware that is busy but making no progress is
  // usually servicing one interrupt over and over, and this is the list that
  // names it.
  struct SourceUse {
    int peripheral;
    int line;
    uint64_t raised;
    bool asserted;  // still asserted now — a level nobody cleared
  };
  std::vector<SourceUse> sourcesUsed() const;

 private:
  struct CoreRouting {
    Cpu* cpu = nullptr;
    std::map<int, int> routing;  // peripheral -> CPU line
    uint32_t enabled = 0;        // the C3's matrix-level mask
  };

  void refresh();
  // The map register at this offset, as (core, peripheral), or core -1 if the
  // offset is not a map register.
  std::pair<int, int> mapRegister(uint32_t offset) const;

  const SocDesc& soc_;
  bool lineZeroIsRouted_;  // true on the S3, false on the C3
  CoreRouting cores_[2];
  std::map<int, bool> active_;    // peripheral -> asserted
  std::map<int, uint64_t> raised_;  // peripheral -> rising edges
};

// ── Timer group ──────────────────────────────────────────────────────────────
// Two things here matter to a booting app. The general-purpose timer, which
// the drivers read as a microsecond counter; and the RTC clock calibration
// block, which measures the slow clock against the crystal. Startup will not
// proceed past the calibration loop, so it has to produce a number consistent
// with the slow clock the firmware selected — a wrong one makes every
// subsequent timeout in the system wrong by the same ratio.
class TimerGroupDevice : public GenericPeripheral {
 public:
  TimerGroupDevice(std::string name, uint32_t base, std::function<uint64_t()> nowUs, uint32_t xtalHz);

  uint32_t read(uint32_t offset) override;
  void write(uint32_t offset, uint32_t value, uint32_t mask) override;

 private:
  uint64_t counter() const;

  std::function<uint64_t()> nowUs_;
  uint32_t xtalHz_;
  uint64_t latched_ = 0;
  uint32_t calibration_ = 0;  // cycles counted, as RTCCALICFG1 reports them
};

// ── RTC controller ───────────────────────────────────────────────────────────
// Clock selection, the always-on counter, the retained STORE registers and the
// reset/wake state. Modelled shallowly except for the counter, which is what
// `esp_rtc_get_time_us()` and deep-sleep timing are built on.
class RtcCntlDevice : public GenericPeripheral {
 public:
  // The sensor block shares this 4 KB page — the RTC controller is at the
  // bottom of it and the SAR/"SENS" registers 2 KB further up — so one model
  // covers both. The part of SENS that has to work is the legacy one-shot
  // conversion: start a measurement, then spin until its done bit appears.
  // With interrupts off, a conversion that never finishes is a boot that
  // never finishes.
  using Sampler = std::function<uint32_t(int unit, int channel)>;

  RtcCntlDevice(uint32_t base, std::function<uint64_t()> nowUs, Sampler sampler = {});

  uint32_t read(uint32_t offset) override;
  void write(uint32_t offset, uint32_t value, uint32_t mask) override;

  // Survives a restart, as the real RTC domain does across a deep-sleep wake.
  void setSlowClockHz(uint32_t hz) { slowHz_ = hz; }

  // Called when the firmware arms the sleep: the chip is about to stop. On
  // silicon the core loses power here and comes back through reset, so there
  // is nothing sensible to keep executing — the machine stops and says so, and
  // whatever is driving it decides when the device wakes.
  //
  // Without this the firmware sits in the loop that waits for a sleep that
  // never happens, which reads as a hang with no cause: the last thing it did
  // was set a bit in a register.
  void setSleepHook(std::function<void(uint64_t sleepUs)> hook) { sleep_ = std::move(hook); }

 private:
  std::function<uint64_t()> nowUs_;
  Sampler sampler_;
  std::function<void(uint64_t)> sleep_;
  uint32_t slowHz_ = 150000;
  uint64_t latched_ = 0;
};

// ── System timer ─────────────────────────────────────────────────────────────
// The 52-bit counter FreeRTOS ticks from and esp_timer schedules on. Its three
// comparators are what turn simulated time into interrupts, so this is the
// block that makes the firmware's scheduler actually run rather than sit in
// its idle loop forever.
class SystimerDevice : public GenericPeripheral {
 public:
  SystimerDevice(uint32_t base, std::function<uint64_t()> nowUs, InterruptMatrixDevice* matrix,
                 int firstInterruptSource);

  uint32_t read(uint32_t offset) override;
  void write(uint32_t offset, uint32_t value, uint32_t mask) override;
  void tick(uint64_t nowUs) override;

  // Microseconds until the next comparator fires, or 0 if none is armed.
  uint64_t nextDeadlineUs() const;

 private:
  static constexpr int kTargets = 3;
  static constexpr uint32_t kTicksPerUs = 16;

  uint64_t counter() const;
  void refreshInterrupts();

  std::function<uint64_t()> nowUs_;
  InterruptMatrixDevice* matrix_;
  int firstSource_;

  uint64_t unitLatched_[2] = {0, 0};
  uint64_t target_[kTargets] = {0, 0, 0};
  uint32_t targetConf_[kTargets] = {0, 0, 0};
  bool workEnabled_[kTargets] = {false, false, false};
  // A one-shot comparator fires once, when the counter passes the target it
  // was given, and then stays quiet until it is given a new one. Without that,
  // a target in the past matches on every pass and the alarm re-fires as fast
  // as the machine can service it — which does not look like a broken timer,
  // it looks like a firmware that boots and then hangs, because the interrupt
  // it is drowning in has a higher priority than the scheduler tick.
  bool armed_[kTargets] = {false, false, false};
  uint64_t periodBase_[kTargets] = {0, 0, 0};
  uint32_t intEnable_ = 0;
  uint32_t intRaw_ = 0;
};

}  // namespace freeink::sim::emu
