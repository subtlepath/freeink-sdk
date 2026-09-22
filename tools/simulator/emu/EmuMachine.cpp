// FreeInk emulator — one emulated device.

#include "EmuMachine.h"

#include "FlashController.h"
#include "RiscvCore.h"
#include "XtensaCore.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>

namespace freeink::sim::emu {
namespace {

std::string hex(uint32_t value) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "0x%08X", value);
  return buf;
}

// Names for the peripheral pages, so a register trace and a fault message say
// "SPI2+0x14" rather than an address. Anything not listed keeps its address.
struct NamedBlock {
  uint32_t base;
  const char* name;
};

std::vector<NamedBlock> namedBlocks(const SocDesc& soc) {
  std::vector<NamedBlock> blocks = {
      {soc.uart0, "UART0"},         {soc.uart1, "UART1"},
      {soc.gpio, "GPIO"},           {soc.ioMux, "IO_MUX"},
      {soc.spi0, "SPI0"},           {soc.spi1, "SPI1"},
      {soc.spi2, "SPI2"},           {soc.spi3, "SPI3"},
      {soc.i2c0, "I2C0"},           {soc.i2c1, "I2C1"},
      {soc.rtcCntl, "RTC_CNTL"},    {soc.efuse, "EFUSE"},
      {soc.timerGroup0, "TIMG0"},   {soc.timerGroup1, "TIMG1"},
      {soc.systimer, "SYSTIMER"},   {soc.interrupt, "INTERRUPT"},
      {soc.system, "SYSTEM"},       {soc.extmem, "EXTMEM"},
      {soc.apbCtrl, "APB_CTRL"},    {soc.ledc, "LEDC"},
      {soc.sdmmc, "SDMMC"},         {soc.gdma, "GDMA"},
      {soc.usbSerialJtag, "USB_SERIAL_JTAG"},
  };
  blocks.erase(std::remove_if(blocks.begin(), blocks.end(), [](const NamedBlock& b) { return b.base == 0; }),
               blocks.end());
  return blocks;
}

}  // namespace

EmuMachine::EmuMachine() = default;
EmuMachine::~EmuMachine() = default;

bool EmuMachine::load(const std::string& imagePath, const EmuOptions& options, std::string* error) {
  if (!flash_.load(imagePath, error)) return false;

  soc_ = socForChip(flash_.app().chip);
  if (!soc_) {
    if (error) {
      *error = imagePath + " is built for " + chipName(flash_.app().chip) +
               ", which the emulator has no model for (it knows esp32c3 and esp32s3)";
    }
    return false;
  }

  if (!options.partitionTablePath.empty() &&
      !flash_.loadPartitionTable(options.partitionTablePath, error)) {
    return false;
  }

  limit_ = options.instructionLimit;
  seed_ = options.seed;
  profiling_ = options.profile;
  bus_ = std::make_unique<Bus>(*soc_, flash_);
  rom_ = std::make_unique<Rom>(*bus_, *soc_);

  std::string romPath = options.romSymbolDir;
  if (romPath.empty()) romPath = "tools/simulator/emu/rom";
  if (!rom_->loadSymbols(romPath + "/" + soc_->name + ".romsyms", error)) return false;

  switch (soc_->arch) {
    case Arch::RiscV:
      cpu_ = std::make_unique<RiscvCore>(*bus_, *soc_, rom_.get());
      break;
    case Arch::Xtensa: {
      cpu_ = std::make_unique<XtensaCore>(*bus_, *soc_, rom_.get(), 0);
      if (soc_->cores > 1) {
        // The APP CPU is held in reset by the hardware until the PRO CPU
        // releases it, so it is created parked and started by the ROM hook
        // below. An IDF app blocks in start_other_core() until it reports in.
        auto app = std::make_unique<XtensaCore>(*bus_, *soc_, rom_.get(), 1);
        app->park();
        cpu1_ = std::move(app);
      }
      break;
    }
  }

  attachPeripherals();

  rom_->setOutput([this](const char* text, size_t length) {
    if (console_) console_("rom", text, length);
  });
  rom_->hooks().delay = [this](uint32_t us) { timeUs_ += us; };
  rom_->hooks().routeInterrupt = [this](int cpu, int peripheral, int line) {
    if (matrix_) matrix_->route(cpu, peripheral, line);
  };
  rom_->hooks().appCpuBoot = [this](uint32_t address) {
    // This is the ROM call the PRO CPU makes to say where the APP CPU should
    // begin. On silicon the app core is already out of reset and spinning in
    // ROM waiting for exactly this; here it starts running from it.
    if (!cpu1_ || !address) return;
    auto* app = dynamic_cast<XtensaCore*>(cpu1_.get());
    if (app && app->parked()) app->start(address);
  };
  rom_->initializeRomData();

  if (!loadSegments(error)) return false;

  for (uint32_t address : options.breakpoints) cpu_->addBreakpoint(address);
  cpu_->reset(flash_.app().entry);
  return true;
}

void EmuMachine::attachPeripherals() {
  // One model per 4 KB page across the peripheral window, named after the
  // block it belongs to. A page with a real model gets it; the rest get the
  // read-back-and-record fallback, which is what makes an unmodelled register
  // visible rather than silently zero.
  const std::vector<NamedBlock> blocks = namedBlocks(*soc_);
  for (uint32_t offset = 0; offset < soc_->peripheral.size; offset += 0x1000) {
    const uint32_t base = soc_->peripheral.base + offset;
    // RTC memory lives inside the peripheral window on some parts; it is
    // memory, and was already mapped as such.
    if (inRange(soc_->rtcFast, base) || inRange(soc_->rtcSlow, base)) continue;

    std::string name;
    for (const NamedBlock& block : blocks) {
      if (base == (block.base & ~0xFFFu)) {
        name = block.name;
        break;
      }
    }
    if (name.empty()) name = "peripheral@" + hex(base);
    auto device = std::make_unique<GenericPeripheral>(name, base, 0x1000);
    generic_.push_back(device.get());
    recording_.emplace_back(base, device.get());
    bus_->attach(base, 0x1000, std::move(device));
  }

  // Replacing a fallback with a real model drops the fallback's recording, so
  // each specific device registers in its place.
  auto replaceRecording = [this](uint32_t base, GenericPeripheral* device) {
    for (auto& entry : recording_) {
      if (entry.first == base) {
        entry.second = device;
        return;
      }
    }
    recording_.emplace_back(base, device);
  };
  {
    auto device = std::make_unique<UartDevice>("UART0", soc_->uart0,
                                               [this](const char* text, size_t length) {
                                                 if (console_) console_("uart0", text, length);
                                               });
    replaceRecording(soc_->uart0, device.get());
    bus_->attach(soc_->uart0, 0x1000, std::move(device));
  }
  {
    auto device = std::make_unique<UartDevice>("UART1", soc_->uart1,
                                               [this](const char* text, size_t length) {
                                                 if (console_) console_("uart1", text, length);
                                               });
    replaceRecording(soc_->uart1, device.get());
    bus_->attach(soc_->uart1, 0x1000, std::move(device));
  }

  if (soc_->usbSerialJtag) {
    bus_->attach(soc_->usbSerialJtag, 0x1000,
                 std::make_unique<UsbSerialJtagDevice>(soc_->usbSerialJtag,
                                                       [this](const char* text, size_t length) {
                                                         if (console_) console_("usb", text, length);
                                                       }));
  }

  if (soc_->saradc) {
    bus_->attach(soc_->saradc, 0x1000,
                 std::make_unique<SarAdcDevice>(soc_->saradc, [this](int unit, int channel) {
                   return board_ ? board_->adcSample(unit, channel) : 4095u;
                 }));
  }

  auto matrix = std::make_unique<InterruptMatrixDevice>(*soc_, *cpu_, cpu1_.get());
  matrix_ = matrix.get();
  replaceRecording(soc_->interrupt, matrix_);
  bus_->attach(soc_->interrupt, 0x1000, std::move(matrix));

  bus_->attach(soc_->apbCtrl, 0x1000, std::make_unique<SysconDevice>(soc_->apbCtrl, seed_));

  if (soc_->analogMaster) {
    auto device = std::make_unique<AnalogMasterDevice>(soc_->analogMaster, soc_->analogMasterStatus,
                                                       soc_->analogMasterReadyBit);
    replaceRecording(soc_->analogMaster, device.get());
    bus_->attach(soc_->analogMaster, 0x1000, std::move(device));
  }

  if (soc_->extmem && soc_->cacheStateReg) {
    auto device = std::make_unique<CacheControlDevice>(soc_->extmem, soc_->cacheStateReg,
                                                       soc_->cacheIdleValue);
    replaceRecording(soc_->extmem, device.get());
    bus_->attach(soc_->extmem, 0x1000, std::move(device));
  }

  // The four "interrupt from CPU" sources sit consecutively in the matrix; the
  // first one's index is its map register's offset, as for every other source.
  {
    auto device = std::make_unique<SystemDevice>(soc_->system, matrix_,
                                                 static_cast<int>(soc_->fromCpuIntSource),
                                                 soc_->systemFromCpu0);
    replaceRecording(soc_->system, device.get());
    bus_->attach(soc_->system, 0x1000, std::move(device));
  }

  auto now = [this]() { return timeUs_; };
  bus_->attach(soc_->timerGroup0, 0x1000,
               std::make_unique<TimerGroupDevice>("TIMG0", soc_->timerGroup0, now, 40000000));
  bus_->attach(soc_->timerGroup1, 0x1000,
               std::make_unique<TimerGroupDevice>("TIMG1", soc_->timerGroup1, now, 40000000));
  {
    auto device = std::make_unique<RtcCntlDevice>(soc_->rtcCntl, now, [this](int unit, int channel) {
      return board_ ? board_->adcSample(unit, channel) : 4095u;
    });
    device->setSleepHook([this](uint64_t requested) {
      asleep_ = true;
      sleepRequestUs_ = requested;
      cpu_->stop(HaltReason::Stopped, "the firmware put the chip into deep sleep");
      if (cpu1_) cpu1_->stop(HaltReason::Stopped, "the firmware put the chip into deep sleep");
    });
    replaceRecording(soc_->rtcCntl, device.get());
    bus_->attach(soc_->rtcCntl, 0x1000, std::move(device));
  }

  // The system timer's three comparators occupy consecutive slots in the
  // interrupt matrix; the first one's index is its map register's offset.
  const int systimerSource = static_cast<int>(soc_->systimerIntSource);
  auto systimer = std::make_unique<SystimerDevice>(soc_->systimer, now, matrix_, systimerSource);
  systimer_ = systimer.get();
  replaceRecording(soc_->systimer, systimer_);
  bus_->attach(soc_->systimer, 0x1000, std::move(systimer));

  // SPI0 shares the flash with the cache; SPI1 is the one software drives.
  bus_->attach(soc_->spi1, 0x1000,
               std::make_unique<FlashControllerDevice>("SPI1 (flash)", soc_->spi1, flash_));
  bus_->attach(soc_->spi0, 0x1000,
               std::make_unique<FlashControllerDevice>("SPI0 (cache)", soc_->spi0, flash_));

  bus_->attach(soc_->mmu.tableBase, 0x1000, std::make_unique<MmuTableDevice>(*bus_));

  // ── The pins, and what hangs off them ──────────────────────────────────────
  // These are the blocks that make an emulated image a *device* rather than a
  // program: the pads, the two buses the board's peripherals sit on, and the
  // DMA a frame of pixels travels through.
  {
    auto device = std::make_unique<IoMuxDevice>(*soc_, board_);
    replaceRecording(soc_->ioMux, device.get());
    bus_->attach(soc_->ioMux, 0x1000, std::move(device));
  }
  {
    auto device = std::make_unique<GpioDevice>(*soc_, board_, matrix_);
    gpio_ = device.get();
    replaceRecording(soc_->gpio, gpio_);
    bus_->attach(soc_->gpio, 0x1000, std::move(device));
  }
  if (soc_->gdma) {
    auto device = std::make_unique<GdmaDevice>(*soc_, *bus_, matrix_);
    gdma_ = device.get();
    replaceRecording(soc_->gdma, gdma_);
    bus_->attach(soc_->gdma, 0x1000, std::move(device));
  }
  {
    auto device = std::make_unique<I2cMasterDevice>("I2C0", soc_->i2c0, 0, board_, matrix_,
                                                    static_cast<int>(soc_->i2c0IntSource));
    replaceRecording(soc_->i2c0, device.get());
    bus_->attach(soc_->i2c0, 0x1000, std::move(device));
  }
  if (soc_->i2c1) {
    auto device = std::make_unique<I2cMasterDevice>("I2C1", soc_->i2c1, 1, board_, matrix_,
                                                    static_cast<int>(soc_->i2c1IntSource));
    replaceRecording(soc_->i2c1, device.get());
    bus_->attach(soc_->i2c1, 0x1000, std::move(device));
  }
  {
    // GDMA numbers SPI2 as peripheral 0 and SPI3 as 1; a driver binds a
    // channel to one of those and the SPI block pulls through whichever it
    // finds bound to itself.
    auto device = std::make_unique<SpiMasterDevice>("SPI2", soc_->spi2, 2, 0, board_, gdma_, matrix_,
                                                    static_cast<int>(soc_->spi2IntSource));
    replaceRecording(soc_->spi2, device.get());
    bus_->attach(soc_->spi2, 0x1000, std::move(device));
  }
  if (soc_->spi3) {
    auto device = std::make_unique<SpiMasterDevice>("SPI3", soc_->spi3, 3, 1, board_, gdma_, matrix_,
                                                    static_cast<int>(soc_->spi3IntSource));
    replaceRecording(soc_->spi3, device.get());
    bus_->attach(soc_->spi3, 0x1000, std::move(device));
  }
  if (soc_->sdmmc) {
    auto device = std::make_unique<SdmmcHostDevice>(soc_->sdmmc, *bus_, board_, matrix_,
                                                    static_cast<int>(soc_->sdmmcIntSource));
    replaceRecording(soc_->sdmmc, device.get());
    bus_->attach(soc_->sdmmc, 0x1000, std::move(device));
  }
}

bool EmuMachine::loadSegments(std::string* error) {
  const AppImage& app = flash_.app();
  for (const ImageSegment& segment : app.segments) {
    const uint32_t flashAddress = flash_.appOffset() + segment.fileOffset;

    // Cached segments are not copied: they are mapped, page by page, the way
    // the bootloader maps them. Copying would work until the first time the
    // firmware remapped a page itself, and then diverge invisibly.
    if (inRange(soc_->iromCache, segment.addr) || inRange(soc_->dromCache, segment.addr)) {
      if (!bus_->mapFlash(segment.addr, flashAddress, segment.length, error)) return false;
      continue;
    }

    std::vector<uint8_t> data(segment.length);
    flash_.read(flashAddress, data.data(), data.size());
    for (uint32_t i = 0; i < segment.length; ++i) {
      bus_->write8(segment.addr + i, data[i]);
      if (bus_->faulted()) {
        if (error) {
          *error = "segment at " + hex(segment.addr) + " does not fit the chip's memory map: " +
                   bus_->fault().detail;
        }
        return false;
      }
    }
  }
  return true;
}

uint64_t EmuMachine::run(uint64_t budget) {
  if (!cpu_) return 0;
  const uint64_t before = cpu_->retired();
  uint64_t done = 0;
  Cpu* cores[2] = {cpu_.get(), cpu1_.get()};

  while (done < budget && !stopped()) {
    // Slices are short enough that a timer comparator is serviced within a few
    // microseconds of when it should be, which is what keeps the firmware's own
    // timing sane without checking the timers on every instruction.
    // Profiling shortens the slice: the sample is taken between slices, so the
    // slice length *is* the sampling interval.
    const uint64_t slice = std::min<uint64_t>(budget - done, profiling_ ? 251 : 20000);

    // An idle core wakes when the matrix raises something it has enabled — and
    // not before, so a core sitting in its wait instruction neither costs host
    // time nor runs on past the wait.
    for (Cpu* core : cores) {
      if (core && core->halted() == HaltReason::WaitingForInterrupt && core->hasPendingInterrupt()) {
        core->resume();
      }
    }

    uint64_t ran = 0;
    for (Cpu* core : cores) {
      if (!core || core->halted() != HaltReason::Running) continue;
      ran = std::max(ran, core->run(slice));
    }

    // Charge the instructions to the clock. One instruction per cycle is
    // optimistic for a real pipeline, but the alternative — no time passing
    // while code runs — makes every timeout in the firmware behave wrongly.
    // The cores run at the same clock at the same time, so a slice costs what
    // the busiest of them took rather than the sum.
    if (profiling_ && ran) {
      for (int index = 0; index < 2; ++index) {
        Cpu* core = cores[index];
        if (!core || core->halted() != HaltReason::Running) continue;
        const uint64_t key = (static_cast<uint64_t>(index) << 32) | core->pc();
        HotSpot& spot = profile_[key];
        spot.pc = core->pc();
        spot.returnAddress = core->returnAddress();
        spot.core = index;
        ++spot.samples;
        ++profileSamples_;
      }
    }

    done += ran;
    timeUs_ += (ran * 1000000ULL) / soc_->cpuHz;
    if (systimer_) systimer_->tick(timeUs_);
    // A pin can change without the firmware touching a register — a button
    // going down, the panel dropping BUSY — and an interrupt armed on that
    // edge has to fire anyway. Nothing else would ever look.
    if (gpio_) gpio_->pollInputs();

    if (ran == 0) {
      bool waiting = false;
      for (Cpu* core : cores) {
        if (core && core->halted() == HaltReason::WaitingForInterrupt) waiting = true;
      }
      if (!waiting) {
        // Nothing ran and nothing is waiting for an interrupt, so nothing is
        // ever going to happen. Returning zero here would look to the caller
        // like the end of a budget; saying so makes it a fault with a cause.
        std::string detail = "no core made progress and none is waiting for an interrupt";
        for (int index = 0; index < 2; ++index) {
          if (!cores[index]) continue;
          char buf[96];
          std::snprintf(buf, sizeof(buf), "; core %d at 0x%08X (%s)", index, cores[index]->pc(),
                        cores[index]->haltDetail().empty() ? "running"
                                                           : cores[index]->haltDetail().c_str());
          detail += buf;
        }
        cpu_->stop(HaltReason::Stopped, detail);
        break;
      }
      // Nothing will happen until a timer fires, so go straight there rather
      // than spending host time simulating an idle CPU. A machine with no
      // armed comparator would idle forever, and says so.
      const uint64_t deadline = systimer_ ? systimer_->nextDeadlineUs() : 0;
      if (deadline == 0) {
        cpu_->stop(HaltReason::Stopped,
                   "the firmware is waiting for an interrupt that nothing will raise");
        break;
      }
      timeUs_ += deadline;
      if (systimer_) systimer_->tick(timeUs_);
      for (Cpu* core : cores) {
        if (core && core->halted() == HaltReason::WaitingForInterrupt) core->resume();
      }
      continue;
    }

    if (limit_ && cpu_->retired() - before >= limit_) {
      cpu_->stop(HaltReason::Stopped,
                 "instruction limit reached (" + std::to_string(limit_) + ") — the firmware is "
                 "running but produced no result in that budget");
      break;
    }
  }
  return done;
}

bool EmuMachine::stopped() const {
  if (!cpu_) return true;
  // Either core ending the run ends it for the machine: an APP CPU that faults
  // takes the application down with it, and reporting only the PRO CPU's view
  // would leave the run hanging on a core that is never coming back.
  const Cpu* cores[2] = {cpu_.get(), cpu1_.get()};
  for (const Cpu* core : cores) {
    if (!core) continue;
    const HaltReason reason = core->halted();
    if (reason != HaltReason::Running && reason != HaltReason::WaitingForInterrupt) return true;
  }
  return false;
}

std::string EmuMachine::stopReason() const {
  if (!cpu_) return "no firmware loaded";
  // Report whichever core actually stopped: on a dual-core part the PRO CPU is
  // often sitting in a perfectly ordinary wait while the APP CPU is the one
  // that hit something.
  const Cpu* core = cpu_.get();
  for (const Cpu* candidate : {cpu_.get(), cpu1_.get()}) {
    if (!candidate) continue;
    const HaltReason reason = candidate->halted();
    if (reason != HaltReason::Running && reason != HaltReason::WaitingForInterrupt) {
      core = candidate;
      break;
    }
  }
  switch (core->halted()) {
    case HaltReason::Running: return "running";
    case HaltReason::WaitingForInterrupt: return "waiting for an interrupt";
    default: break;
  }
  std::string message = core->haltDetail();
  if (bus_ && bus_->faulted() && message != bus_->fault().detail) {
    message += " (bus: " + bus_->fault().detail + ")";
  }
  return message + "\n  " + core->describeState();
}

std::string EmuMachine::firstExceptionReport() const {
  // Both cores keep one; whichever took an exception first is the one worth
  // reporting, and on a single-core part there is only ever one.
  struct Report {
    bool seen = false;
    uint32_t cause = 0;
    uint32_t pc = 0;
    uint32_t detail = 0;
    const char* detailName = "";
    std::string name;
    int core = 0;
    std::vector<std::pair<uint32_t, uint32_t>> trace;
  };
  Report best;

  auto fromRiscv = [](const RiscvCore* core) {
    Report out;
    if (!core || !core->firstException().seen) return out;
    const RiscvCore::FirstException& first = core->firstException();
    // The RISC-V machine-mode exception causes, in the firmware's own terms.
    const char* name = "exception";
    switch (first.cause) {
      case 0: name = "instruction address misaligned"; break;
      case 1: name = "instruction access fault"; break;
      case 2: name = "illegal instruction"; break;
      case 3: name = "breakpoint"; break;
      case 4: name = "load address misaligned"; break;
      case 5: name = "load access fault"; break;
      case 6: name = "store address misaligned"; break;
      case 7: name = "store access fault"; break;
      case 11: name = "environment call"; break;
      default: break;
    }
    out.seen = true;
    out.cause = first.cause;
    out.pc = first.pc;
    out.detail = first.tval;
    out.detailName = "mtval";
    out.name = name;
    out.trace = first.trace;
    return out;
  };
  auto fromXtensa = [](const XtensaCore* core) {
    Report out;
    if (!core || !core->firstException().seen) return out;
    const XtensaCore::FirstException& first = core->firstException();
    out.seen = true;
    out.cause = first.cause;
    out.pc = first.pc;
    out.detail = first.vaddr;
    out.detailName = "excvaddr";
    out.name = XtensaCore::causeName(first.cause);
    out.core = core->coreId();
    out.trace = first.trace;
    return out;
  };

  for (const Cpu* candidate : {cpu_.get(), cpu1_.get()}) {
    if (!candidate) continue;
    Report report = fromRiscv(dynamic_cast<const RiscvCore*>(candidate));
    if (!report.seen) report = fromXtensa(dynamic_cast<const XtensaCore*>(candidate));
    if (!report.seen) continue;
    if (!best.seen) best = report;
  }
  if (!best.seen) return {};

  char buf[320];
  std::string where = soc_ && soc_->cores > 1 ? (best.core == 0 ? "on the PRO CPU " : "on the APP CPU ")
                                              : std::string();
  std::snprintf(buf, sizeof(buf), "%s %sat 0x%08X (%s 0x%08X), in %s", best.name.c_str(),
                where.c_str(), best.pc, best.detailName, best.detail,
                bus_->describe(best.pc).c_str());
  std::string out = buf;
  if (!best.trace.empty()) {
    out += "\n  how it got there (most recent last):";
    for (const auto& call : best.trace) {
      std::snprintf(buf, sizeof(buf), "\n    0x%08X -> 0x%08X", call.first, call.second);
      out += buf;
    }
  }
  return out;
}

std::vector<std::string> EmuMachine::gaps() const {
  std::vector<std::string> out;
  if (rom_) {
    for (const std::string& name : rom_->missing()) out.push_back("ROM: " + name);
  }
  if (bus_ && bus_->faulted()) out.push_back("bus: " + bus_->fault().detail);
  return out;
}

std::vector<RegisterAccess> EmuMachine::registersAt(uint32_t base) const {
  const uint32_t page = base & ~0xFFFu;
  for (const auto& entry : recording_) {
    if (entry.first != page) continue;
    std::vector<RegisterAccess> accesses = entry.second->accesses();
    std::sort(accesses.begin(), accesses.end(), [](const RegisterAccess& a, const RegisterAccess& b) {
      return a.address == b.address ? (a.write && !b.write) : a.address < b.address;
    });
    return accesses;
  }
  return {};
}

std::vector<std::pair<uint32_t, std::string>> EmuMachine::modelledBlocks() const {
  std::vector<std::pair<uint32_t, std::string>> out;
  for (const auto& entry : recording_) out.emplace_back(entry.first, entry.second->name());
  std::sort(out.begin(), out.end());
  return out;
}

std::vector<EmuMachine::HotSpot> EmuMachine::hotSpots(size_t limit) const {
  std::vector<HotSpot> out;
  out.reserve(profile_.size());
  for (const auto& entry : profile_) out.push_back(entry.second);
  std::sort(out.begin(), out.end(),
            [](const HotSpot& a, const HotSpot& b) { return a.samples > b.samples; });
  if (out.size() > limit) out.resize(limit);
  return out;
}

std::vector<EmuMachine::Task> EmuMachine::tasks() const {
  std::vector<Task> out;
  if (!bus_ || !cpu_) return out;

  // A FreeRTOS control block holds, in order: the saved stack pointer, two
  // list items, the priority, a pointer to the bottom of the task's stack, and
  // then the task's name. The name is what makes one findable without symbols:
  // a short printable string immediately after a word that points into RAM,
  // with a plausible priority before it and a saved stack pointer 48 bytes
  // earlier that lies inside the same stack.
  constexpr uint32_t kPxStackToName = 4;
  constexpr uint32_t kPriorityToPxStack = 4;
  constexpr uint32_t kTcbToPxStack = 48;
  constexpr size_t kMaxNameLength = 16;
  constexpr uint32_t kMaxPriority = 32;  // configMAX_PRIORITIES in IDF is 25

  const std::vector<Bus::RamRange> regions = bus_->ramRegions();
  auto inRam = [&regions](uint32_t address) {
    for (const Bus::RamRange& region : regions) {
      if (address >= region.base && address - region.base < region.size) return true;
    }
    return false;
  };
  auto word = [this](uint32_t address, uint32_t* value) {
    std::vector<uint8_t> bytes;
    if (!bus_->peek(address, 4, &bytes)) return false;
    *value = static_cast<uint32_t>(bytes[0]) | (static_cast<uint32_t>(bytes[1]) << 8) |
             (static_cast<uint32_t>(bytes[2]) << 16) | (static_cast<uint32_t>(bytes[3]) << 24);
    return true;
  };

  for (const Bus::RamRange& region : regions) {
    if (region.alias) continue;  // the same bytes, already walked
    for (uint32_t offset = kTcbToPxStack; offset + 64 < region.size; offset += 4) {
      const uint32_t at = region.base + offset;
      uint32_t stack = 0;
      if (!word(at, &stack) || (stack & 3) != 0 || !inRam(stack)) continue;

      uint32_t priority = 0;
      if (!word(at - kPriorityToPxStack, &priority) || priority >= kMaxPriority) continue;

      uint32_t topOfStack = 0;
      if (!word(at - kTcbToPxStack, &topOfStack) || !inRam(topOfStack)) continue;
      // The saved stack pointer has to be standing in this task's own stack,
      // which is what tells a control block apart from any other pair of
      // pointers that happens to sit next to a string.
      if (topOfStack < stack || topOfStack - stack > 64 * 1024) continue;

      std::vector<uint8_t> name;
      if (!bus_->peek(at + kPxStackToName, kMaxNameLength, &name)) continue;
      size_t length = 0;
      while (length < kMaxNameLength && name[length] >= 0x20 && name[length] < 0x7F) ++length;
      // Three characters is the shortest FreeRTOS name that turns up in
      // practice ("tiT", "IDLE").
      if (length < 3 || length >= kMaxNameLength || name[length] != 0) continue;

      Task task;
      task.name.assign(name.begin(), name.begin() + static_cast<long>(length));
      task.stack = stack;
      task.tcb = at - kTcbToPxStack;
      task.topOfStack = topOfStack;
      task.priority = priority;
      // A suspended task's registers are pushed at the top of its stack, and
      // the saved program counter is the first thing in that frame on RISC-V
      // and the second on Xtensa (whose frame starts with its own exit hook).
      word(topOfStack + (soc_->arch == Arch::RiscV ? 0 : 4), &task.resumePc);
      // Everything still on the stack that could be a return address into the
      // firmware's own code. The application's routines live in the cached
      // flash window; the SDK's and IDF's hot paths live in IRAM.
      for (uint32_t probe = topOfStack; probe < topOfStack + 2048 && task.stackTrail.size() < 12;
           probe += 4) {
        uint32_t candidate = 0;
        if (!word(probe, &candidate)) break;
        const bool code = inRange(soc_->iromCache, candidate) ||
                          (candidate >= soc_->sram[0].iramBase &&
                           candidate < soc_->sram[0].iramBase + soc_->sram[0].size);
        if (!code || (candidate & 1)) continue;
        if (!task.stackTrail.empty() && task.stackTrail.back() == candidate) continue;
        task.stackTrail.push_back(candidate);
      }
      task.running = false;
      out.push_back(task);
    }
  }

  // The running task is the one whose stack the stack pointer is standing in.
  // Stacks are contiguous and grow down, so the owner is the task whose base
  // is the closest one below it.
  const uint32_t sp = cpu_->stackPointer();
  size_t best = out.size();
  uint32_t bestBase = 0;
  for (size_t index = 0; index < out.size(); ++index) {
    if (out[index].stack <= sp && out[index].stack >= bestBase) {
      bestBase = out[index].stack;
      best = index;
    }
  }
  if (best < out.size()) out[best].running = true;

  std::sort(out.begin(), out.end(), [](const Task& a, const Task& b) { return a.stack < b.stack; });
  return out;
}

std::vector<EmuMachine::BlockUse> EmuMachine::blocksTouched() const {
  std::vector<BlockUse> out;
  for (const auto& entry : recording_) {
    uint64_t total = 0;
    for (const RegisterAccess& access : entry.second->accesses()) total += access.count;
    if (total == 0) continue;
    const bool fallback =
        std::find(generic_.begin(), generic_.end(), entry.second) != generic_.end();
    out.push_back({entry.first, entry.second->name(), total, !fallback});
  }
  std::sort(out.begin(), out.end(),
            [](const BlockUse& a, const BlockUse& b) { return a.accesses > b.accesses; });
  return out;
}

std::vector<InterruptMatrixDevice::SourceUse> EmuMachine::interruptsUsed() const {
  return matrix_ ? matrix_->sourcesUsed() : std::vector<InterruptMatrixDevice::SourceUse>{};
}

std::vector<RegisterAccess> EmuMachine::unmodelledRegisters(size_t limit) const {
  std::vector<RegisterAccess> all;
  for (const GenericPeripheral* device : generic_) {
    const std::vector<RegisterAccess> accesses = device->accesses();
    all.insert(all.end(), accesses.begin(), accesses.end());
  }
  std::sort(all.begin(), all.end(),
            [](const RegisterAccess& a, const RegisterAccess& b) { return a.count > b.count; });
  if (all.size() > limit) all.resize(limit);
  return all;
}

}  // namespace freeink::sim::emu
