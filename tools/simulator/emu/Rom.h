#pragma once

// FreeInk emulator — mask ROM, emulated at the call rather than the
// instruction.
//
// An ESP32 app is not self-contained. It calls into the chip's mask ROM for
// libc (memcpy, strlen, printf), for libgcc's 64-bit and soft-float helpers,
// for early UART and GPIO setup, and for flash access — several hundred
// entry points at fixed addresses. The ROM itself is a binary Espressif
// publishes separately, and this emulator deliberately ships no blob.
//
// So calls into ROM address space are intercepted: the core notices its PC has
// entered the ROM window, this layer looks the address up in a symbol map
// (generated from ESP-IDF's published linker scripts — addresses and names, no
// code), runs a native implementation against the guest's registers and
// memory, and returns to the caller.
//
// The honest limitation is that only implemented routines work. An address
// with no implementation stops the machine and says which routine it was, by
// name, which is a diagnostic rather than a mystery: "ROM routine
// `esp_rom_spiflash_erase_block` (0x40000148) is not implemented". Adding one
// is a table entry.

#include "Cpu.h"

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace freeink::sim::emu {

class Machine;

// What a ROM routine needs from the rest of the machine. Filled in by the
// emulated machine when it wires its peripherals up; every hook is optional,
// and an unset one simply makes its routine a no-op that returns success.
//
// These are the ROM routines that are not pure computation — the ones that
// touch time, pins or the reset line — kept behind function objects so that
// Rom.cpp depends on nothing but the bus.
struct RomHooks {
  std::function<void(uint32_t us)> delay;
  std::function<void(const char* reason)> reset;
  std::function<void(int gpio, uint32_t signal, bool invert, bool oenInvert)> matrixOut;
  std::function<void(int gpio, uint32_t signal, bool invert)> matrixIn;
  std::function<void(int gpio)> padSelect;
  // mode: 0 float, 1 pull-up, 2 pull-down.
  std::function<void(int gpio, int mode)> padPull;
  std::function<void(int gpio, bool hold)> padHold;
  std::function<void(int cpu, int peripheral, int line)> routeInterrupt;
  std::function<void(uint32_t address)> appCpuBoot;
};

class Rom {
 public:
  Rom(Bus& bus, const SocDesc& soc);

  // Loads `<chip>.romsyms`, as produced by rom/import-rom-symbols.py.
  bool loadSymbols(const std::string& path, std::string* error);
  size_t symbolCount() const { return byAddress_.size(); }
  // The ROM symbol at exactly this address, or nullptr.
  const char* symbolAt(uint32_t address) const;

  // Fills in the ROM-owned data structures an app expects to find already
  // initialised — the legacy flash chip descriptor above all, which the ROM
  // populates during its own boot and IDF reads as `g_rom_flashchip`.
  void initializeRomData();

  // Handles a call whose target is in ROM. Returns false if the address has no
  // implementation, leaving the caller to report it.
  bool call(Cpu& cpu, uint32_t address);

  // Where ets_printf and the UART character routines send their output.
  void setOutput(std::function<void(const char*, size_t)> sink) { output_ = std::move(sink); }
  RomHooks& hooks() { return hooks_; }

  // ets_delay_us and friends hand time back to the machine's clock rather than
  // spinning: a firmware that waits 100 ms for a panel must cost 100 ms of
  // simulated time, not 100 ms of the host's.
  void delayUs(uint32_t us) {
    if (hooks_.delay) hooks_.delay(us);
  }
  void requestReset(const char* reason) {
    if (hooks_.reset) hooks_.reset(reason);
  }

  // Names of ROM routines that were called but are not implemented, in call
  // order, for the status report.
  const std::vector<std::string>& missing() const { return missing_; }
  // Every implemented routine that has actually been called, with a count.
  const std::map<std::string, uint64_t>& callCounts() const { return callCounts_; }

  Bus& bus() { return bus_; }
  const SocDesc& soc() const { return soc_; }

  // ── Guest memory helpers, used by the handlers ─────────────────────────────
  std::string readString(uint32_t address, size_t limit = 4096);
  void writeBytes(uint32_t address, const void* data, size_t length);
  void readBytes(uint32_t address, void* data, size_t length);
  void emit(const std::string& text);
  // Writes through the firmware's installed character sink if there is one,
  // falling back to the console when there is not (before ets_install_putc1
  // runs, which is most of early boot).
  void emitThroughPutc(Cpu& cpu, const std::string& text);

  // The analog register file behind rom_i2c_readReg/writeReg: the PLL, the
  // regulators and the RF block are all poked through it during clock setup.
  // Modelled as storage that reads back what was written, which is enough for
  // the polling loops in rtc_clk to complete.
  uint8_t analogRead(uint8_t block, uint8_t reg);
  void analogWrite(uint8_t block, uint8_t reg, uint8_t value);

  // Copies a string into the ROM's scratch page and returns its guest address.
  // ROM routines that hand back a `const char *` need somewhere real to point
  // at: returning zero from one of those is not a harmless stub, it is a null
  // dereference a few hundred instructions later, in the caller's printf.
  uint32_t internString(const std::string& text);

  // Where the ROM-owned structures were put, and the MAC inside them.
  uint32_t romScratch() const { return romScratch_; }
  uint32_t macAddress() const { return romScratch_ ? romScratch_ + 32 : 0; }

  // The ROM's rand(). Seeded deterministically so two runs of the same
  // firmware produce the same sequence.
  void setRandomState(uint32_t state) { random_ = state; }
  uint32_t nextRandom() {
    random_ = random_ * 1103515245u + 12345u;
    return random_ >> 1;
  }

  uint32_t cpuFrequencyMhz() const { return cpuMhz_; }
  void setCpuFrequencyMhz(uint32_t mhz) { cpuMhz_ = mhz; }

  // ets_install_putc1 hands the ROM a character sink. The emulator keeps the
  // pointer for reporting but writes output directly rather than calling back
  // into guest code for every character — same bytes, a great deal faster.
  void setPutc1(uint32_t handler) { putc1_ = handler; }
  uint32_t putc1() const { return putc1_; }

 private:
  using Handler = void (*)(Rom&, Cpu&);
  // Defined in RomHandlers.cpp, which holds the implementations themselves;
  // the flash and MD5 groups live beside the models they need.
  void registerHandlers();
  void registerFlashHandlers();
  void initializeRomLayout(uint32_t address);
  void registerMd5Handlers();
  void bind(const char* name, Handler handler);

  Bus& bus_;
  const SocDesc& soc_;
  std::unordered_map<uint32_t, std::string> byAddress_;
  std::unordered_map<std::string, uint32_t> byName_;
  std::unordered_map<uint32_t, Handler> handlers_;
  std::unordered_map<std::string, Handler> handlersByName_;

  std::function<void(const char*, size_t)> output_;
  RomHooks hooks_;
  std::vector<std::string> missing_;
  std::map<std::string, uint64_t> callCounts_;
  std::map<uint16_t, uint8_t> analog_;
  uint32_t cpuMhz_ = 160;
  uint32_t romScratch_ = 0;
  uint32_t random_ = 1;
  uint32_t stringPool_ = 0;
  std::unordered_map<std::string, uint32_t> interned_;
  // ets_install_putc1 redirects console output at the character level.
  uint32_t putc1_ = 0;
};

// Formats a printf-style call against a source of 32-bit arguments. Two
// sources exist: the guest's argument registers (spilling to its stack), and a
// va_list, which on both these ABIs is a pointer walking the same words. One
// formatter serves ets_printf, the UART variants and the newlib family.
using GuestArgSource = std::function<uint32_t()>;
std::string formatGuestPrintf(Rom& rom, const std::string& format, const GuestArgSource& nextArg);
// Arguments from a CPU's registers, starting at `firstArgIndex`.
GuestArgSource cpuArgSource(Cpu& cpu, int firstArgIndex);
// Arguments from a va_list pointer in guest memory.
GuestArgSource vaListArgSource(Rom& rom, uint32_t vaList);

// The ROM's checksums, which NVS, the partition table and esp_ota all rely on.
uint32_t romCrc32Le(uint32_t crc, const uint8_t* data, size_t length);
uint16_t romCrc16Le(uint16_t crc, const uint8_t* data, size_t length);
uint8_t romCrc8Le(uint8_t crc, const uint8_t* data, size_t length);

}  // namespace freeink::sim::emu
