#pragma once

// FreeInk emulator — the address space.
//
// Every load, store and instruction fetch goes through here, so the shape of
// this file is mostly about not being slow: a flat first-level table indexed by
// the top 12 address bits, and 4 KB pages below that for the peripheral block.
// A RAM access costs a table lookup and a pointer add.
//
// Two things are more than plumbing:
//
//   * **The flash MMU is real.** The cached instruction and data windows are
//     not preloaded copies of the image — they translate through the chip's
//     64 KB page table into the flash model, exactly as the silicon does. That
//     is what makes `spi_flash_mmap()`, OTA reads and an app that remaps its
//     own pages behave correctly instead of reading stale bytes.
//
//   * **An unmapped access is a fault, not a crash.** It stops the core and is
//     reported with the address, the PC and what the region would have been, so
//     a firmware bug reads as a firmware bug rather than as the emulator dying.

#include "Image.h"
#include "Soc.h"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace freeink::sim::emu {

// A memory-mapped peripheral. Registers are 32 bits wide; byte and halfword
// accesses are widened by the bus, with `mask` marking the bytes written, so a
// device model never has to think about access width.
class MmioDevice {
 public:
  virtual ~MmioDevice() = default;
  virtual const char* name() const = 0;
  virtual uint32_t read(uint32_t offset) = 0;
  virtual void write(uint32_t offset, uint32_t value, uint32_t mask) = 0;
  // Called once per emulated millisecond for devices that keep time.
  virtual void tick(uint64_t /*nowUs*/) {}
};

enum class FaultKind { None, Unmapped, UnmappedFetch, MmuInvalid, Misaligned, Unimplemented };

struct Fault {
  FaultKind kind = FaultKind::None;
  uint32_t address = 0;
  uint32_t pc = 0;
  std::string detail;
};

class Bus {
 public:
  Bus(const SocDesc& soc, FlashImage& flash);
  ~Bus();

  const SocDesc& soc() const { return soc_; }
  FlashImage& flash() { return flash_; }

  // ── Wiring ─────────────────────────────────────────────────────────────────
  // Devices are owned by the bus so peripheral lifetimes match the machine's.
  void attach(uint32_t base, uint32_t size, std::unique_ptr<MmioDevice> device);
  // A block of host-backed memory, optionally visible at a second address
  // (the instruction/data aliases of internal SRAM).
  void addRam(uint32_t base, uint32_t size, const char* name);
  void aliasRam(uint32_t existingBase, uint32_t aliasBase, uint32_t size);

  // ── Access ─────────────────────────────────────────────────────────────────
  uint8_t read8(uint32_t addr);
  uint16_t read16(uint32_t addr);
  uint32_t read32(uint32_t addr);
  void write8(uint32_t addr, uint8_t value);
  void write16(uint32_t addr, uint16_t value);
  void write32(uint32_t addr, uint32_t value);

  // Host pointer for a run of bytes, or nullptr if the range is not plain
  // memory. Used for instruction fetch and for bulk copies (ROM memcpy, SPI
  // DMA) that would otherwise cost a bus round trip per byte.
  const uint8_t* hostRead(uint32_t addr, uint32_t length);
  uint8_t* hostWrite(uint32_t addr, uint32_t length);

  // Reads that must not fault — used by the debugger surface (`sim mem`), which
  // should be able to poke at anything without stopping the machine.
  bool peek(uint32_t addr, uint32_t length, std::vector<uint8_t>* out);

  // ── MMU ────────────────────────────────────────────────────────────────────
  // Maps `length` bytes of flash at `paddr` to `vaddr`, as the bootloader does
  // for the app's cached segments. Both must be 64 KB aligned in their low bits.
  bool mapFlash(uint32_t vaddr, uint32_t paddr, uint32_t length, std::string* error);
  void setMmuEntry(uint32_t index, uint32_t value);
  uint32_t mmuEntry(uint32_t index) const;
  uint32_t mmuEntryCount() const { return soc_.mmu.entryCount; }
  // vaddr -> flash offset, or false when the page is not mapped.
  bool translate(uint32_t vaddr, uint32_t* paddr) const;

  // ── Faults ─────────────────────────────────────────────────────────────────
  bool faulted() const { return fault_.kind != FaultKind::None; }
  const Fault& fault() const { return fault_; }
  void clearFault() { fault_ = Fault{}; }
  void raise(FaultKind kind, uint32_t address, const std::string& detail);
  // The core keeps this current so a fault report names the instruction.
  void setPc(uint32_t pc) { pc_ = pc; }
  uint32_t pc() const { return pc_; }

  // Human-readable name for an address, for fault messages and traces:
  // "IRAM", "peripheral SPI2+0x14", "flash-mapped DROM page 42".
  std::string describe(uint32_t addr) const;

  // Accounting the status command reports.
  uint64_t unmappedReads() const { return unmappedReads_; }

  // The chip's own RAM. Used by the diagnostics that go looking through a
  // firmware's memory for structures it has no symbols for — which need both
  // views: one to walk (an alias would find everything twice) and both to
  // recognise a pointer, since the same bytes are addressed either way.
  struct RamRange {
    uint32_t base;
    uint32_t size;
    bool alias;  // the second window onto a block already listed
  };
  std::vector<RamRange> ramRegions() const;

 private:
  enum class Kind : uint8_t { Unmapped, Ram, Mmio, FlashCache, Rom };

  struct Page {  // one 4 KB page
    Kind kind = Kind::Unmapped;
    uint8_t* host = nullptr;     // for Ram: host address of this page
    MmioDevice* device = nullptr;
    uint32_t deviceBase = 0;
  };

  struct RamRegion {
    uint32_t base;
    uint32_t size;
    std::string name;
    std::vector<uint8_t> storage;  // empty for an alias
    std::vector<uint8_t>* backing;
  };

  static constexpr uint32_t kPageBits = 12;
  static constexpr uint32_t kPageSize = 1u << kPageBits;

  Page* pageFor(uint32_t addr);
  const Page* pageFor(uint32_t addr) const;
  void mapPages(uint32_t base, uint32_t size, const Page& prototype, bool ramWalk);
  uint32_t mmuIndexFor(uint32_t vaddr) const;

  const SocDesc& soc_;
  FlashImage& flash_;
  // Sparse first level: 1 MB per entry, allocated on demand, so the table costs
  // a few pages rather than 8 MB of empty Page structs.
  std::vector<std::unique_ptr<Page[]>> level1_;
  std::vector<std::unique_ptr<RamRegion>> ram_;
  std::vector<std::unique_ptr<MmioDevice>> devices_;
  std::vector<uint32_t> mmuTable_;

  Fault fault_;
  uint32_t pc_ = 0;
  uint64_t unmappedReads_ = 0;
};

}  // namespace freeink::sim::emu
