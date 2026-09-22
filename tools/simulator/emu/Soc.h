#pragma once

// FreeInk emulator — what a chip is.
//
// Everything downstream of here is chip-generic: the bus, the MMU, the flash
// model and the peripheral set all read their facts from one SocDesc. Adding a
// chip means adding a descriptor and a core, not editing the machinery.
//
// The numbers are the SoC's own, taken from ESP-IDF's soc headers
// (components/soc/<chip>/{include,register}/soc/*.h). Where a field cites a
// constant by name, that is the IDF macro it mirrors, so the two can be
// compared when a new IDF release moves something.

#include <cstddef>
#include <cstdint>

namespace freeink::sim::emu {

// esp_chip_id_t values from esp_app_format.h. The image header carries this,
// so a firmware .bin identifies its own silicon and the loader never guesses.
enum class Chip : uint16_t {
  Unknown = 0xFFFF,
  Esp32 = 0x0000,
  Esp32S2 = 0x0002,
  Esp32C3 = 0x0005,
  Esp32S3 = 0x0009,
  Esp32C2 = 0x000C,
  Esp32C6 = 0x000D,
  Esp32H2 = 0x0010,
  Esp32P4 = 0x0012,
};

enum class Arch { RiscV, Xtensa };

struct MemRange {
  uint32_t base = 0;
  uint32_t size = 0;
};

// One block of internal SRAM, and the two addresses it answers to.
//
// On these parts instruction and data buses see the *same* memory through
// different windows: C3 SRAM is at 0x3FC7C000 as data and 0x4037C000 as
// instructions. Modelling that as one backing store with two windows is not a
// detail — IDF puts .data at DRAM addresses and IRAM functions at IRAM
// addresses in the same bytes, and a heap block handed to a driver may be
// written through one view and executed through the other. Two independent
// buffers would silently diverge.
struct SramBlock {
  uint32_t iramBase;  // 0 when the block is data-only
  uint32_t dramBase;  // 0 when the block is instruction-only
  uint32_t size;
};

// The MMU maps 64 KB flash pages into the cached instruction and data buses.
// Its entry encoding differs per chip in ways that matter: on the C3 an entry
// is a page number with bit 8 meaning *invalid*, on the S3 the valid bit is
// bit 14 and bit 15 selects flash or PSRAM. Get this wrong and the app reads
// the wrong 64 KB of itself, which looks like random corruption rather than a
// mapping bug — hence keeping the semantics explicit per chip.
struct MmuDesc {
  uint32_t tableBase;     // DR_REG_MMU_TABLE
  uint32_t entryCount;    // SOC_MMU_ENTRY_NUM
  uint32_t pageSize;      // SPI_FLASH_MMU_PAGE_SIZE — 64 KB on every chip here
  uint32_t valueMask;     // SOC_MMU_VALID_VAL_MASK — the page-number field
  uint32_t invalidBit;    // SOC_MMU_INVALID
  uint32_t typeBit;       // SOC_MMU_TYPE: set = PSRAM rather than flash (0 if absent)
  uint32_t linearMask;    // SOC_MMU_LINEAR_ADDR_MASK
  uint32_t dbusBase;      // SOC_MMU_DBUS_VADDR_BASE
  uint32_t ibusBase;      // SOC_MMU_IBUS_VADDR_BASE
};

struct SocDesc {
  Chip chip;
  Arch arch;
  const char* name;
  int cores;

  SramBlock sram[4];   // terminated by a zero-size entry
  MemRange iromCache;  // SOC_IRAM0_CACHE_ADDRESS_LOW/HIGH — flash-backed via MMU
  MemRange dromCache;  // SOC_DRAM0_CACHE_ADDRESS_LOW/HIGH — flash-backed via MMU
  // The stack the mask ROM runs on, and leaves the app running on until
  // FreeRTOS gives each task its own. Starting the emulated app anywhere else
  // puts its earliest stack frames on top of memory the ROM owns, which shows
  // up much later as a corrupted return address.
  MemRange romStack;   // SOC_ROM_STACK_START - SOC_ROM_STACK_SIZE, SOC_ROM_STACK_SIZE
  MemRange rtcFast;    // RTC fast memory as the CPU sees it
  MemRange rtcSlow;    // RTC slow memory
  MemRange rom;        // mask ROM: never executed, calls into it are intercepted
  MemRange romData;    // mask ROM data, if the chip maps it separately
  MemRange peripheral;

  MmuDesc mmu;

  // Register bases the peripheral models bind to (DR_REG_*_BASE).
  uint32_t uart0;
  uint32_t uart1;
  uint32_t gpio;
  uint32_t ioMux;
  uint32_t spi0;
  uint32_t spi1;
  uint32_t spi2;
  uint32_t spi3;  // 0 when the chip has no third SPI
  uint32_t i2c0;
  uint32_t i2c1;  // 0 when absent
  uint32_t rtcCntl;
  uint32_t efuse;
  uint32_t timerGroup0;
  uint32_t timerGroup1;
  uint32_t systimer;
  uint32_t interrupt;  // DR_REG_INTERRUPT_BASE — the interrupt matrix
  uint32_t system;     // DR_REG_SYSTEM_BASE — clock and reset gating
  uint32_t extmem;     // DR_REG_EXTMEM_BASE — cache control
  uint32_t apbCtrl;
  uint32_t ledc;
  uint32_t usbSerialJtag;
  uint32_t saradc;

  // Where things sit in the interrupt matrix. A peripheral's slot is its map
  // register's offset divided by four, and the numbering is renumbered on
  // every chip — so these are facts about the part, not constants.
  uint32_t systimerIntSource;  // the first of the system timer's three
  uint32_t fromCpuIntSource;   // the first of the four "interrupt from CPU"
  // The pin-facing blocks' slots. A driver that allocates an interrupt writes
  // its line number into the slot at (source * 4) within the matrix block, so
  // these are what a peripheral model raises its condition on. Getting one
  // wrong is silent — the firmware's handler simply never runs — so each is
  // cross-checked against the map register the firmware actually wrote, and
  // the machine says so when they disagree.
  uint32_t gpioIntSource;
  uint32_t i2c0IntSource;
  uint32_t i2c1IntSource;  // 0 where the chip has one I2C
  uint32_t spi2IntSource;
  uint32_t spi3IntSource;  // 0 where the chip has two SPI masters
  // GDMA's per-channel receive and transmit slots, the first of each. SPI
  // transfers longer than the peripheral's 64-byte register file go through
  // DMA, and their completion is what the driver waits on.
  uint32_t dmaInIntSource;
  uint32_t dmaOutIntSource;
  uint32_t dmaChannels;
  uint32_t systemFromCpu0;     // SYSTEM_CPU_INTR_FROM_CPU_0_REG, within SYSTEM
  uint32_t matrixMapEnd;       // end of one core's map-register range
  // A multi-core part gives each CPU its own set of map registers in the same
  // block. Zero on a single-core part.
  uint32_t matrixCore1Offset;

  // The cache controller's state register, within the EXTMEM block, and the
  // value that means "idle". Firmware that reconfigures the cache — every S3
  // app does, at boot — polls this before touching anything, so a zero here
  // is not a missing detail, it is a hang. Zero offset means the chip has no
  // such register (or no firmware asks).
  uint32_t cacheStateReg;
  uint32_t cacheIdleValue;

  // The general-purpose DMA the SPI masters pull through, and the external RAM
  // window the MMU maps PSRAM-type pages into.
  uint32_t sdmmc;       // DR_REG_SDMMC_BASE, 0 where the part has no card host
  uint32_t sdmmcIntSource;
  uint32_t gdma;        // DR_REG_GDMA_BASE
  uint32_t psramCache;  // SOC_EXTRAM_DATA_LOW, 0 where the part has no PSRAM
  uint32_t psramSize;   // how much the board fits, 0 for a part with none

  // The analog-register master: the block the PLL, the regulators and the RF
  // front end are configured through. Its own status register reports when a
  // PLL calibration has finished, and clock setup waits for that — with
  // interrupts off, so a firmware that never sees it stops there and nothing
  // else in the machine gets a chance to explain why.
  uint32_t analogMaster;          // I2C_ANA_MST base, 0 if the chip has none
  uint32_t analogMasterStatus;    // I2C_MST_ANA_CONF0_REG, as an offset
  uint32_t analogMasterReadyBit;  // I2C_MST_BBPLL_CAL_DONE, 0 where absent

  uint32_t cpuHz;  // CPU clock the emulator charges instructions at
  uint32_t apbHz;
};

// Returns nullptr for a chip the emulator has no descriptor for; the caller
// reports that by name rather than running something it cannot model.
const SocDesc* socForChip(Chip chip);
const char* chipName(Chip chip);

inline bool inRange(const MemRange& range, uint32_t addr) {
  return range.size != 0 && addr >= range.base && addr - range.base < range.size;
}

}  // namespace freeink::sim::emu
