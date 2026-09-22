// FreeInk emulator — chip descriptors.

#include "Soc.h"

namespace freeink::sim::emu {
namespace {

// ── ESP32-C3 ─────────────────────────────────────────────────────────────────
// Single RISC-V core, 400 KB of SRAM aliased into both buses, 8 KB of RTC fast
// memory, no PSRAM. This is the X3/X4 silicon.
constexpr SocDesc kEsp32C3 = {
    /*chip*/ Chip::Esp32C3,
    /*arch*/ Arch::RiscV,
    /*name*/ "esp32c3",
    /*cores*/ 1,
    /*sram*/
    {
        // 400 KB, visible at 0x3FC7C000 (data) and 0x4037C000 (instructions).
        {0x4037C000, 0x3FC7C000, 0x64000},
        {},
    },
    /*iromCache*/ {0x42000000, 0x00800000},
    /*dromCache*/ {0x3C000000, 0x00800000},
    /*romStack*/ {0x3FCDC710, 0x2000},  // top at SOC_ROM_STACK_START = 0x3FCDE710
    /*rtcFast*/ {0x50000000, 0x2000},
    /*rtcSlow*/ {},  // the C3's RTC fast memory is the only RTC RAM the CPU sees
    /*rom*/ {0x40000000, 0x00060000},
    /*romData*/ {0x3FF00000, 0x00020000},
    /*peripheral*/ {0x60000000, 0x00100000},
    /*mmu*/
    {
        /*tableBase*/ 0x600C5000,
        /*entryCount*/ 128,
        /*pageSize*/ 0x10000,
        /*valueMask*/ 0xFF,
        /*invalidBit*/ 1u << 8,
        /*typeBit*/ 0,
        /*linearMask*/ 0x7FFFFF,
        /*dbusBase*/ 0x3C000000,
        /*ibusBase*/ 0x42000000,
    },
    /*uart0*/ 0x60000000,
    /*uart1*/ 0x60010000,
    /*gpio*/ 0x60004000,
    /*ioMux*/ 0x60009000,
    /*spi0*/ 0x60003000,
    /*spi1*/ 0x60002000,
    /*spi2*/ 0x60024000,
    /*spi3*/ 0,
    /*i2c0*/ 0x60013000,
    /*i2c1*/ 0,
    /*rtcCntl*/ 0x60008000,
    /*efuse*/ 0x60008800,
    /*timerGroup0*/ 0x6001F000,
    /*timerGroup1*/ 0x60020000,
    /*systimer*/ 0x60023000,
    /*interrupt*/ 0x600C2000,
    /*system*/ 0x600C0000,
    /*extmem*/ 0x600C4000,
    /*apbCtrl*/ 0x60026000,
    /*ledc*/ 0x60019000,
    /*usbSerialJtag*/ 0x60043000,
    /*saradc*/ 0x60040000,
    /*systimerIntSource*/ 0x094 / 4,   // ETS_SYSTIMER_TARGET0_INTR_SOURCE = 37
    /*fromCpuIntSource*/ 0x0C8 / 4,    // ETS_FROM_CPU_INTR0_SOURCE = 50
    /*gpioIntSource*/ 16,              // ETS_GPIO_INTR_SOURCE
    /*i2c0IntSource*/ 29,              // ETS_I2C_EXT0_INTR_SOURCE
    /*i2c1IntSource*/ 0,               // the C3 has one I2C
    /*spi2IntSource*/ 19,              // ETS_SPI2_INTR_SOURCE
    /*spi3IntSource*/ 0,               // the C3 has one general-purpose SPI
    /*dmaInIntSource*/ 44,             // ETS_DMA_CH0_INTR_SOURCE — one slot per channel
    /*dmaOutIntSource*/ 44,            // the C3 shares one slot per channel for both directions
    /*dmaChannels*/ 3,
    /*systemFromCpu0*/ 0x28,
    /*matrixMapEnd*/ 0x100,
    /*matrixCore1Offset*/ 0,
    /*cacheStateReg*/ 0,
    /*cacheIdleValue*/ 0,
    /*sdmmc*/ 0,  // the C3 has no SDMMC host: its card is on SPI
    /*sdmmcIntSource*/ 0,
    /*gdma*/ 0x6003F000,
    /*psramCache*/ 0,  // the C3 has no external RAM interface
    /*psramSize*/ 0,
    /*analogMaster*/ 0x6000E000,
    /*analogMasterStatus*/ 0x40,
    // The C3's ANA_CONF0 has no calibration-done bit: its clock setup does
    // not wait on one.
    /*analogMasterReadyBit*/ 0,
    /*cpuHz*/ 160000000,
    /*apbHz*/ 80000000,
};

// ── ESP32-S3 ─────────────────────────────────────────────────────────────────
// Dual Xtensa LX7, 512 KB of SRAM in three blocks with different bus
// visibility, RTC fast memory at two addresses, and an MMU wide enough to map
// PSRAM as well as flash. This is the X4 Pro silicon.
constexpr SocDesc kEsp32S3 = {
    /*chip*/ Chip::Esp32S3,
    /*arch*/ Arch::Xtensa,
    /*name*/ "esp32s3",
    /*cores*/ 2,
    /*sram*/
    {
        {0x40370000, 0, 0x8000},               // SRAM0: instructions only
        {0x40378000, 0x3FC88000, 0x68000},     // SRAM1: both buses
        {0, 0x3FCF0000, 0x10000},              // SRAM2: data only
        {},
    },
    /*iromCache*/ {0x42000000, 0x02000000},
    /*dromCache*/ {0x3C000000, 0x02000000},
    /*romStack*/ {0x3FCE9710, 0x2000},  // top at SOC_ROM_STACK_START = 0x3FCEB710
    /*rtcFast*/ {0x600FE000, 0x2000},
    /*rtcSlow*/ {0x50000000, 0x2000},
    /*rom*/ {0x40000000, 0x00060000},
    /*romData*/ {0x3FF00000, 0x00020000},
    /*peripheral*/ {0x60000000, 0x00100000},
    /*mmu*/
    {
        /*tableBase*/ 0x600C5000,
        /*entryCount*/ 512,
        /*pageSize*/ 0x10000,
        /*valueMask*/ 0x3FFF,
        /*invalidBit*/ 1u << 14,
        /*typeBit*/ 1u << 15,
        /*linearMask*/ 0x1FFFFFF,
        /*dbusBase*/ 0x3C000000,
        /*ibusBase*/ 0x42000000,
    },
    /*uart0*/ 0x60000000,
    /*uart1*/ 0x60010000,
    /*gpio*/ 0x60004000,
    /*ioMux*/ 0x60009000,
    /*spi0*/ 0x60003000,
    /*spi1*/ 0x60002000,
    /*spi2*/ 0x60024000,
    /*spi3*/ 0x60025000,
    /*i2c0*/ 0x60013000,
    /*i2c1*/ 0x60027000,
    /*rtcCntl*/ 0x60008000,
    /*efuse*/ 0x60007000,
    /*timerGroup0*/ 0x6001F000,
    /*timerGroup1*/ 0x60020000,
    /*systimer*/ 0x60023000,
    /*interrupt*/ 0x600C2000,
    /*system*/ 0x600C0000,
    /*extmem*/ 0x600C4000,
    /*apbCtrl*/ 0x60026000,
    /*ledc*/ 0x60019000,
    /*usbSerialJtag*/ 0x60038000,
    /*saradc*/ 0x60040000,
    /*systimerIntSource*/ 0x0E4 / 4,   // ETS_SYSTIMER_TARGET0_EDGE_INTR_SOURCE = 57
    /*fromCpuIntSource*/ 0x13C / 4,    // ETS_FROM_CPU_INTR0_SOURCE = 79
    /*gpioIntSource*/ 16,              // ETS_GPIO_INTR_SOURCE
    /*i2c0IntSource*/ 40,              // ETS_I2C_EXT0_INTR_SOURCE
    /*i2c1IntSource*/ 41,              // ETS_I2C_EXT1_INTR_SOURCE
    /*spi2IntSource*/ 22,              // ETS_SPI2_INTR_SOURCE
    /*spi3IntSource*/ 23,              // ETS_SPI3_INTR_SOURCE
    /*dmaInIntSource*/ 66,             // ETS_DMA_IN_CH0_INTR_SOURCE
    /*dmaOutIntSource*/ 71,            // ETS_DMA_OUT_CH0_INTR_SOURCE
    /*dmaChannels*/ 5,
    /*systemFromCpu0*/ 0x30,
    /*matrixMapEnd*/ 0x190,
    // The APP CPU's map registers share the block, 0x800 further in.
    /*matrixCore1Offset*/ 0x800,
    // EXTMEM_CACHE_STATE_REG: both FSMs idle, its reset value.
    /*cacheStateReg*/ 0x130,
    /*cacheIdleValue*/ 0x001001,
    /*sdmmc*/ 0x60028000,
    /*sdmmcIntSource*/ 30,  // ETS_SDIO_HOST_INTR_SOURCE
    /*gdma*/ 0x6003F000,
    // SOC_EXTRAM_DATA_LOW. The X4 Pro fits 8 MB of octal PSRAM; the X4 Classic
    // fits none, and its firmware reports the chip missing and carries on.
    /*psramCache*/ 0x3C000000,
    /*psramSize*/ 8u * 1024 * 1024,
    /*analogMaster*/ 0x6000E000,
    /*analogMasterStatus*/ 0x40,
    /*analogMasterReadyBit*/ 1u << 24,  // I2C_MST_BBPLL_CAL_DONE
    /*cpuHz*/ 240000000,
    /*apbHz*/ 80000000,
};

}  // namespace

const SocDesc* socForChip(Chip chip) {
  switch (chip) {
    case Chip::Esp32C3: return &kEsp32C3;
    case Chip::Esp32S3: return &kEsp32S3;
    default: return nullptr;
  }
}

const char* chipName(Chip chip) {
  switch (chip) {
    case Chip::Esp32: return "esp32";
    case Chip::Esp32S2: return "esp32s2";
    case Chip::Esp32S3: return "esp32s3";
    case Chip::Esp32C2: return "esp32c2";
    case Chip::Esp32C3: return "esp32c3";
    case Chip::Esp32C6: return "esp32c6";
    case Chip::Esp32H2: return "esp32h2";
    case Chip::Esp32P4: return "esp32p4";
    default: return "unknown";
  }
}

}  // namespace freeink::sim::emu
