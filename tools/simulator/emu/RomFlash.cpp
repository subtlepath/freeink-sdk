// FreeInk emulator — ROM flash routines, and the ROM-owned data an app expects
// to find already set up.
//
// The legacy flash API is the one the ROM itself uses during boot and that
// IDF still reaches for in the bootloader-adjacent paths. Everything here acts
// on the FlashImage the machine was loaded with, so a firmware that writes its
// own OTA slot, erases a storage partition or reads its own image back sees a
// consistent flash — including NOR's "erase before you can set a bit" rule,
// which FlashImage enforces.

#include "Rom.h"

#include "FlashController.h"

#include <algorithm>
#include <cstring>
#include <vector>

namespace freeink::sim::emu {
namespace {

// esp_rom_spiflash_result_t
constexpr uint32_t kOk = 0;
constexpr uint32_t kError = 1;

// esp_rom_spiflash_chip_t, as IDF declares it.
struct GuestFlashChip {
  uint32_t deviceId;
  uint32_t chipSize;
  uint32_t blockSize;
  uint32_t sectorSize;
  uint32_t pageSize;
  uint32_t statusMask;
};

// esp_rom_spiflash_legacy_data_t
struct GuestLegacyData {
  GuestFlashChip chip;
  uint8_t dummyLenPlus[3];
  uint8_t sigMatrix;
};

}  // namespace

void Rom::initializeRomData() {
  // The ROM keeps its own variables in the top page of DRAM, which IDF's linker
  // scripts deliberately leave free. We put the structures in the same page,
  // just below the pointers that refer to them, so nothing the app links
  // against can collide with them.
  auto symbol = byName_.find("rom_spiflash_legacy_data");
  if (symbol == byName_.end()) return;
  const uint32_t pointerAddress = symbol->second;
  const uint32_t scratch = (pointerAddress & ~0xFFFu) + 0xE00;

  GuestLegacyData legacy{};
  legacy.chip.deviceId = 0x001640C8;  // a GigaDevice part, as most of these boards carry
  legacy.chip.chipSize = bus_.flash().size();
  legacy.chip.blockSize = 64 * 1024;
  legacy.chip.sectorSize = 4 * 1024;
  legacy.chip.pageSize = 256;
  legacy.chip.statusMask = 0xFFFF;
  legacy.sigMatrix = 0;
  writeBytes(scratch, &legacy, sizeof(legacy));
  bus_.write32(pointerAddress, scratch);

  // A MAC address for ets_efuse_get_mac(). Locally administered, derived from
  // nothing, and stable across runs so a firmware that keys anything off its
  // MAC behaves the same way every time.
  static const uint8_t kMac[6] = {0x7C, 0xDF, 0xA1, 0x00, 0x00, 0x01};
  writeBytes(scratch + 0x20, kMac, sizeof(kMac));
  romScratch_ = scratch;

  initializeRomLayout(scratch + 0x40);
  // Strings handed back by ROM routines live above the structures, still
  // inside the page the ROM owns and the heap is told to keep clear of.
  stringPool_ = scratch + 0x140;
}

void Rom::initializeRomLayout(uint32_t address) {
  // `ets_rom_layout_p` points at a table describing which parts of DRAM the
  // mask ROM owns: its stack, and the regions its Bluetooth, Wi-Fi, USB and
  // UART code keeps data in. IDF reads it during heap initialisation to decide
  // what the application may use, and dereferences it unconditionally — so on
  // a machine where the ROM never ran, it is a null pointer crash before the
  // first log line. The ROM fills this in during its own boot; here we do.
  //
  // Every region is given as a zero-length range at the base of the ROM's page.
  // That says "the ROM reserves nothing you could reclaim", which is the
  // conservative answer: the heap gets the memory it is entitled to and never
  // hands out memory the ROM might be using.
  auto symbol = byName_.find("ets_rom_layout_p");
  if (symbol == byName_.end()) return;

  if (soc_.romStack.size == 0) return;
  const uint32_t stackBottom = soc_.romStack.base;
  const uint32_t stackTop = soc_.romStack.base + soc_.romStack.size;

  // Wide enough to cover the longest of these structures across the chips the
  // emulator models; the tail is simply never read on the shorter ones.
  constexpr int kFields = 64;
  uint32_t layout[kFields];
  for (int i = 0; i < kFields; ++i) layout[i] = stackBottom;
  layout[0] = stackBottom;  // dram0_stack_shared_mem_start
  layout[1] = stackBottom;  // dram0_rtos_reserved_start
  layout[2] = stackBottom;  // stack_sentry
  layout[3] = stackTop;     // stack
  layout[4] = stackBottom;  // stack_sentry_app
  layout[5] = stackTop;     // stack_app
  writeBytes(address, layout, sizeof(layout));

  // The pointer itself lives in the ROM's data region, which is read-only on
  // the device and plain memory here, so it can be set up the way the ROM
  // would have left it.
  bus_.write32(symbol->second, address);
}

uint32_t Rom::internString(const std::string& text) {
  if (!stringPool_) return 0;
  auto existing = interned_.find(text);
  if (existing != interned_.end()) return existing->second;
  const uint32_t address = stringPool_;
  writeBytes(address, text.c_str(), text.size() + 1);
  stringPool_ += static_cast<uint32_t>((text.size() + 4) & ~3u);
  interned_.emplace(text, address);
  return address;
}

void Rom::registerFlashHandlers() {
  bind("esp_rom_spiflash_read", [](Rom& rom, Cpu& cpu) {
    const uint32_t source = cpu.arg(0);
    const uint32_t destination = cpu.arg(1);
    const int32_t length = static_cast<int32_t>(cpu.arg(2));
    if (length < 0) {
      cpu.setReturn(kError);
      return;
    }
    std::vector<uint8_t> data(static_cast<size_t>(length));
    rom.bus().flash().read(source, data.data(), data.size());
    rom.writeBytes(destination, data.data(), data.size());
    cpu.setReturn(kOk);
  });
  bind("esp_rom_spiflash_write", [](Rom& rom, Cpu& cpu) {
    const uint32_t destination = cpu.arg(0);
    const uint32_t source = cpu.arg(1);
    const int32_t length = static_cast<int32_t>(cpu.arg(2));
    if (length < 0) {
      cpu.setReturn(kError);
      return;
    }
    std::vector<uint8_t> data(static_cast<size_t>(length));
    rom.readBytes(source, data.data(), data.size());
    rom.bus().flash().write(destination, data.data(), data.size());
    cpu.setReturn(kOk);
  });
  bind("esp_rom_spiflash_erase_sector", [](Rom& rom, Cpu& cpu) {
    rom.bus().flash().erase(cpu.arg(0) * 4096, 4096);
    cpu.setReturn(kOk);
  });
  bind("esp_rom_spiflash_erase_block", [](Rom& rom, Cpu& cpu) {
    rom.bus().flash().erase(cpu.arg(0) * 65536, 65536);
    cpu.setReturn(kOk);
  });
  bind("esp_rom_spiflash_erase_area", [](Rom& rom, Cpu& cpu) {
    rom.bus().flash().erase(cpu.arg(0), cpu.arg(1));
    cpu.setReturn(kOk);
  });
  bind("esp_rom_spiflash_erase_chip", [](Rom& rom, Cpu& cpu) {
    rom.bus().flash().erase(0, rom.bus().flash().size());
    cpu.setReturn(kOk);
  });
  bind("esp_rom_spiflash_config_param", [](Rom& rom, Cpu& cpu) {
    // The caller is telling the ROM what chip it found. Record it so a later
    // read of g_rom_flashchip agrees with what the app believes.
    if (rom.romScratch()) {
      GuestFlashChip chip{};
      chip.deviceId = cpu.arg(0);
      chip.chipSize = cpu.arg(1);
      chip.blockSize = cpu.arg(2);
      chip.sectorSize = cpu.arg(3);
      chip.pageSize = cpu.arg(4);
      chip.statusMask = cpu.arg(5);
      rom.writeBytes(rom.romScratch(), &chip, sizeof(chip));
    }
    cpu.setReturn(kOk);
  });
  bind("esp_rom_spiflash_read_status", [](Rom& rom, Cpu& cpu) {
    if (cpu.arg(1)) rom.bus().write32(cpu.arg(1), 0);  // not busy, not write-protected
    cpu.setReturn(kOk);
  });
  bind("esp_rom_spiflash_read_statushigh", [](Rom& rom, Cpu& cpu) {
    if (cpu.arg(1)) rom.bus().write32(cpu.arg(1), 0);
    cpu.setReturn(kOk);
  });
  bind("esp_rom_spiflash_read_user_cmd", [](Rom& rom, Cpu& cpu) {
    if (cpu.arg(0)) rom.bus().write32(cpu.arg(0), 0);
    cpu.setReturn(kOk);
  });

  // Everything that configures the flash bus rather than moving data: there is
  // no real SPI timing to set up here, so they succeed.
  for (const char* name : {"esp_rom_spiflash_wait_idle", "esp_rom_spiflash_unlock",
                           "esp_rom_spiflash_clear_bp", "esp_rom_spiflash_write_enable",
                           "esp_rom_spiflash_write_disable", "esp_rom_spiflash_attach",
                           "esp_rom_spiflash_config_clk", "esp_rom_spiflash_config_readmode",
                           "esp_rom_spiflash_select_qio_pins", "esp_rom_spiflash_select_padsfunc",
                           "esp_rom_spiflash_set_drvs", "esp_rom_spiflash_fix_dummylen",
                           "esp_rom_spiflash_write_status", "esp_rom_spiflash_auto_wait_idle",
                           "spi_flash_attach", "spi_flash_boot_attach", "SPI_init", "SPILock",
                           "SPI_WakeUp", "SPI_write_enable", "spi_dummy_len_fix",
                           "esp_rom_spi_flash_auto_sus_res", "esp_rom_spi_flash_send_resume"}) {
    bind(name, [](Rom&, Cpu& cpu) { cpu.setReturn(kOk); });
  }
  bind("spi_flash_get_chip_size", [](Rom& rom, Cpu& cpu) { cpu.setReturn(rom.bus().flash().size()); });

  // ── Octal flash ────────────────────────────────────────────────────────────
  // The S3 can drive its flash and PSRAM on an 8-bit bus, and its ROM has a
  // second driver for it. The flash model here is a NOR array behind a
  // controller, with no bus width and no DTR timing of its own, so the pin,
  // mode and timing calls succeed and the data calls go to the same array the
  // quad path uses. What the width would really change — throughput — is not
  // modelled either way.
  for (const char* name : {"esp_rom_opiflash_pin_config", "esp_rom_opiflash_mode_reset",
                           "esp_rom_opiflash_soft_reset", "esp_rom_opiflash_wait_idle",
                           "esp_rom_opiflash_wren", "esp_rom_opiflash_cache_mode_config",
                           "esp_rom_opiflash_legacy_driver_init", "esp_rom_opiflash_exec_cmd",
                           "esp_rom_opiflash_exit_continuous_read_mode",
                           "esp_rom_spi_set_op_mode", "esp_rom_spi_set_dtr_swap_mode",
                           "esp_rom_spi_set_address_bit_len", "esp_rom_spi_cmd_config",
                           "esp_rom_spi_cmd_start", "esp_rom_opiflash_set_required_regs"}) {
    bind(name, [](Rom&, Cpu& cpu) { cpu.setReturn(kOk); });
  }
  bind("esp_rom_opiflash_rdsr", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  // The same JEDEC id the flash controller answers with, so a driver that
  // probes the chip through the ROM and one that probes it through the
  // controller find the same part.
  bind("esp_rom_opiflash_read_id", [](Rom&, Cpu& cpu) { cpu.setReturn(FlashControllerDevice::kDefaultJedecId); });
  bind("esp_rom_spi_flash_update_id", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("esp_rom_opiflash_read", [](Rom& rom, Cpu& cpu) {
    std::vector<uint8_t> data(cpu.arg(2));
    rom.bus().flash().read(cpu.arg(0), data.data(), data.size());
    rom.writeBytes(cpu.arg(1), data.data(), data.size());
    cpu.setReturn(kOk);
  });
  bind("esp_rom_opiflash_read_raw", [](Rom& rom, Cpu& cpu) {
    std::vector<uint8_t> data(cpu.arg(2));
    rom.bus().flash().read(cpu.arg(0), data.data(), data.size());
    rom.writeBytes(cpu.arg(1), data.data(), data.size());
    cpu.setReturn(kOk);
  });
  bind("esp_rom_opiflash_write", [](Rom& rom, Cpu& cpu) {
    std::vector<uint8_t> data(cpu.arg(2));
    rom.readBytes(cpu.arg(1), data.data(), data.size());
    rom.bus().flash().write(cpu.arg(0), data.data(), data.size());
    cpu.setReturn(kOk);
  });
  bind("esp_rom_opiflash_erase_sector", [](Rom& rom, Cpu& cpu) {
    rom.bus().flash().erase(cpu.arg(0) * 4096, 4096);
    cpu.setReturn(kOk);
  });
  bind("esp_rom_opiflash_erase_block_64k", [](Rom& rom, Cpu& cpu) {
    rom.bus().flash().erase(cpu.arg(0) * 65536, 65536);
    cpu.setReturn(kOk);
  });
  bind("esp_rom_opiflash_erase_area", [](Rom& rom, Cpu& cpu) {
    rom.bus().flash().erase(cpu.arg(0), cpu.arg(1));
    cpu.setReturn(kOk);
  });

  // ── eFuse ──────────────────────────────────────────────────────────────────
  // Zero here means "the factory default": flash on the default pins, no
  // secure boot, no flash encryption, UART download enabled.
  for (const char* name : {"ets_efuse_get_spiconfig", "esp_rom_efuse_get_flash_gpio_info",
                           "esp_rom_efuse_get_flash_wp_gpio", "ets_efuse_get_wp_pad",
                           "ets_efuse_secure_boot_enabled", "ets_efuse_cache_encryption_enabled",
                           "ets_efuse_download_modes_disabled", "ets_efuse_get_uart_print_control",
                           "ets_efuse_usb_print_is_disabled", "ets_efuse_jtag_disabled",
                           "ets_efuse_security_download_modes_enabled",
                           "ets_efuse_legacy_spi_boot_mode_disabled", "ets_efuse_force_send_resume",
                           "ets_efuse_secure_boot_aggressive_revoke_enabled",
                           "esp_rom_efuse_is_secure_boot_enabled", "ets_efuse_flash_opi_5pads_power_sel_vddspi",
                           "ets_efuse_usb_serial_jtag_print_is_disabled"}) {
    bind(name, [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  }
  bind("ets_efuse_get_flash_delay_us", [](Rom&, Cpu& cpu) { cpu.setReturn(2000); });
  bind("ets_efuse_read", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("ets_efuse_get_mac", [](Rom& rom, Cpu& cpu) { cpu.setReturn(rom.macAddress()); });
  bind("esp_rom_efuse_mac_address_crc8", [](Rom& rom, Cpu& cpu) {
    std::vector<uint8_t> data(cpu.arg(1));
    if (!data.empty()) rom.readBytes(cpu.arg(0), data.data(), data.size());
    cpu.setReturn(romCrc8Le(0, data.data(), data.size()));
  });
}

}  // namespace freeink::sim::emu
