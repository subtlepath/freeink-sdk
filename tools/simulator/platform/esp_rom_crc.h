#pragma once

// FreeInk simulator — esp_rom_crc shim (bitwise CRC32, matching the ROM's
// reflected polynomial so checksums computed here match the device).

#include <cstddef>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif
uint32_t esp_rom_crc32_le(uint32_t crc, const uint8_t* buf, uint32_t len);
#ifdef __cplusplus
}
#endif
