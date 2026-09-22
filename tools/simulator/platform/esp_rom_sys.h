#pragma once

// FreeInk simulator — esp_rom_sys shim.
//
// esp_rom_printf is the SDK's ISR-safe / early-boot logger. It routes to the
// same daemon log ring as Serial so one `freeink-sim log` shows both.

#include <cstdarg>
#include <cstdio>
#include <freeink_sim_abi.h>

#ifdef __cplusplus
extern "C" {
#endif
int esp_rom_printf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
void esp_rom_delay_us(uint32_t us);
void esp_rom_install_channel_putc(int channel, void (*putc)(char c));
#ifdef __cplusplus
}
#endif
