#pragma once

// FreeInk simulator — esp_cpu shim. The cycle counter is synthesized from the
// simulated clock at a nominal 160 MHz so timing code sees a monotonic count.

#include <cstdint>
#include <freeink_sim_abi.h>

inline uint32_t esp_cpu_get_cycle_count(void) { return static_cast<uint32_t>(fsim_micros() * 160ULL); }
inline void esp_cpu_set_cycle_count(uint32_t) {}
