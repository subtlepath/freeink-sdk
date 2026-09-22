#pragma once

// FreeInk emulator — MD5.
//
// Needed in two places that must agree: the ROM's MD5 routines, which an app
// calls to verify its own partition table, and the synthesized partition table
// itself, which has to carry a checksum that verification accepts.

#include <cstddef>
#include <cstdint>

namespace freeink::sim::emu {

// One 64-byte block into the running state. Exposed because the ROM's API is
// incremental and keeps its context in guest memory between calls.
void md5Transform(uint32_t state[4], const uint32_t block[16]);

// The whole thing, for callers that have the data in hand.
void md5Digest(const uint8_t* data, size_t length, uint8_t out[16]);

}  // namespace freeink::sim::emu
