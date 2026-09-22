#pragma once

// FreeInk simulator — a FAT32 volume built from a host directory.
//
// Split out from VirtualCard because it is a self-contained piece of format
// knowledge: everything here is the on-disk layout Microsoft's specification
// describes, and nothing here knows what an SD card is.

#include <cstdint>
#include <string>
#include <vector>

namespace freeink::sim {

// Builds a partitioned card image: MBR at LBA 0, one FAT32 partition, with the
// tree under `directory` in it. `capacity` is the whole image in bytes and is
// adjusted up if the content will not fit. Returns false with `error` set.
bool buildFat32Image(const std::string& directory, uint64_t capacity, std::vector<uint8_t>* image,
                     std::string* error);

}  // namespace freeink::sim
