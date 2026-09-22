#pragma once

// FreeInk emulator — firmware images and the flash they live in.
//
// What the user hands us is normally an *app* image: the thing `idf.py build`
// produces and the thing you flash at 0x10000. It is not a whole flash. A
// device also has a bootloader at 0x0 and a partition table at 0x8000, and an
// IDF app reads both — `esp_ota_get_running_partition()`, the NVS partition
// lookup and every `esp_partition_find()` go through that table.
//
// So the loader builds a flash around the image: the app at its offset, a
// synthesized partition table, and a bootloader header that is well-formed but
// never executed (the emulator enters at the app's entry point, with the MMU
// already mapped the way the bootloader would have left it). A real dump —
// anything that already has a partition table at 0x8000 — is used verbatim
// instead, and a real `partitions.bin` can be supplied to override the
// synthesized one.

#include "Soc.h"

#include <cstdint>
#include <string>
#include <vector>

namespace freeink::sim::emu {

// One segment of an ESP32 image: a chunk of bytes and the address it loads at.
struct ImageSegment {
  uint32_t addr = 0;
  uint32_t fileOffset = 0;  // offset within the image, i.e. of its first byte
  uint32_t length = 0;
};

// esp_app_desc_t, the 256-byte block at the start of the first segment.
struct AppDescription {
  bool present = false;
  uint32_t secureVersion = 0;
  std::string version;
  std::string projectName;
  std::string time;
  std::string date;
  std::string idfVersion;
};

struct AppImage {
  Chip chip = Chip::Unknown;
  uint32_t entry = 0;
  uint8_t spiMode = 0;
  uint8_t spiSpeedCode = 0;
  uint8_t spiSizeCode = 0;
  uint32_t flashSizeBytes = 0;
  uint32_t minChipRev = 0;
  std::vector<ImageSegment> segments;
  AppDescription desc;
  uint32_t imageLength = 0;  // bytes the image occupies, checksum and hash included
  bool hashAppended = false;
};

// A partition as the table describes it.
struct Partition {
  uint8_t type = 0;
  uint8_t subtype = 0;
  uint32_t offset = 0;
  uint32_t size = 0;
  std::string label;
};

// The whole SPI flash, as the chip sees it.
class FlashImage {
 public:
  // Loads a firmware file. Accepts an app image (the usual case) or a full
  // flash dump. `error` explains a refusal in the terms the user gave it —
  // wrong magic, a chip the emulator has no model for, a truncated file.
  bool load(const std::string& path, std::string* error);
  // Replaces the synthesized table with a real partitions.bin.
  bool loadPartitionTable(const std::string& path, std::string* error);

  const AppImage& app() const { return app_; }
  const std::vector<Partition>& partitions() const { return partitions_; }
  // Where the app image was placed; also where the app's own partition starts.
  uint32_t appOffset() const { return appOffset_; }
  bool synthesizedTable() const { return synthesizedTable_; }
  const std::string& sourcePath() const { return sourcePath_; }

  uint32_t size() const { return static_cast<uint32_t>(data_.size()); }
  const uint8_t* bytes() const { return data_.data(); }
  uint8_t* bytes() { return data_.data(); }

  // Reads/writes as the SPI flash controller would: out-of-range reads return
  // 0xFF (an erased, absent chip) rather than faulting the emulator.
  void read(uint32_t offset, uint8_t* out, size_t len) const;
  void write(uint32_t offset, const uint8_t* in, size_t len);
  void erase(uint32_t offset, size_t len);

 private:
  void synthesizePartitionTable();
  void synthesizeBootloader();
  bool parsePartitionTable();

  std::vector<uint8_t> data_;
  AppImage app_;
  std::vector<Partition> partitions_;
  uint32_t appOffset_ = 0x10000;
  bool synthesizedTable_ = false;
  std::string sourcePath_;
};

// Parses an image header at `offset`. Exposed so `freeink-sim image` can
// report on a file without building a flash around it.
bool parseAppImage(const uint8_t* data, size_t size, uint32_t offset, AppImage* out, std::string* error);

// Flash size encoded in the image header's size/speed byte.
uint32_t flashSizeFromCode(uint8_t code);
const char* spiModeName(uint8_t mode);
const char* spiSpeedName(uint8_t code);
const char* partitionTypeName(uint8_t type, uint8_t subtype);

}  // namespace freeink::sim::emu
