// FreeInk emulator — firmware images and the flash they live in.

#include "Image.h"

#include "Md5.h"

#include <cstdio>
#include <cstring>
#include <fstream>

namespace freeink::sim::emu {
namespace {

constexpr uint8_t kImageMagic = 0xE9;
constexpr uint32_t kAppDescMagic = 0xABCD5432;
constexpr uint32_t kPartitionTableOffset = 0x8000;
constexpr uint16_t kPartitionMagic = 0x50AA;
constexpr uint16_t kPartitionMd5Magic = 0xEBEB;
constexpr size_t kPartitionEntrySize = 32;
constexpr size_t kHeaderSize = 24;
constexpr size_t kAppDescSize = 256;

uint16_t read16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
uint32_t read32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
void write16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}
void write32(uint8_t* p, uint32_t v) {
  for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
}

std::string fixedString(const uint8_t* p, size_t max) {
  size_t len = 0;
  while (len < max && p[len] != 0) ++len;
  return std::string(reinterpret_cast<const char*>(p), len);
}

uint32_t roundUp(uint32_t value, uint32_t to) { return (value + to - 1) / to * to; }

}  // namespace

uint32_t flashSizeFromCode(uint8_t code) {
  // esp_image_header_t.spi_size: 1, 2, 4, 8, 16, 32, 64, 128 MB.
  if (code > 7) return 4u * 1024 * 1024;
  return (1u << code) * 1024 * 1024;
}

const char* spiModeName(uint8_t mode) {
  switch (mode) {
    case 0: return "qio";
    case 1: return "qout";
    case 2: return "dio";
    case 3: return "dout";
    case 4: return "fast_read";
    case 5: return "slow_read";
    default: return "unknown";
  }
}

const char* spiSpeedName(uint8_t code) {
  switch (code) {
    case 0x0: return "40MHz";
    case 0x1: return "26MHz";
    case 0x2: return "20MHz";
    case 0xf: return "80MHz";
    default: return "unknown";
  }
}

const char* partitionTypeName(uint8_t type, uint8_t subtype) {
  if (type == 0) {
    switch (subtype) {
      case 0x00: return "app/factory";
      case 0x10: return "app/ota_0";
      case 0x11: return "app/ota_1";
      case 0x20: return "app/test";
      default: return "app";
    }
  }
  if (type == 1) {
    switch (subtype) {
      case 0x00: return "data/ota";
      case 0x01: return "data/phy";
      case 0x02: return "data/nvs";
      case 0x04: return "data/nvs_keys";
      case 0x06: return "data/efuse";
      case 0x80: return "data/esphttpd";
      case 0x81: return "data/fat";
      case 0x82: return "data/spiffs";
      case 0x83: return "data/littlefs";
      default: return "data";
    }
  }
  return "custom";
}

bool parseAppImage(const uint8_t* data, size_t size, uint32_t offset, AppImage* out, std::string* error) {
  auto fail = [&](const std::string& message) {
    if (error) *error = message;
    return false;
  };
  if (offset + kHeaderSize > size) return fail("file is too small to hold an image header");
  const uint8_t* header = data + offset;
  if (header[0] != kImageMagic) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "not an ESP32 image: magic is 0x%02X at 0x%X, expected 0xE9", header[0], offset);
    return fail(buf);
  }

  AppImage image;
  image.spiMode = header[2];
  image.spiSpeedCode = header[3] & 0x0F;
  image.spiSizeCode = header[3] >> 4;
  image.flashSizeBytes = flashSizeFromCode(image.spiSizeCode);
  image.entry = read32(header + 4);
  image.chip = static_cast<Chip>(read16(header + 12));
  image.minChipRev = read16(header + 15);
  image.hashAppended = header[23] == 1;

  const int segmentCount = header[1];
  if (segmentCount <= 0 || segmentCount > 16) {
    return fail("image header claims " + std::to_string(segmentCount) + " segments, which is not a real image");
  }

  size_t cursor = offset + kHeaderSize;
  for (int i = 0; i < segmentCount; ++i) {
    if (cursor + 8 > size) return fail("image is truncated in the segment table");
    ImageSegment segment;
    segment.addr = read32(data + cursor);
    segment.length = read32(data + cursor + 4);
    segment.fileOffset = static_cast<uint32_t>(cursor + 8 - offset);
    cursor += 8;
    if (cursor + segment.length > size) {
      char buf[160];
      std::snprintf(buf, sizeof(buf), "image is truncated: segment %d at 0x%08X wants %u bytes, %zu remain", i,
                    segment.addr, segment.length, size - cursor);
      return fail(buf);
    }
    cursor += segment.length;
    image.segments.push_back(segment);
  }

  // One checksum byte after padding to a 16-byte boundary, then the optional
  // SHA-256 of everything before it.
  cursor = (cursor + 16) & ~static_cast<size_t>(15);
  if (image.hashAppended) cursor += 32;
  image.imageLength = static_cast<uint32_t>(cursor - offset);

  // The app descriptor sits at the start of the first segment, which is why a
  // bootloader (no descriptor) and an app (descriptor) can be told apart.
  if (!image.segments.empty()) {
    const uint32_t descOffset = offset + image.segments[0].fileOffset;
    if (descOffset + kAppDescSize <= size && read32(data + descOffset) == kAppDescMagic) {
      const uint8_t* desc = data + descOffset;
      image.desc.present = true;
      image.desc.secureVersion = read32(desc + 4);
      image.desc.version = fixedString(desc + 16, 32);
      image.desc.projectName = fixedString(desc + 48, 32);
      image.desc.time = fixedString(desc + 80, 16);
      image.desc.date = fixedString(desc + 96, 16);
      image.desc.idfVersion = fixedString(desc + 112, 32);
    }
  }

  *out = image;
  return true;
}

bool FlashImage::load(const std::string& path, std::string* error) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    if (error) *error = "cannot open " + path;
    return false;
  }
  std::vector<uint8_t> raw((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  if (raw.size() < kHeaderSize) {
    if (error) *error = path + " is too small to be a firmware image";
    return false;
  }
  sourcePath_ = path;

  // A full flash dump already has a partition table where the ROM looks for
  // one. Anything else is an app image, and we build a flash around it.
  const bool isDump = raw.size() > kPartitionTableOffset + kPartitionEntrySize &&
                      read16(raw.data() + kPartitionTableOffset) == kPartitionMagic;

  if (isDump) {
    data_ = std::move(raw);
    if (!parsePartitionTable()) {
      if (error) *error = path + ": the partition table at 0x8000 is malformed";
      return false;
    }
    // The running app is the first app partition holding a valid image.
    bool found = false;
    for (const Partition& partition : partitions_) {
      if (partition.type != 0) continue;
      AppImage candidate;
      if (parseAppImage(data_.data(), data_.size(), partition.offset, &candidate, nullptr)) {
        app_ = candidate;
        appOffset_ = partition.offset;
        found = true;
        break;
      }
    }
    if (!found) {
      if (error) *error = path + ": no app partition in the table holds a loadable image";
      return false;
    }
    return true;
  }

  AppImage image;
  if (!parseAppImage(raw.data(), raw.size(), 0, &image, error)) return false;
  if (!image.desc.present) {
    // A bootloader image would land here. Running one would mean emulating the
    // ROM's own loader, which the emulator deliberately does not do.
    if (error) {
      *error = path +
               " has no application descriptor — this looks like a bootloader or a raw segment "
               "dump, not an app image. Pass the app .bin (the one flashed at 0x10000).";
    }
    return false;
  }

  app_ = image;
  appOffset_ = 0x10000;

  uint32_t flashBytes = image.flashSizeBytes;
  if (flashBytes < appOffset_ + image.imageLength) {
    flashBytes = roundUp(appOffset_ + image.imageLength, 1024 * 1024);
  }
  data_.assign(flashBytes, 0xFF);
  std::memcpy(data_.data() + appOffset_, raw.data(), raw.size());

  synthesizeBootloader();
  synthesizePartitionTable();
  return true;
}

void FlashImage::synthesizeBootloader() {
  // Never executed — the emulator enters at the app's entry point with the MMU
  // already mapped. It exists so that code reading the bootloader header (the
  // ROM's own checks, `esp_ota` sanity reads) sees a well-formed one rather
  // than erased flash.
  if (data_.size() < 0x1000 + kHeaderSize) return;
  uint8_t* header = data_.data() + 0x1000;
  std::memset(header, 0, kHeaderSize);
  header[0] = kImageMagic;
  header[1] = 0;  // no segments: there is nothing here to load
  header[2] = app_.spiMode;
  header[3] = static_cast<uint8_t>((app_.spiSizeCode << 4) | app_.spiSpeedCode);
  write32(header + 4, 0);
  write16(header + 12, static_cast<uint16_t>(app_.chip));
}

void FlashImage::synthesizePartitionTable() {
  // An app .bin does not carry the table it was built against, so this is a
  // guess — a deliberately conventional one, matching IDF's single-factory
  // layout with the app partition grown to fit the image. `--partitions`
  // replaces it with the real thing when exactness matters (an app that reads
  // its own storage partition, say).
  partitions_.clear();
  synthesizedTable_ = true;

  const uint32_t appSize = roundUp(app_.imageLength, 0x10000);
  const uint32_t flashSize = static_cast<uint32_t>(data_.size());

  partitions_.push_back({1, 0x02, 0x9000, 0x5000, "nvs"});
  partitions_.push_back({1, 0x01, 0xE000, 0x1000, "phy_init"});
  partitions_.push_back({0, 0x00, appOffset_, appSize, "factory"});
  const uint32_t storageStart = appOffset_ + appSize;
  if (flashSize > storageStart + 0x10000) {
    partitions_.push_back({1, 0x82, storageStart, flashSize - storageStart, "storage"});
  }

  if (data_.size() < kPartitionTableOffset + 0x1000) return;
  uint8_t* table = data_.data() + kPartitionTableOffset;
  std::memset(table, 0xFF, 0x1000);
  size_t cursor = 0;
  for (const Partition& partition : partitions_) {
    uint8_t* entry = table + cursor;
    std::memset(entry, 0, kPartitionEntrySize);
    write16(entry, kPartitionMagic);
    entry[2] = partition.type;
    entry[3] = partition.subtype;
    write32(entry + 4, partition.offset);
    write32(entry + 8, partition.size);
    std::memcpy(entry + 12, partition.label.c_str(), std::min<size_t>(partition.label.size(), 16));
    cursor += kPartitionEntrySize;
  }
  // The MD5 entry. IDF treats a table without one as malformed and refuses to
  // load it, so a synthesized table has to carry a real checksum over the
  // entries above it.
  uint8_t* md5Entry = table + cursor;
  std::memset(md5Entry, 0xFF, kPartitionEntrySize);
  write16(md5Entry, kPartitionMd5Magic);
  md5Digest(table, cursor, md5Entry + 16);
}

bool FlashImage::parsePartitionTable() {
  partitions_.clear();
  synthesizedTable_ = false;
  if (data_.size() < kPartitionTableOffset + kPartitionEntrySize) return false;
  for (size_t cursor = kPartitionTableOffset; cursor + kPartitionEntrySize <= data_.size() &&
                                              cursor < kPartitionTableOffset + 0x1000;
       cursor += kPartitionEntrySize) {
    const uint8_t* entry = data_.data() + cursor;
    const uint16_t magic = read16(entry);
    if (magic == kPartitionMd5Magic) continue;
    if (magic != kPartitionMagic) break;
    Partition partition;
    partition.type = entry[2];
    partition.subtype = entry[3];
    partition.offset = read32(entry + 4);
    partition.size = read32(entry + 8);
    partition.label = fixedString(entry + 12, 16);
    partitions_.push_back(partition);
  }
  return !partitions_.empty();
}

bool FlashImage::loadPartitionTable(const std::string& path, std::string* error) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    if (error) *error = "cannot open partition table " + path;
    return false;
  }
  std::vector<uint8_t> raw((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  if (raw.size() < kPartitionEntrySize || read16(raw.data()) != kPartitionMagic) {
    if (error) *error = path + " is not a partition table (no 0x50AA entry at its start)";
    return false;
  }
  if (data_.size() < kPartitionTableOffset + raw.size()) {
    if (error) *error = "the flash image is too small to hold this partition table";
    return false;
  }
  std::memset(data_.data() + kPartitionTableOffset, 0xFF, 0x1000);
  std::memcpy(data_.data() + kPartitionTableOffset, raw.data(), std::min<size_t>(raw.size(), 0x1000));
  if (!parsePartitionTable()) {
    if (error) *error = path + ": no usable entries";
    return false;
  }
  return true;
}

void FlashImage::read(uint32_t offset, uint8_t* out, size_t len) const {
  for (size_t i = 0; i < len; ++i) {
    const size_t address = offset + i;
    out[i] = address < data_.size() ? data_[address] : 0xFF;
  }
}

void FlashImage::write(uint32_t offset, const uint8_t* in, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    const size_t address = offset + i;
    if (address >= data_.size()) return;
    // NOR flash clears bits on write; it cannot set them without an erase.
    // Modelling that is what makes a firmware that forgets to erase behave
    // here the way it does on the device.
    data_[address] &= in[i];
  }
}

void FlashImage::erase(uint32_t offset, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    const size_t address = offset + i;
    if (address < data_.size()) data_[address] = 0xFF;
  }
}

}  // namespace freeink::sim::emu
