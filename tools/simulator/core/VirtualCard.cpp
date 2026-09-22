// FreeInk simulator — the SD card, as blocks.

#include "VirtualCard.h"

#include "Fat32Image.h"

#include <cstring>
#include <fstream>

namespace freeink::sim {

bool VirtualCard::mountDirectory(const std::string& directory, uint64_t capacity,
                                 std::string* error) {
  std::vector<uint8_t> image;
  if (!buildFat32Image(directory, capacity, &image, error)) return false;
  image_ = std::move(image);
  source_ = directory;
  writeBack_.clear();
  buildIdentity();
  return true;
}

bool VirtualCard::mountImage(const std::string& path, std::string* error) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) {
    if (error) *error = "cannot read " + path;
    return false;
  }
  const std::streamoff size = file.tellg();
  if (size <= 0 || size % kSectorSize != 0) {
    if (error) *error = path + " is not a whole number of 512-byte sectors";
    return false;
  }
  file.seekg(0);
  image_.resize(static_cast<size_t>(size));
  file.read(reinterpret_cast<char*>(image_.data()), size);
  source_ = path;
  writeBack_ = path;
  buildIdentity();
  return true;
}

void VirtualCard::unmount() {
  flush();
  image_.clear();
  source_.clear();
  writeBack_.clear();
}

void VirtualCard::buildIdentity() {
  // A plausible CID: manufacturer 0x03 (SanDisk), "FRINK" as the product name,
  // revision 1.0, a fixed serial so a run reproduces, and a manufacture date.
  std::memset(cid_, 0, sizeof(cid_));
  cid_[0] = 0x03;
  cid_[1] = 'F';  cid_[2] = 'I';
  std::memcpy(cid_ + 3, "FRINK", 5);
  cid_[8] = 0x10;
  cid_[9] = 0xF2; cid_[10] = 0xEE; cid_[11] = 0x14; cid_[12] = 0x01;
  cid_[13] = 0x01;
  cid_[14] = 0x5A;  // 2025, September
  cid_[15] = 0x01;  // the CRC slot; controllers here do not check it

  // CSD version 2 (SDHC/SDXC), which reports capacity as (C_SIZE + 1) * 512 KB.
  std::memset(csd_, 0, sizeof(csd_));
  csd_[0] = 0x40;                     // CSD_STRUCTURE = 1
  csd_[1] = 0x0E;                     // TAAC
  csd_[2] = 0x00;                     // NSAC
  csd_[3] = 0x32;                     // TRAN_SPEED: 25 MHz
  csd_[4] = 0x5B; csd_[5] = 0x59; csd_[6] = 0x00;  // CCC and READ_BL_LEN
  const uint32_t cSize = static_cast<uint32_t>(image_.size() / (512ULL * 1024)) - 1;
  csd_[7] = static_cast<uint8_t>((cSize >> 16) & 0x3F);
  csd_[8] = static_cast<uint8_t>((cSize >> 8) & 0xFF);
  csd_[9] = static_cast<uint8_t>(cSize & 0xFF);
  csd_[10] = 0x7F; csd_[11] = 0x80;
  csd_[12] = 0x0A; csd_[13] = 0x40; csd_[14] = 0x00;
  csd_[15] = 0x01;
}

uint32_t VirtualCard::ocr() const {
  // Powered up, high capacity, and happy with every voltage in the 2.7–3.6 V
  // window — which is what a modern card reports.
  return 0xC0FF8000u;
}

bool VirtualCard::readBlocks(uint32_t lba, uint32_t count, uint8_t* out) const {
  if (!out || count == 0) return false;
  const uint64_t offset = static_cast<uint64_t>(lba) * kSectorSize;
  const uint64_t length = static_cast<uint64_t>(count) * kSectorSize;
  if (offset + length > image_.size()) {
    std::memset(out, 0, static_cast<size_t>(length));
    return false;
  }
  std::memcpy(out, image_.data() + offset, static_cast<size_t>(length));
  return true;
}

bool VirtualCard::writeBlocks(uint32_t lba, uint32_t count, const uint8_t* in) {
  if (!in || count == 0) return false;
  const uint64_t offset = static_cast<uint64_t>(lba) * kSectorSize;
  const uint64_t length = static_cast<uint64_t>(count) * kSectorSize;
  if (offset + length > image_.size()) return false;
  std::memcpy(image_.data() + offset, in, static_cast<size_t>(length));
  return true;
}

void VirtualCard::flush() const {
  if (writeBack_.empty() || image_.empty()) return;
  std::ofstream file(writeBack_, std::ios::binary | std::ios::trunc);
  if (!file) return;
  file.write(reinterpret_cast<const char*>(image_.data()),
             static_cast<std::streamsize>(image_.size()));
}

}  // namespace freeink::sim
