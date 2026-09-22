#pragma once

// FreeInk simulator — the SD card, as blocks.
//
// A firmware *bundle* gets its files through the platform shim: it calls
// open() and the daemon hands it a host path inside the mounted directory. An
// emulated *image* cannot — it talks to a card controller, clocks out SD
// commands and expects 512-byte sectors back. So for an image the card has to
// be a card: a block device with a real FAT volume on it.
//
// That is what this builds. A host directory is turned into a FAT32 volume —
// MBR, boot sector, FAT chains, long file names and all — which the firmware's
// own FAT driver then mounts and reads. Nothing about the filesystem is
// shortcut, because the point is to exercise the firmware's storage path
// rather than to hand it a list of books.
//
// Writes land in the image and, for a card backed by a file, are flushed back
// to it. A card built from a directory is a snapshot: the firmware can write
// (and must be able to, or it cannot save a reading position) but the host
// directory is not modified, which keeps a test run from editing its fixtures.

#include <cstdint>
#include <string>
#include <vector>

namespace freeink::sim {

class VirtualCard {
 public:
  static constexpr uint32_t kSectorSize = 512;

  // Builds a FAT32 volume holding everything under `directory`. `capacity` is
  // rounded up to hold the content with room to write; zero picks a size from
  // the content.
  bool mountDirectory(const std::string& directory, uint64_t capacity, std::string* error);
  // Uses an existing raw card image. Writes are flushed back to it.
  bool mountImage(const std::string& path, std::string* error);
  void unmount();

  bool present() const { return !image_.empty(); }
  uint64_t capacityBytes() const { return image_.size(); }
  uint32_t sectorCount() const { return static_cast<uint32_t>(image_.size() / kSectorSize); }
  const std::string& source() const { return source_; }

  // Block access, as the controller models use it. Out-of-range reads return
  // zeroes and out-of-range writes are dropped, which is what a card does with
  // a block address past its end.
  bool readBlocks(uint32_t lba, uint32_t count, uint8_t* out) const;
  bool writeBlocks(uint32_t lba, uint32_t count, const uint8_t* in);
  void flush() const;

  // The card's identity registers, as the controllers report them.
  const uint8_t* cid() const { return cid_; }
  const uint8_t* csd() const { return csd_; }
  uint32_t ocr() const;

 private:
  void buildIdentity();

  std::vector<uint8_t> image_;
  std::string source_;
  std::string writeBack_;  // file to flush to, empty for a directory snapshot
  uint8_t cid_[16] = {};
  uint8_t csd_[16] = {};
};

}  // namespace freeink::sim
