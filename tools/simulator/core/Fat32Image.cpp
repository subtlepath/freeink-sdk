// FreeInk simulator — a FAT32 volume built from a host directory.
//
// The layout is the ordinary one an SD card ships with, because that is what
// the firmware's driver expects to find: an MBR at LBA 0 with a single
// partition starting at LBA 2048, and a FAT32 volume in it with two FATs, a
// root directory that is an ordinary cluster chain, and long file names.
//
// Files are laid out contiguously. That is not what a real card looks like
// after a year of use, and it means this does not exercise fragmented-chain
// handling — but every other part of the driver's path is real: it reads the
// MBR, parses the BPB, walks FAT chains, decodes LFN entries and checksums
// them, and reads file data through its own cache.

#include "Fat32Image.h"

#include <algorithm>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <sys/stat.h>

namespace freeink::sim {
namespace {

constexpr uint32_t kSectorSize = 512;
// One sector per cluster. FAT32 needs at least 65525 clusters to be FAT32 at
// all, so the cluster size sets the smallest volume that can exist: at 4 KB
// that floor is 280 MB, which is a lot of host memory to hold for a card with
// three files on it. At 512 bytes the floor is 35 MB, and a card this size is
// not being used for throughput.
constexpr uint32_t kSectorsPerCluster = 1;
constexpr uint32_t kClusterSize = kSectorSize * kSectorsPerCluster;
constexpr uint32_t kReservedSectors = 32;
constexpr uint32_t kPartitionStart = 2048;
constexpr uint32_t kFatCount = 2;
constexpr uint32_t kDirEntrySize = 32;
constexpr uint32_t kEntriesPerCluster = kClusterSize / kDirEntrySize;

struct Node {
  std::string name;      // as it appears on the host
  std::string hostPath;
  bool directory = false;
  uint64_t size = 0;
  std::vector<Node> children;

  // Filled in during layout.
  uint32_t firstCluster = 0;
  uint32_t clusters = 0;
};

void put16(uint8_t* at, uint16_t value) {
  at[0] = static_cast<uint8_t>(value);
  at[1] = static_cast<uint8_t>(value >> 8);
}

void put32(uint8_t* at, uint32_t value) {
  at[0] = static_cast<uint8_t>(value);
  at[1] = static_cast<uint8_t>(value >> 8);
  at[2] = static_cast<uint8_t>(value >> 16);
  at[3] = static_cast<uint8_t>(value >> 24);
}

bool scan(const std::string& path, Node* node) {
  DIR* dir = ::opendir(path.c_str());
  if (!dir) return false;
  std::vector<Node> entries;
  while (dirent* entry = ::readdir(dir)) {
    const std::string name = entry->d_name;
    if (name == "." || name == "..") continue;
    // A host directory is a person's directory: dotfiles in it are the host's
    // business, not the device's.
    if (!name.empty() && name[0] == '.') continue;
    const std::string child = path + "/" + name;
    struct stat info {};
    if (::stat(child.c_str(), &info) != 0) continue;

    Node made;
    made.name = name;
    made.hostPath = child;
    made.directory = S_ISDIR(info.st_mode);
    if (made.directory) {
      scan(child, &made);
    } else if (S_ISREG(info.st_mode)) {
      made.size = static_cast<uint64_t>(info.st_size);
    } else {
      continue;
    }
    entries.push_back(std::move(made));
  }
  ::closedir(dir);
  std::sort(entries.begin(), entries.end(),
            [](const Node& a, const Node& b) { return a.name < b.name; });
  node->children = std::move(entries);
  return true;
}

// ── Names ────────────────────────────────────────────────────────────────────

bool fitsShortName(const std::string& name, char out[11]) {
  // The 8.3 form is only usable when the name already is one: upper case, no
  // spaces, no second dot, and short enough. Anything else gets a generated
  // alias and a long-name entry.
  std::memset(out, ' ', 11);
  const size_t dot = name.rfind('.');
  const std::string stem = dot == std::string::npos ? name : name.substr(0, dot);
  const std::string extension = dot == std::string::npos ? "" : name.substr(dot + 1);
  if (stem.empty() || stem.size() > 8 || extension.size() > 3) return false;
  if (stem.find('.') != std::string::npos) return false;
  auto acceptable = [](char c) {
    return (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || std::strchr("$%'-_@~`!(){}^#&", c);
  };
  for (char c : stem) {
    if (!acceptable(c)) return false;
  }
  for (char c : extension) {
    if (!acceptable(c)) return false;
  }
  std::memcpy(out, stem.data(), stem.size());
  std::memcpy(out + 8, extension.data(), extension.size());
  return true;
}

void makeShortAlias(const std::string& name, uint32_t ordinal, char out[11]) {
  std::memset(out, ' ', 11);
  const size_t dot = name.rfind('.');
  const std::string stem = dot == std::string::npos ? name : name.substr(0, dot);
  const std::string extension = dot == std::string::npos ? "" : name.substr(dot + 1);

  std::string base;
  for (char c : stem) {
    if (base.size() >= 8) break;
    const unsigned char u = static_cast<unsigned char>(c);
    if (u >= 'a' && u <= 'z') base += static_cast<char>(u - 32);
    else if ((u >= 'A' && u <= 'Z') || (u >= '0' && u <= '9')) base += static_cast<char>(u);
    else base += '_';
  }
  if (base.empty()) base = "FILE";

  const std::string tail = "~" + std::to_string(ordinal);
  if (base.size() + tail.size() > 8) base.resize(8 - tail.size());
  base += tail;
  std::memcpy(out, base.data(), base.size());

  for (size_t index = 0; index < extension.size() && index < 3; ++index) {
    const unsigned char u = static_cast<unsigned char>(extension[index]);
    out[8 + index] = (u >= 'a' && u <= 'z') ? static_cast<char>(u - 32) : static_cast<char>(u);
  }
}

uint8_t shortNameChecksum(const char name[11]) {
  uint8_t sum = 0;
  for (int index = 0; index < 11; ++index) {
    sum = static_cast<uint8_t>(((sum & 1) ? 0x80 : 0) + (sum >> 1) + static_cast<uint8_t>(name[index]));
  }
  return sum;
}

// UTF-8 to UTF-16, well enough for the basic multilingual plane. A name the
// host holds as UTF-8 has to reach the device as UTF-16 or its own driver will
// show mojibake for every non-ASCII book title.
std::vector<uint16_t> toUtf16(const std::string& text) {
  std::vector<uint16_t> out;
  for (size_t index = 0; index < text.size();) {
    const unsigned char lead = static_cast<unsigned char>(text[index]);
    uint32_t code = lead;
    size_t length = 1;
    if (lead >= 0xF0) { code = lead & 0x07; length = 4; }
    else if (lead >= 0xE0) { code = lead & 0x0F; length = 3; }
    else if (lead >= 0xC0) { code = lead & 0x1F; length = 2; }
    if (index + length > text.size()) length = 1;
    for (size_t extra = 1; extra < length; ++extra) {
      code = (code << 6) | (static_cast<unsigned char>(text[index + extra]) & 0x3F);
    }
    index += length;
    if (code >= 0x10000) {
      code -= 0x10000;
      out.push_back(static_cast<uint16_t>(0xD800 + (code >> 10)));
      out.push_back(static_cast<uint16_t>(0xDC00 + (code & 0x3FF)));
    } else {
      out.push_back(static_cast<uint16_t>(code));
    }
  }
  return out;
}

// ── Layout ───────────────────────────────────────────────────────────────────

uint32_t clustersFor(uint64_t bytes) {
  return static_cast<uint32_t>((bytes + kClusterSize - 1) / kClusterSize);
}

// How many directory entries a node needs: one short entry plus one long entry
// per 13 characters of its name.
uint32_t entriesFor(const Node& node) {
  char unused[11];
  if (fitsShortName(node.name, unused)) return 1;
  const size_t characters = toUtf16(node.name).size();
  return 1 + static_cast<uint32_t>((characters + 12) / 13);
}

uint32_t directoryEntries(const Node& node, bool isRoot) {
  uint32_t entries = isRoot ? 0 : 2;  // "." and ".."
  for (const Node& child : node.children) entries += entriesFor(child);
  return entries;
}

// Walks the tree assigning cluster ranges, depth first, and returns the total.
uint32_t assignClusters(Node* node, bool isRoot, uint32_t* next) {
  node->firstCluster = *next;
  const uint32_t entries = directoryEntries(*node, isRoot);
  node->clusters = std::max<uint32_t>(1, (entries + kEntriesPerCluster - 1) / kEntriesPerCluster);
  *next += node->clusters;

  for (Node& child : node->children) {
    if (child.directory) {
      assignClusters(&child, false, next);
    } else {
      child.firstCluster = child.size ? *next : 0;
      child.clusters = clustersFor(child.size);
      *next += child.clusters;
    }
  }
  return *next;
}

class Writer {
 public:
  Writer(std::vector<uint8_t>* image, uint32_t dataStart, uint32_t fatStart, uint32_t fatSectors)
      : image_(*image), dataStart_(dataStart), fatStart_(fatStart), fatSectors_(fatSectors) {}

  uint8_t* clusterAt(uint32_t cluster) {
    const uint64_t offset =
        static_cast<uint64_t>(dataStart_ + (cluster - 2) * kSectorsPerCluster) * kSectorSize;
    return offset + kClusterSize <= image_.size() ? image_.data() + offset : nullptr;
  }

  void setFat(uint32_t cluster, uint32_t value) {
    for (uint32_t copy = 0; copy < kFatCount; ++copy) {
      const uint64_t offset = static_cast<uint64_t>(fatStart_ + copy * fatSectors_) * kSectorSize +
                              static_cast<uint64_t>(cluster) * 4;
      if (offset + 4 > image_.size()) continue;
      put32(image_.data() + offset, value);
    }
  }

  // Links a run of clusters into a chain ending in the end-of-chain marker.
  void chain(uint32_t first, uint32_t count) {
    for (uint32_t index = 0; index + 1 < count; ++index) setFat(first + index, first + index + 1);
    if (count) setFat(first + count - 1, 0x0FFFFFFF);
  }

 private:
  std::vector<uint8_t>& image_;
  uint32_t dataStart_;
  uint32_t fatStart_;
  uint32_t fatSectors_;
};

void writeShortEntry(uint8_t* at, const char name[11], uint8_t attributes, uint32_t cluster,
                     uint32_t size) {
  std::memset(at, 0, kDirEntrySize);
  std::memcpy(at, name, 11);
  at[11] = attributes;
  // A fixed timestamp: 2026-01-01 00:00:00. A wall-clock one would make every
  // generated image differ from the last, and a card whose contents change
  // between runs is not a fixture.
  put16(at + 14, (0 << 11) | (0 << 5) | 0);
  put16(at + 16, ((2026 - 1980) << 9) | (1 << 5) | 1);
  put16(at + 18, ((2026 - 1980) << 9) | (1 << 5) | 1);
  put16(at + 20, static_cast<uint16_t>(cluster >> 16));
  put16(at + 22, (0 << 11) | (0 << 5) | 0);
  put16(at + 24, ((2026 - 1980) << 9) | (1 << 5) | 1);
  put16(at + 26, static_cast<uint16_t>(cluster & 0xFFFF));
  put32(at + 28, size);
}

void writeLongEntries(uint8_t* at, const std::vector<uint16_t>& name, uint8_t checksum,
                      uint32_t count) {
  // Long-name entries are stored in reverse: the last fragment comes first on
  // disk, and the one holding the *start* of the name sits immediately before
  // the short entry.
  for (uint32_t index = 0; index < count; ++index) {
    uint8_t* entry = at + index * kDirEntrySize;
    const uint32_t sequence = count - index;  // 1-based, counting down the disk
    std::memset(entry, 0, kDirEntrySize);
    entry[0] = static_cast<uint8_t>(sequence | (index == 0 ? 0x40 : 0));
    entry[11] = 0x0F;  // the long-name attribute set
    entry[12] = 0;
    entry[13] = checksum;
    put16(entry + 26, 0);

    static const int kSlots[13] = {1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30};
    for (int slot = 0; slot < 13; ++slot) {
      const size_t character = (sequence - 1) * 13 + static_cast<size_t>(slot);
      uint16_t value = 0xFFFF;
      if (character < name.size()) value = name[character];
      else if (character == name.size()) value = 0;
      put16(entry + kSlots[slot], value);
    }
  }
}

bool emit(Node* node, bool isRoot, Writer& writer, std::vector<uint8_t>& image, uint32_t dataStart,
          uint32_t parentCluster, std::string* error) {
  writer.chain(node->firstCluster, node->clusters);
  uint8_t* cursor = writer.clusterAt(node->firstCluster);
  if (!cursor) {
    if (error) *error = "the card image is too small for its directory tree";
    return false;
  }
  uint8_t* end = cursor + static_cast<size_t>(node->clusters) * kClusterSize;
  std::memset(cursor, 0, static_cast<size_t>(node->clusters) * kClusterSize);

  if (!isRoot) {
    char dot[11];
    std::memset(dot, ' ', 11);
    dot[0] = '.';
    writeShortEntry(cursor, dot, 0x10, node->firstCluster, 0);
    cursor += kDirEntrySize;
    dot[1] = '.';
    writeShortEntry(cursor, dot, 0x10, parentCluster, 0);
    cursor += kDirEntrySize;
  }

  uint32_t ordinal = 1;
  for (Node& child : node->children) {
    char shortName[11];
    const bool plain = fitsShortName(child.name, shortName);
    if (!plain) makeShortAlias(child.name, ordinal++, shortName);

    if (!plain) {
      const std::vector<uint16_t> utf16 = toUtf16(child.name);
      const uint32_t fragments = static_cast<uint32_t>((utf16.size() + 12) / 13);
      if (cursor + (fragments + 1) * kDirEntrySize > end) {
        if (error) *error = "a directory outgrew the clusters reserved for it";
        return false;
      }
      writeLongEntries(cursor, utf16, shortNameChecksum(shortName), fragments);
      cursor += fragments * kDirEntrySize;
    }
    if (cursor + kDirEntrySize > end) {
      if (error) *error = "a directory outgrew the clusters reserved for it";
      return false;
    }
    writeShortEntry(cursor, shortName, child.directory ? 0x10 : 0x20, child.firstCluster,
                    static_cast<uint32_t>(child.size));
    cursor += kDirEntrySize;
  }

  for (Node& child : node->children) {
    if (child.directory) {
      if (!emit(&child, false, writer, image, dataStart, node->firstCluster, error)) return false;
      continue;
    }
    if (!child.size) continue;
    writer.chain(child.firstCluster, child.clusters);
    uint8_t* at = writer.clusterAt(child.firstCluster);
    if (!at || at + child.size > image.data() + image.size()) {
      if (error) *error = "the card image is too small for " + child.name;
      return false;
    }
    std::ifstream file(child.hostPath, std::ios::binary);
    if (!file) {
      if (error) *error = "could not read " + child.hostPath;
      return false;
    }
    file.read(reinterpret_cast<char*>(at), static_cast<std::streamsize>(child.size));
  }
  return true;
}

}  // namespace

bool buildFat32Image(const std::string& directory, uint64_t capacity, std::vector<uint8_t>* image,
                     std::string* error) {
  Node root;
  root.directory = true;
  root.hostPath = directory;
  if (!scan(directory, &root)) {
    if (error) *error = "cannot read " + directory;
    return false;
  }

  uint32_t next = 2;
  const uint32_t contentClusters = assignClusters(&root, true, &next) - 2;

  // Room for the content, plus somewhere for the firmware to write: an
  // e-reader that cannot save a reading position is not one you can interact
  // with. A quarter of the volume free, and never less than 4 MB.
  const uint64_t contentBytes = static_cast<uint64_t>(contentClusters) * kClusterSize;
  uint64_t wanted = std::max<uint64_t>(capacity, contentBytes + contentBytes / 3 + 4u * 1024 * 1024);
  wanted = std::max<uint64_t>(wanted, 16u * 1024 * 1024);
  // FAT32 needs at least 65525 clusters to be FAT32 rather than FAT16, and a
  // driver that works that out from the cluster count will refuse a volume
  // that claims FAT32 and has fewer.
  wanted = std::max<uint64_t>(wanted, static_cast<uint64_t>(70000) * kClusterSize);

  uint32_t totalSectors = static_cast<uint32_t>((wanted + kSectorSize - 1) / kSectorSize);
  totalSectors = (totalSectors + kSectorsPerCluster - 1) / kSectorsPerCluster * kSectorsPerCluster;
  const uint32_t partitionSectors = totalSectors;

  // The FAT has to hold an entry for every cluster, and the clusters are what
  // is left after the FAT — so solve it by settling.
  uint32_t fatSectors = 1;
  uint32_t clusters = 0;
  for (int pass = 0; pass < 8; ++pass) {
    const uint32_t dataSectors = partitionSectors - kReservedSectors - kFatCount * fatSectors;
    clusters = dataSectors / kSectorsPerCluster;
    const uint32_t needed = ((clusters + 2) * 4 + kSectorSize - 1) / kSectorSize;
    if (needed == fatSectors) break;
    fatSectors = needed;
  }
  if (contentClusters + 2 > clusters) {
    if (error) *error = "the directory does not fit in a card of that size";
    return false;
  }

  image->assign(static_cast<size_t>(kPartitionStart + partitionSectors) * kSectorSize, 0);
  uint8_t* base = image->data();

  // ── MBR ────────────────────────────────────────────────────────────────────
  uint8_t* mbr = base;
  uint8_t* entry = mbr + 446;
  entry[0] = 0x00;        // not bootable
  entry[1] = 0x20;        // CHS start, nominal
  entry[2] = 0x21;
  entry[3] = 0x00;
  entry[4] = 0x0C;        // FAT32 with LBA
  entry[5] = 0xFE;        // CHS end, nominal
  entry[6] = 0xFF;
  entry[7] = 0xFF;
  put32(entry + 8, kPartitionStart);
  put32(entry + 12, partitionSectors);
  put16(mbr + 510, 0xAA55);

  // ── Boot sector ────────────────────────────────────────────────────────────
  const uint32_t volumeStart = kPartitionStart;
  const uint32_t fatStart = volumeStart + kReservedSectors;
  const uint32_t dataStart = fatStart + kFatCount * fatSectors;

  auto writeBootSector = [&](uint32_t lba) {
    uint8_t* boot = base + static_cast<size_t>(lba) * kSectorSize;
    boot[0] = 0xEB; boot[1] = 0x58; boot[2] = 0x90;
    std::memcpy(boot + 3, "MSWIN4.1", 8);
    put16(boot + 11, kSectorSize);
    boot[13] = kSectorsPerCluster;
    put16(boot + 14, kReservedSectors);
    boot[16] = kFatCount;
    put16(boot + 17, 0);          // no fixed root directory on FAT32
    put16(boot + 19, 0);          // total sectors lives in the 32-bit field
    boot[21] = 0xF8;              // fixed disk
    put16(boot + 22, 0);          // FAT size lives in the 32-bit field
    put16(boot + 24, 63);
    put16(boot + 26, 255);
    put32(boot + 28, volumeStart);
    put32(boot + 32, partitionSectors);
    put32(boot + 36, fatSectors);
    put16(boot + 40, 0);          // both FATs live, first is active
    put16(boot + 42, 0);          // version 0.0
    put32(boot + 44, 2);          // the root directory starts at cluster 2
    put16(boot + 48, 1);          // FSInfo sector, relative to the volume
    put16(boot + 50, 6);          // backup boot sector
    boot[64] = 0x80;
    boot[66] = 0x29;              // an extended boot signature follows
    put32(boot + 67, 0xF2EE1401);
    std::memcpy(boot + 71, "FREEINK SD ", 11);
    std::memcpy(boot + 82, "FAT32   ", 8);
    put16(boot + 510, 0xAA55);
  };
  writeBootSector(volumeStart);
  writeBootSector(volumeStart + 6);

  auto writeFsInfo = [&](uint32_t lba) {
    uint8_t* info = base + static_cast<size_t>(lba) * kSectorSize;
    put32(info + 0, 0x41615252);
    put32(info + 484, 0x61417272);
    put32(info + 488, clusters - contentClusters);
    put32(info + 492, contentClusters + 2);
    put32(info + 508, 0xAA550000);
  };
  writeFsInfo(volumeStart + 1);
  writeFsInfo(volumeStart + 7);

  Writer writer(image, dataStart, fatStart, fatSectors);
  writer.setFat(0, 0x0FFFFFF8);
  writer.setFat(1, 0x0FFFFFFF);

  if (!emit(&root, true, writer, *image, dataStart, 0, error)) return false;
  return true;
}

}  // namespace freeink::sim
