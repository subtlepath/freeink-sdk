#pragma once

// FreeInk simulator — SdFat shim over a host directory.
//
// The virtual card is a directory the CLI mounts (`freeink-sim sd mount
// ./cards/default`); every firmware path is resolved inside it, so the reader's
// file I/O is real file I/O and an EPUB dropped in that directory shows up in
// the library exactly as it would on a card.
//
// What this does NOT model: the FAT layer itself. Cluster allocation, the
// 8.3/LFN encoding, corrupt-volume recovery and raw sector access are the real
// card's business — code that depends on those needs hardware. Free/used space
// is reported from the host filesystem, and rawBlockDevice() has no sectors to
// hand out, so the USB-MSC path reports "no raw device" rather than pretending.
// Everything a reader firmware normally does — open, list, read, write, mkdir,
// rename, remove — behaves.

#include <Arduino.h>
#include <freeink_sim_abi.h>

#include <cstdio>
#include <string>

// ── Open flags (SdFat's names; they map onto POSIX modes) ────────────────────
typedef uint8_t oflag_t;
#ifndef O_RDONLY
#define O_RDONLY 0x00
#endif
#ifndef O_WRONLY
#define O_WRONLY 0x01
#endif
#ifndef O_RDWR
#define O_RDWR 0x02
#endif
#ifndef O_APPEND
#define O_APPEND 0x08
#endif
#ifndef O_CREAT
#define O_CREAT 0x10
#endif
#ifndef O_TRUNC
#define O_TRUNC 0x20
#endif
#ifndef O_EXCL
#define O_EXCL 0x40
#endif
#define O_AT_END 0x80
#define O_WRITE O_WRONLY
#define O_READ O_RDONLY
#define FILE_READ O_RDONLY
#define FILE_WRITE (O_RDWR | O_CREAT | O_AT_END)

// Sector I/O interface. The simulator has no sector-addressable card behind the
// host directory, so the volume hands back nullptr and callers take their
// "raw access unavailable" path.
class FsBlockDeviceInterface {
 public:
  virtual ~FsBlockDeviceInterface() = default;
  virtual bool readSector(uint32_t sector, uint8_t* dst) = 0;
  virtual bool writeSector(uint32_t sector, const uint8_t* src) = 0;
  virtual bool readSectors(uint32_t sector, uint8_t* dst, size_t n) = 0;
  virtual bool writeSectors(uint32_t sector, const uint8_t* src, size_t n) = 0;
  virtual uint32_t sectorCount() = 0;
  virtual bool syncDevice() = 0;
  virtual bool isBusy() { return false; }
};

class FsFile : public Print {
 public:
  FsFile() = default;
  FsFile(const FsFile&) = delete;
  FsFile& operator=(const FsFile&) = delete;
  FsFile(FsFile&& o) noexcept { moveFrom(o); }
  FsFile& operator=(FsFile&& o) noexcept {
    if (this != &o) {
      close();
      moveFrom(o);
    }
    return *this;
  }
  ~FsFile() override { close(); }

  bool open(const char* path, oflag_t oflag = O_RDONLY);
  bool isOpen() const { return _fp != nullptr || _dir != nullptr; }
  explicit operator bool() const { return isOpen(); }
  void close();

  bool isDirectory() const { return _dir != nullptr; }
  bool isDir() const { return isDirectory(); }
  bool isFile() const { return _fp != nullptr; }

  // Directory iteration. Returns a closed file when the directory is exhausted.
  FsFile openNextFile(oflag_t oflag = O_RDONLY);
  void rewindDirectory();
  void rewind();

  size_t getName(char* out, size_t cap) const;
  const char* name() const { return _name.c_str(); }

  uint64_t fileSize() const;
  uint64_t size() const { return fileSize(); }
  int available();
  uint64_t curPosition() const;
  uint64_t position() const { return curPosition(); }
  bool seekSet(uint64_t pos);
  bool seekCur(int64_t delta);
  bool seekEnd(int64_t delta = 0);
  bool seek(uint64_t pos) { return seekSet(pos); }

  int read();
  int read(void* buf, size_t count);
  int peek();
  size_t write(uint8_t b) override;
  size_t write(const uint8_t* buf, size_t count) override;
  size_t write(const char* s) { return write(reinterpret_cast<const uint8_t*>(s), strlen(s)); }
  size_t print(const char* s) { return write(s); }
  size_t print(const String& s) { return write(s.c_str()); }
  size_t println(const char* s) { return write(s) + write("\n"); }
  bool truncate(uint64_t length = 0);
  bool sync();
  void flush() { sync(); }

 private:
  void moveFrom(FsFile& o);

  FILE* _fp = nullptr;
  void* _dir = nullptr;  // DIR*, opaque to keep <dirent.h> out of this header
  std::string _hostPath;
  std::string _name;
};

// A mounted volume. SdFat's FsVolume and the SDMMC path's bare FsVolume are the
// same object here, since both resolve to the one mounted host directory.
class FsVolume {
 public:
  bool begin(FsBlockDeviceInterface* = nullptr, bool = true, uint8_t = 1) { return fsim_sd_present() != 0; }
  void end() {}

  FsFile open(const char* path, oflag_t oflag = O_RDONLY) {
    FsFile f;
    f.open(path, oflag);
    return f;
  }
  bool exists(const char* path);
  bool mkdir(const char* path, bool createParents = true);
  bool remove(const char* path);
  bool rmdir(const char* path);
  bool rename(const char* path, const char* newPath);

  // Reported from the host filesystem holding the mount, scaled to the card
  // size the CLI configured, so a firmware's "card full" logic can be driven.
  uint32_t bytesPerCluster() const { return 32768; }
  uint32_t clusterCount() const;
  int32_t freeClusterCount() const;
  uint8_t fatType() const { return 32; }
};

// SdFat itself: a volume plus the card bring-up the SPI path calls.
class SdFat : public FsVolume {
 public:
  bool begin(uint8_t = 0, uint32_t = 0) { return fsim_sd_present() != 0; }
  template <typename ConfigT>
  bool begin(const ConfigT&) {
    return fsim_sd_present() != 0;
  }
  void end() {}
  FsBlockDeviceInterface* card() { return nullptr; }
  uint8_t sdErrorCode() const { return fsim_sd_present() ? 0 : 0x01; }
  uint8_t sdErrorData() const { return 0; }
  void errorHalt(Print&) {}
  void initErrorHalt(Print&) {}
};

using SdFs = SdFat;
using File32 = FsFile;
using FsBaseFile = FsFile;

// SPI/SDIO card configuration objects. Accepted and ignored: the simulator's
// card does not ride a bus.
struct SdSpiConfig {
  SdSpiConfig() = default;
  SdSpiConfig(uint8_t, uint8_t, uint32_t) {}
  SdSpiConfig(uint8_t, uint8_t, uint32_t, void*) {}
};
struct SdioConfig {
  SdioConfig() = default;
  explicit SdioConfig(uint8_t) {}
};
#define SHARED_SPI 0
#define DEDICATED_SPI 1
#define FIFO_SDIO 0
#define SD_SCK_MHZ(n) ((uint32_t)(n) * 1000000UL)
