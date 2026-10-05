#pragma once
// Browser card uses the native simulator's real file-backed SdFat seam.
#include <SdFat.h>
#include <string>
#include <vector>
class SDCardManager {
  FsVolume volume_;
 public:
  static SDCardManager& getInstance() { static SDCardManager card; return card; }
  bool begin() { return volume_.begin(); }
  bool ready() const { return fsim_sd_present() != 0; }
  void shutdown() {}
  uint64_t sdTotalBytes() const { return fsim_sd_capacity_bytes(); }
  uint64_t sdUsedBytes() { return 0; }
  FsFile open(const char* path, oflag_t flags = O_RDONLY) { return volume_.open(path, flags); }
  bool mkdir(const char* path, bool parents = true) { return volume_.mkdir(path, parents); }
  bool exists(const char* path) { return volume_.exists(path); }
  bool remove(const char* path) { return volume_.remove(path); }
  bool rmdir(const char* path) { return volume_.rmdir(path); }
  bool rename(const char* path, const char* target) { return volume_.rename(path, target); }
  bool ensureDirectoryExists(const char* path) { return exists(path) || mkdir(path); }
  bool openFileForRead(const char*, const char* path, FsFile& file) { file = open(path); return bool(file); }
  bool openFileForRead(const char* tag, const std::string& path, FsFile& file) { return openFileForRead(tag, path.c_str(), file); }
  bool openFileForRead(const char* tag, const String& path, FsFile& file) { return openFileForRead(tag, path.c_str(), file); }
  bool openFileForWrite(const char*, const char* path, FsFile& file) { file = open(path, O_RDWR | O_CREAT | O_TRUNC); return bool(file); }
  bool openFileForWrite(const char* tag, const std::string& path, FsFile& file) { return openFileForWrite(tag, path.c_str(), file); }
  bool openFileForWrite(const char* tag, const String& path, FsFile& file) { return openFileForWrite(tag, path.c_str(), file); }
  size_t readFileToBuffer(const char* path, char* buffer, size_t capacity, size_t limit = 0) {
    if (!capacity) return 0;
    auto file = open(path); if (!file) { buffer[0] = 0; return 0; }
    const auto count = std::min(capacity - 1, limit ? limit : capacity - 1);
    const auto read = file.read(buffer, count); const size_t n = read > 0 ? read : 0;
    buffer[n] = 0; return n;
  }
  String readFile(const char* path) { auto file = open(path); if (!file) return String(); std::string text(file.fileSize(), '\0'); const int n = file.read(text.data(), text.size()); text.resize(n > 0 ? n : 0); return String(text); }
  bool readFileToStream(const char* path, Print& out, size_t chunk = 256) { auto file = open(path); if (!file) return false; std::vector<uint8_t> buffer(std::max<size_t>(1, chunk)); int n; while ((n = file.read(buffer.data(), buffer.size())) > 0) if (out.write(buffer.data(), n) != static_cast<size_t>(n)) return false; return n == 0; }
  bool writeFile(const char* path, const String& text) { auto file = open(path, O_RDWR | O_CREAT | O_TRUNC); return file && file.write(text.c_str()) == text.length() && file.sync(); }
  std::vector<String> listFiles(const char* path = "/", int limit = 200) { std::vector<String> result; auto dir = open(path); for (int i=0; i<limit; i++) { auto file = dir.openNextFile(); if (!file) break; result.push_back(String(file.name())); } return result; }
  bool removeDir(const char* path) { auto dir = open(path); if (!dir) return false; for (;;) { auto file = dir.openNextFile(); if (!file) break; std::string child = std::string(path)+"/"+file.name(); if (file.isDirectory()) { if (!removeDir(child.c_str())) return false; } else { file.close(); if (!remove(child.c_str())) return false; } } dir.close(); return rmdir(path); }
};
