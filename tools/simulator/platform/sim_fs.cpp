// FreeInk simulator — SdFat-over-host-directory and virtual flash.
//
// Firmware paths ("/books/x.epub") are resolved by the daemon into the mounted
// host directory, which also refuses traversal outside it, so a firmware bug
// that walks "../.." cannot reach the developer's home directory.

#include <SdFat.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <spi_flash_mmap.h>

#include <cstdio>
#include <ctime>
#include <cstring>
#include <string>
#include <vector>

#include <dirent.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <unistd.h>

namespace {

// Resolves a firmware path against the mounted card. Empty on failure (no card,
// or the path escapes the mount).
std::string hostPathFor(const char* fwPath) {
  if (!fwPath) return {};
  char buf[4096];
  if (fsim_sd_resolve(fwPath, buf, sizeof(buf)) != 0) return {};
  return std::string(buf);
}

const char* modeStringFor(oflag_t oflag) {
  const bool write = (oflag & (O_WRONLY | O_RDWR)) != 0;
  if (!write) return "rb";
  if (oflag & O_APPEND) return (oflag & O_RDWR) ? "a+b" : "ab";
  if (oflag & O_TRUNC) return (oflag & O_RDWR) ? "w+b" : "wb";
  if (oflag & O_CREAT) return "r+b";  // opened below, created first if missing
  return (oflag & O_RDWR) ? "r+b" : "wb";
}

std::string baseName(const std::string& path) {
  const size_t slash = path.find_last_of('/');
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

}  // namespace

// ── FsFile ───────────────────────────────────────────────────────────────────
void FsFile::moveFrom(FsFile& o) {
  _fp = o._fp;
  _dir = o._dir;
  _hostPath = std::move(o._hostPath);
  _name = std::move(o._name);
  o._fp = nullptr;
  o._dir = nullptr;
}

bool FsFile::open(const char* path, oflag_t oflag) {
  close();
  const std::string host = hostPathFor(path);
  if (host.empty()) return false;

  struct stat st {};
  const bool exists = ::stat(host.c_str(), &st) == 0;
  if (exists && S_ISDIR(st.st_mode)) {
    DIR* dir = ::opendir(host.c_str());
    if (!dir) return false;
    _dir = dir;
    _hostPath = host;
    _name = baseName(host);
    if (_name.empty()) _name = "/";
    return true;
  }

  if (!exists) {
    if (!(oflag & O_CREAT)) return false;
    FILE* created = ::fopen(host.c_str(), "wb");
    if (!created) return false;
    ::fclose(created);
  } else if ((oflag & O_EXCL) && (oflag & O_CREAT)) {
    return false;
  }

  _fp = ::fopen(host.c_str(), modeStringFor(oflag));
  if (!_fp) return false;
  _hostPath = host;
  _name = baseName(host);
  if (oflag & O_AT_END) ::fseeko(_fp, 0, SEEK_END);
  return true;
}

bool FsFile::close() {
  bool ok = true;
  if (_fp) {
    ok = ::fclose(_fp) == 0;
    _fp = nullptr;
  }
  if (_dir) {
    ok = (::closedir(static_cast<DIR*>(_dir)) == 0) && ok;
    _dir = nullptr;
  }
  _hostPath.clear();
  _name.clear();
  return ok;
}

bool FsFile::getModifyDateTime(uint16_t* date, uint16_t* time) const {
  struct stat st{};
  if (!date || !time || ::stat(_hostPath.c_str(), &st) != 0) return false;
  struct tm parts{};
  if (!localtime_r(&st.st_mtime, &parts) || parts.tm_year < 80) return false;
  *date = static_cast<uint16_t>(((parts.tm_year - 80) << 9) | ((parts.tm_mon + 1) << 5) | parts.tm_mday);
  *time = static_cast<uint16_t>((parts.tm_hour << 11) | (parts.tm_min << 5) | (parts.tm_sec / 2));
  return true;
}

bool FsFile::rename(const char* newPath) {
  char resolved[4096];
  if (fsim_sd_resolve(newPath, resolved, sizeof resolved) != 0) return false;
  if (::rename(_hostPath.c_str(), resolved) != 0) return false;
  _hostPath = resolved;
  _name = baseName(_hostPath);
  return true;
}

FsFile FsFile::openNextFile(oflag_t oflag) {
  FsFile next;
  if (!_dir) return next;
  // Skip "." and ".." — SdFat's directory iteration does not report them.
  for (;;) {
    struct dirent* entry = ::readdir(static_cast<DIR*>(_dir));
    if (!entry) return next;
    if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) continue;

    const std::string childHost = _hostPath + "/" + entry->d_name;
    struct stat st {};
    if (::stat(childHost.c_str(), &st) != 0) continue;
    if (S_ISDIR(st.st_mode)) {
      DIR* dir = ::opendir(childHost.c_str());
      if (!dir) continue;
      next._dir = dir;
    } else {
      FILE* fp = ::fopen(childHost.c_str(), modeStringFor(oflag));
      if (!fp) continue;
      next._fp = fp;
    }
    next._hostPath = childHost;
    next._name = entry->d_name;
    return next;
  }
}

void FsFile::rewindDirectory() {
  if (_dir) ::rewinddir(static_cast<DIR*>(_dir));
}

void FsFile::rewind() {
  if (_fp) ::fseeko(_fp, 0, SEEK_SET);
  rewindDirectory();
}

size_t FsFile::getName(char* out, size_t cap) const {
  if (!out || cap == 0) return 0;
  const size_t n = _name.size() < cap - 1 ? _name.size() : cap - 1;
  memcpy(out, _name.data(), n);
  out[n] = '\0';
  return n;
}

uint64_t FsFile::fileSize() const {
  if (_hostPath.empty()) return 0;
  struct stat st {};
  if (::stat(_hostPath.c_str(), &st) != 0) return 0;
  return static_cast<uint64_t>(st.st_size);
}

int FsFile::available() {
  if (!_fp) return 0;
  const uint64_t size = fileSize();
  const uint64_t pos = curPosition();
  return pos >= size ? 0 : static_cast<int>(size - pos);
}

uint64_t FsFile::curPosition() const {
  if (!_fp) return 0;
  const off_t pos = ::ftello(_fp);
  return pos < 0 ? 0 : static_cast<uint64_t>(pos);
}

bool FsFile::seekSet(uint64_t pos) { return _fp && ::fseeko(_fp, static_cast<off_t>(pos), SEEK_SET) == 0; }
bool FsFile::seekCur(int64_t delta) { return _fp && ::fseeko(_fp, static_cast<off_t>(delta), SEEK_CUR) == 0; }
bool FsFile::seekEnd(int64_t delta) { return _fp && ::fseeko(_fp, static_cast<off_t>(delta), SEEK_END) == 0; }

int FsFile::read() {
  if (!_fp) return -1;
  const int c = ::fgetc(_fp);
  return c == EOF ? -1 : c;
}

int FsFile::read(void* buf, size_t count) {
  if (!_fp || !buf) return -1;
  const size_t n = ::fread(buf, 1, count, _fp);
  if (n == 0 && ::ferror(_fp)) return -1;
  return static_cast<int>(n);
}

int FsFile::peek() {
  if (!_fp) return -1;
  const int c = ::fgetc(_fp);
  if (c == EOF) return -1;
  ::ungetc(c, _fp);
  return c;
}

size_t FsFile::write(uint8_t b) { return write(&b, 1); }

size_t FsFile::write(const uint8_t* buf, size_t count) {
  if (!_fp || !buf) return 0;
  return ::fwrite(buf, 1, count, _fp);
}

bool FsFile::truncate(uint64_t length) {
  if (!_fp) return false;
  ::fflush(_fp);
  return ::ftruncate(::fileno(_fp), static_cast<off_t>(length)) == 0;
}

bool FsFile::sync() {
  if (!_fp) return true;
  return ::fflush(_fp) == 0;
}

// ── FsVolume ─────────────────────────────────────────────────────────────────
bool FsVolume::exists(const char* path) {
  const std::string host = hostPathFor(path);
  if (host.empty()) return false;
  struct stat st {};
  return ::stat(host.c_str(), &st) == 0;
}

bool FsVolume::mkdir(const char* path, bool createParents) {
  const std::string host = hostPathFor(path);
  if (host.empty()) return false;
  if (!createParents) return ::mkdir(host.c_str(), 0755) == 0 || errno == EEXIST;

  // Create each missing component, as SdFat's pFlag does.
  std::string partial;
  size_t start = 0;
  while (start <= host.size()) {
    const size_t slash = host.find('/', start);
    partial = host.substr(0, slash == std::string::npos ? host.size() : slash);
    if (!partial.empty() && ::mkdir(partial.c_str(), 0755) != 0 && errno != EEXIST) return false;
    if (slash == std::string::npos) break;
    start = slash + 1;
  }
  return true;
}

bool FsVolume::remove(const char* path) {
  const std::string host = hostPathFor(path);
  return !host.empty() && ::remove(host.c_str()) == 0;
}

bool FsVolume::rmdir(const char* path) {
  const std::string host = hostPathFor(path);
  return !host.empty() && ::rmdir(host.c_str()) == 0;
}

bool FsVolume::rename(const char* path, const char* newPath) {
  const std::string from = hostPathFor(path);
  const std::string to = hostPathFor(newPath);
  return !from.empty() && !to.empty() && ::rename(from.c_str(), to.c_str()) == 0;
}

uint32_t FsVolume::clusterCount() const {
  const uint64_t capacity = fsim_sd_capacity_bytes();
  return static_cast<uint32_t>(capacity / bytesPerCluster());
}

int32_t FsVolume::freeClusterCount() const {
  // Free space comes from the host filesystem holding the mount, capped at the
  // virtual card's configured capacity so a small simulated card really does
  // report itself full.
  char buf[4096];
  if (fsim_sd_resolve("/", buf, sizeof(buf)) != 0) return -1;
  struct statvfs vfs {};
  if (::statvfs(buf, &vfs) != 0) return -1;
  const uint64_t hostFree = static_cast<uint64_t>(vfs.f_bavail) * vfs.f_frsize;
  const uint64_t capacity = fsim_sd_capacity_bytes();
  const uint64_t usable = hostFree < capacity ? hostFree : capacity;
  return static_cast<int32_t>(usable / bytesPerCluster());
}

// ── Virtual flash: partitions, OTA, mmap ─────────────────────────────────────
//
// A single host file stands in for the flash chip. The layout below matches the
// stock ESP32-S3 16 MB "default_16MB"-style table the FreeInk sample configs
// use; it is enough for the SDK's recovery/OTA paths to find the partitions
// they look for by type and label.
namespace {

constexpr uint32_t kFlashSize = 16u * 1024 * 1024;

esp_partition_t g_partitions[] = {
    {nullptr, ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, 0x009000, 0x005000, 0x1000, "nvs", false, false},
    {nullptr, ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, 0x00e000, 0x002000, 0x1000, "otadata", false,
     false},
    {nullptr, ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, 0x010000, 0x640000, 0x1000, "app0", false,
     false},
    {nullptr, ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, 0x650000, 0x640000, 0x1000, "app1", false,
     false},
    {nullptr, ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, 0xc90000, 0x360000, 0x1000, "spiffs", false,
     false},
};
constexpr size_t kPartitionCount = sizeof(g_partitions) / sizeof(g_partitions[0]);

const esp_partition_t* g_bootPartition = &g_partitions[2];

// The flash file lives beside the daemon's state; the daemon resolves the path.
FILE* flashFile() {
  static FILE* fp = nullptr;
  if (fp) return fp;
  char path[4096];
  if (fsim_sd_resolve("\x01flash", path, sizeof(path)) != 0) {
    // No state directory: fall back to an anonymous temp file so partition I/O
    // still behaves, it just does not persist across daemon restarts.
    fp = ::tmpfile();
  } else {
    fp = ::fopen(path, "r+b");
    if (!fp) fp = ::fopen(path, "w+b");
  }
  if (fp) {
    ::fseeko(fp, 0, SEEK_END);
    if (::ftello(fp) < static_cast<off_t>(kFlashSize)) {
      ::ftruncate(::fileno(fp), kFlashSize);
    }
  }
  return fp;
}

struct OtaSession {
  const esp_partition_t* partition;
  size_t offset;
};
std::vector<OtaSession> g_otaSessions;

}  // namespace

extern "C" const esp_partition_t* esp_partition_find_first(esp_partition_type_t type, esp_partition_subtype_t subtype,
                                                           const char* label) {
  for (size_t i = 0; i < kPartitionCount; ++i) {
    const esp_partition_t& p = g_partitions[i];
    if (type != ESP_PARTITION_TYPE_ANY && p.type != type) continue;
    if (subtype != ESP_PARTITION_SUBTYPE_ANY && p.subtype != subtype) continue;
    if (label && strcmp(p.label, label) != 0) continue;
    return &p;
  }
  return nullptr;
}

// The iterator is an index biased by one so a null handle stays distinguishable.
extern "C" esp_partition_iterator_t esp_partition_find(esp_partition_type_t type, esp_partition_subtype_t subtype,
                                                       const char* label) {
  for (size_t i = 0; i < kPartitionCount; ++i) {
    const esp_partition_t& p = g_partitions[i];
    if (type != ESP_PARTITION_TYPE_ANY && p.type != type) continue;
    if (subtype != ESP_PARTITION_SUBTYPE_ANY && p.subtype != subtype) continue;
    if (label && strcmp(p.label, label) != 0) continue;
    return reinterpret_cast<esp_partition_iterator_t>(i + 1);
  }
  return nullptr;
}

extern "C" const esp_partition_t* esp_partition_get(esp_partition_iterator_t it) {
  const size_t index = reinterpret_cast<size_t>(it);
  return (index >= 1 && index <= kPartitionCount) ? &g_partitions[index - 1] : nullptr;
}

extern "C" esp_partition_iterator_t esp_partition_next(esp_partition_iterator_t it) {
  const size_t index = reinterpret_cast<size_t>(it);
  return index < kPartitionCount ? reinterpret_cast<esp_partition_iterator_t>(index + 1) : nullptr;
}

extern "C" void esp_partition_iterator_release(esp_partition_iterator_t) {}

extern "C" esp_err_t esp_partition_read(const esp_partition_t* p, size_t off, void* dst, size_t size) {
  if (!p || !dst) return ESP_ERR_INVALID_ARG;
  if (off + size > p->size) return ESP_ERR_INVALID_SIZE;
  FILE* fp = flashFile();
  if (!fp) return ESP_FAIL;
  if (::fseeko(fp, static_cast<off_t>(p->address + off), SEEK_SET) != 0) return ESP_FAIL;
  return ::fread(dst, 1, size, fp) == size ? ESP_OK : ESP_FAIL;
}

extern "C" esp_err_t esp_partition_write(const esp_partition_t* p, size_t off, const void* src, size_t size) {
  if (!p || !src) return ESP_ERR_INVALID_ARG;
  if (off + size > p->size) return ESP_ERR_INVALID_SIZE;
  FILE* fp = flashFile();
  if (!fp) return ESP_FAIL;
  if (::fseeko(fp, static_cast<off_t>(p->address + off), SEEK_SET) != 0) return ESP_FAIL;
  const bool ok = ::fwrite(src, 1, size, fp) == size;
  ::fflush(fp);
  return ok ? ESP_OK : ESP_FAIL;
}

extern "C" esp_err_t esp_partition_erase_range(const esp_partition_t* p, size_t off, size_t size) {
  if (!p) return ESP_ERR_INVALID_ARG;
  if (off + size > p->size) return ESP_ERR_INVALID_SIZE;
  // Erased flash reads as 0xFF; writing that back is what makes a subsequent
  // "is this partition blank?" check behave like the real thing.
  std::vector<uint8_t> ones(size, 0xFF);
  return esp_partition_write(p, off, ones.data(), size);
}

extern "C" esp_err_t esp_partition_read_raw(const esp_partition_t* p, size_t off, void* dst, size_t size) {
  return esp_partition_read(p, off, dst, size);
}
extern "C" esp_err_t esp_partition_write_raw(const esp_partition_t* p, size_t off, const void* src, size_t size) {
  return esp_partition_write(p, off, src, size);
}

extern "C" const esp_partition_t* esp_ota_get_running_partition(void) { return g_bootPartition; }
extern "C" const esp_partition_t* esp_ota_get_boot_partition(void) { return g_bootPartition; }

extern "C" const esp_partition_t* esp_ota_get_next_update_partition(const esp_partition_t* start_from) {
  const esp_partition_t* running = start_from ? start_from : g_bootPartition;
  return running == &g_partitions[2] ? &g_partitions[3] : &g_partitions[2];
}

extern "C" esp_err_t esp_ota_set_boot_partition(const esp_partition_t* partition) {
  if (!partition || partition->type != ESP_PARTITION_TYPE_APP) return ESP_ERR_INVALID_ARG;
  g_bootPartition = partition;
  return ESP_OK;
}

extern "C" esp_err_t esp_ota_begin(const esp_partition_t* partition, size_t, esp_ota_handle_t* out_handle) {
  if (!partition || !out_handle) return ESP_ERR_INVALID_ARG;
  g_otaSessions.push_back({partition, 0});
  *out_handle = static_cast<esp_ota_handle_t>(g_otaSessions.size());
  return ESP_OK;
}

extern "C" esp_err_t esp_ota_write(esp_ota_handle_t handle, const void* data, size_t size) {
  if (handle == 0 || handle > g_otaSessions.size()) return ESP_ERR_INVALID_ARG;
  OtaSession& session = g_otaSessions[handle - 1];
  const esp_err_t err = esp_partition_write(session.partition, session.offset, data, size);
  if (err == ESP_OK) session.offset += size;
  return err;
}

extern "C" esp_err_t esp_ota_end(esp_ota_handle_t handle) {
  return (handle == 0 || handle > g_otaSessions.size()) ? ESP_ERR_INVALID_ARG : ESP_OK;
}

extern "C" esp_err_t spi_flash_mmap(size_t src_addr, size_t size, spi_flash_mmap_memory_t, const void** out_ptr,
                                    spi_flash_mmap_handle_t* out_handle) {
  if (!out_ptr) return ESP_ERR_INVALID_ARG;
  FILE* fp = flashFile();
  if (!fp) return ESP_FAIL;
  // Map page-aligned and hand back the offset pointer, as the real mmap does.
  const size_t pageSize = static_cast<size_t>(::getpagesize());
  const size_t alignedAddr = (src_addr / pageSize) * pageSize;
  const size_t delta = src_addr - alignedAddr;
  void* base = ::mmap(nullptr, size + delta, PROT_READ, MAP_PRIVATE, ::fileno(fp), static_cast<off_t>(alignedAddr));
  if (base == MAP_FAILED) return ESP_FAIL;
  *out_ptr = static_cast<const uint8_t*>(base) + delta;
  if (out_handle) *out_handle = 0;
  return ESP_OK;
}

extern "C" void spi_flash_munmap(spi_flash_mmap_handle_t) {}
