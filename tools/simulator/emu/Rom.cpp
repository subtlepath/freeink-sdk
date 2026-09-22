// FreeInk emulator — mask ROM, emulated at the call rather than the
// instruction. See Rom.h for why.

#include "Rom.h"

#include "Md5.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>

namespace freeink::sim::emu {
namespace {

// ── Guest memory access ──────────────────────────────────────────────────────
// Bulk copies take the host-pointer fast path when the range is plain memory,
// which matters: an IDF boot moves megabytes through ROM memcpy.

void guestCopyOut(Bus& bus, uint32_t address, void* out, size_t length) {
  auto* dst = static_cast<uint8_t*>(out);
  size_t done = 0;
  while (done < length) {
    const uint32_t at = address + static_cast<uint32_t>(done);
    const uint32_t chunk = static_cast<uint32_t>(std::min<size_t>(length - done, 0x1000 - (at & 0xFFF)));
    if (const uint8_t* host = bus.hostRead(at, chunk)) {
      std::memcpy(dst + done, host, chunk);
      done += chunk;
      continue;
    }
    dst[done] = bus.read8(at);
    ++done;
    if (bus.faulted()) return;
  }
}

void guestCopyIn(Bus& bus, uint32_t address, const void* in, size_t length) {
  const auto* src = static_cast<const uint8_t*>(in);
  size_t done = 0;
  while (done < length) {
    const uint32_t at = address + static_cast<uint32_t>(done);
    const uint32_t chunk = static_cast<uint32_t>(std::min<size_t>(length - done, 0x1000 - (at & 0xFFF)));
    if (uint8_t* host = bus.hostWrite(at, chunk)) {
      std::memcpy(host, src + done, chunk);
      done += chunk;
      continue;
    }
    bus.write8(at, src[done]);
    ++done;
    if (bus.faulted()) return;
  }
}






// ── MD5 ──────────────────────────────────────────────────────────────────────
// The ROM's MD5 is used to verify a partition table's checksum, so it has to
// be the real algorithm operating on the guest's own context struct
// (uint32 buf[4]; uint32 bits[2]; uint8 in[64]) — an app may interleave
// updates with other work, and the context lives in guest memory between them.

struct GuestMd5 {
  uint32_t buf[4];
  uint32_t bits[2];
  uint8_t in[64];
};

void md5Load(Bus& bus, uint32_t address, GuestMd5* out) { guestCopyOut(bus, address, out, sizeof(GuestMd5)); }
void md5Store(Bus& bus, uint32_t address, const GuestMd5& value) {
  guestCopyIn(bus, address, &value, sizeof(GuestMd5));
}

// ── CRC ──────────────────────────────────────────────────────────────────────
// The ROM's crc32_le is the standard reflected CRC-32 with pre/post inversion
// applied by the caller, which is why the seed is passed straight through.
uint32_t crc32LeImpl(uint32_t crc, const uint8_t* data, size_t length) {
  crc = ~crc;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ (0xEDB88320u & (~(crc & 1) + 1));
  }
  return ~crc;
}

uint16_t crc16LeImpl(uint16_t crc, const uint8_t* data, size_t length) {
  crc = static_cast<uint16_t>(~crc);
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = static_cast<uint16_t>((crc >> 1) ^ (0x8408u & (~(crc & 1) + 1)));
    }
  }
  return static_cast<uint16_t>(~crc);
}

uint8_t crc8LeImpl(uint8_t crc, const uint8_t* data, size_t length) {
  crc = static_cast<uint8_t>(~crc);
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = static_cast<uint8_t>((crc >> 1) ^ (0x8Cu & (~(crc & 1) + 1)));
    }
  }
  return static_cast<uint8_t>(~crc);
}

}  // namespace

uint32_t romCrc32Le(uint32_t crc, const uint8_t* data, size_t length) { return crc32LeImpl(crc, data, length); }
uint16_t romCrc16Le(uint16_t crc, const uint8_t* data, size_t length) { return crc16LeImpl(crc, data, length); }
uint8_t romCrc8Le(uint8_t crc, const uint8_t* data, size_t length) { return crc8LeImpl(crc, data, length); }

// ── MD5 ──────────────────────────────────────────────────────────────────────
// Bound here rather than in RomHandlers.cpp so the handlers sit next to the
// context marshalling they depend on.
void Rom::registerMd5Handlers() {
  bind("MD5Init", [](Rom& rom, Cpu& cpu) {
    GuestMd5 context{};
    context.buf[0] = 0x67452301;
    context.buf[1] = 0xefcdab89;
    context.buf[2] = 0x98badcfe;
    context.buf[3] = 0x10325476;
    context.bits[0] = 0;
    context.bits[1] = 0;
    md5Store(rom.bus(), cpu.arg(0), context);
    cpu.setReturn(0);
  });
  bind("MD5Update", [](Rom& rom, Cpu& cpu) {
    const uint32_t contextAddress = cpu.arg(0);
    const uint32_t source = cpu.arg(1);
    uint32_t length = cpu.arg(2);
    GuestMd5 context{};
    md5Load(rom.bus(), contextAddress, &context);

    const uint32_t previousBits = context.bits[0];
    context.bits[0] += length << 3;
    if (context.bits[0] < previousBits) ++context.bits[1];
    context.bits[1] += length >> 29;

    std::vector<uint8_t> data(length);
    if (length) rom.readBytes(source, data.data(), length);
    size_t offset = 0;
    uint32_t used = (previousBits >> 3) & 0x3F;

    if (used) {
      const uint32_t space = 64 - used;
      if (length < space) {
        std::memcpy(context.in + used, data.data(), length);
        md5Store(rom.bus(), contextAddress, context);
        cpu.setReturn(0);
        return;
      }
      std::memcpy(context.in + used, data.data(), space);
      uint32_t words[16];
      std::memcpy(words, context.in, 64);
      md5Transform(context.buf, words);
      offset = space;
      length -= space;
    }
    while (length >= 64) {
      uint32_t words[16];
      std::memcpy(words, data.data() + offset, 64);
      md5Transform(context.buf, words);
      offset += 64;
      length -= 64;
    }
    if (length) std::memcpy(context.in, data.data() + offset, length);
    md5Store(rom.bus(), contextAddress, context);
    cpu.setReturn(0);
  });
  bind("MD5Final", [](Rom& rom, Cpu& cpu) {
    const uint32_t digestAddress = cpu.arg(0);
    const uint32_t contextAddress = cpu.arg(1);
    GuestMd5 context{};
    md5Load(rom.bus(), contextAddress, &context);

    uint32_t used = (context.bits[0] >> 3) & 0x3F;
    context.in[used++] = 0x80;
    if (used > 56) {
      std::memset(context.in + used, 0, 64 - used);
      uint32_t words[16];
      std::memcpy(words, context.in, 64);
      md5Transform(context.buf, words);
      std::memset(context.in, 0, 56);
    } else {
      std::memset(context.in + used, 0, 56 - used);
    }
    std::memcpy(context.in + 56, context.bits, 8);
    uint32_t words[16];
    std::memcpy(words, context.in, 64);
    md5Transform(context.buf, words);

    rom.writeBytes(digestAddress, context.buf, 16);
    cpu.setReturn(0);
  });
}

// ── printf ───────────────────────────────────────────────────────────────────
// A small formatter over the guest's argument registers. It covers what ESP-IDF
// and the ROM itself actually emit; an unrecognised conversion is copied
// through verbatim rather than silently dropped, so a wrong log line is visibly
// wrong instead of missing.
GuestArgSource cpuArgSource(Cpu& cpu, int firstArgIndex) {
  auto index = std::make_shared<int>(firstArgIndex);
  return [&cpu, index]() { return cpu.arg((*index)++); };
}

GuestArgSource vaListArgSource(Rom& rom, uint32_t vaList) {
  auto cursor = std::make_shared<uint32_t>(vaList);
  return [&rom, cursor]() {
    const uint32_t value = rom.bus().read32(*cursor);
    *cursor += 4;
    return value;
  };
}

std::string formatGuestPrintf(Rom& rom, const std::string& format, const GuestArgSource& nextArg) {
  std::string out;
  // A 64-bit value arrives as two consecutive words on both ABIs.
  auto nextArg64 = [&]() {
    const uint64_t low = nextArg();
    const uint64_t high = nextArg();
    return low | (high << 32);
  };

  for (size_t i = 0; i < format.size(); ++i) {
    if (format[i] != '%') {
      out += format[i];
      continue;
    }
    if (i + 1 >= format.size()) break;

    std::string spec = "%";
    ++i;
    // flags, width, precision
    while (i < format.size() && std::strchr("-+ #0", format[i])) spec += format[i++];
    while (i < format.size() && (std::isdigit(static_cast<unsigned char>(format[i])) || format[i] == '*')) {
      if (format[i] == '*') {
        spec += std::to_string(static_cast<int32_t>(nextArg()));
        ++i;
      } else {
        spec += format[i++];
      }
    }
    if (i < format.size() && format[i] == '.') {
      spec += format[i++];
      while (i < format.size() && (std::isdigit(static_cast<unsigned char>(format[i])) || format[i] == '*')) {
        if (format[i] == '*') {
          spec += std::to_string(static_cast<int32_t>(nextArg()));
          ++i;
        } else {
          spec += format[i++];
        }
      }
    }
    int longCount = 0;
    while (i < format.size() && std::strchr("lhzjt", format[i])) {
      if (format[i] == 'l') ++longCount;
      ++i;
    }
    if (i >= format.size()) break;

    const char conversion = format[i];
    char buffer[512];
    switch (conversion) {
      case 'd':
      case 'i': {
        if (longCount >= 2) {
          const int64_t value = static_cast<int64_t>(nextArg64());
          std::snprintf(buffer, sizeof(buffer), (spec + "lld").c_str(), static_cast<long long>(value));
        } else {
          std::snprintf(buffer, sizeof(buffer), (spec + "d").c_str(), static_cast<int32_t>(nextArg()));
        }
        out += buffer;
        break;
      }
      case 'u':
      case 'x':
      case 'X':
      case 'o': {
        if (longCount >= 2) {
          const uint64_t value = nextArg64();
          std::snprintf(buffer, sizeof(buffer), (spec + "ll" + conversion).c_str(),
                        static_cast<unsigned long long>(value));
        } else {
          std::snprintf(buffer, sizeof(buffer), (spec + conversion).c_str(), nextArg());
        }
        out += buffer;
        break;
      }
      case 'c':
        std::snprintf(buffer, sizeof(buffer), (spec + "c").c_str(), static_cast<int>(nextArg()));
        out += buffer;
        break;
      case 'p':
        std::snprintf(buffer, sizeof(buffer), "0x%08x", nextArg());
        out += buffer;
        break;
      case 's': {
        const uint32_t pointer = nextArg();
        const std::string value = pointer ? rom.readString(pointer) : "(null)";
        std::snprintf(buffer, sizeof(buffer), (spec + "s").c_str(), value.c_str());
        out += buffer;
        break;
      }
      case 'f':
      case 'F':
      case 'g':
      case 'G':
      case 'e':
      case 'E': {
        const uint64_t raw = nextArg64();
        double value;
        std::memcpy(&value, &raw, 8);
        std::snprintf(buffer, sizeof(buffer), (spec + conversion).c_str(), value);
        out += buffer;
        break;
      }
      case '%':
        out += '%';
        break;
      default:
        out += spec;
        out += conversion;
        break;
    }
  }
  return out;
}

// ── Rom ──────────────────────────────────────────────────────────────────────

Rom::Rom(Bus& bus, const SocDesc& soc) : bus_(bus), soc_(soc) {
  cpuMhz_ = soc.cpuHz / 1000000;
  registerHandlers();
}

bool Rom::loadSymbols(const std::string& path, std::string* error) {
  std::ifstream file(path);
  if (!file) {
    if (error) {
      *error = "cannot read the ROM symbol map " + path +
               " — generate it with tools/simulator/emu/rom/import-rom-symbols.py";
    }
    return false;
  }
  byAddress_.clear();
  byName_.clear();
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream parser(line);
    std::string address;
    std::string name;
    if (!(parser >> address >> name)) continue;
    const uint32_t value = static_cast<uint32_t>(std::stoul(address, nullptr, 16));
    // Several names can share an address (aliases like esp_rom_printf and
    // ets_printf). Keep the first, which sorts deterministically.
    byAddress_.emplace(value, name);
    byName_.emplace(name, value);
  }
  if (byAddress_.empty()) {
    if (error) *error = path + " holds no symbols";
    return false;
  }

  // Bind the implementations to the addresses this chip puts them at.
  //
  // Some routines are published twice under different names, at different
  // addresses, and IDF calls whichever its headers gave it. They are the same
  // routine, so one implementation serves both rather than the table carrying
  // every name twice:
  //
  //   rom_Cache_Suspend_DCache   is Cache_Suspend_DCache          (S3)
  //   esp_rom_output_tx_one_char is esp_rom_uart_tx_one_char      (IDF v5.5
  //                                 renamed the console routines)
  static const struct {
    const char* prefix;
    const char* canonical;
  } kAliases[] = {
      {"rom_", ""},
      {"esp_rom_output_", "esp_rom_uart_"},
  };

  handlers_.clear();
  for (const auto& entry : byName_) {
    auto handler = handlersByName_.find(entry.first);
    for (const auto& alias : kAliases) {
      if (handler != handlersByName_.end()) break;
      const std::string prefix = alias.prefix;
      if (entry.first.rfind(prefix, 0) != 0) continue;
      handler = handlersByName_.find(alias.canonical + entry.first.substr(prefix.size()));
    }
    if (handler != handlersByName_.end()) handlers_.emplace(entry.second, handler->second);
  }
  return true;
}

const char* Rom::symbolAt(uint32_t address) const {
  auto it = byAddress_.find(address);
  return it == byAddress_.end() ? nullptr : it->second.c_str();
}

void Rom::bind(const char* name, Handler handler) { handlersByName_[name] = handler; }

bool Rom::call(Cpu& cpu, uint32_t address) {
  auto handler = handlers_.find(address);
  if (handler == handlers_.end()) {
    const char* name = symbolAt(address);
    char buf[192];
    if (name) {
      std::snprintf(buf, sizeof(buf), "ROM routine `%s` (0x%08X) is not implemented", name, address);
      missing_.push_back(name);
    } else {
      std::snprintf(buf, sizeof(buf),
                    "call to 0x%08X, which is inside mask ROM but is not a known entry point", address);
      missing_.push_back(std::string(buf));
    }
    cpu.stop(HaltReason::Unimplemented, buf);
    return false;
  }
  if (const char* name = symbolAt(address)) ++callCounts_[name];
  handler->second(*this, cpu);
  // The handler has run in place of the routine's body; now return the way the
  // routine would have. A handler that redirected control flow itself (a reset,
  // a longjmp-alike) has already moved the PC, so leave it alone.
  if (cpu.pc() == address) cpu.returnToCaller();
  return true;
}

std::string Rom::readString(uint32_t address, size_t limit) {
  std::string out;
  for (size_t i = 0; i < limit; ++i) {
    const uint8_t byte = bus_.read8(address + static_cast<uint32_t>(i));
    if (bus_.faulted() || byte == 0) break;
    out += static_cast<char>(byte);
  }
  return out;
}

void Rom::writeBytes(uint32_t address, const void* data, size_t length) {
  guestCopyIn(bus_, address, data, length);
}

void Rom::readBytes(uint32_t address, void* data, size_t length) {
  guestCopyOut(bus_, address, data, length);
}

void Rom::emit(const std::string& text) {
  if (output_ && !text.empty()) output_(text.data(), text.size());
}

void Rom::emitThroughPutc(Cpu& cpu, const std::string& text) {
  if (!putc1_) {
    emit(text);
    return;
  }
  for (char c : text) {
    const uint32_t argument = static_cast<uint32_t>(static_cast<unsigned char>(c));
    uint32_t ignored = 0;
    if (!cpu.callGuest(putc1_, &argument, 1, &ignored)) {
      // The sink faulted. Falling back keeps the rest of the message — losing
      // the log at the exact moment something went wrong would be the worst
      // possible time to lose it.
      putc1_ = 0;
      emit(text);
      return;
    }
  }
}

uint8_t Rom::analogRead(uint8_t block, uint8_t reg) {
  auto it = analog_.find(static_cast<uint16_t>((block << 8) | reg));
  return it == analog_.end() ? 0 : it->second;
}

void Rom::analogWrite(uint8_t block, uint8_t reg, uint8_t value) {
  analog_[static_cast<uint16_t>((block << 8) | reg)] = value;
}

}  // namespace freeink::sim::emu
