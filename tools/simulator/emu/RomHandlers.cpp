// FreeInk emulator — the native implementations behind intercepted ROM calls.
//
// Each handler reads the guest's argument registers, does the work against
// guest memory or the machine, and leaves a result. Rom::call() returns to the
// caller afterwards, so a handler only redirects control flow if it means to.
//
// The table is grouped the way ESP-IDF's own linker scripts are: libc, libgcc,
// console, timing, flash, clocks and pads. What is *not* here is as important:
// anything with no implementation stops the machine by name rather than
// returning a plausible-looking zero, because a ROM routine that silently does
// nothing produces a failure thousands of instructions later, in a place that
// tells you nothing.

#include "Rom.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace freeink::sim::emu {
namespace {

// Guest-side string and buffer helpers, kept short because the handlers below
// read almost identically to their C originals when they are.
std::vector<uint8_t> readBuffer(Rom& rom, uint32_t address, uint32_t length) {
  std::vector<uint8_t> data(length);
  if (length) rom.readBytes(address, data.data(), length);
  return data;
}

uint32_t guestStrlen(Rom& rom, uint32_t address) {
  Bus& bus = rom.bus();
  uint32_t length = 0;
  while (length < (1u << 20)) {
    if (bus.read8(address + length) == 0 || bus.faulted()) break;
    ++length;
  }
  return length;
}

int guestStrcmp(Rom& rom, uint32_t a, uint32_t b, uint32_t limit, bool caseless) {
  Bus& bus = rom.bus();
  for (uint32_t i = 0; i < limit; ++i) {
    int left = bus.read8(a + i);
    int right = bus.read8(b + i);
    if (caseless) {
      left = std::tolower(left);
      right = std::tolower(right);
    }
    if (left != right) return left - right;
    if (left == 0) return 0;
  }
  return 0;
}

float argFloat(Cpu& cpu, int index) {
  const uint32_t raw = cpu.arg(index);
  float value;
  std::memcpy(&value, &raw, 4);
  return value;
}

double argDouble(Cpu& cpu, int index) {
  const uint64_t raw = static_cast<uint64_t>(cpu.arg(index)) | (static_cast<uint64_t>(cpu.arg(index + 1)) << 32);
  double value;
  std::memcpy(&value, &raw, 8);
  return value;
}

void returnFloat(Cpu& cpu, float value) {
  uint32_t raw;
  std::memcpy(&raw, &value, 4);
  cpu.setReturn(raw);
}

void returnDouble(Cpu& cpu, double value) {
  uint64_t raw;
  std::memcpy(&raw, &value, 8);
  cpu.setReturn64(raw);
}

}  // namespace

void Rom::registerHandlers() {
  // ── libc: memory ───────────────────────────────────────────────────────────
  bind("memcpy", [](Rom& rom, Cpu& cpu) {
    const uint32_t dst = cpu.arg(0), src = cpu.arg(1), len = cpu.arg(2);
    if (len) {
      std::vector<uint8_t> data = readBuffer(rom, src, len);
      rom.writeBytes(dst, data.data(), len);
    }
    cpu.setReturn(dst);
  });
  bind("memmove", [](Rom& rom, Cpu& cpu) {
    const uint32_t dst = cpu.arg(0), src = cpu.arg(1), len = cpu.arg(2);
    if (len) {
      std::vector<uint8_t> data = readBuffer(rom, src, len);  // a copy makes overlap safe
      rom.writeBytes(dst, data.data(), len);
    }
    cpu.setReturn(dst);
  });
  bind("memset", [](Rom& rom, Cpu& cpu) {
    const uint32_t dst = cpu.arg(0), len = cpu.arg(2);
    if (len) {
      std::vector<uint8_t> data(len, static_cast<uint8_t>(cpu.arg(1)));
      rom.writeBytes(dst, data.data(), len);
    }
    cpu.setReturn(dst);
  });
  bind("bzero", [](Rom& rom, Cpu& cpu) {
    const uint32_t dst = cpu.arg(0), len = cpu.arg(1);
    if (len) {
      std::vector<uint8_t> data(len, 0);
      rom.writeBytes(dst, data.data(), len);
    }
    cpu.setReturn(0);
  });
  bind("memcmp", [](Rom& rom, Cpu& cpu) {
    const uint32_t len = cpu.arg(2);
    std::vector<uint8_t> a = readBuffer(rom, cpu.arg(0), len);
    std::vector<uint8_t> b = readBuffer(rom, cpu.arg(1), len);
    cpu.setReturn(static_cast<uint32_t>(len ? std::memcmp(a.data(), b.data(), len) : 0));
  });
  bind("memchr", [](Rom& rom, Cpu& cpu) {
    const uint32_t base = cpu.arg(0), len = cpu.arg(2);
    const uint8_t wanted = static_cast<uint8_t>(cpu.arg(1));
    std::vector<uint8_t> data = readBuffer(rom, base, len);
    for (uint32_t i = 0; i < len; ++i) {
      if (data[i] == wanted) {
        cpu.setReturn(base + i);
        return;
      }
    }
    cpu.setReturn(0);
  });

  // ── libc: strings ──────────────────────────────────────────────────────────
  bind("strlen", [](Rom& rom, Cpu& cpu) { cpu.setReturn(guestStrlen(rom, cpu.arg(0))); });
  bind("strnlen", [](Rom& rom, Cpu& cpu) {
    cpu.setReturn(std::min(guestStrlen(rom, cpu.arg(0)), cpu.arg(1)));
  });
  bind("strcmp", [](Rom& rom, Cpu& cpu) {
    cpu.setReturn(static_cast<uint32_t>(guestStrcmp(rom, cpu.arg(0), cpu.arg(1), 0xFFFFFFFFu, false)));
  });
  bind("strncmp", [](Rom& rom, Cpu& cpu) {
    cpu.setReturn(static_cast<uint32_t>(guestStrcmp(rom, cpu.arg(0), cpu.arg(1), cpu.arg(2), false)));
  });
  bind("strcasecmp", [](Rom& rom, Cpu& cpu) {
    cpu.setReturn(static_cast<uint32_t>(guestStrcmp(rom, cpu.arg(0), cpu.arg(1), 0xFFFFFFFFu, true)));
  });
  bind("strncasecmp", [](Rom& rom, Cpu& cpu) {
    cpu.setReturn(static_cast<uint32_t>(guestStrcmp(rom, cpu.arg(0), cpu.arg(1), cpu.arg(2), true)));
  });
  bind("strcpy", [](Rom& rom, Cpu& cpu) {
    const std::string value = rom.readString(cpu.arg(1), 1u << 20);
    rom.writeBytes(cpu.arg(0), value.c_str(), value.size() + 1);
    cpu.setReturn(cpu.arg(0));
  });
  bind("strncpy", [](Rom& rom, Cpu& cpu) {
    const uint32_t limit = cpu.arg(2);
    std::string value = rom.readString(cpu.arg(1), limit);
    value.resize(limit, '\0');  // strncpy pads to the full width
    if (limit) rom.writeBytes(cpu.arg(0), value.data(), limit);
    cpu.setReturn(cpu.arg(0));
  });
  bind("strlcpy", [](Rom& rom, Cpu& cpu) {
    const std::string value = rom.readString(cpu.arg(1), 1u << 20);
    const uint32_t size = cpu.arg(2);
    if (size) {
      const size_t copy = std::min<size_t>(value.size(), size - 1);
      rom.writeBytes(cpu.arg(0), value.c_str(), copy);
      const uint8_t nul = 0;
      rom.writeBytes(cpu.arg(0) + static_cast<uint32_t>(copy), &nul, 1);
    }
    cpu.setReturn(static_cast<uint32_t>(value.size()));
  });
  bind("strcat", [](Rom& rom, Cpu& cpu) {
    const uint32_t dst = cpu.arg(0);
    const uint32_t end = dst + guestStrlen(rom, dst);
    const std::string value = rom.readString(cpu.arg(1), 1u << 20);
    rom.writeBytes(end, value.c_str(), value.size() + 1);
    cpu.setReturn(dst);
  });
  bind("strchr", [](Rom& rom, Cpu& cpu) {
    const uint32_t base = cpu.arg(0);
    const char wanted = static_cast<char>(cpu.arg(1));
    const std::string value = rom.readString(base, 1u << 20);
    const size_t at = value.find(wanted);
    cpu.setReturn(at == std::string::npos ? (wanted == 0 ? base + value.size() : 0)
                                          : base + static_cast<uint32_t>(at));
  });
  bind("strrchr", [](Rom& rom, Cpu& cpu) {
    const uint32_t base = cpu.arg(0);
    const std::string value = rom.readString(base, 1u << 20);
    const size_t at = value.rfind(static_cast<char>(cpu.arg(1)));
    cpu.setReturn(at == std::string::npos ? 0 : base + static_cast<uint32_t>(at));
  });
  bind("strstr", [](Rom& rom, Cpu& cpu) {
    const uint32_t base = cpu.arg(0);
    const std::string haystack = rom.readString(base, 1u << 20);
    const std::string needle = rom.readString(cpu.arg(1), 1u << 16);
    const size_t at = haystack.find(needle);
    cpu.setReturn(at == std::string::npos ? 0 : base + static_cast<uint32_t>(at));
  });
  bind("atoi", [](Rom& rom, Cpu& cpu) {
    cpu.setReturn(static_cast<uint32_t>(std::atoi(rom.readString(cpu.arg(0), 64).c_str())));
  });
  // itoa/utoa are the ROM's own, not newlib's: the base argument goes up to
  // 36 and the buffer is the caller's, so the result is written back rather
  // than interned.
  bind("itoa", [](Rom& rom, Cpu& cpu) {
    const int32_t value = static_cast<int32_t>(cpu.arg(0));
    const uint32_t buffer = cpu.arg(1);
    const int base = static_cast<int>(cpu.arg(2));
    std::string text;
    if (base == 10 && value < 0) {
      text = std::to_string(value);
    } else {
      uint32_t magnitude = static_cast<uint32_t>(value);
      if (base < 2 || base > 36) {
        cpu.setReturn(0);
        return;
      }
      do {
        text.insert(text.begin(), "0123456789abcdefghijklmnopqrstuvwxyz"[magnitude % base]);
        magnitude /= static_cast<uint32_t>(base);
      } while (magnitude);
    }
    rom.writeBytes(buffer, text.c_str(), text.size() + 1);
    cpu.setReturn(buffer);
  });
  bind("utoa", [](Rom& rom, Cpu& cpu) {
    uint32_t value = cpu.arg(0);
    const uint32_t buffer = cpu.arg(1);
    const int base = static_cast<int>(cpu.arg(2));
    if (base < 2 || base > 36) {
      cpu.setReturn(0);
      return;
    }
    std::string text;
    do {
      text.insert(text.begin(), "0123456789abcdefghijklmnopqrstuvwxyz"[value % base]);
      value /= static_cast<uint32_t>(base);
    } while (value);
    rom.writeBytes(buffer, text.c_str(), text.size() + 1);
    cpu.setReturn(buffer);
  });
  bind("strtol", [](Rom& rom, Cpu& cpu) {
    const std::string text = rom.readString(cpu.arg(0), 256);
    char* end = nullptr;
    const long value = std::strtol(text.c_str(), &end, static_cast<int>(cpu.arg(2)));
    if (cpu.arg(1)) {
      const uint32_t consumed = cpu.arg(0) + static_cast<uint32_t>(end - text.c_str());
      rom.bus().write32(cpu.arg(1), consumed);
    }
    cpu.setReturn(static_cast<uint32_t>(value));
  });
  bind("strtoul", [](Rom& rom, Cpu& cpu) {
    const std::string text = rom.readString(cpu.arg(0), 256);
    char* end = nullptr;
    const unsigned long value = std::strtoul(text.c_str(), &end, static_cast<int>(cpu.arg(2)));
    if (cpu.arg(1)) {
      rom.bus().write32(cpu.arg(1), cpu.arg(0) + static_cast<uint32_t>(end - text.c_str()));
    }
    cpu.setReturn(static_cast<uint32_t>(value));
  });
  bind("abs", [](Rom&, Cpu& cpu) {
    const int32_t value = static_cast<int32_t>(cpu.arg(0));
    cpu.setReturn(static_cast<uint32_t>(value < 0 ? -value : value));
  });
  bind("isalnum", [](Rom&, Cpu& cpu) { cpu.setReturn(std::isalnum(static_cast<int>(cpu.arg(0))) ? 1 : 0); });
  bind("isalpha", [](Rom&, Cpu& cpu) { cpu.setReturn(std::isalpha(static_cast<int>(cpu.arg(0))) ? 1 : 0); });
  bind("isdigit", [](Rom&, Cpu& cpu) { cpu.setReturn(std::isdigit(static_cast<int>(cpu.arg(0))) ? 1 : 0); });
  bind("isspace", [](Rom&, Cpu& cpu) { cpu.setReturn(std::isspace(static_cast<int>(cpu.arg(0))) ? 1 : 0); });
  bind("toupper", [](Rom&, Cpu& cpu) { cpu.setReturn(std::toupper(static_cast<int>(cpu.arg(0)))); });
  bind("tolower", [](Rom&, Cpu& cpu) { cpu.setReturn(std::tolower(static_cast<int>(cpu.arg(0)))); });

  // ── libc: the rest ─────────────────────────────────────────────────────────
  // Pure computation, so these are the originals in all but instruction
  // encoding. They are bound as a group because an IDF app reaches for them
  // unpredictably and each one missing costs a boot.
  bind("div", [](Rom&, Cpu& cpu) {
    const int32_t numerator = static_cast<int32_t>(cpu.arg(0));
    const int32_t denominator = static_cast<int32_t>(cpu.arg(1));
    // div_t is two words, returned the same way a 64-bit value is.
    const int32_t quotient = denominator ? numerator / denominator : 0;
    const int32_t remainder = denominator ? numerator % denominator : 0;
    cpu.setReturn64((static_cast<uint64_t>(static_cast<uint32_t>(remainder)) << 32) |
                    static_cast<uint32_t>(quotient));
  });
  bind("ldiv", [](Rom&, Cpu& cpu) {
    const int32_t numerator = static_cast<int32_t>(cpu.arg(0));
    const int32_t denominator = static_cast<int32_t>(cpu.arg(1));
    const int32_t quotient = denominator ? numerator / denominator : 0;
    const int32_t remainder = denominator ? numerator % denominator : 0;
    cpu.setReturn64((static_cast<uint64_t>(static_cast<uint32_t>(remainder)) << 32) |
                    static_cast<uint32_t>(quotient));
  });
  bind("labs", [](Rom&, Cpu& cpu) {
    const int32_t value = static_cast<int32_t>(cpu.arg(0));
    cpu.setReturn(static_cast<uint32_t>(value < 0 ? -value : value));
  });
  bind("atol", [](Rom& rom, Cpu& cpu) {
    cpu.setReturn(static_cast<uint32_t>(std::atol(rom.readString(cpu.arg(0), 64).c_str())));
  });
  // A deterministic generator: two runs of the same firmware produce the same
  // sequence, which is what makes an emulator run reproducible.
  bind("srand", [](Rom& rom, Cpu& cpu) {
    rom.setRandomState(cpu.arg(0));
    cpu.setReturn(0);
  });
  bind("rand", [](Rom& rom, Cpu& cpu) { cpu.setReturn(rom.nextRandom() & 0x7FFFFFFF); });
  bind("rand_r", [](Rom& rom, Cpu& cpu) {
    const uint32_t statePointer = cpu.arg(0);
    uint32_t state = statePointer ? rom.bus().read32(statePointer) : 0;
    state = state * 1103515245u + 12345u;
    if (statePointer) rom.bus().write32(statePointer, state);
    cpu.setReturn((state >> 1) & 0x7FFFFFFF);
  });
  bind("strspn", [](Rom& rom, Cpu& cpu) {
    const std::string text = rom.readString(cpu.arg(0), 1u << 16);
    const std::string accept = rom.readString(cpu.arg(1), 1u << 12);
    cpu.setReturn(static_cast<uint32_t>(std::min(text.find_first_not_of(accept), text.size())));
  });
  bind("strcspn", [](Rom& rom, Cpu& cpu) {
    const std::string text = rom.readString(cpu.arg(0), 1u << 16);
    const std::string reject = rom.readString(cpu.arg(1), 1u << 12);
    cpu.setReturn(static_cast<uint32_t>(std::min(text.find_first_of(reject), text.size())));
  });
  bind("strlcat", [](Rom& rom, Cpu& cpu) {
    const uint32_t dst = cpu.arg(0);
    const uint32_t size = cpu.arg(2);
    const uint32_t used = std::min(guestStrlen(rom, dst), size);
    const std::string tail = rom.readString(cpu.arg(1), 1u << 16);
    if (used < size) {
      const size_t copy = std::min<size_t>(tail.size(), size - used - 1);
      rom.writeBytes(dst + used, tail.c_str(), copy);
      const uint8_t nul = 0;
      rom.writeBytes(dst + used + static_cast<uint32_t>(copy), &nul, 1);
    }
    cpu.setReturn(used + static_cast<uint32_t>(tail.size()));
  });
  bind("strncat", [](Rom& rom, Cpu& cpu) {
    const uint32_t dst = cpu.arg(0);
    const uint32_t end = dst + guestStrlen(rom, dst);
    std::string tail = rom.readString(cpu.arg(1), cpu.arg(2));
    rom.writeBytes(end, tail.c_str(), tail.size() + 1);
    cpu.setReturn(dst);
  });
  bind("memccpy", [](Rom& rom, Cpu& cpu) {
    const uint32_t dst = cpu.arg(0), src = cpu.arg(1), len = cpu.arg(3);
    const uint8_t stop = static_cast<uint8_t>(cpu.arg(2));
    std::vector<uint8_t> data = readBuffer(rom, src, len);
    for (uint32_t i = 0; i < len; ++i) {
      if (data[i] == stop) {
        rom.writeBytes(dst, data.data(), i + 1);
        cpu.setReturn(dst + i + 1);
        return;
      }
    }
    if (len) rom.writeBytes(dst, data.data(), len);
    cpu.setReturn(0);
  });
  bind("memrchr", [](Rom& rom, Cpu& cpu) {
    const uint32_t base = cpu.arg(0), len = cpu.arg(2);
    const uint8_t wanted = static_cast<uint8_t>(cpu.arg(1));
    std::vector<uint8_t> data = readBuffer(rom, base, len);
    for (uint32_t i = len; i > 0; --i) {
      if (data[i - 1] == wanted) {
        cpu.setReturn(base + i - 1);
        return;
      }
    }
    cpu.setReturn(0);
  });
  bind("isupper", [](Rom&, Cpu& cpu) { cpu.setReturn(std::isupper(static_cast<int>(cpu.arg(0))) ? 1 : 0); });
  bind("islower", [](Rom&, Cpu& cpu) { cpu.setReturn(std::islower(static_cast<int>(cpu.arg(0))) ? 1 : 0); });
  bind("isprint", [](Rom&, Cpu& cpu) { cpu.setReturn(std::isprint(static_cast<int>(cpu.arg(0))) ? 1 : 0); });
  bind("ispunct", [](Rom&, Cpu& cpu) { cpu.setReturn(std::ispunct(static_cast<int>(cpu.arg(0))) ? 1 : 0); });
  bind("iscntrl", [](Rom&, Cpu& cpu) { cpu.setReturn(std::iscntrl(static_cast<int>(cpu.arg(0))) ? 1 : 0); });
  bind("isgraph", [](Rom&, Cpu& cpu) { cpu.setReturn(std::isgraph(static_cast<int>(cpu.arg(0))) ? 1 : 0); });
  bind("isxdigit", [](Rom&, Cpu& cpu) { cpu.setReturn(std::isxdigit(static_cast<int>(cpu.arg(0))) ? 1 : 0); });

  // ── libgcc: 64-bit integer helpers ─────────────────────────────────────────
  // A 64-bit argument takes an aligned register pair on both ABIs, so arg(0)
  // and arg(1) are the halves of the first one either way.
  auto arg64 = [](Cpu& cpu, int index) {
    return static_cast<uint64_t>(cpu.arg(index)) | (static_cast<uint64_t>(cpu.arg(index + 1)) << 32);
  };
  (void)arg64;
  bind("__udivdi3", [](Rom&, Cpu& cpu) {
    const uint64_t a = static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32);
    const uint64_t b = static_cast<uint64_t>(cpu.arg(2)) | (static_cast<uint64_t>(cpu.arg(3)) << 32);
    cpu.setReturn64(b == 0 ? ~0ULL : a / b);
  });
  bind("__umoddi3", [](Rom&, Cpu& cpu) {
    const uint64_t a = static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32);
    const uint64_t b = static_cast<uint64_t>(cpu.arg(2)) | (static_cast<uint64_t>(cpu.arg(3)) << 32);
    cpu.setReturn64(b == 0 ? a : a % b);
  });
  bind("__divdi3", [](Rom&, Cpu& cpu) {
    const int64_t a = static_cast<int64_t>(static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32));
    const int64_t b = static_cast<int64_t>(static_cast<uint64_t>(cpu.arg(2)) | (static_cast<uint64_t>(cpu.arg(3)) << 32));
    cpu.setReturn64(b == 0 ? ~0ULL : static_cast<uint64_t>(a / b));
  });
  bind("__moddi3", [](Rom&, Cpu& cpu) {
    const int64_t a = static_cast<int64_t>(static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32));
    const int64_t b = static_cast<int64_t>(static_cast<uint64_t>(cpu.arg(2)) | (static_cast<uint64_t>(cpu.arg(3)) << 32));
    cpu.setReturn64(b == 0 ? static_cast<uint64_t>(a) : static_cast<uint64_t>(a % b));
  });
  bind("__udivmoddi4", [](Rom& rom, Cpu& cpu) {
    const uint64_t a = static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32);
    const uint64_t b = static_cast<uint64_t>(cpu.arg(2)) | (static_cast<uint64_t>(cpu.arg(3)) << 32);
    const uint32_t remainderOut = cpu.arg(4);
    const uint64_t quotient = b == 0 ? ~0ULL : a / b;
    const uint64_t remainder = b == 0 ? a : a % b;
    if (remainderOut) rom.writeBytes(remainderOut, &remainder, 8);
    cpu.setReturn64(quotient);
  });
  bind("__muldi3", [](Rom&, Cpu& cpu) {
    const uint64_t a = static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32);
    const uint64_t b = static_cast<uint64_t>(cpu.arg(2)) | (static_cast<uint64_t>(cpu.arg(3)) << 32);
    cpu.setReturn64(a * b);
  });
  bind("__ashldi3", [](Rom&, Cpu& cpu) {
    const uint64_t a = static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32);
    cpu.setReturn64(a << (cpu.arg(2) & 63));
  });
  bind("__lshrdi3", [](Rom&, Cpu& cpu) {
    const uint64_t a = static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32);
    cpu.setReturn64(a >> (cpu.arg(2) & 63));
  });
  bind("__ashrdi3", [](Rom&, Cpu& cpu) {
    const int64_t a = static_cast<int64_t>(static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32));
    cpu.setReturn64(static_cast<uint64_t>(a >> (cpu.arg(2) & 63)));
  });
  bind("__negdi2", [](Rom&, Cpu& cpu) {
    const uint64_t a = static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32);
    cpu.setReturn64(~a + 1);
  });
  bind("__cmpdi2", [](Rom&, Cpu& cpu) {
    const int64_t a = static_cast<int64_t>(static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32));
    const int64_t b = static_cast<int64_t>(static_cast<uint64_t>(cpu.arg(2)) | (static_cast<uint64_t>(cpu.arg(3)) << 32));
    cpu.setReturn(a < b ? 0 : (a == b ? 1 : 2));
  });
  bind("__ucmpdi2", [](Rom&, Cpu& cpu) {
    const uint64_t a = static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32);
    const uint64_t b = static_cast<uint64_t>(cpu.arg(2)) | (static_cast<uint64_t>(cpu.arg(3)) << 32);
    cpu.setReturn(a < b ? 0 : (a == b ? 1 : 2));
  });
  bind("__clzsi2", [](Rom&, Cpu& cpu) {
    const uint32_t value = cpu.arg(0);
    cpu.setReturn(value == 0 ? 32 : static_cast<uint32_t>(__builtin_clz(value)));
  });
  bind("__ctzsi2", [](Rom&, Cpu& cpu) {
    const uint32_t value = cpu.arg(0);
    cpu.setReturn(value == 0 ? 32 : static_cast<uint32_t>(__builtin_ctz(value)));
  });
  bind("__popcountsi2", [](Rom&, Cpu& cpu) {
    cpu.setReturn(static_cast<uint32_t>(__builtin_popcount(cpu.arg(0))));
  });
  // The 64-bit bit-counting helpers. They return an int, not a 64-bit value,
  // even though their argument is 64 bits wide.
  bind("__clzdi2", [](Rom&, Cpu& cpu) {
    const uint64_t value = static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32);
    cpu.setReturn(value == 0 ? 64 : static_cast<uint32_t>(__builtin_clzll(value)));
  });
  bind("__ctzdi2", [](Rom&, Cpu& cpu) {
    const uint64_t value = static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32);
    cpu.setReturn(value == 0 ? 64 : static_cast<uint32_t>(__builtin_ctzll(value)));
  });
  bind("__popcountdi2", [](Rom&, Cpu& cpu) {
    const uint64_t value = static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32);
    cpu.setReturn(static_cast<uint32_t>(__builtin_popcountll(value)));
  });
  bind("__ffsdi2", [](Rom&, Cpu& cpu) {
    const uint64_t value = static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32);
    cpu.setReturn(value == 0 ? 0 : static_cast<uint32_t>(__builtin_ctzll(value) + 1));
  });
  bind("__paritysi2", [](Rom&, Cpu& cpu) {
    cpu.setReturn(static_cast<uint32_t>(__builtin_parity(cpu.arg(0))));
  });
  bind("__bswapdi2", [](Rom&, Cpu& cpu) {
    const uint64_t value = static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32);
    cpu.setReturn64(__builtin_bswap64(value));
  });
  bind("__ffssi2", [](Rom&, Cpu& cpu) {
    cpu.setReturn(static_cast<uint32_t>(__builtin_ffs(static_cast<int>(cpu.arg(0)))));
  });
  bind("__bswapsi2", [](Rom&, Cpu& cpu) { cpu.setReturn(__builtin_bswap32(cpu.arg(0))); });
  bind("__divsi3", [](Rom&, Cpu& cpu) {
    const int32_t a = static_cast<int32_t>(cpu.arg(0)), b = static_cast<int32_t>(cpu.arg(1));
    cpu.setReturn(b == 0 ? 0xFFFFFFFFu : static_cast<uint32_t>(a / b));
  });
  bind("__udivsi3", [](Rom&, Cpu& cpu) {
    cpu.setReturn(cpu.arg(1) == 0 ? 0xFFFFFFFFu : cpu.arg(0) / cpu.arg(1));
  });
  bind("__modsi3", [](Rom&, Cpu& cpu) {
    const int32_t a = static_cast<int32_t>(cpu.arg(0)), b = static_cast<int32_t>(cpu.arg(1));
    cpu.setReturn(b == 0 ? static_cast<uint32_t>(a) : static_cast<uint32_t>(a % b));
  });
  bind("__umodsi3", [](Rom&, Cpu& cpu) {
    cpu.setReturn(cpu.arg(1) == 0 ? cpu.arg(0) : cpu.arg(0) % cpu.arg(1));
  });
  bind("__mulsi3", [](Rom&, Cpu& cpu) { cpu.setReturn(cpu.arg(0) * cpu.arg(1)); });

  // ── libgcc: soft float ─────────────────────────────────────────────────────
  // The C3 has no FPU, so every float operation an app performs lands here.
  bind("__addsf3", [](Rom&, Cpu& cpu) { returnFloat(cpu, argFloat(cpu, 0) + argFloat(cpu, 1)); });
  bind("__subsf3", [](Rom&, Cpu& cpu) { returnFloat(cpu, argFloat(cpu, 0) - argFloat(cpu, 1)); });
  bind("__mulsf3", [](Rom&, Cpu& cpu) { returnFloat(cpu, argFloat(cpu, 0) * argFloat(cpu, 1)); });
  bind("__divsf3", [](Rom&, Cpu& cpu) { returnFloat(cpu, argFloat(cpu, 0) / argFloat(cpu, 1)); });
  bind("__negsf2", [](Rom&, Cpu& cpu) { returnFloat(cpu, -argFloat(cpu, 0)); });
  bind("__adddf3", [](Rom&, Cpu& cpu) { returnDouble(cpu, argDouble(cpu, 0) + argDouble(cpu, 2)); });
  bind("__subdf3", [](Rom&, Cpu& cpu) { returnDouble(cpu, argDouble(cpu, 0) - argDouble(cpu, 2)); });
  bind("__muldf3", [](Rom&, Cpu& cpu) { returnDouble(cpu, argDouble(cpu, 0) * argDouble(cpu, 2)); });
  bind("__divdf3", [](Rom&, Cpu& cpu) { returnDouble(cpu, argDouble(cpu, 0) / argDouble(cpu, 2)); });
  bind("__negdf2", [](Rom&, Cpu& cpu) { returnDouble(cpu, -argDouble(cpu, 0)); });
  bind("__extendsfdf2", [](Rom&, Cpu& cpu) { returnDouble(cpu, static_cast<double>(argFloat(cpu, 0))); });
  bind("__truncdfsf2", [](Rom&, Cpu& cpu) { returnFloat(cpu, static_cast<float>(argDouble(cpu, 0))); });
  bind("__floatsisf", [](Rom&, Cpu& cpu) {
    returnFloat(cpu, static_cast<float>(static_cast<int32_t>(cpu.arg(0))));
  });
  bind("__floatunsisf", [](Rom&, Cpu& cpu) { returnFloat(cpu, static_cast<float>(cpu.arg(0))); });
  bind("__floatsidf", [](Rom&, Cpu& cpu) {
    returnDouble(cpu, static_cast<double>(static_cast<int32_t>(cpu.arg(0))));
  });
  bind("__floatunsidf", [](Rom&, Cpu& cpu) { returnDouble(cpu, static_cast<double>(cpu.arg(0))); });
  bind("__floatdidf", [](Rom&, Cpu& cpu) {
    const int64_t value = static_cast<int64_t>(static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32));
    returnDouble(cpu, static_cast<double>(value));
  });
  bind("__floatundidf", [](Rom&, Cpu& cpu) {
    const uint64_t value = static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32);
    returnDouble(cpu, static_cast<double>(value));
  });
  bind("__floatdisf", [](Rom&, Cpu& cpu) {
    const int64_t value = static_cast<int64_t>(static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32));
    returnFloat(cpu, static_cast<float>(value));
  });
  bind("__floatundisf", [](Rom&, Cpu& cpu) {
    const uint64_t value = static_cast<uint64_t>(cpu.arg(0)) | (static_cast<uint64_t>(cpu.arg(1)) << 32);
    returnFloat(cpu, static_cast<float>(value));
  });
  bind("__fixsfsi", [](Rom&, Cpu& cpu) { cpu.setReturn(static_cast<uint32_t>(static_cast<int32_t>(argFloat(cpu, 0)))); });
  bind("__fixunssfsi", [](Rom&, Cpu& cpu) { cpu.setReturn(static_cast<uint32_t>(argFloat(cpu, 0))); });
  bind("__fixdfsi", [](Rom&, Cpu& cpu) { cpu.setReturn(static_cast<uint32_t>(static_cast<int32_t>(argDouble(cpu, 0)))); });
  bind("__fixunsdfsi", [](Rom&, Cpu& cpu) { cpu.setReturn(static_cast<uint32_t>(argDouble(cpu, 0))); });
  bind("__fixsfdi", [](Rom&, Cpu& cpu) { cpu.setReturn64(static_cast<uint64_t>(static_cast<int64_t>(argFloat(cpu, 0)))); });
  bind("__fixdfdi", [](Rom&, Cpu& cpu) { cpu.setReturn64(static_cast<uint64_t>(static_cast<int64_t>(argDouble(cpu, 0)))); });

  // Comparisons follow libgcc's contract: a sign, not a boolean.
  bind("__eqsf2", [](Rom&, Cpu& cpu) { cpu.setReturn(argFloat(cpu, 0) == argFloat(cpu, 1) ? 0 : 1); });
  bind("__nesf2", [](Rom&, Cpu& cpu) { cpu.setReturn(argFloat(cpu, 0) == argFloat(cpu, 1) ? 0 : 1); });
  bind("__ltsf2", [](Rom&, Cpu& cpu) {
    const float a = argFloat(cpu, 0), b = argFloat(cpu, 1);
    cpu.setReturn(static_cast<uint32_t>(a < b ? -1 : (a == b ? 0 : 1)));
  });
  bind("__lesf2", [](Rom&, Cpu& cpu) {
    const float a = argFloat(cpu, 0), b = argFloat(cpu, 1);
    cpu.setReturn(static_cast<uint32_t>(a < b ? -1 : (a == b ? 0 : 1)));
  });
  bind("__gtsf2", [](Rom&, Cpu& cpu) {
    const float a = argFloat(cpu, 0), b = argFloat(cpu, 1);
    cpu.setReturn(static_cast<uint32_t>(a > b ? 1 : (a == b ? 0 : -1)));
  });
  bind("__gesf2", [](Rom&, Cpu& cpu) {
    const float a = argFloat(cpu, 0), b = argFloat(cpu, 1);
    cpu.setReturn(static_cast<uint32_t>(a > b ? 1 : (a == b ? 0 : -1)));
  });
  bind("__unordsf2", [](Rom&, Cpu& cpu) {
    cpu.setReturn(std::isnan(argFloat(cpu, 0)) || std::isnan(argFloat(cpu, 1)) ? 1 : 0);
  });
  bind("__eqdf2", [](Rom&, Cpu& cpu) { cpu.setReturn(argDouble(cpu, 0) == argDouble(cpu, 2) ? 0 : 1); });
  bind("__nedf2", [](Rom&, Cpu& cpu) { cpu.setReturn(argDouble(cpu, 0) == argDouble(cpu, 2) ? 0 : 1); });
  bind("__ltdf2", [](Rom&, Cpu& cpu) {
    const double a = argDouble(cpu, 0), b = argDouble(cpu, 2);
    cpu.setReturn(static_cast<uint32_t>(a < b ? -1 : (a == b ? 0 : 1)));
  });
  bind("__ledf2", [](Rom&, Cpu& cpu) {
    const double a = argDouble(cpu, 0), b = argDouble(cpu, 2);
    cpu.setReturn(static_cast<uint32_t>(a < b ? -1 : (a == b ? 0 : 1)));
  });
  bind("__gtdf2", [](Rom&, Cpu& cpu) {
    const double a = argDouble(cpu, 0), b = argDouble(cpu, 2);
    cpu.setReturn(static_cast<uint32_t>(a > b ? 1 : (a == b ? 0 : -1)));
  });
  bind("__gedf2", [](Rom&, Cpu& cpu) {
    const double a = argDouble(cpu, 0), b = argDouble(cpu, 2);
    cpu.setReturn(static_cast<uint32_t>(a > b ? 1 : (a == b ? 0 : -1)));
  });
  bind("__unorddf2", [](Rom&, Cpu& cpu) {
    cpu.setReturn(std::isnan(argDouble(cpu, 0)) || std::isnan(argDouble(cpu, 2)) ? 1 : 0);
  });

  // ── Console ────────────────────────────────────────────────────────────────
  bind("ets_printf", [](Rom& rom, Cpu& cpu) {
    const std::string format = rom.readString(cpu.arg(0));
    const std::string text = formatGuestPrintf(rom, format, cpuArgSource(cpu, 1));
    // The real routine emits through whatever character sink was installed with
    // ets_install_putc1 — on an IDF app, one that writes to the console UART.
    // Going through it matters: emitting here *as well* would show every log
    // line twice, once from the ROM and once from the UART the sink wrote to.
    rom.emitThroughPutc(cpu, text);
    cpu.setReturn(static_cast<uint32_t>(text.size()));
  });
  bind("uart_tx_one_char", [](Rom& rom, Cpu& cpu) {
    const char c = static_cast<char>(cpu.arg(0));
    rom.emit(std::string(1, c));
    cpu.setReturn(0);
  });
  bind("uart_tx_one_char2", [](Rom& rom, Cpu& cpu) {
    rom.emit(std::string(1, static_cast<char>(cpu.arg(0))));
    cpu.setReturn(0);
  });
  bind("esp_rom_uart_tx_one_char", [](Rom& rom, Cpu& cpu) {
    rom.emit(std::string(1, static_cast<char>(cpu.arg(0))));
    cpu.setReturn(0);
  });
  bind("ets_install_putc1", [](Rom& rom, Cpu& cpu) {
    rom.setPutc1(cpu.arg(0));
    cpu.setReturn(0);
  });
  bind("ets_install_putc2", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("ets_install_uart_printf", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("ets_get_printf_channel", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  // Which UART is "the console" does not change where the output goes here —
  // the machine records every byte with the port it came out of — so these
  // only have to succeed.
  for (const char* name : {"esp_rom_uart_set_as_console", "uart_tx_switch",
                           "esp_rom_uart_usb_acm_init", "esp_rom_install_channel_putc"}) {
    bind(name, [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  }
  bind("esp_rom_uart_putc", [](Rom& rom, Cpu& cpu) {
    const char character = static_cast<char>(cpu.arg(0));
    rom.emitThroughPutc(cpu, std::string(1, character));
    cpu.setReturn(0);
  });
  bind("esp_rom_uart_rx_string", [](Rom&, Cpu& cpu) { cpu.setReturn(1); });

  bind("uart_tx_wait_idle", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("esp_rom_uart_tx_wait_idle", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("uart_tx_flush", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("esp_rom_uart_flush_tx", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("uart_rx_one_char", [](Rom&, Cpu& cpu) { cpu.setReturn(1); });  // nothing to read
  bind("esp_rom_uart_rx_one_char", [](Rom&, Cpu& cpu) { cpu.setReturn(1); });
  bind("uart_div_modify", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("uartAttach", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("ets_is_print_boot", [](Rom&, Cpu& cpu) { cpu.setReturn(1); });

  // ── The newlib printf family in ROM ────────────────────────────────────────
  bind("printf", [](Rom& rom, Cpu& cpu) {
    const std::string text = formatGuestPrintf(rom, rom.readString(cpu.arg(0)), cpuArgSource(cpu, 1));
    rom.emit(text);
    cpu.setReturn(static_cast<uint32_t>(text.size()));
  });
  bind("sprintf", [](Rom& rom, Cpu& cpu) {
    const std::string text = formatGuestPrintf(rom, rom.readString(cpu.arg(1)), cpuArgSource(cpu, 2));
    rom.writeBytes(cpu.arg(0), text.c_str(), text.size() + 1);
    cpu.setReturn(static_cast<uint32_t>(text.size()));
  });
  bind("snprintf", [](Rom& rom, Cpu& cpu) {
    const uint32_t limit = cpu.arg(1);
    const std::string text = formatGuestPrintf(rom, rom.readString(cpu.arg(2)), cpuArgSource(cpu, 3));
    if (limit) {
      const size_t copy = std::min<size_t>(text.size(), limit - 1);
      rom.writeBytes(cpu.arg(0), text.c_str(), copy);
      const uint8_t nul = 0;
      rom.writeBytes(cpu.arg(0) + static_cast<uint32_t>(copy), &nul, 1);
    }
    // snprintf returns what it *would* have written, not what it did.
    cpu.setReturn(static_cast<uint32_t>(text.size()));
  });
  bind("vsnprintf", [](Rom& rom, Cpu& cpu) {
    const uint32_t limit = cpu.arg(1);
    const std::string text =
        formatGuestPrintf(rom, rom.readString(cpu.arg(2)), vaListArgSource(rom, cpu.arg(3)));
    if (limit) {
      const size_t copy = std::min<size_t>(text.size(), limit - 1);
      rom.writeBytes(cpu.arg(0), text.c_str(), copy);
      const uint8_t nul = 0;
      rom.writeBytes(cpu.arg(0) + static_cast<uint32_t>(copy), &nul, 1);
    }
    cpu.setReturn(static_cast<uint32_t>(text.size()));
  });
  bind("vprintf", [](Rom& rom, Cpu& cpu) {
    const std::string text =
        formatGuestPrintf(rom, rom.readString(cpu.arg(0)), vaListArgSource(rom, cpu.arg(1)));
    rom.emit(text);
    cpu.setReturn(static_cast<uint32_t>(text.size()));
  });

  // ── Locks and process-level stubs ──────────────────────────────────────────
  // The ROM's newlib needs locks before FreeRTOS exists to provide them. There
  // is only ever one thread running inside the emulator at a time, so taking
  // one is a no-op — and once FreeRTOS is up, IDF replaces these anyway.
  for (const char* name : {"esp_rom_newlib_init_common_mutexes", "__retarget_lock_init",
                           "__retarget_lock_init_recursive", "__retarget_lock_close",
                           "__retarget_lock_close_recursive", "__retarget_lock_acquire",
                           "__retarget_lock_acquire_recursive", "__retarget_lock_release",
                           "__retarget_lock_release_recursive", "__retarget_lock_try_acquire",
                           "__retarget_lock_try_acquire_recursive", "_lock_init", "_lock_init_recursive",
                           "_lock_close", "_lock_close_recursive", "_lock_acquire",
                           "_lock_acquire_recursive", "_lock_release", "_lock_release_recursive",
                           "esp_rom_set_rtc_wake_addr", "esp_rom_install_channel_putc",
                           "esp_rom_install_uart_printf", "ets_install_lock"}) {
    bind(name, [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  }
  bind("abort", [](Rom& rom, Cpu& cpu) {
    rom.emit("\n[abort() called by firmware]\n");
    cpu.stop(HaltReason::Panic, "the firmware called abort()");
  });

  // ── Timing ─────────────────────────────────────────────────────────────────
  bind("ets_delay_us", [](Rom& rom, Cpu& cpu) {
    rom.delayUs(cpu.arg(0));
    cpu.setReturn(0);
  });
  bind("ets_get_cpu_frequency", [](Rom& rom, Cpu& cpu) { cpu.setReturn(rom.cpuFrequencyMhz()); });
  bind("ets_update_cpu_frequency", [](Rom& rom, Cpu& cpu) {
    rom.setCpuFrequencyMhz(cpu.arg(0));
    cpu.setReturn(0);
  });
  bind("ets_get_apb_freq", [](Rom& rom, Cpu& cpu) { cpu.setReturn(rom.soc().apbHz); });

  // ── Checksums ──────────────────────────────────────────────────────────────
  bind("crc32_le", [](Rom& rom, Cpu& cpu) {
    const std::vector<uint8_t> data = readBuffer(rom, cpu.arg(1), cpu.arg(2));
    cpu.setReturn(romCrc32Le(cpu.arg(0), data.data(), data.size()));
  });
  bind("crc16_le", [](Rom& rom, Cpu& cpu) {
    const std::vector<uint8_t> data = readBuffer(rom, cpu.arg(1), cpu.arg(2));
    cpu.setReturn(romCrc16Le(static_cast<uint16_t>(cpu.arg(0)), data.data(), data.size()));
  });
  bind("crc8_le", [](Rom& rom, Cpu& cpu) {
    const std::vector<uint8_t> data = readBuffer(rom, cpu.arg(1), cpu.arg(2));
    cpu.setReturn(romCrc8Le(static_cast<uint8_t>(cpu.arg(0)), data.data(), data.size()));
  });

  // ── Reset and power ────────────────────────────────────────────────────────
  // A fresh boot: POWERON_RESET, no wake cause. The machine overrides these
  // when it restarts the firmware for a deep-sleep wake.
  bind("rtc_get_reset_reason", [](Rom&, Cpu& cpu) { cpu.setReturn(1); });
  bind("esp_rom_get_reset_reason", [](Rom&, Cpu& cpu) { cpu.setReturn(1); });
  bind("rtc_get_wakeup_cause", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("software_reset", [](Rom& rom, Cpu& cpu) {
    rom.requestReset("software_reset");
    cpu.stop(HaltReason::Panic, "the firmware called software_reset()");
  });
  bind("software_reset_cpu", [](Rom& rom, Cpu& cpu) {
    rom.requestReset("software_reset_cpu");
    cpu.stop(HaltReason::Panic, "the firmware called software_reset_cpu()");
  });
  bind("analog_super_wdt_reset_happened", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("jtag_cpu_reset_happened", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("rtc_unhold_all_pads", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("rtc_select_apb_bridge", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("rtc_boot_control", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("set_rtc_memory_crc", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("cacl_rtc_memory_crc", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });

  // ── Cache ──────────────────────────────────────────────────────────────────
  // The emulator has no cache to keep coherent: the MMU translation in Bus is
  // consulted on every access, so invalidating, suspending or resuming one is
  // a no-op that must still return the state the caller will hand back later.
  for (const char* name : {"Cache_Invalidate_ICache_All", "Cache_Invalidate_DCache_All",
                           "Cache_Clean_All", "Cache_WriteBack_All", "Cache_Flush",
                           "Cache_Invalidate_Addr", "Cache_WriteBack_Addr", "Cache_Disable_ICache",
                           "Cache_Disable_DCache", "Cache_Enable_ICache", "Cache_Enable_DCache",
                           "Cache_MMU_Init", "Cache_Owner_Init", "Cache_Set_Default_Mode",
                           "Cache_Enable_Defalut_Mode", "esp_rom_Cache_Invalidate_Addr",
                           "Cache_Freeze_ICache_Enable", "Cache_Freeze_ICache_Disable",
                           "Cache_Freeze_DCache_Enable", "Cache_Freeze_DCache_Disable"}) {
    bind(name, [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  }
  bind("Cache_Suspend_ICache", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Suspend_DCache", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Resume_ICache", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Resume_DCache", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Set_IDROM_MMU_Size", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Get_IROM_MMU_End", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Get_DROM_MMU_End", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Set_IDROM_MMU_Info", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });

  // The S3 brings its cache up through the ROM rather than through registers,
  // and an app that skipped these would run with its flash unmapped. There is
  // no cache in this model — the MMU translates straight into the flash — so
  // everything that only configures, locks, freezes or invalidates a cache is
  // honestly a no-op here. The two that are not are the MMU setters: those
  // decide what the firmware's own pages point at.
  bind("rom_config_instruction_cache_mode", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("rom_config_data_cache_mode", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Enable_ICache", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Enable_DCache", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Disable_ICache", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Disable_DCache", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Invalidate_ICache_All", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Invalidate_DCache_All", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Invalidate_Addr", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_WriteBack_All", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_WriteBack_Addr", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Clean_All", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Occupy_ICache_MEMORY", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Occupy_Addr", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Owner_Init", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Set_Default_Mode", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_MMU_Init", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Freeze_ICache_Enable", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Freeze_ICache_Disable", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Freeze_DCache_Enable", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Freeze_DCache_Disable", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("Cache_Get_ICache_Line_Size", [](Rom&, Cpu& cpu) { cpu.setReturn(32); });
  bind("Cache_Get_DCache_Line_Size", [](Rom&, Cpu& cpu) { cpu.setReturn(32); });
  bind("Cache_Get_Mode", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });

  // Cache_{I,D}bus_MMU_Set(ext_ram, vaddr, paddr, psize_kb, pages, fixed):
  // what esp_mmu and spi_flash_mmap are built on. Getting this wrong does not
  // fail — it silently points the firmware's own code or data at the wrong
  // 64 KB of flash — so it is modelled rather than stubbed.
  //
  // Returns MMU_SET_ADDR_ALIGNED_ERROR (0x200) the way the ROM does when it is
  // asked for something it cannot map.
  auto mmuSet = [](Rom& rom, Cpu& cpu) {
    const uint32_t external = cpu.arg(0);
    const uint32_t vaddr = cpu.arg(1);
    const uint32_t paddr = cpu.arg(2);
    const uint32_t pageKb = cpu.arg(3) ? cpu.arg(3) : 64;
    const uint32_t pages = cpu.arg(4);
    const bool fixed = cpu.arg(5) != 0;
    if (external) {
      // PSRAM is not modelled, and pretending a mapping succeeded would hand
      // the firmware a window onto nothing.
      cpu.setReturn(0x200);
      return;
    }
    const uint32_t pageBytes = pageKb * 1024;
    for (uint32_t page = 0; page < pages; ++page) {
      const uint32_t source = fixed ? paddr : paddr + page * pageBytes;
      if (!rom.bus().mapFlash(vaddr + page * pageBytes, source, pageBytes, nullptr)) {
        cpu.setReturn(0x200);
        return;
      }
    }
    cpu.setReturn(0);
  };
  bind("Cache_Ibus_MMU_Set", mmuSet);
  bind("Cache_Dbus_MMU_Set", mmuSet);

  // ── Pads and signal routing ────────────────────────────────────────────────
  bind("gpio_pad_select_gpio", [](Rom& rom, Cpu& cpu) {
    if (rom.hooks().padSelect) rom.hooks().padSelect(static_cast<int>(cpu.arg(0)));
    cpu.setReturn(0);
  });
  bind("gpio_pad_pullup", [](Rom& rom, Cpu& cpu) {
    if (rom.hooks().padPull) rom.hooks().padPull(static_cast<int>(cpu.arg(0)), 1);
    cpu.setReturn(0);
  });
  bind("gpio_pad_pulldown", [](Rom& rom, Cpu& cpu) {
    if (rom.hooks().padPull) rom.hooks().padPull(static_cast<int>(cpu.arg(0)), 2);
    cpu.setReturn(0);
  });
  bind("gpio_pad_unhold", [](Rom& rom, Cpu& cpu) {
    if (rom.hooks().padHold) rom.hooks().padHold(static_cast<int>(cpu.arg(0)), false);
    cpu.setReturn(0);
  });
  bind("gpio_pad_hold", [](Rom& rom, Cpu& cpu) {
    if (rom.hooks().padHold) rom.hooks().padHold(static_cast<int>(cpu.arg(0)), true);
    cpu.setReturn(0);
  });
  bind("gpio_matrix_out", [](Rom& rom, Cpu& cpu) {
    if (rom.hooks().matrixOut) {
      rom.hooks().matrixOut(static_cast<int>(cpu.arg(0)), cpu.arg(1), cpu.arg(2) != 0, cpu.arg(3) != 0);
    }
    cpu.setReturn(0);
  });
  bind("gpio_matrix_in", [](Rom& rom, Cpu& cpu) {
    if (rom.hooks().matrixIn) {
      rom.hooks().matrixIn(static_cast<int>(cpu.arg(1)), cpu.arg(0), cpu.arg(2) != 0);
    }
    cpu.setReturn(0);
  });
  bind("gpio_output_disable", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("gpio_bypass_matrix_in", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });

  // ── Analog register file (PLL, regulators, RF) ─────────────────────────────
  bind("rom_i2c_readReg", [](Rom& rom, Cpu& cpu) {
    cpu.setReturn(rom.analogRead(static_cast<uint8_t>(cpu.arg(0)), static_cast<uint8_t>(cpu.arg(2))));
  });
  bind("rom_i2c_writeReg", [](Rom& rom, Cpu& cpu) {
    rom.analogWrite(static_cast<uint8_t>(cpu.arg(0)), static_cast<uint8_t>(cpu.arg(2)),
                    static_cast<uint8_t>(cpu.arg(3)));
    cpu.setReturn(0);
  });
  bind("rom_i2c_readReg_Mask", [](Rom& rom, Cpu& cpu) {
    const uint8_t value = rom.analogRead(static_cast<uint8_t>(cpu.arg(0)), static_cast<uint8_t>(cpu.arg(2)));
    const uint32_t msb = cpu.arg(3), lsb = cpu.arg(4);
    const uint32_t mask = msb >= lsb && msb < 8 ? ((1u << (msb - lsb + 1)) - 1) : 0xFF;
    cpu.setReturn((value >> lsb) & mask);
  });
  bind("rom_i2c_writeReg_Mask", [](Rom& rom, Cpu& cpu) {
    const uint8_t block = static_cast<uint8_t>(cpu.arg(0));
    const uint8_t reg = static_cast<uint8_t>(cpu.arg(2));
    const uint32_t msb = cpu.arg(3), lsb = cpu.arg(4);
    const uint32_t mask = msb >= lsb && msb < 8 ? (((1u << (msb - lsb + 1)) - 1) << lsb) : 0xFF;
    const uint8_t old = rom.analogRead(block, reg);
    rom.analogWrite(block, reg, static_cast<uint8_t>((old & ~mask) | ((cpu.arg(5) << lsb) & mask)));
    cpu.setReturn(0);
  });

  // ── Interrupt matrix ───────────────────────────────────────────────────────
  bind("esp_rom_route_intr_matrix", [](Rom& rom, Cpu& cpu) {
    if (rom.hooks().routeInterrupt) {
      rom.hooks().routeInterrupt(static_cast<int>(cpu.arg(0)), static_cast<int>(cpu.arg(1)),
                                 static_cast<int>(cpu.arg(2)));
    }
    cpu.setReturn(0);
  });
  bind("intr_matrix_set", [](Rom& rom, Cpu& cpu) {
    if (rom.hooks().routeInterrupt) {
      rom.hooks().routeInterrupt(static_cast<int>(cpu.arg(0)), static_cast<int>(cpu.arg(1)),
                                 static_cast<int>(cpu.arg(2)));
    }
    cpu.setReturn(0);
  });
  bind("ets_set_appcpu_boot_addr", [](Rom& rom, Cpu& cpu) {
    if (rom.hooks().appCpuBoot) rom.hooks().appCpuBoot(cpu.arg(0));
    cpu.setReturn(0);
  });

  // ── Callbacks into firmware ────────────────────────────────────────────────
  // qsort's comparator is application code, so the sort runs here but the
  // comparisons run in the guest. A merge sort over an index permutation keeps
  // the number of callbacks down and, unlike handing a foreign comparator to
  // std::sort, stays in bounds even if that comparator is inconsistent.
  bind("qsort", [](Rom& rom, Cpu& cpu) {
    const uint32_t base = cpu.arg(0);
    const uint32_t count = cpu.arg(1);
    const uint32_t size = cpu.arg(2);
    const uint32_t comparator = cpu.arg(3);
    if (count < 2 || size == 0 || size > 4096 || !comparator) return;

    std::vector<uint32_t> order(count);
    for (uint32_t i = 0; i < count; ++i) order[i] = i;

    bool failed = false;
    auto before = [&](uint32_t left, uint32_t right) {
      if (failed) return false;
      const uint32_t args[2] = {base + left * size, base + right * size};
      uint32_t verdict = 0;
      if (!cpu.callGuest(comparator, args, 2, &verdict)) {
        failed = true;
        return false;
      }
      return static_cast<int32_t>(verdict) < 0;
    };

    std::vector<uint32_t> scratch(count);
    for (uint32_t width = 1; width < count && !failed; width *= 2) {
      for (uint32_t start = 0; start < count; start += 2 * width) {
        const uint32_t middle = std::min(start + width, count);
        const uint32_t end = std::min(start + 2 * width, count);
        uint32_t left = start, right = middle, out = start;
        while (left < middle && right < end) {
          scratch[out++] = before(order[right], order[left]) ? order[right++] : order[left++];
        }
        while (left < middle) scratch[out++] = order[left++];
        while (right < end) scratch[out++] = order[right++];
      }
      order.swap(scratch);
    }
    if (failed) return;

    // Apply the permutation as whole-element copies, through a host-side copy
    // of the array so overlapping moves cannot corrupt it.
    std::vector<uint8_t> original(static_cast<size_t>(count) * size);
    rom.readBytes(base, original.data(), original.size());
    std::vector<uint8_t> sorted(original.size());
    for (uint32_t i = 0; i < count; ++i) {
      std::memcpy(sorted.data() + static_cast<size_t>(i) * size,
                  original.data() + static_cast<size_t>(order[i]) * size, size);
    }
    rom.writeBytes(base, sorted.data(), sorted.size());
  });

  bind("bsearch", [](Rom& rom, Cpu& cpu) {
    const uint32_t key = cpu.arg(0);
    const uint32_t base = cpu.arg(1);
    const uint32_t count = cpu.arg(2);
    const uint32_t size = cpu.arg(3);
    const uint32_t comparator = cpu.arg(4);
    (void)rom;
    if (!count || !size || !comparator) {
      cpu.setReturn(0);
      return;
    }
    uint32_t low = 0, high = count;
    while (low < high) {
      const uint32_t middle = low + (high - low) / 2;
      const uint32_t args[2] = {key, base + middle * size};
      uint32_t verdict = 0;
      if (!cpu.callGuest(comparator, args, 2, &verdict)) {
        cpu.setReturn(0);
        return;
      }
      const int32_t signedVerdict = static_cast<int32_t>(verdict);
      if (signedVerdict == 0) {
        cpu.setReturn(base + middle * size);
        return;
      }
      if (signedVerdict < 0) {
        high = middle;
      } else {
        low = middle + 1;
      }
    }
    cpu.setReturn(0);
  });

  // ── The C3's RISC-V interrupt controller ───────────────────────────────────
  // These are thin wrappers over the interrupt matrix registers in the real
  // ROM too, so implementing them as register writes keeps one source of truth:
  // firmware that pokes the matrix directly and firmware that calls these end
  // up in the same place.
  bind("esprv_intc_int_enable", [](Rom& rom, Cpu& cpu) {
    const uint32_t reg = rom.soc().interrupt + 0x104;
    rom.bus().write32(reg, rom.bus().read32(reg) | cpu.arg(0));
    cpu.setReturn(0);
  });
  bind("esprv_intc_int_disable", [](Rom& rom, Cpu& cpu) {
    const uint32_t reg = rom.soc().interrupt + 0x104;
    rom.bus().write32(reg, rom.bus().read32(reg) & ~cpu.arg(0));
    cpu.setReturn(0);
  });
  bind("esprv_intc_int_set_priority", [](Rom& rom, Cpu& cpu) {
    const int line = static_cast<int>(cpu.arg(0));
    if (line >= 0 && line < 32) rom.bus().write32(rom.soc().interrupt + 0x114 + line * 4, cpu.arg(1));
    cpu.setReturn(0);
  });
  bind("esprv_intc_int_set_threshold", [](Rom& rom, Cpu& cpu) {
    rom.bus().write32(rom.soc().interrupt + 0x194, cpu.arg(0));
    cpu.setReturn(0);
  });
  bind("esprv_intc_int_set_type", [](Rom& rom, Cpu& cpu) {
    const uint32_t reg = rom.soc().interrupt + 0x108;
    const uint32_t mask = 1u << (cpu.arg(0) & 31);
    const uint32_t current = rom.bus().read32(reg);
    rom.bus().write32(reg, cpu.arg(1) ? (current | mask) : (current & ~mask));
    cpu.setReturn(0);
  });
  // The APB backup DMA lock, and the other ROM-side locking hooks IDF installs
  // during startup. There is no second bus master here to arbitrate with.
  for (const char* name : {"ets_apb_backup_init_lock_func", "ets_backup_dma_lock",
                           "ets_backup_dma_unlock", "ets_apb_backup_fill_dslp_mem"}) {
    bind(name, [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  }
  // ── Watchdogs ──────────────────────────────────────────────────────────────
  // The S3 keeps IDF's watchdog HAL in ROM, so an app reaches these rather
  // than code of its own. No watchdog is modelled: simulated time only moves
  // when the firmware lets it, so a "hang" here is a firmware that stopped
  // asking for time rather than one a watchdog should shoot. The honest
  // consequence is that firmware relying on a watchdog reset to recover will
  // wait here instead — which `freeink-sim emu` reports as waiting.
  //
  // The context is still filled in: it is the caller's struct, and handing it
  // back uninitialised would have the firmware pass stack garbage to the next
  // call in the sequence.
  bind("wdt_hal_init", [](Rom& rom, Cpu& cpu) {
    const uint32_t context = cpu.arg(0);
    const uint32_t instance = cpu.arg(1);
    // wdt_hal_context_t: { wdt_inst_t inst; <timer group or rtc> *dev; }
    static constexpr uint32_t kTimerGroupStride = 0x1000;
    const uint32_t device = instance == 2 ? rom.soc().rtcCntl
                                          : rom.soc().timerGroup0 + instance * kTimerGroupStride;
    rom.bus().write32(context, instance);
    rom.bus().write32(context + 4, device);
    cpu.setReturn(0);
  });
  for (const char* name : {"wdt_hal_deinit", "wdt_hal_enable", "wdt_hal_disable", "wdt_hal_feed",
                           "wdt_hal_config_stage", "wdt_hal_handle_intr",
                           "wdt_hal_write_protect_disable", "wdt_hal_write_protect_enable",
                           "wdt_hal_set_flashboot_en", "wdt_hal_is_enabled"}) {
    bind(name, [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  }

  // ── The ROM's Xtensa OS layer ──────────────────────────────────────────────
  // An S3 app reaches these during startup and whenever it needs a critical
  // section before its own port is up. They are pure CPU state, so the core
  // does the work and the ROM table just names it.
  bind("_xtos_set_intlevel", [](Rom&, Cpu& cpu) {
    cpu.setReturn(cpu.maskInterruptsAbove(cpu.arg(0)));
  });
  bind("_xtos_set_min_intlevel", [](Rom&, Cpu& cpu) {
    cpu.setReturn(cpu.maskInterruptsAbove(cpu.arg(0)));
  });
  bind("_xtos_restore_intlevel", [](Rom&, Cpu& cpu) {
    cpu.restoreInterruptMask(cpu.arg(0));
    cpu.setReturn(0);
  });
  bind("_xtos_ints_on", [](Rom&, Cpu& cpu) {
    cpu.setInterruptEnableMask(cpu.interruptEnableMask() | cpu.arg(0));
    cpu.setReturn(0);
  });
  bind("_xtos_ints_off", [](Rom&, Cpu& cpu) {
    cpu.setInterruptEnableMask(cpu.interruptEnableMask() & ~cpu.arg(0));
    cpu.setReturn(0);
  });

  bind("ets_intr_lock", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("ets_intr_unlock", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("ets_isr_mask", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("ets_isr_unmask", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  bind("ets_isr_attach", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });

  // ── Radio: coexistence and PHY ─────────────────────────────────────────────
  // The emulator models no radio at all — see the limits in docs/simulator.md.
  // What it does is let the *initialisation* path through, so firmware that
  // brings Wi-Fi or Bluetooth up during startup reaches the rest of its code
  // instead of stopping at the first ROM call. Anything that actually
  // transmits will find a radio that never sees a packet, which is a modelled
  // absence rather than a pretence.
  // Returns a version *string*, which the caller logs and measures.
  bind("esp_coex_rom_version_get", [](Rom& rom, Cpu& cpu) {
    cpu.setReturn(rom.internString("emulated-coex"));
  });
  for (const char* name : {"coex_bt_release", "coex_bt_request", "coex_core_release",
                           "coex_core_request", "coex_core_status_get", "coex_core_pti_get",
                           "coex_core_ble_conn_dyn_prio_get", "coex_event_duration_get",
                           "coex_hw_timer_disable", "coex_hw_timer_enable", "coex_hw_timer_set",
                           "coex_schm_interval_set", "coex_schm_lock", "coex_schm_unlock",
                           "coex_wifi_release", "esp_coex_ble_conn_dynamic_prio_get",
                           "rom_disable_agc", "rom_enable_agc", "rom_phy_disable_cca",
                           "rom_phy_enable_cca", "rom_phy_set_bbfreq_init", "rom_disable_wifi_agc",
                           "rom_enable_wifi_agc", "rom_phy_xpd_rf", "rom_phy_track_pll_cap",
                           "rom_phy_pwdet_always_en", "rom_phy_pwdet_onetime_en",
                           "rom_phy_en_hw_set_freq", "rom_phy_dis_hw_set_freq",
                           "rom_i2c_master_reset", "phy_get_romfuncs"}) {
    bind(name, [](Rom&, Cpu& cpu) { cpu.setReturn(0); });
  }
  bind("rom_phy_get_noisefloor", [](Rom&, Cpu& cpu) {
    cpu.setReturn(static_cast<uint32_t>(-90));  // a quiet band
  });
  bind("rom_phy_get_rx_freq", [](Rom&, Cpu& cpu) { cpu.setReturn(2412); });
  bind("rom_phy_byte_to_word", [](Rom&, Cpu& cpu) { cpu.setReturn(0); });

  registerFlashHandlers();
  registerMd5Handlers();
}

}  // namespace freeink::sim::emu
