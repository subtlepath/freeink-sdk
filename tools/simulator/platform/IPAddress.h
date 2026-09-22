#pragma once

// FreeInk simulator — IPAddress.

#include <Arduino.h>

class IPAddress {
 public:
  IPAddress() = default;
  explicit IPAddress(uint32_t raw) : _raw(raw) {}
  IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
      : _raw(static_cast<uint32_t>(a) | (static_cast<uint32_t>(b) << 8) | (static_cast<uint32_t>(c) << 16) |
             (static_cast<uint32_t>(d) << 24)) {}
  operator uint32_t() const { return _raw; }
  uint8_t operator[](int i) const { return static_cast<uint8_t>((_raw >> (8 * i)) & 0xFF); }
  String toString() const {
    char buf[16];
    snprintf(buf, sizeof(buf), "%u.%u.%u.%u", (*this)[0], (*this)[1], (*this)[2], (*this)[3]);
    return String(buf);
  }

 private:
  uint32_t _raw = 0;
};
