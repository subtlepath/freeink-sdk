#pragma once
#include "../../emu/Md5.h"
#include <Arduino.h>
#include <vector>
class MD5Builder {
  std::vector<uint8_t> data_;
  uint8_t digest_[16]{};
 public:
  void begin() { data_.clear(); }
  void add(const uint8_t* data, size_t length) { data_.insert(data_.end(), data, data + length); }
  void add(uint8_t* data, size_t length) { add(static_cast<const uint8_t*>(data), length); }
  void add(const char* data) { add(reinterpret_cast<const uint8_t*>(data), strlen(data)); }
  void calculate() { freeink::sim::emu::md5Digest(data_.data(), data_.size(), digest_); }
  String toString() { char hex[33]; for (int i = 0; i < 16; i++) snprintf(hex + 2*i, 3, "%02x", digest_[i]); return String(hex); }
};
