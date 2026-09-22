#pragma once

// FreeInk simulator — Arduino Preferences over the daemon's NVS store.
//
// The store is persisted next to the daemon's state, so settings survive a
// simulated restart exactly as they survive a reboot on device, and
// `freeink-sim nvs` can read and write them — which is how a test puts the
// firmware into a known settings state without driving the UI.

#include <Arduino.h>
#include <freeink_sim_abi.h>

class Preferences {
 public:
  bool begin(const char* name, bool readOnly = false, const char* = nullptr) {
    _ns = name ? name : "";
    _ro = readOnly;
    return true;
  }
  void end() { _ns.clear(); }

  bool clear() { return !_ro && fsim_nvs_clear(_ns.c_str()) == 0; }
  bool remove(const char* key) { return !_ro && fsim_nvs_erase(_ns.c_str(), key) == 0; }
  bool isKey(const char* key) const {
    size_t len = 0;
    return fsim_nvs_get(_ns.c_str(), key, nullptr, 0, &len) == 0;
  }

  size_t putBytes(const char* key, const void* value, size_t len) { return put(key, value, len) ? len : 0; }
  size_t getBytes(const char* key, void* buf, size_t maxLen) {
    size_t len = 0;
    if (fsim_nvs_get(_ns.c_str(), key, buf, maxLen, &len) != 0) return 0;
    return len;
  }
  size_t getBytesLength(const char* key) {
    size_t len = 0;
    return fsim_nvs_get(_ns.c_str(), key, nullptr, 0, &len) == 0 ? len : 0;
  }

  size_t putChar(const char* k, int8_t v) { return putScalar(k, v); }
  size_t putUChar(const char* k, uint8_t v) { return putScalar(k, v); }
  size_t putShort(const char* k, int16_t v) { return putScalar(k, v); }
  size_t putUShort(const char* k, uint16_t v) { return putScalar(k, v); }
  size_t putInt(const char* k, int32_t v) { return putScalar(k, v); }
  size_t putUInt(const char* k, uint32_t v) { return putScalar(k, v); }
  size_t putLong(const char* k, int32_t v) { return putScalar(k, v); }
  size_t putULong(const char* k, uint32_t v) { return putScalar(k, v); }
  size_t putLong64(const char* k, int64_t v) { return putScalar(k, v); }
  size_t putULong64(const char* k, uint64_t v) { return putScalar(k, v); }
  size_t putFloat(const char* k, float v) { return putScalar(k, v); }
  size_t putDouble(const char* k, double v) { return putScalar(k, v); }
  size_t putBool(const char* k, bool v) { return putScalar(k, static_cast<uint8_t>(v ? 1 : 0)); }
  size_t putString(const char* k, const char* v) { return put(k, v, v ? strlen(v) + 1 : 0) ? strlen(v) : 0; }
  size_t putString(const char* k, const String& v) { return putString(k, v.c_str()); }

  int8_t getChar(const char* k, int8_t d = 0) { return getScalar(k, d); }
  uint8_t getUChar(const char* k, uint8_t d = 0) { return getScalar(k, d); }
  int16_t getShort(const char* k, int16_t d = 0) { return getScalar(k, d); }
  uint16_t getUShort(const char* k, uint16_t d = 0) { return getScalar(k, d); }
  int32_t getInt(const char* k, int32_t d = 0) { return getScalar(k, d); }
  uint32_t getUInt(const char* k, uint32_t d = 0) { return getScalar(k, d); }
  int32_t getLong(const char* k, int32_t d = 0) { return getScalar(k, d); }
  uint32_t getULong(const char* k, uint32_t d = 0) { return getScalar(k, d); }
  int64_t getLong64(const char* k, int64_t d = 0) { return getScalar(k, d); }
  uint64_t getULong64(const char* k, uint64_t d = 0) { return getScalar(k, d); }
  float getFloat(const char* k, float d = 0.0f) { return getScalar(k, d); }
  double getDouble(const char* k, double d = 0.0) { return getScalar(k, d); }
  bool getBool(const char* k, bool d = false) { return getScalar<uint8_t>(k, d ? 1 : 0) != 0; }
  String getString(const char* k, const String& d = String()) {
    size_t len = 0;
    if (fsim_nvs_get(_ns.c_str(), k, nullptr, 0, &len) != 0 || len == 0) return d;
    std::string buf(len, '\0');
    if (fsim_nvs_get(_ns.c_str(), k, &buf[0], len, &len) != 0) return d;
    if (!buf.empty() && buf.back() == '\0') buf.pop_back();
    return String(buf);
  }
  size_t getString(const char* k, char* out, size_t cap) {
    size_t len = 0;
    if (fsim_nvs_get(_ns.c_str(), k, out, cap, &len) != 0) return 0;
    return len;
  }
  size_t freeEntries() { return 1000; }

 private:
  bool put(const char* key, const void* data, size_t len) {
    return !_ro && fsim_nvs_set(_ns.c_str(), key, data, len) == 0;
  }
  template <typename T>
  size_t putScalar(const char* key, T v) {
    return put(key, &v, sizeof(T)) ? sizeof(T) : 0;
  }
  template <typename T>
  T getScalar(const char* key, T fallback) {
    T v{};
    size_t len = 0;
    if (fsim_nvs_get(_ns.c_str(), key, &v, sizeof(T), &len) != 0 || len != sizeof(T)) return fallback;
    return v;
  }

  std::string _ns;
  bool _ro = false;
};
