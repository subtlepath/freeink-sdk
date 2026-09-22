#pragma once

// FreeInk simulator — WiFiClient over host TCP, gated by the daemon's network
// policy (see WiFi.h). Refused connections surface as connect() == 0, the same
// as an unreachable host on device.

#include <Arduino.h>
#include <freeink_sim_abi.h>

#include "Client.h"

class WiFiClient : public Client {
 public:
  WiFiClient() = default;
  ~WiFiClient() override { stop(); }

  int connect(const char* host, uint16_t port) override {
    stop();
    _sock = fsim_net_connect(host, port);
    return _sock >= 0 ? 1 : 0;
  }
  int connect(uint32_t ip, uint16_t port) override {
    char host[16];
    snprintf(host, sizeof(host), "%u.%u.%u.%u", ip & 0xFF, (ip >> 8) & 0xFF, (ip >> 16) & 0xFF, (ip >> 24) & 0xFF);
    return connect(host, port);
  }
  size_t write(uint8_t b) override { return write(&b, 1); }
  size_t write(const uint8_t* buf, size_t size) override {
    if (_sock < 0) return 0;
    const int n = fsim_net_write(_sock, buf, size);
    return n > 0 ? static_cast<size_t>(n) : 0;
  }
  int available() override { return _peeked >= 0 ? 1 : 0; }
  int read() override {
    if (_peeked >= 0) {
      const int v = _peeked;
      _peeked = -1;
      return v;
    }
    uint8_t b = 0;
    return read(&b, 1) == 1 ? b : -1;
  }
  int read(uint8_t* buf, size_t size) override {
    if (_sock < 0 || size == 0) return -1;
    size_t off = 0;
    if (_peeked >= 0) {
      buf[0] = static_cast<uint8_t>(_peeked);
      _peeked = -1;
      off = 1;
      if (size == 1) return 1;
    }
    const int n = fsim_net_read(_sock, buf + off, size - off);
    if (n <= 0) return off > 0 ? static_cast<int>(off) : n;
    return static_cast<int>(off) + n;
  }
  int peek() override {
    if (_peeked < 0) {
      uint8_t b = 0;
      if (read(&b, 1) == 1) _peeked = b;
    }
    return _peeked;
  }
  void flush() override {}
  void stop() override {
    if (_sock >= 0) fsim_net_close(_sock);
    _sock = -1;
    _peeked = -1;
  }
  uint8_t connected() override { return _sock >= 0 ? 1 : 0; }
  operator bool() override { return _sock >= 0; }
  void setTimeout(uint32_t) {}
  void setNoDelay(bool) {}

 private:
  int _sock = -1;
  int _peeked = -1;
};
