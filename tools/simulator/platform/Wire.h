#pragma once

// FreeInk simulator — Arduino Wire (I2C) shim.
//
// Buffers a beginTransmission()/write()/endTransmission() sequence and hands it
// to the daemon's I2C model as one transaction, then serves requestFrom() out
// of the model's reply. That keeps the real driver code — GT911, FT6336,
// BM8563, CW2017, QMI8658, LM3630A — running against virtual devices, so
// probe/NACK behaviour matches hardware: an address nothing answers returns a
// nonzero endTransmission() and firmware takes its device-absent path.
//
// Like the ESP32 core's TwoWire, a transaction owns the bus from
// beginTransmission() until endTransmission(true) or the end of requestFrom(),
// so a repeated-start register read in one task can't interleave with
// another task's. Without that, two tasks sharing Wire (touch polling in the
// loop, the battery gauge read while rendering) race on the buffers below.

#include <Arduino.h>

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>

class SimWire {
 public:
  explicit SimWire(int bus = 0) : _bus(bus) {
    // Fixed capacity, as the core's buffers are: assign()/clear() never
    // reallocate under a reader still draining a previous reply.
    _tx.reserve(kBufferSize);
    _rx.reserve(kBufferSize);
  }

  bool begin(int sda = -1, int scl = -1, uint32_t hz = 100000) {
    _sda = sda;
    _scl = scl;
    _hz = hz;
    fsim_i2c_begin(_bus, sda, scl, hz);
    return true;
  }
  bool end() { return true; }
  void setClock(uint32_t hz) {
    _hz = hz;
    fsim_i2c_begin(_bus, _sda, _scl, hz);
  }
  void setTimeOut(uint16_t) {}
  void setTimeout(uint32_t) {}

  void beginTransmission(uint8_t addr) {
    acquire();
    _addr = addr;
    _tx.clear();
  }
  size_t write(uint8_t b) {
    _tx.push_back(b);
    return 1;
  }
  size_t write(const uint8_t* data, size_t len) {
    _tx.insert(_tx.end(), data, data + len);
    return len;
  }

  // 0 = success (ACK), 2 = address NACK — the codes firmware branches on.
  uint8_t endTransmission(bool sendStop = true) {
    const int rc = fsim_i2c_xfer(_bus, _addr, _tx.data(), _tx.size(), nullptr, 0);
    _tx.clear();
    // Hardware keeps the bus for the repeated-start read that follows a
    // non-stop write. This model sends the write now and can NACK it, after
    // which drivers skip requestFrom(), so release on failure too.
    if (sendStop || rc != 0) release();
    return rc == 0 ? 0 : 2;
  }

  uint8_t requestFrom(uint8_t addr, size_t len, bool = true) {
    acquire();
    struct Release {
      SimWire& wire;
      ~Release() { wire.release(); }
    } releaseOnReturn{*this};
    _addr = addr;
    _rx.assign(len, 0);
    _rxPos = 0;
    // Any bytes still pending from a write become the register-pointer phase of
    // a repeated-start read, which is how every driver here addresses a
    // register before reading it.
    const int rc = fsim_i2c_xfer(_bus, addr, _tx.data(), _tx.size(), _rx.data(), len);
    _tx.clear();
    if (rc != 0) {
      _rx.clear();
      return 0;
    }
    return static_cast<uint8_t>(_rx.size());
  }
  uint8_t requestFrom(int addr, int len) { return requestFrom(static_cast<uint8_t>(addr), static_cast<size_t>(len)); }

  int available() { return static_cast<int>(_rx.size() - _rxPos); }
  int read() { return _rxPos < _rx.size() ? _rx[_rxPos++] : -1; }
  int peek() { return _rxPos < _rx.size() ? _rx[_rxPos] : -1; }
  void flush() {}

 private:
  static constexpr size_t kBufferSize = 128;  // the ESP32 core's default I2C buffer

  // Re-entrant for the owning task, as the core's currentTaskHandle check is.
  void acquire() {
    const auto self = std::this_thread::get_id();
    if (_holder.load() == self) return;
    _lock.lock();
    _holder.store(self);
  }
  void release() {
    if (_holder.load() != std::this_thread::get_id()) return;
    _holder.store(std::thread::id());
    _lock.unlock();
  }

  std::mutex _lock;
  std::atomic<std::thread::id> _holder{};
  int _bus;
  int _sda = -1;
  int _scl = -1;
  uint32_t _hz = 100000;
  uint8_t _addr = 0;
  std::vector<uint8_t> _tx;
  std::vector<uint8_t> _rx;
  size_t _rxPos = 0;
};

extern SimWire Wire;
extern SimWire Wire1;
