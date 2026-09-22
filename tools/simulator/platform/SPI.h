#pragma once

// FreeInk simulator — Arduino SPI shim.
//
// This is the seam that makes the panel simulation faithful: the real EpdBus
// and the real panel drivers run unmodified, and their SPI traffic is decoded
// by a virtual controller in the daemon. Command framing (the DC pin level at
// the moment of each byte) is read from the GPIO matrix, exactly as the glass
// would. Nothing here interprets panel commands.

#include <Arduino.h>

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3

#define HSPI 0
#define VSPI 0
#define FSPI 0

class SPISettings {
 public:
  SPISettings() = default;
  SPISettings(uint32_t hz, uint8_t order, uint8_t mode) : _hz(hz), _order(order), _mode(mode) {}
  uint32_t hz() const { return _hz; }
  uint8_t order() const { return _order; }
  uint8_t mode() const { return _mode; }

 private:
  uint32_t _hz = 1000000;
  uint8_t _order = MSBFIRST;
  uint8_t _mode = SPI_MODE0;
};

class SimSPI {
 public:
  explicit SimSPI(int bus = 0) : _bus(bus) {}

  void begin(int8_t sclk = -1, int8_t miso = -1, int8_t mosi = -1, int8_t cs = -1) {
    fsim_spi_begin(_bus, sclk, miso, mosi, cs);
  }
  void end() { fsim_spi_end(_bus); }

  void beginTransaction(const SPISettings& s) { fsim_spi_settings(_bus, s.hz(), s.order(), s.mode()); }
  void endTransaction() {}

  void setFrequency(uint32_t hz) { fsim_spi_settings(_bus, hz, MSBFIRST, SPI_MODE0); }
  void setDataMode(uint8_t) {}
  void setBitOrder(uint8_t) {}

  uint8_t transfer(uint8_t data) {
    uint8_t rx = 0xFF;
    fsim_spi_transfer(_bus, &data, &rx, 1);
    return rx;
  }
  uint16_t transfer16(uint16_t data) {
    const uint8_t tx[2] = {static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)};
    uint8_t rx[2] = {0xFF, 0xFF};
    fsim_spi_transfer(_bus, tx, rx, 2);
    return static_cast<uint16_t>((rx[0] << 8) | rx[1]);
  }
  // In-place full-duplex transfer, the Arduino-ESP32 signature.
  void transfer(void* data, uint32_t size) {
    uint8_t* p = static_cast<uint8_t*>(data);
    fsim_spi_transfer(_bus, p, p, size);
  }
  void writeBytes(const uint8_t* data, uint32_t size) { fsim_spi_transfer(_bus, data, nullptr, size); }
  void writePixels(const void* data, uint32_t size) {
    fsim_spi_transfer(_bus, static_cast<const uint8_t*>(data), nullptr, size);
  }

 private:
  int _bus;
};

extern SimSPI SPI;
