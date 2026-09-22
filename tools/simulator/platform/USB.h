#pragma once

// FreeInk simulator — USB device shim. The daemon models cable presence, which
// the CLI toggles (`freeink-sim usb plug/unplug`), so USB-gated firmware paths
// are reachable without a cable.

#include <Arduino.h>
#include <freeink_sim_abi.h>

class SimUSB {
 public:
  bool begin() { return true; }
  void end() {}
  void productName(const char*) {}
  void manufacturerName(const char*) {}
  void serialNumber(const char*) {}
  void VID(uint16_t) {}
  void PID(uint16_t) {}
  operator bool() const { return fsim_usb_connected() != 0; }
};

extern SimUSB USB;
