#pragma once
#include <esp_system.h>
#include <cassert>
#include <Arduino.h>
#include <freertos/task.h>
inline bool setCpuFrequencyMhz(uint32_t) { return true; }
inline uint32_t getCpuFrequencyMhz() { return 160; }
inline void configTzTime(const char*, const char*, const char* = nullptr, const char* = nullptr) {}
#define GPIO_NUM_13 13
struct WebESP {
  void restart() { esp_restart(); }
  uint32_t getFreeHeap() { return 32 * 1024 * 1024; }
  uint32_t getHeapSize() { return 64 * 1024 * 1024; }
  uint32_t getMinFreeHeap() { return getFreeHeap(); }
  uint32_t getMaxAllocHeap() { return getFreeHeap(); }
  uint32_t getFreePsram() { return 8 * 1024 * 1024; }
  uint32_t getPsramSize() { return 8 * 1024 * 1024; }
  const char* getChipModel() { return "WebAssembly"; }
  uint32_t getChipRevision() { return 0; }
  uint32_t getFlashChipSize() { return 16 * 1024 * 1024; }
};
inline WebESP ESP;
