#pragma once

// FreeInk simulator — WiFi shim.
//
// Association is entirely virtual: the CLI decides which networks exist, how
// long a join takes, and whether it succeeds, so firmware retry and
// failure-path code can be driven deterministically instead of depending on a
// real access point. Sockets opened after association go through the host, but
// only when the daemon was started with networking enabled — a simulated device
// should not reach the internet unless the operator says so.

#include <Arduino.h>
#include <freeink_sim_abi.h>

#include <string>
#include <vector>

#include "Client.h"
#include "IPAddress.h"

typedef enum {
  WL_NO_SHIELD = 255,
  WL_IDLE_STATUS = 0,
  WL_NO_SSID_AVAIL = 1,
  WL_SCAN_COMPLETED = 2,
  WL_CONNECTED = 3,
  WL_CONNECT_FAILED = 4,
  WL_CONNECTION_LOST = 5,
  WL_DISCONNECTED = 6,
} wl_status_t;

typedef enum { WIFI_MODE_NULL = 0, WIFI_MODE_STA, WIFI_MODE_AP, WIFI_MODE_APSTA } wifi_mode_t;
#define WIFI_STA WIFI_MODE_STA
#define WIFI_AP WIFI_MODE_AP
#define WIFI_OFF WIFI_MODE_NULL
#define WIFI_SCAN_RUNNING -1
#define WIFI_SCAN_FAILED -2
#define WIFI_AUTH_OPEN 0
#define WIFI_ALL_CHANNEL_SCAN 1
#define WIFI_CONNECT_AP_BY_SIGNAL 1

class SimWiFiClass {
 public:
  bool mode(wifi_mode_t m) {
    _mode = m;
    if (m == WIFI_MODE_NULL) fsim_wifi_disconnect();
    return true;
  }
  wl_status_t begin(const char* ssid, const char* pass = nullptr) {
    fsim_wifi_begin(ssid, pass ? pass : "");
    return status();
  }
  wl_status_t status() {
    switch (fsim_wifi_status()) {
      case FSIM_WIFI_CONNECTED: return WL_CONNECTED;
      case FSIM_WIFI_CONNECTING: return WL_IDLE_STATUS;
      case FSIM_WIFI_FAILED: return WL_CONNECT_FAILED;
      default: return WL_DISCONNECTED;
    }
  }
  bool isConnected() { return status() == WL_CONNECTED; }
  bool disconnect(bool = false, bool = false) {
    fsim_wifi_disconnect();
    return true;
  }
  IPAddress localIP() { return IPAddress(fsim_wifi_local_ip()); }
  IPAddress gatewayIP() { return IPAddress(fsim_wifi_local_ip() & 0x00FFFFFFu); }
  IPAddress subnetMask() { return IPAddress(255, 255, 255, 0); }
  String macAddress() { return String("02:46:52:45:45:00"); }
  int32_t RSSI() { return isConnected() ? -55 : -100; }
  void setSleep(bool) {}
  wifi_mode_t getMode() const { return _mode; }
  int16_t scanComplete() const { return static_cast<int16_t>(_scan.size()); }
  void setScanMethod(int) {}
  void setSortMethod(int) {}
  void setHostname(const char*) {}
  String SSID() { return String(); }
  bool softAP(const char*, const char* = nullptr, int = 1, bool = false, int = 4) { return false; }
  bool softAPdisconnect(bool = false) { return true; }
  IPAddress softAPIP() { return IPAddress(); }
  const uint8_t* BSSID(uint8_t = 0) { static const uint8_t empty[6]{}; return empty; }
  void BSSID(uint8_t* out) { if (out) memset(out, 0, 6); }
  int channel(uint8_t = 0) { return 0; }
  void setAutoReconnect(bool) {}
  void persistent(bool) {}

  // Scan results come from the daemon's virtual network list, which the CLI
  // populates (`freeink-sim wifi add-network`).
  int16_t scanNetworks(bool = false, bool = false);
  String SSID(uint8_t index);
  int32_t RSSI(uint8_t index);
  uint8_t encryptionType(uint8_t index);
  void scanDelete();

 private:
  struct ScanEntry {
    std::string ssid;
    int32_t rssi;
    uint8_t enc;
  };

  wifi_mode_t _mode = WIFI_MODE_NULL;
  std::vector<ScanEntry> _scan;
};

extern SimWiFiClass WiFi;
