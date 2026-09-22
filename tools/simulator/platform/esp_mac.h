#pragma once

// FreeInk simulator — esp_mac shim. A stable, obviously-simulated locally
// administered MAC, so firmware that derives an ID from it is reproducible.

#include <cstdint>
#include <cstring>

#include "esp_err.h"

typedef enum { ESP_MAC_WIFI_STA = 0, ESP_MAC_WIFI_SOFTAP, ESP_MAC_BT, ESP_MAC_ETH } esp_mac_type_t;

inline esp_err_t esp_read_mac(uint8_t* mac, esp_mac_type_t type) {
  if (!mac) return ESP_ERR_INVALID_ARG;
  const uint8_t base[6] = {0x02, 0x46, 0x52, 0x45, 0x45, 0x00};
  memcpy(mac, base, 6);
  mac[5] = static_cast<uint8_t>(type);
  return ESP_OK;
}
inline esp_err_t esp_efuse_mac_get_default(uint8_t* mac) { return esp_read_mac(mac, ESP_MAC_WIFI_STA); }
