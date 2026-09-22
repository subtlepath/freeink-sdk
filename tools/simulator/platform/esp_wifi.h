#pragma once
// FreeInk simulator — esp_wifi shim. Radio tuning calls are accepted and
// recorded; association itself lives in the WiFi shim.
#include <cstdint>

#include "esp_err.h"
#include "esp_mac.h"

typedef enum { WIFI_PS_NONE = 0, WIFI_PS_MIN_MODEM, WIFI_PS_MAX_MODEM } wifi_ps_type_t;
typedef enum { WIFI_IF_STA = 0, WIFI_IF_AP = 1 } wifi_interface_t;
typedef enum { WIFI_SECOND_CHAN_NONE = 0, WIFI_SECOND_CHAN_ABOVE, WIFI_SECOND_CHAN_BELOW } wifi_second_chan_t;

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t esp_wifi_set_ps(wifi_ps_type_t type);
esp_err_t esp_wifi_set_channel(uint8_t primary, wifi_second_chan_t second);
esp_err_t esp_wifi_get_mac(wifi_interface_t ifx, uint8_t mac[6]);
esp_err_t esp_wifi_start(void);
esp_err_t esp_wifi_stop(void);
#ifdef __cplusplus
}
#endif
