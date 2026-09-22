#pragma once
// FreeInk simulator — ESP-NOW shim.
//
// Peers are real: two simulator instances sharing a daemon channel exchange
// frames, so NearbyTransfer can be driven device-to-device without hardware. A
// single instance with no peer sees its sends succeed and nothing arrive.
#include <cstdint>

#include "esp_err.h"

#define ESP_NOW_ETH_ALEN 6
#define ESP_NOW_MAX_DATA_LEN 250

typedef struct {
  uint8_t peer_addr[ESP_NOW_ETH_ALEN];
  uint8_t lmk[16];
  uint8_t channel;
  int ifidx;
  bool encrypt;
  void* priv;
} esp_now_peer_info_t;

typedef struct {
  uint8_t src_addr[ESP_NOW_ETH_ALEN];
  uint8_t des_addr[ESP_NOW_ETH_ALEN];
  void* rx_ctrl;
} esp_now_recv_info_t;

typedef void (*esp_now_recv_cb_t)(const esp_now_recv_info_t* info, const uint8_t* data, int len);
typedef void (*esp_now_send_cb_t)(const uint8_t* mac_addr, int status);

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t esp_now_init(void);
esp_err_t esp_now_deinit(void);
esp_err_t esp_now_add_peer(const esp_now_peer_info_t* peer);
esp_err_t esp_now_del_peer(const uint8_t* peer_addr);
bool esp_now_is_peer_exist(const uint8_t* peer_addr);
esp_err_t esp_now_send(const uint8_t* peer_addr, const uint8_t* data, size_t len);
esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t cb);
esp_err_t esp_now_unregister_recv_cb(void);
esp_err_t esp_now_register_send_cb(esp_now_send_cb_t cb);
esp_err_t esp_now_unregister_send_cb(void);
#ifdef __cplusplus
}
#endif
