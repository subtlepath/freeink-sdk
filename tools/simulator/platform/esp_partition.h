#pragma once

// FreeInk simulator — esp_partition shim.
//
// Backed by a host file acting as the flash image, so OTA/recovery code that
// reads, erases and writes partitions runs for real against a virtual flash the
// CLI can dump and diff. The layout comes from the partition CSV the bundle was
// built with, or a built-in default when none was supplied.

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

typedef enum { ESP_PARTITION_TYPE_APP = 0x00, ESP_PARTITION_TYPE_DATA = 0x01, ESP_PARTITION_TYPE_ANY = 0xff } esp_partition_type_t;
typedef enum {
  ESP_PARTITION_SUBTYPE_APP_FACTORY = 0x00,
  ESP_PARTITION_SUBTYPE_APP_OTA_0 = 0x10,
  ESP_PARTITION_SUBTYPE_APP_OTA_1 = 0x11,
  ESP_PARTITION_SUBTYPE_DATA_OTA = 0x00,
  ESP_PARTITION_SUBTYPE_DATA_NVS = 0x02,
  ESP_PARTITION_SUBTYPE_DATA_COREDUMP = 0x03,
  ESP_PARTITION_SUBTYPE_DATA_FAT = 0x81,
  ESP_PARTITION_SUBTYPE_DATA_SPIFFS = 0x82,
  ESP_PARTITION_SUBTYPE_ANY = 0xff,
} esp_partition_subtype_t;

typedef struct {
  void* flash_chip;
  esp_partition_type_t type;
  esp_partition_subtype_t subtype;
  uint32_t address;
  uint32_t size;
  uint32_t erase_size;
  char label[17];
  bool encrypted;
  bool readonly;
} esp_partition_t;

typedef struct esp_partition_iterator_opaque_* esp_partition_iterator_t;

#ifdef __cplusplus
extern "C" {
#endif
const esp_partition_t* esp_partition_find_first(esp_partition_type_t type, esp_partition_subtype_t subtype, const char* label);
esp_partition_iterator_t esp_partition_find(esp_partition_type_t type, esp_partition_subtype_t subtype, const char* label);
const esp_partition_t* esp_partition_get(esp_partition_iterator_t it);
esp_partition_iterator_t esp_partition_next(esp_partition_iterator_t it);
void esp_partition_iterator_release(esp_partition_iterator_t it);
esp_err_t esp_partition_read(const esp_partition_t* p, size_t off, void* dst, size_t size);
esp_err_t esp_partition_write(const esp_partition_t* p, size_t off, const void* src, size_t size);
esp_err_t esp_partition_erase_range(const esp_partition_t* p, size_t off, size_t size);
esp_err_t esp_partition_read_raw(const esp_partition_t* p, size_t off, void* dst, size_t size);
esp_err_t esp_partition_write_raw(const esp_partition_t* p, size_t off, const void* src, size_t size);
#ifdef __cplusplus
}
#endif
