#pragma once

// FreeInk simulator — raw NVS C API shim, over the same key/value store the
// Preferences shim uses, so both views see one namespace.

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

typedef uint32_t nvs_handle_t;
typedef nvs_handle_t nvs_handle;
typedef enum { NVS_READONLY = 0, NVS_READWRITE = 1 } nvs_open_mode_t;

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t nvs_flash_init(void);
esp_err_t nvs_flash_erase(void);
esp_err_t nvs_open(const char* name, nvs_open_mode_t mode, nvs_handle_t* out_handle);
void nvs_close(nvs_handle_t handle);
esp_err_t nvs_commit(nvs_handle_t handle);
esp_err_t nvs_get_u8(nvs_handle_t h, const char* key, uint8_t* out);
esp_err_t nvs_set_u8(nvs_handle_t h, const char* key, uint8_t v);
esp_err_t nvs_get_u16(nvs_handle_t h, const char* key, uint16_t* out);
esp_err_t nvs_set_u16(nvs_handle_t h, const char* key, uint16_t v);
esp_err_t nvs_get_u32(nvs_handle_t h, const char* key, uint32_t* out);
esp_err_t nvs_set_u32(nvs_handle_t h, const char* key, uint32_t v);
esp_err_t nvs_get_i32(nvs_handle_t h, const char* key, int32_t* out);
esp_err_t nvs_set_i32(nvs_handle_t h, const char* key, int32_t v);
esp_err_t nvs_get_str(nvs_handle_t h, const char* key, char* out, size_t* length);
esp_err_t nvs_set_str(nvs_handle_t h, const char* key, const char* v);
esp_err_t nvs_get_blob(nvs_handle_t h, const char* key, void* out, size_t* length);
esp_err_t nvs_set_blob(nvs_handle_t h, const char* key, const void* v, size_t length);
esp_err_t nvs_erase_key(nvs_handle_t h, const char* key);
esp_err_t nvs_erase_all(nvs_handle_t h);
#ifdef __cplusplus
}
#endif
