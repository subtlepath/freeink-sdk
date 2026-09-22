#pragma once

// FreeInk simulator — esp_ota_ops shim over the virtual flash. Setting the boot
// partition is recorded and honoured by the next simulated restart, so an OTA
// or recovery flow can be driven end to end from the CLI.

#include "esp_err.h"
#include "esp_partition.h"

typedef uint32_t esp_ota_handle_t;

#ifdef __cplusplus
extern "C" {
#endif
const esp_partition_t* esp_ota_get_running_partition(void);
const esp_partition_t* esp_ota_get_boot_partition(void);
const esp_partition_t* esp_ota_get_next_update_partition(const esp_partition_t* start_from);
esp_err_t esp_ota_set_boot_partition(const esp_partition_t* partition);
esp_err_t esp_ota_begin(const esp_partition_t* partition, size_t image_size, esp_ota_handle_t* out_handle);
esp_err_t esp_ota_write(esp_ota_handle_t handle, const void* data, size_t size);
esp_err_t esp_ota_end(esp_ota_handle_t handle);
#ifdef __cplusplus
}
#endif
