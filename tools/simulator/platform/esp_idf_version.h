#pragma once

// FreeInk simulator — esp_idf_version shim. Reports the IDF release the
// simulator's shims are modelled on, so version-gated SDK code takes the same
// branch it takes on device.

#define ESP_IDF_VERSION_MAJOR 5
#define ESP_IDF_VERSION_MINOR 3
#define ESP_IDF_VERSION_PATCH 0
#define ESP_IDF_VERSION_VAL(major, minor, patch) ((major) * 10000 + (minor) * 100 + (patch))
#define ESP_IDF_VERSION ESP_IDF_VERSION_VAL(ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH)
