#pragma once

// FreeInk simulator — esp_heap_caps shim.
//
// Every capability maps to the host heap. The daemon still accounts the
// requests per pool, so `freeink-sim mem` reports how much the firmware asked
// for as PSRAM vs internal — the ratio that decides whether a build fits a real
// device, even though the host has one flat heap.

#include <cstdlib>
#include <cstring>
#include <freeink_sim_abi.h>

#define MALLOC_CAP_EXEC (1 << 0)
#define MALLOC_CAP_32BIT (1 << 1)
#define MALLOC_CAP_8BIT (1 << 2)
#define MALLOC_CAP_DMA (1 << 3)
#define MALLOC_CAP_SPIRAM (1 << 10)
#define MALLOC_CAP_INTERNAL (1 << 11)
#define MALLOC_CAP_DEFAULT (1 << 12)

#ifdef __cplusplus
extern "C" {
#endif
// Implemented in sim_bridge.cpp so the accounting lives in one place.
void* heap_caps_malloc(size_t size, uint32_t caps);
void* heap_caps_calloc(size_t n, size_t size, uint32_t caps);
void* heap_caps_realloc(void* ptr, size_t size, uint32_t caps);
void* heap_caps_aligned_alloc(size_t alignment, size_t size, uint32_t caps);
void heap_caps_free(void* ptr);
size_t heap_caps_get_free_size(uint32_t caps);
size_t heap_caps_get_largest_free_block(uint32_t caps);
size_t heap_caps_get_total_size(uint32_t caps);
size_t heap_caps_get_minimum_free_size(uint32_t caps);
#ifdef __cplusplus
}
#endif
