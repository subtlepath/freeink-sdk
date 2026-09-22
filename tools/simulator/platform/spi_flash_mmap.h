#pragma once

// FreeInk simulator — spi_flash_mmap shim. The virtual flash file is mapped
// into the host address space, so code that reads a memory-mapped partition
// gets a real pointer with real contents.

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

typedef uint32_t spi_flash_mmap_handle_t;
typedef enum { SPI_FLASH_MMAP_DATA = 0, SPI_FLASH_MMAP_INST } spi_flash_mmap_memory_t;

#ifdef __cplusplus
extern "C" {
#endif
esp_err_t spi_flash_mmap(size_t src_addr, size_t size, spi_flash_mmap_memory_t memory, const void** out_ptr,
                         spi_flash_mmap_handle_t* out_handle);
void spi_flash_munmap(spi_flash_mmap_handle_t handle);
#ifdef __cplusplus
}
#endif
