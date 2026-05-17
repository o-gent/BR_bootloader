/*
 * Bootloader flash write API.
 *
 * Wraps the vendored stm32_flash_* primitives (lib/libArduinoDroneCAN/src/flash.cpp)
 * with sector-aware, append-only semantics for receiving a firmware image over
 * DroneCAN FileRead.
 *
 * Usage:
 *   bootloader_flash_begin();                       // erase first app page
 *   bootloader_flash_write(offset, data, len);      // append chunk
 *   ...
 *   bootloader_flash_finalize();                    // flush + lock
 *
 * Notes:
 *   - On STM32L4 the underlying flash primitive requires 8-byte aligned writes.
 *     Chunks are buffered internally and flushed in 8-byte units; the final
 *     write is padded with 0xFF.
 *   - Pages are erased lazily as the write address crosses page boundaries,
 *     so we don't pay erase latency for pages we never touch.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Prepare for a new image: reset offset, erase the first app page. */
bool bootloader_flash_begin(void);

/* Append a chunk at the given linear offset (0 == start of app image). The
   offset is checked against the internal cursor; out-of-order chunks fail. */
bool bootloader_flash_write(uint32_t offset, const uint8_t *data, uint16_t len);

/* Flush any buffered tail (padded with 0xFF) and lock the flash. */
bool bootloader_flash_finalize(void);

/* Convenience: matches the DroneCAN::firmware_write_fn signature. */
void bootloader_flash_write_cb(uint32_t offset, const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif
