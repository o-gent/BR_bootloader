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

/* True if any begin/write/finalize call has failed since the last begin(). */
bool bootloader_flash_failed(void);

/* Number of bytes successfully committed to flash (post-finalize == image size). */
uint32_t bootloader_flash_bytes_written(void);

/* Localized failure code -- set on the first failing call, never cleared
   except by a successful bootloader_flash_begin(). LED blink patterns in
   main.cpp map directly to this value (1 = BEGIN_ERASE, etc.). */
typedef enum {
    BL_FLASH_OK                = 0,
    BL_FLASH_ERR_BEGIN_ERASE   = 1,  /* initial page-20 erase failed */
    BL_FLASH_ERR_NOT_STARTED   = 2,  /* write called before successful begin */
    BL_FLASH_ERR_OFFSET        = 3,  /* chunk arrived at unexpected offset */
    BL_FLASH_ERR_OVERFLOW      = 4,  /* chunk + pending > internal buffer */
    BL_FLASH_ERR_PAGE_ERASE    = 5,  /* mid-image page erase failed */
    BL_FLASH_ERR_WRITE         = 6,  /* stm32_flash_write reported failure */
    BL_FLASH_ERR_FINALIZE      = 7,  /* finalize-time padded write failed */
    BL_FLASH_ERR_OUT_OF_RANGE  = 8,  /* page address outside app region */
} bl_flash_err_t;

bl_flash_err_t bootloader_flash_last_error(void);

/* STM32L4 FLASH peripheral snapshot captured at the point of failure.
   Lets us tell apart WRP errors from PGS errors from option-byte issues
   without attaching a debugger. Only valid when bootloader_flash_failed(). */
struct bl_flash_diag {
    uint32_t sr;       /* FLASH_SR after the failed op (WRPERR/PGSERR/etc.) */
    uint32_t cr;       /* FLASH_CR after the failed op (PER/PG/LOCK state) */
    uint32_t optr;     /* FLASH_OPTR (RDP level, dual-bank, etc.) */
    uint32_t wrp1ar;   /* FLASH_WRP1AR (write-protected area A bounds) */
    uint32_t wrp1br;   /* FLASH_WRP1BR (write-protected area B bounds) */
    uint32_t pcrop1sr; /* FLASH_PCROP1SR (PCROP zone start) */
    uint32_t pcrop1er; /* FLASH_PCROP1ER (PCROP zone end + RDP_ERASE flag) */
    uint32_t acr;      /* FLASH_ACR (cache + prefetch state) */
};
void bootloader_flash_get_diag(struct bl_flash_diag *out);

#ifdef __cplusplus
}
#endif
