#include "bootloader_flash.h"
#include "board_config.h"
#include <flash.h>
#include <string.h>

/*
 * Internal state for an in-progress image write.
 *   cursor      -- next absolute flash address to write
 *   pending_*   -- short tail buffer for sub-aligned chunks
 *   highest_page_erased -- so we erase each app page exactly once
 *   started     -- gate begin() / write() ordering
 */
#define APP_FIRST_PAGE  (APP_START_ADDRESS / BL_FLASH_PAGE_SIZE)
#define APP_LAST_PAGE   ((BOARD_FLASH_BYTES / BL_FLASH_PAGE_SIZE) - 1)

static uint32_t cursor;
static uint8_t  pending[8];
static uint8_t  pending_len;
static uint32_t highest_page_erased;
static bool     started;

static uint32_t addr_to_page(uint32_t addr)
{
    return (addr - 0x08000000UL) / BL_FLASH_PAGE_SIZE;
}

static bool ensure_page_erased(uint32_t addr)
{
    uint32_t page = addr_to_page(addr);
    if (page < APP_FIRST_PAGE || page > APP_LAST_PAGE) {
        return false;
    }
    if (started && page <= highest_page_erased) {
        return true;
    }
    if (!stm32_flash_erasepage(page)) {
        return false;
    }
    highest_page_erased = page;
    return true;
}

static bool flush_aligned(const uint8_t *src, uint32_t len)
{
    /* len is a multiple of 8 here. Erase any pages we're about to touch. */
    uint32_t remaining = len;
    uint32_t addr      = cursor;
    const uint8_t *p   = src;
    while (remaining > 0) {
        if (!ensure_page_erased(addr)) {
            return false;
        }
        uint32_t page_end   = (addr_to_page(addr) + 1) * BL_FLASH_PAGE_SIZE + 0x08000000UL;
        uint32_t chunk_room = page_end - addr;
        uint32_t chunk      = remaining < chunk_room ? remaining : chunk_room;
        /* Round chunk down to 8-byte multiple; the partial slice (if any)
           goes into the next page's chunk after an erase. */
        chunk &= ~7U;
        if (chunk == 0) {
            /* shouldn't happen given remaining is 8-byte aligned, but guard. */
            return false;
        }
        if (!stm32_flash_write(addr, p, chunk)) {
            return false;
        }
        addr      += chunk;
        p         += chunk;
        remaining -= chunk;
    }
    cursor = addr;
    return true;
}

bool bootloader_flash_begin(void)
{
    cursor = APP_START_ADDRESS;
    pending_len = 0;
    highest_page_erased = 0;
    started = false;
    if (!stm32_flash_erasepage(APP_FIRST_PAGE)) {
        return false;
    }
    highest_page_erased = APP_FIRST_PAGE;
    started = true;
    return true;
}

bool bootloader_flash_write(uint32_t offset, const uint8_t *data, uint16_t len)
{
    if (!started) {
        return false;
    }
    /* Enforce strict append semantics; out-of-order chunks indicate a state bug. */
    uint32_t expected_offset = (cursor - APP_START_ADDRESS) + pending_len;
    if (offset != expected_offset) {
        return false;
    }
    /* Combine the small pending tail with the new chunk, write whole 8-byte
       groups, keep the residue for next time. */
    uint8_t buf[8 + 512];  /* DroneCAN file-read max payload is 256B; 512 is generous. */
    uint32_t total = pending_len + len;
    if (total > sizeof(buf)) {
        /* Force a flush of pending+head first if a giant chunk ever arrives. */
        return false;
    }
    memcpy(buf, pending, pending_len);
    memcpy(buf + pending_len, data, len);

    uint32_t aligned = total & ~7U;
    uint32_t tail    = total - aligned;

    if (aligned > 0) {
        if (!flush_aligned(buf, aligned)) {
            return false;
        }
    }
    memcpy(pending, buf + aligned, tail);
    pending_len = (uint8_t)tail;
    return true;
}

bool bootloader_flash_finalize(void)
{
    if (!started) {
        return false;
    }
    if (pending_len > 0) {
        uint8_t padded[8];
        memset(padded, 0xFF, sizeof(padded));
        memcpy(padded, pending, pending_len);
        if (!flush_aligned(padded, 8)) {
            return false;
        }
        pending_len = 0;
    }
    started = false;
    return true;
}

void bootloader_flash_write_cb(uint32_t offset, const uint8_t *data, uint16_t len)
{
    bootloader_flash_write(offset, data, len);
}
