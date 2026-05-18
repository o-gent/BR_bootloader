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
 *
 * Write granule: 32 bytes (256 bits). This is the H7 minimum; L4 only needs
 * 8 bytes but accepts any 8-byte multiple, so 32-byte alignment is safe for
 * both. Universal alignment simplifies the code at a cost of up to ~31 bytes
 * of 0xFF padding at the end of the image.
 */
/* Page numbers are zero-based from the flash base (0x08000000), so the app
   start needs the flash base subtracted before dividing. The original macro
   used the absolute address and resolved to ~65556 -- wildly out of range --
   which is why every erase path (lib, raw register, HAL) silently failed. */
#define APP_FIRST_PAGE  ((APP_START_ADDRESS - 0x08000000UL) / BL_FLASH_PAGE_SIZE)
#define APP_LAST_PAGE   ((BOARD_FLASH_BYTES / BL_FLASH_PAGE_SIZE) - 1)
#define BL_WRITE_ALIGN  32U

static uint32_t cursor;
static uint8_t  pending[BL_WRITE_ALIGN];
static uint8_t  pending_len;
static uint32_t highest_page_erased;
static bool     started;
static bool     failed;
static uint32_t bytes_written;
static bl_flash_err_t last_err = BL_FLASH_OK;
static struct bl_flash_diag diag;

/* Deferred vector-table write. The image's first 8 bytes (initial SP +
   reset_handler) are what app_is_valid() looks at -- if a power-loss leaves
   those bytes valid but the rest of the image incomplete, the bootloader
   would jump to broken code on the next boot.
 *
 * We defer the entire first 32-byte write granule (not just 8 bytes), for
 * two reasons:
 *   1. On H7, stm32_flash_write rejects re-programming partially-programmed
 *      memory (check_all_ones gate). If we wrote bytes 8-31 of the image
 *      during streaming, a later 32-byte write at the same address would
 *      fail. By deferring all 32 bytes, the first block stays erased (0xFF)
 *      until finalize.
 *   2. The L4 write path's skip-if-matched optimization makes writing 0xFF
 *      to an erased cell a no-op, so this has zero cost on both families.
 */
static uint8_t  deferred_vt[BL_WRITE_ALIGN];
static bool     deferred_vt_armed;

/* STM32L4 FLASH peripheral register addresses (RM0394 Table 51). */
#define BL_FLASH_BASE   0x40022000UL
#define BL_FLASH_ACR    (*(volatile uint32_t *)(BL_FLASH_BASE + 0x00))
#define BL_FLASH_KEYR   (*(volatile uint32_t *)(BL_FLASH_BASE + 0x08))
#define BL_FLASH_SR     (*(volatile uint32_t *)(BL_FLASH_BASE + 0x10))
#define BL_FLASH_CR     (*(volatile uint32_t *)(BL_FLASH_BASE + 0x14))
#define BL_FLASH_OPTR   (*(volatile uint32_t *)(BL_FLASH_BASE + 0x20))
#define BL_FLASH_WRP1AR  (*(volatile uint32_t *)(BL_FLASH_BASE + 0x2C))
#define BL_FLASH_WRP1BR  (*(volatile uint32_t *)(BL_FLASH_BASE + 0x30))
#define BL_FLASH_PCROP1SR (*(volatile uint32_t *)(BL_FLASH_BASE + 0x24))
#define BL_FLASH_PCROP1ER (*(volatile uint32_t *)(BL_FLASH_BASE + 0x28))


static void capture_diag(void)
{
    diag.sr       = BL_FLASH_SR;
    diag.cr       = BL_FLASH_CR;
    diag.optr     = BL_FLASH_OPTR;
    diag.wrp1ar   = BL_FLASH_WRP1AR;
    diag.wrp1br   = BL_FLASH_WRP1BR;
    diag.pcrop1sr = BL_FLASH_PCROP1SR;
    diag.pcrop1er = BL_FLASH_PCROP1ER;
    diag.acr      = BL_FLASH_ACR;
}

static void fail(bl_flash_err_t err)
{
    failed = true;
    if (last_err == BL_FLASH_OK) {
        last_err = err;  /* keep the first failure -- root cause, not consequences */
    }
}

static uint32_t addr_to_page(uint32_t addr)
{
    return (addr - 0x08000000UL) / BL_FLASH_PAGE_SIZE;
}

static bool ensure_page_erased(uint32_t addr)
{
    uint32_t page = addr_to_page(addr);
    if (page < APP_FIRST_PAGE || page > APP_LAST_PAGE) {
        fail(BL_FLASH_ERR_OUT_OF_RANGE);
        return false;
    }
    if (started && page <= highest_page_erased) {
        return true;
    }
    if (!stm32_flash_erasepage(page)) {
        fail(BL_FLASH_ERR_PAGE_ERASE);
        return false;
    }
    highest_page_erased = page;
    return true;
}

static bool flush_aligned(const uint8_t *src, uint32_t len)
{
    /* len is a multiple of BL_WRITE_ALIGN here. Erase any pages we're about to touch. */
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
        chunk &= ~(BL_WRITE_ALIGN - 1U);
        if (chunk == 0) {
            fail(BL_FLASH_ERR_WRITE);
            return false;
        }
        if (!stm32_flash_write(addr, p, chunk)) {
            fail(BL_FLASH_ERR_WRITE);
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
    bytes_written = 0;
    started = false;
    failed = false;
    last_err = BL_FLASH_OK;
    deferred_vt_armed = false;
    memset(deferred_vt, 0xFF, sizeof(deferred_vt));

    if (!stm32_flash_erasepage(APP_FIRST_PAGE)) {
        capture_diag();
        fail(BL_FLASH_ERR_BEGIN_ERASE);
        return false;
    }
    highest_page_erased = APP_FIRST_PAGE;
    started = true;
    return true;
}

bool bootloader_flash_write(uint32_t offset, const uint8_t *data, uint16_t len)
{
    if (!started) {
        fail(BL_FLASH_ERR_NOT_STARTED);
        return false;
    }
    uint32_t expected_offset = (cursor - APP_START_ADDRESS) + pending_len;
    if (offset != expected_offset) {
        fail(BL_FLASH_ERR_OFFSET);
        return false;
    }
    uint8_t buf[BL_WRITE_ALIGN + 512];
    uint32_t total = pending_len + len;
    if (total > sizeof(buf)) {
        fail(BL_FLASH_ERR_OVERFLOW);
        return false;
    }
    memcpy(buf, pending, pending_len);
    memcpy(buf + pending_len, data, len);

    /* Deferred VT: if this chunk contains the image's first write-granule,
       stash the bytes and substitute 0xFF in the buffer we're about to
       flash. offset==0 is sufficient because writes are strictly append-only
       (asserted by the expected_offset gate above) -- the first BL_WRITE_ALIGN
       bytes can only ever appear in the very first chunk. */
    if (!deferred_vt_armed && offset == 0 && total >= BL_WRITE_ALIGN) {
        memcpy(deferred_vt, buf, BL_WRITE_ALIGN);
        memset(buf, 0xFF, BL_WRITE_ALIGN);
        deferred_vt_armed = true;
    }

    uint32_t aligned = total & ~(BL_WRITE_ALIGN - 1U);
    uint32_t tail    = total - aligned;

    if (aligned > 0) {
        if (!flush_aligned(buf, aligned)) {
            /* flush_aligned already set the specific error code. */
            return false;
        }
    }
    memcpy(pending, buf + aligned, tail);
    pending_len = (uint8_t)tail;
    bytes_written = (cursor - APP_START_ADDRESS);
    return true;
}

bool bootloader_flash_finalize(void)
{
    if (!started) {
        fail(BL_FLASH_ERR_FINALIZE);
        return false;
    }
    if (pending_len > 0) {
        uint8_t padded[BL_WRITE_ALIGN];
        memset(padded, 0xFF, sizeof(padded));
        memcpy(padded, pending, pending_len);
        if (!flush_aligned(padded, BL_WRITE_ALIGN)) {
            return false;
        }
        pending_len = 0;
    }

    /* Commit the deferred vector table as the very last operation. Before
       this write the first BL_WRITE_ALIGN bytes at APP_START are still 0xFF
       (we wrote 0xFF during streaming; both the L4 and H7 write paths skip
       the actual program operation when source matches destination, so the
       erased state survives intact). The real 32-byte write here completes
       the image and makes app_is_valid() return true on the next boot. */
    if (deferred_vt_armed) {
        if (!stm32_flash_write(APP_START_ADDRESS, deferred_vt, BL_WRITE_ALIGN)) {
            fail(BL_FLASH_ERR_WRITE);
            return false;
        }
        deferred_vt_armed = false;
    }

    bytes_written = (cursor - APP_START_ADDRESS);
    started = false;
    return true;
}

bool bootloader_flash_failed(void)
{
    return failed;
}

uint32_t bootloader_flash_bytes_written(void)
{
    return bytes_written;
}

bl_flash_err_t bootloader_flash_last_error(void)
{
    return last_err;
}

void bootloader_flash_get_diag(struct bl_flash_diag *out)
{
    if (out != NULL) {
        *out = diag;
    }
}

void bootloader_flash_write_cb(uint32_t offset, const uint8_t *data, uint16_t len)
{
    bootloader_flash_write(offset, data, len);
}
