/*
 * Per-MCU configuration constants for the BR bootloader.
 *
 * Selected at build time via the platformio.ini build flag for the target
 * (-DCANL431 for MicroNode, -DCANH7 for CoreNode/MicroNodePlus).
 */

#pragma once

#include <stdint.h>

#if defined(CANL431)

/* STM32L431 (MicroNode) ----------------------------------------------------*/

/* Bootloader occupies the first 40 KB of flash; app starts on the next 2 KB
   page boundary so the bootloader is safe to leave in place during app erase. */
#define APP_START_ADDRESS   0x0800A000UL
#define BL_FLASH_PAGE_SIZE  0x800UL          /* 2 KB pages on L4 */
#define BOARD_FLASH_BYTES   (256UL * 1024UL) /* L431CC */

/* Arduino pin 19 == PB3 on the MicroNode variant header. */
#define BOOTLOADER_LED_PIN  19

/* 96-bit unique ID base address (per RM0394). */
#define UDID_BASE           0x1FFF7590UL

#elif defined(CANH7)

/* STM32H7 family (CoreNode / MicroNodePlus) -- filled in during Phase 3. */
#define APP_START_ADDRESS   0x08020000UL     /* sector 1 boundary on H7 */
#define BL_FLASH_PAGE_SIZE  0x20000UL        /* 128 KB sectors on H7 */
#define UDID_BASE           0x1FF1E800UL

#else
#error "Define CANL431 or CANH7 for the target MCU."
#endif
