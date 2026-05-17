/*
 * Jump to application image at APP_START_ADDRESS.
 *
 * The application's vector table layout (Cortex-M):
 *   [0] = initial main stack pointer
 *   [1] = Reset_Handler address
 *
 * jump_to_app() validates the SP is inside SRAM, points VTOR at the app
 * vector table, loads SP, and branches to the reset handler. It returns
 * (without jumping) if no valid app is present so the caller can fall back
 * to bootloader idle behaviour.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool app_is_valid(uint32_t app_address);
void jump_to_app(uint32_t app_address);

#ifdef __cplusplus
}
#endif
