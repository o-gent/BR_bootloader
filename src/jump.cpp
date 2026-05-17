#include <Arduino.h>
#include "jump.h"

/*
 * SRAM bounds used to sanity-check the application's initial stack pointer.
 * STM32L4 main SRAM is at 0x20000000 (96 KB max on this family); H7 has
 * larger DTCMRAM/AXISRAM regions. The range below is intentionally generous:
 * we only need to reject obviously-erased flash (0xFFFFFFFF) and zeroed RAM.
 */
#define SRAM_BASE_MIN  0x20000000UL
#define SRAM_BASE_MAX  0x24080000UL  /* covers L4 (0x20018000) and H7 AXISRAM */

bool app_is_valid(uint32_t app_address)
{
    const uint32_t *vt = (const uint32_t *)app_address;
    uint32_t sp    = vt[0];
    uint32_t reset = vt[1];

    if (sp < SRAM_BASE_MIN || sp > SRAM_BASE_MAX) {
        return false;
    }
    /* Reset handler must live in flash and have the Thumb bit set. */
    if ((reset & 1U) == 0U) {
        return false;
    }
    if (reset < 0x08000000UL || reset >= 0x08200000UL) {
        return false;
    }
    return true;
}

void jump_to_app(uint32_t app_address)
{
    if (!app_is_valid(app_address)) {
        return;
    }

    const uint32_t *vt = (const uint32_t *)app_address;
    uint32_t sp    = vt[0];
    uint32_t reset = vt[1];

    /* Disable interrupts and clear pending so the app starts clean. */
    __disable_irq();
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    /* Point the vector table at the app and force the change to be visible. */
    SCB->VTOR = app_address;
    __DSB();
    __ISB();

    /* Load SP and branch -- noreturn. */
    __set_MSP(sp);
    __enable_irq();
    ((void (*)(void))reset)();

    /* Should never reach here. */
    while (1) { }
}
