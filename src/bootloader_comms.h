/*
 * Shared bootloader <-> application IPC region.
 *
 * Layout MUST match Arduino-DroneCAN's app_bootloader_comms (lib/libArduinoDroneCAN/src/dronecan.h)
 * and AP_Periph's app_comms.h. The struct lives at BOOTCOM_ADDR (0x20000000), 256 bytes
 * carved out of RAM by the linker scripts on both sides.
 */

#pragma once

#include <stdint.h>

#define APP_BOOTLOADER_COMMS_MAGIC 0xc544ad9a
#define BOOTCOM_ADDR               0x20000000UL

struct app_bootloader_comms {
    uint32_t magic;
    uint32_t ip;
    uint32_t netmask;
    uint32_t gateway;
    uint32_t reserved;
    uint8_t  server_node_id;
    uint8_t  my_node_id;
    uint8_t  path[201];
};

static inline volatile struct app_bootloader_comms *bootcom(void)
{
    return (volatile struct app_bootloader_comms *)BOOTCOM_ADDR;
}
