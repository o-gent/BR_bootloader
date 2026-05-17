/*
 * BR_bootloader -- Phase 2 entry point.
 *
 * Boot flow:
 *   1. If BOOTCOM magic is present, the app requested an update -> stay in
 *      the bootloader, reuse the node ID it left for us, and immediately
 *      bring CAN up so the GCS can re-deliver BeginFirmwareUpdate.
 *   2. Otherwise, if a valid app vector table exists -> jump to app.
 *   3. Otherwise, bring CAN up (DNA from scratch) so the bootloader is
 *      discoverable and recoverable from a totally-empty board.
 *
 * After CAN is up we loop: process DroneCAN, drive any in-flight FileRead,
 * and -- once a download completes -- validate and jump.
 */

#include <Arduino.h>
#include "board_config.h"
#include "bootloader_comms.h"
#include "bootloader_dronecan.h"
#include "jump.h"

static void clear_bootcom_magic()
{
    bootcom()->magic = 0;
}

void setup()
{
    pinMode(BOOTLOADER_LED_PIN, OUTPUT);

    volatile struct app_bootloader_comms *bc = bootcom();
    const bool update_requested = (bc->magic == APP_BOOTLOADER_COMMS_MAGIC);
    const uint8_t bootcom_nid   = update_requested ? bc->my_node_id : 0;

    if (!update_requested && app_is_valid(APP_START_ADDRESS)) {
        jump_to_app(APP_START_ADDRESS);
        /* jump_to_app returns only if it changed its mind; fall through to CAN. */
    }

    /* Consume the request so a watchdog reboot during a stalled update
       eventually returns to normal boot. The new BeginFirmwareUpdate from
       the GCS will re-set it. */
    clear_bootcom_magic();

    bl_can_start(bootcom_nid);
}

void loop()
{
    bl_can_cycle();

    if (bl_can_update_done() && app_is_valid(APP_START_ADDRESS)) {
        jump_to_app(APP_START_ADDRESS);
    }
}
