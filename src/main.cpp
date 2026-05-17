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
#include "bootloader_flash.h"
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

    /* Visible failure signal: blink the bootloader error code on the LED.
       Pattern is N short flashes (200 ms on / 200 ms off) followed by a 1 s
       pause, repeating. The code is the bl_flash_err_t value, so 1 blink ==
       BL_FLASH_ERR_BEGIN_ERASE, 2 == NOT_STARTED, etc. See bootloader_flash.h
       for the full mapping. We never jump in this state -- the old vector
       table is still intact at APP_START and would otherwise look "valid". */
    if (bl_can_update_failed()) {
        const uint8_t code = (uint8_t)bootloader_flash_last_error();
        const uint32_t now = millis();
        static uint32_t phase_start;
        static uint8_t  blinks_done;
        static bool     led_on;
        static bool     initialized;
        if (!initialized) {
            initialized = true;
            phase_start = now;
            blinks_done = 0;
            led_on = false;
            digitalWrite(BOOTLOADER_LED_PIN, LOW);
        }
        const uint32_t in_phase = now - phase_start;

        if (blinks_done < code) {
            /* mid-pattern: 200 ms on, 200 ms off */
            if (in_phase < 200 && !led_on) {
                led_on = true;
                digitalWrite(BOOTLOADER_LED_PIN, HIGH);
            } else if (in_phase >= 200 && in_phase < 400 && led_on) {
                led_on = false;
                digitalWrite(BOOTLOADER_LED_PIN, LOW);
            } else if (in_phase >= 400) {
                blinks_done++;
                phase_start = now;
                led_on = false;
            }
        } else {
            /* pause between pattern repeats */
            digitalWrite(BOOTLOADER_LED_PIN, LOW);
            if (in_phase >= 1000) {
                blinks_done = 0;
                phase_start = now;
            }
        }
    }
}
