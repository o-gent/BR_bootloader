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

/*
 * Snapshot RCC->CSR reset cause flags and clear them so the next boot starts
 * fresh. Captured before any other setup so we don't lose the flags to an
 * intervening peripheral init that might clear them. IWDGRSTF indicates the
 * previous boot was killed by the independent watchdog -- treat that as an
 * app crash and stay in the bootloader.
 */
static bool g_app_crashed_via_iwdg;

static void capture_reset_cause()
{
    g_app_crashed_via_iwdg = (RCC->CSR & RCC_CSR_IWDGRSTF) != 0;
    RCC->CSR |= RCC_CSR_RMVF;
}

void setup()
{
    capture_reset_cause();
    pinMode(BOOTLOADER_LED_PIN, OUTPUT);

    volatile struct app_bootloader_comms *bc = bootcom();
    const bool update_requested = (bc->magic == APP_BOOTLOADER_COMMS_MAGIC);
    /* If the app handed us a node ID via BOOTCOM, reuse it. Otherwise fall
       back to a fixed default so the bootloader is immediately discoverable
       on the bus after a cold boot -- we don't want to depend on a GCS-side
       DNA allocator being running just to recover from a bricked update. */
    const uint8_t bootcom_nid   = update_requested ? bc->my_node_id
                                                   : BL_FALLBACK_NODE_ID;

    /* Stay in the bootloader if the app explicitly requested it (BOOTCOM)
       OR if the previous boot died to a watchdog reset (app crash, hang). */
    const bool stay_in_bootloader = update_requested || g_app_crashed_via_iwdg;

    if (!stay_in_bootloader && app_is_valid(APP_START_ADDRESS)) {
        jump_to_app(APP_START_ADDRESS);
        /* jump_to_app returns only if it changed its mind; fall through to CAN. */
    }

    /* Consume the request so a watchdog reboot during a stalled update
       eventually returns to normal boot. The new BeginFirmwareUpdate from
       the GCS will re-set it. */
    clear_bootcom_magic();

    bl_can_start(bootcom_nid, g_app_crashed_via_iwdg);
}

/*
 * Bootloader LED state machine.
 *
 *   FAILED:      N short flashes (200 ms on/off) + 1 s pause; N = bl_flash_err_t.
 *                Never jumps -- the operator needs to see the error code.
 *   UPDATING:    fast even blink at ~5 Hz (100 ms on/off). "Something is
 *                actively happening."
 *   IDLE:        slow 1 Hz heartbeat (1000 ms on/off). "Bootloader alive,
 *                waiting for a BeginFirmwareUpdate."
 */
static void drive_led()
{
    const uint32_t now = millis();
    static uint32_t toggle_at;
    static bool     led_on;

    if (bl_can_update_failed()) {
        /* Error-code blink pattern -- preserved from earlier diagnostic work. */
        const uint8_t code = (uint8_t)bootloader_flash_last_error();
        static uint32_t phase_start;
        static uint8_t  blinks_done;
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
            digitalWrite(BOOTLOADER_LED_PIN, LOW);
            if (in_phase >= 1000) {
                blinks_done = 0;
                phase_start = now;
            }
        }
        return;
    }

    const uint32_t period = bl_can_update_in_progress() ? 100 : 1000;
    if ((int32_t)(now - toggle_at) >= 0) {
        led_on = !led_on;
        digitalWrite(BOOTLOADER_LED_PIN, led_on);
        toggle_at = now + period;
    }
}

void loop()
{
    bl_can_cycle();

    if (bl_can_update_done() && app_is_valid(APP_START_ADDRESS)) {
        jump_to_app(APP_START_ADDRESS);
    }

    drive_led();
}
