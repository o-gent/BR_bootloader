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
 * Snapshot the IWDG reset cause flag and clear all reset flags so the next
 * boot starts fresh. Captured before any other setup so we don't lose the
 * flags to an intervening peripheral init that might clear them. An IWDG
 * reset indicates the previous boot was killed by the watchdog -- treat
 * that as an app crash and stay in the bootloader.
 *
 * Register layout differs between MCU families:
 *   L4: RCC->CSR.IWDGRSTF (bit 29), cleared by writing RMVF
 *   H7: RCC->RSR.IWDG1RSTF (different bit), cleared by writing RMVF
 */
static bool g_app_crashed_via_iwdg;

static void capture_reset_cause()
{
#if defined(STM32H7xx) || defined(STM32H7)
    g_app_crashed_via_iwdg = (RCC->RSR & RCC_RSR_IWDG1RSTF) != 0;
    RCC->RSR |= RCC_RSR_RMVF;
#else
    g_app_crashed_via_iwdg = (RCC->CSR & RCC_CSR_IWDGRSTF) != 0;
    RCC->CSR |= RCC_CSR_RMVF;
#endif
}

void setup()
{
    capture_reset_cause();
#ifdef BOOTLOADER_LED_PIN
    pinMode(BOOTLOADER_LED_PIN, OUTPUT);
#endif

    volatile struct app_bootloader_comms *bc = bootcom();
    const bool update_requested = (bc->magic == APP_BOOTLOADER_COMMS_MAGIC);
    /* If the app handed us a *valid* node ID via BOOTCOM, reuse it. Otherwise
       fall back to a fixed default so the bootloader is immediately
       discoverable on the bus -- we don't want to depend on a GCS-side DNA
       allocator just to recover from a bricked update. A handed-over ID of
       zero is the common failure case: it means the app received
       BeginFirmwareUpdate before its own DNA had completed (typically on
       first boot after flashing), so my_node_id was still 0 (broadcast).
       Without this guard the bootloader would silently sit invisible on the
       bus forever -- canard refuses to broadcast NodeStatus without a node
       ID, but no error fires. */
    uint8_t bootcom_nid = BL_FALLBACK_NODE_ID;
    if (update_requested && bc->my_node_id > 0 && bc->my_node_id <= 127) {
        bootcom_nid = bc->my_node_id;
    }

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
 *
 * When BOOTLOADER_LED_PIN is not defined (e.g. CoreNode currently has no
 * on-board LED), the function compiles to a no-op. The DroneCAN
 * debug.LogMessage stream and the NodeStatus health/mode override remain
 * as the diagnostic channels in that case.
 */
#ifdef BOOTLOADER_LED_PIN
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
#else
static inline void drive_led() {}
#endif

void loop()
{
    bl_can_cycle();

    if (bl_can_update_done() && app_is_valid(APP_START_ADDRESS)) {
        /* Clear BOOTCOM magic before handing control to the freshly-installed
           app. The BeginFirmwareUpdate handler set magic so a reset mid-update
           would resume here; now that the update completed cleanly that
           pending-resume flag must be cleared, otherwise the next NRST press
           (which preserves RAM on STM32) would re-enter the bootloader from
           a perfectly healthy app. */
        clear_bootcom_magic();
        jump_to_app(APP_START_ADDRESS);
    }

    drive_led();
}
