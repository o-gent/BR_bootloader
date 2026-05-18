/*
 * Bootloader-side DroneCAN integration.
 *
 * Wraps the vendored DroneCAN class with bootloader-appropriate behavior:
 *   - Custom canard dispatcher (handles BeginFirmwareUpdate as a server, routes
 *     FileRead responses to the lib's existing client).
 *   - Single global instance to avoid C++ static init order surprises.
 *   - Pumps the file-read state machine; emits NodeStatus while idle so the
 *     bootloader is discoverable from a GCS / DroneCAN GUI tool.
 *
 * Flow:
 *   bl_can_start(preferred_node_id) -> CANInit, canard, GetNodeInfo identity
 *   bl_can_cycle()                  -> call frequently from main loop
 *   bl_can_update_done()            -> true once the server signals EOF
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Bring up CAN and identify on the bus.
 *   preferred_node_id  -- non-zero to reuse a node ID handed over from the
 *                         app via BOOTCOM (skips a DNA round-trip).
 *   iwdg_reset         -- true if previous boot was killed by the watchdog;
 *                         logged on the startup banner so the operator knows
 *                         the bootloader is here because the app crashed.
 */
void bl_can_start(uint8_t preferred_node_id, bool iwdg_reset);
void bl_can_cycle(void);

/* True once a download has completed (server returned a short/empty chunk). */
bool bl_can_update_done(void);

/* True between BeginFirmwareUpdate receipt and update_done. Used by main()
   to decide whether to short-circuit a jump while an update is in flight. */
bool bl_can_update_in_progress(void);

/* True if any flash operation has failed since the last begin().
   When set, main() should refuse to jump (the old vector table is still at
   APP_START and would otherwise look valid) and signal the error visibly. */
bool bl_can_update_failed(void);

#ifdef __cplusplus
}
#endif
