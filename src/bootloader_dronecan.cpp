#include "bootloader_dronecan.h"
#include "bootloader_flash.h"
#include "bootloader_comms.h"
#include "board_config.h"

#include <dronecan.h>
#include <dronecan_msgs.h>
#include <stdarg.h>
#include <stdio.h>

#define BOOTLOADER_NODE_NAME "org.beyondrobotix.bootloader"

/* DroneCAN LogMessage level constants -- match uavcan.protocol.debug.LogLevel. */
#define LOG_DEBUG   0
#define LOG_INFO    1
#define LOG_WARNING 2
#define LOG_ERROR   3

static DroneCAN g_dronecan;
static bool     g_last_chunk_seen = false;
static uint32_t g_last_chunk_offset = 0;
static bl_flash_err_t g_logged_err = BL_FLASH_OK;
static uint32_t g_last_progress_log_kb = 0;

/* Format a short message and broadcast via DroneCAN debug.LogMessage. The
   wire-level limit is ~90 bytes; we cap the buffer well below that. */
static void bl_logf(uint8_t level, const char *fmt, ...)
{
    char msg[80];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    g_dronecan.debug(msg, level);
}

/*
 * BeginFirmwareUpdate handler (server role).
 *
 * The app-side handler in dronecan.cpp writes BOOTCOM and resets. In the
 * bootloader we are *already* the responder, so we:
 *   - decode the request,
 *   - reply OK,
 *   - prime the lib's FileRead client with (server_node_id, image_path).
 *
 * Subsequent bl_can_cycle() calls drive send_firmware_read(), and chunks
 * are persisted via the firmware_write_cb registered in bl_can_start().
 */
static void handle_begin_firmware_update_server(CanardInstance *ins, CanardRxTransfer *transfer)
{
    uavcan_protocol_file_BeginFirmwareUpdateRequest req;
    if (uavcan_protocol_file_BeginFirmwareUpdateRequest_decode(transfer, &req)) {
        bl_logf(LOG_ERROR, "BeginFirmwareUpdate: decode failed");
        return;
    }

    bl_logf(LOG_INFO, "BeginFirmwareUpdate src=%u plen=%u",
            (unsigned)transfer->source_node_id,
            (unsigned)req.image_file_remote_path.path.len);

    /* Arm flash state first; if the page erase fails we surface the error in
       the response instead of silently dropping every chunk that follows. */
    bool flash_ready = bootloader_flash_begin();
    g_last_chunk_seen     = false;
    g_last_chunk_offset   = 0;
    g_last_progress_log_kb = 0;
    g_logged_err          = BL_FLASH_OK;

    if (!flash_ready) {
        bl_logf(LOG_ERROR, "flash begin FAILED err=%u",
                (unsigned)bootloader_flash_last_error());
        struct bl_flash_diag d;
        bootloader_flash_get_diag(&d);
        /* Top byte of ACR is repurposed as a path tag: 0xAA=raw worked,
           0xBB=HAL worked, 0xFF=neither (and that's why we're here). */
        bl_logf(LOG_ERROR, "FLASH sr=%08lx cr=%08lx",
                (unsigned long)d.sr, (unsigned long)d.cr);
        bl_logf(LOG_ERROR, "OPTR=%08lx WRP1A=%08lx WRP1B=%08lx",
                (unsigned long)d.optr,
                (unsigned long)d.wrp1ar,
                (unsigned long)d.wrp1br);
        bl_logf(LOG_ERROR, "PCROP1S=%08lx PCROP1E=%08lx ACR=%08lx",
                (unsigned long)d.pcrop1sr,
                (unsigned long)d.pcrop1er,
                (unsigned long)d.acr);
    } else {
        bl_logf(LOG_INFO, "flash begin OK, starting download");
    }

    uavcan_protocol_file_BeginFirmwareUpdateResponse reply{};
    reply.error = flash_ready
        ? UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK
        : UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_UNKNOWN;
    uint8_t buf[UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE];
    uint16_t len = uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(&reply, buf);

    CanardTxTransfer reply_xfer = {
        .transfer_type       = CanardTransferTypeResponse,
        .data_type_signature = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE,
        .data_type_id        = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID,
        .inout_transfer_id   = &transfer->transfer_id,
        .priority            = transfer->priority,
        .payload             = buf,
        .payload_len         = len,
    };
    canardRequestOrRespondObj(ins, transfer->source_node_id, &reply_xfer);

    /* If flash erase failed, don't start the FileRead client -- the bootloader
       will stay alive on the bus so the operator can see it and retry. */
    if (!flash_ready) {
        return;
    }

    /* Per req.source_node_id semantics, the server may delegate to a third
       node; fall back to the request originator if not specified. */
    uint8_t server = req.source_node_id ? req.source_node_id : transfer->source_node_id;

    char path[201];
    uint16_t plen = req.image_file_remote_path.path.len;
    if (plen > sizeof(path) - 1) {
        plen = sizeof(path) - 1;
    }
    memcpy(path, req.image_file_remote_path.path.data, plen);
    path[plen] = 0;

    g_dronecan.begin_firmware_download(server, path);

    /* Mirror app-side BOOTCOM convention so a watchdog/reset mid-update
       still resumes in bootloader mode. */
    volatile struct app_bootloader_comms *bc = bootcom();
    bc->magic          = APP_BOOTLOADER_COMMS_MAGIC;
    bc->server_node_id = server;
    bc->my_node_id     = canardGetLocalNodeID(&g_dronecan.canard);
    memcpy((void *)bc->path, path, plen);
    bc->path[plen] = 0;
}

/*
 * Custom dispatcher. Routes the small set of services the bootloader cares
 * about; everything else is ignored (lighter than pulling in the full app-side
 * dispatcher's param/restart handlers).
 */
static void bl_on_transfer_received(CanardInstance *ins, CanardRxTransfer *transfer)
{
    switch (transfer->transfer_type) {
    case CanardTransferTypeBroadcast:
        if (transfer->data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID) {
            g_dronecan.handle_DNA_Allocation(transfer);
        }
        break;

    case CanardTransferTypeRequest:
        if (transfer->data_type_id == UAVCAN_PROTOCOL_GETNODEINFO_ID) {
            g_dronecan.handle_GetNodeInfo(transfer);
        } else if (transfer->data_type_id == UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID) {
            handle_begin_firmware_update_server(ins, transfer);
        }
        break;

    case CanardTransferTypeResponse:
        if (transfer->data_type_id == UAVCAN_PROTOCOL_FILE_READ_ID) {
            uavcan_protocol_file_ReadResponse peek;
            bool decode_ok = (uavcan_protocol_file_ReadResponse_decode(transfer, &peek) == 0);
            uint32_t before = g_dronecan.firmware_download_offset();

            g_dronecan.handle_file_read_response(transfer);

            uint32_t after = g_dronecan.firmware_download_offset();

            /* progress log every 8 KB so the bus isn't drowned in chatter */
            uint32_t kb = after / 1024;
            if (kb >= g_last_progress_log_kb + 8) {
                g_last_progress_log_kb = kb;
                bl_logf(LOG_DEBUG, "rx %u KB written=%u",
                        (unsigned)kb,
                        (unsigned)bootloader_flash_bytes_written());
            }

            /* A short chunk (< 256B) signals EOF in DroneCAN FileRead. */
            if (decode_ok && (after - before) < 256) {
                bool fin_ok = bootloader_flash_finalize();
                g_last_chunk_offset = after;
                g_last_chunk_seen   = true;
                g_dronecan.firmware_download_finish();
                if (fin_ok && !bootloader_flash_failed()) {
                    bl_logf(LOG_INFO, "download done bytes=%u",
                            (unsigned)bootloader_flash_bytes_written());
                } else {
                    bl_logf(LOG_ERROR, "download FAILED err=%u after=%u",
                            (unsigned)bootloader_flash_last_error(),
                            (unsigned)after);
                }
            }
        }
        break;
    }
}

/* libcanard adapter -> our dispatcher, using user_reference for context. */
static void canard_dispatch(CanardInstance *ins, CanardRxTransfer *transfer)
{
    bl_on_transfer_received(ins, transfer);
}

void bl_can_start(uint8_t preferred_node_id)
{
    g_dronecan.version_major = 0;
    g_dronecan.version_minor = 2;  /* Phase 2 */

    g_dronecan.set_firmware_write_callback(bootloader_flash_write_cb);

    g_dronecan.init(canard_dispatch,
                    DroneCANshouldAcceptTransfer,
                    BOOTLOADER_NODE_NAME,
                    preferred_node_id);

    /* One-shot startup banner so we can confirm the bootloader booted cleanly
       and which node ID it ended up with. */
    bl_logf(LOG_INFO, "bootloader up, preferred_nid=%u",
            (unsigned)preferred_node_id);
}

void bl_can_cycle(void)
{
    g_dronecan.cycle();
    if (g_dronecan.firmware_download_active()) {
        g_dronecan.send_firmware_read();
    }

    /* Report a flash failure the first time we observe it. The failure may
       have been set deep inside the write callback (handle_file_read_response
       chain) where calling debug() would re-enter the canard tx queue --
       better to surface it here, after the response transfer is fully drained. */
    bl_flash_err_t err = bootloader_flash_last_error();
    if (err != BL_FLASH_OK && err != g_logged_err) {
        g_logged_err = err;
        bl_logf(LOG_ERROR, "flash op FAILED err=%u written=%u",
                (unsigned)err,
                (unsigned)bootloader_flash_bytes_written());

        /* Flag the failure in NodeStatus so the GCS node list shows ERROR
           health and MAINTENANCE mode instead of looking healthy while
           silently refusing to jump. VSSC carries the error code. */
        g_dronecan.set_node_status_override(
            UAVCAN_PROTOCOL_NODESTATUS_HEALTH_ERROR,
            UAVCAN_PROTOCOL_NODESTATUS_MODE_MAINTENANCE,
            (uint16_t)err);
    }
}

bool bl_can_update_done(void)
{
    /* "Done" only if we saw EOF *and* every flash op along the way succeeded.
       If a write was dropped, we'd otherwise jump to the still-intact old
       vector table and the old firmware would run, masking the failure. */
    return g_last_chunk_seen && !bootloader_flash_failed();
}

bool bl_can_update_failed(void)
{
    return bootloader_flash_failed();
}

bool bl_can_update_in_progress(void)
{
    return g_dronecan.firmware_download_active() || g_last_chunk_seen;
}
