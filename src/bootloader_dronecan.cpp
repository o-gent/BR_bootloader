#include "bootloader_dronecan.h"
#include "bootloader_flash.h"
#include "bootloader_comms.h"
#include "board_config.h"

#include <dronecan.h>
#include <dronecan_msgs.h>

#define BOOTLOADER_NODE_NAME "org.beyondrobotix.bootloader"

static DroneCAN g_dronecan;
static bool     g_last_chunk_seen = false;
static uint32_t g_last_chunk_offset = 0;

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
        return;
    }

    /* Reply OK to acknowledge the request before kicking off the download. */
    uavcan_protocol_file_BeginFirmwareUpdateResponse reply{};
    reply.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;
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

    /* Per req.source_node_id semantics, the server may delegate to a third
       node; fall back to the request originator if not specified. */
    uint8_t server = req.source_node_id ? req.source_node_id : transfer->source_node_id;

    /* Re-arm flash state (idempotent if already armed). */
    bootloader_flash_begin();
    g_last_chunk_seen   = false;
    g_last_chunk_offset = 0;

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
            /* A short chunk (< 256B) signals EOF in DroneCAN FileRead. */
            uavcan_protocol_file_ReadResponse peek;
            bool decode_ok = (uavcan_protocol_file_ReadResponse_decode(transfer, &peek) == 0);
            uint32_t before = g_dronecan.firmware_download_offset();

            g_dronecan.handle_file_read_response(transfer);

            uint32_t after = g_dronecan.firmware_download_offset();
            if (decode_ok && (after - before) < 256) {
                bootloader_flash_finalize();
                g_last_chunk_offset = after;
                g_last_chunk_seen   = true;
                g_dronecan.firmware_download_finish();
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
}

void bl_can_cycle(void)
{
    g_dronecan.cycle();
    if (g_dronecan.firmware_download_active()) {
        g_dronecan.send_firmware_read();
    }
}

bool bl_can_update_done(void)
{
    return g_last_chunk_seen;
}

bool bl_can_update_in_progress(void)
{
    return g_dronecan.firmware_download_active() || g_last_chunk_seen;
}
