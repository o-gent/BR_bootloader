# BR_bootloader

A PlatformIO + Arduino-framework bootloader for Beyond Robotix STM32 boards.
Supports DroneCAN firmware updates and is binary-compatible with PX4,
Ardupilot, and Arduino-DroneCAN applications.

See [PLAN.md](PLAN.md) for the phased implementation plan and
[SPECIFICATION.md](SPECIFICATION.md) for the design goals.

## Status

| Phase | Scope | Status |
|------|-------|--------|
| 1 | Minimal boot (MicroNode) | Verified on hardware |
| 2 | DroneCAN firmware update (MicroNode) | Builds, HW verification in progress |
| 3 | H7 multi-board (MicroNodePlus, CoreNode) | Not started |
| 4 | Robustness (watchdog, LED patterns) | Not started |
| 5 | Binary distribution | Not started |
| 6 | Firmware signing (optional) | Not started |

## Memory layout

| Region | Address | Size | Notes |
|--------|---------|------|-------|
| Bootloader | `0x08000000` | 40 KB (L4) / 128 KB (H7, planned) | First flash sector(s). |
| App | `0x0800A000` (L4) / `0x08020000` (H7) | Remaining flash | Aligned to a single flash page so erasing the app never touches the bootloader. |
| BOOTCOM | `0x20000000` | 256 B | Shared RAM IPC, magic `0xc544ad9a`. |

## Build & flash

You need [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html) and an ST-Link (V2 or V3).

### Bootloader

```powershell
pio run -e MicroNode-Bootloader -t upload
```

This writes only the first 40 KB of flash, leaving any installed app intact.

### Application

Use the matching `*-Bootloader` env from
[Arduino-DroneCAN](https://github.com/BeyondRobotix/Arduino-DroneCAN) (or the
equivalent linker-aware build in your own project). For the stock test app:

```powershell
cd ..\Arduino-DroneCAN
pio run -e Micro-Node-Bootloader -t upload
```

The `-Bootloader` linker variant places the app at `0x0800A000`, so it boots
*through* this bootloader. An app built for `0x08000000` will overwrite the
bootloader.

## Updating firmware over DroneCAN

Once both bootloader and app are flashed, firmware updates run over the CAN
bus end-to-end:

1. From a DroneCAN GUI tool (e.g.
   [dronecan_gui_tool](https://github.com/DroneCAN/gui_tool)), connect to the
   bus and find the app node (default name from
   [Arduino-DroneCAN](https://github.com/BeyondRobotix/Arduino-DroneCAN)).
2. Send a **Begin Firmware Update** request pointing at your new `.bin`.
3. The app writes BOOTCOM and resets. The bootloader takes over, presents
   itself as `org.beyondrobotix.bootloader` with the same node ID, and pulls
   the file via DroneCAN `file.Read`.
4. When the transfer completes, the bootloader validates and jumps to the
   new image.

If the board resets mid-update (power loss, watchdog), the bootloader sees
the BOOTCOM magic and stays in update mode — re-send the **Begin Firmware
Update** request to resume.

## Recovery commands

### Empty-app recovery

If you flash a broken app or want to force the bootloader to stay in CAN
mode indefinitely, erase the app region (everything from `0x0800A000`
onward, leaving the bootloader at `0x08000000` untouched):

```powershell
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "init; reset halt; stm32l4x mass_erase 0 0x0800a000 0x36000; reset run; exit"
```

For MicroNode (L431CC, 256 KB total): app region is 216 KB (`0x36000`)
starting at `0x0800A000`. Adjust length for other L4 variants.

After erase the bootloader will boot, find no valid app, and sit on the CAN
bus waiting for a `BeginFirmwareUpdate`. Use the DroneCAN GUI tool to push a
new image (see above).

### Full chip erase (nuclear option)

If you suspect the bootloader itself is corrupt, wipe everything and re-flash
from scratch:

```powershell
pio run -e MicroNode-Bootloader -t erase
pio run -e MicroNode-Bootloader -t upload
```

Then re-flash the app via DroneCAN or ST-Link.

## Repository layout

```
BR_bootloader/
├── platformio.ini             # MicroNode-Bootloader env
├── linker/
│   └── bootloader_l4.ld       # 40 KB @ 0x08000000, BOOTCOM @ 0x20000000
├── src/
│   ├── main.cpp               # Boot decision + DroneCAN loop
│   ├── board_config.h         # Per-MCU constants (CANL431, CANH7 stubs)
│   ├── bootloader_comms.h     # BOOTCOM struct (mirrors app/AP_Periph)
│   ├── jump.h / jump.cpp      # VTOR + SP swap into the app
│   ├── bootloader_flash.h/.cpp     # Append-only flash writer for FileRead chunks
│   └── bootloader_dronecan.h/.cpp  # Custom canard dispatcher + update server
├── lib/                       # Vendored from Arduino-DroneCAN
│   ├── libcanard/             #   unmodified
│   ├── dronecan/              #   unmodified DSDL headers
│   └── libArduinoDroneCAN/    #   additive patches (see CHANGES below)
├── AP_Bootloader/             # In-repo reference, not built
└── PX4-Bootloader-main/       # In-repo reference, not built
```

## Patches to vendored libArduinoDroneCAN

All patches under `lib/libArduinoDroneCAN/` are additive and intended to be
PR'd back to mainline:

- **`dronecan.h/cpp`**: new bare `init(handler, accept, name, preferred_node_id)`
  overload that skips parameter-storage setup; `set_firmware_write_callback()`
  hook; `begin_firmware_download()` + accessors so a bootloader can drive the
  existing `send_firmware_read()` / `handle_file_read_response()` machinery.
- **`dronecan.cpp`** bug fix: `handle_file_read_response` previously discarded
  `pkt.data` entirely. It now invokes the registered write callback before
  advancing the offset.
- **`canL431.cpp`**: removed CAN-init and send-failure `Serial.println` debug
  prints.
