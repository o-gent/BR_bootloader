# BR_bootloader

PlatformIO + Arduino-framework bootloader for Beyond Robotix STM32 boards.
Lets Arduino-DroneCAN apps update themselves over the CAN bus.

Supported boards: **MicroNode** (STM32L431), **MicroNodePlus** (STM32H723),
**CoreNode** (STM32H743).

## Using it (app developers)

If you're writing an Arduino-DroneCAN app, you don't need to build this
repo. The compiled bootloader binaries ship inside the `br-boards`
platform_package that Arduino-DroneCAN already depends on. Just use the
matching `-Bootloader` env:

```
cd Arduino-DroneCAN
pio run -e Micro-Node-App      -t upload   # MicroNode
pio run -e Micro-Node-Plus-App -t upload   # MicroNodePlus
pio run -e Core-Node-App       -t upload   # CoreNode
```

A single ST-Link command flashes both the bootloader and your app at the
right addresses. After that, firmware updates run over the CAN bus —
trigger **Begin Firmware Update** in any DroneCAN GUI tool, the app
reboots into the bootloader, the bootloader downloads and flashes the new
image, then jumps to it.

## Building from source (bootloader developers)

To flash a freshly-built bootloader directly (e.g. while iterating):

```
cd BR_bootloader
pio run -e <board>-Bootloader -t upload
```