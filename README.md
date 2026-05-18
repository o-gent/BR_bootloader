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
pio run -e Micro-Node-Bootloader      -t upload   # MicroNode
pio run -e Micro-Node-Plus-Bootloader -t upload   # MicroNodePlus
pio run -e Core-Node-Bootloader       -t upload   # CoreNode
```

A single ST-Link command flashes both the bootloader and your app at the
right addresses. After that, firmware updates run over the CAN bus —
trigger **Begin Firmware Update** in any DroneCAN GUI tool, the app
reboots into the bootloader, the bootloader downloads and flashes the new
image, then jumps to it.

## Building from source (bootloader developers)

You'll need [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html)
and an ST-Link.

```
cd BR_bootloader
pio run -e MicroNode-Bootloader -e MicroNodePlus-Bootloader -e CoreNode-Bootloader
cd ..
python tools/distribute_bins.py
cd br_platformio_hwdef && git diff   # review, commit, push
```

Pushing `br_platformio_hwdef` makes the new bins available to every
downstream user on their next `pio run`.

To flash a freshly-built bootloader directly (e.g. while iterating):

```
cd BR_bootloader
pio run -e <board>-Bootloader -t upload
```

## Brick recovery

If a bad app upload leaves the board unresponsive, erase the app region
via ST-Link (leaves the bootloader intact):

```
# MicroNode (L431, 256 KB total)
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg \
  -c "init; reset halt; stm32l4x mass_erase 0 0x0800a000 0x36000; reset run; exit"

# MicroNodePlus / CoreNode (H7) -- erase from sector 1 onward
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
  -c "init; reset halt; flash erase_sector 0 1 last; reset run; exit"
```

The bootloader then sits on the CAN bus at node ID **100** as
`org.beyondrobotix.bootloader` in MAINTENANCE mode, waiting for a
**Begin Firmware Update** request to recover the board.

If you suspect the bootloader itself is corrupt, full-wipe and reflash:

```
pio run -e <board>-Bootloader -t erase
pio run -e <board>-Bootloader -t upload
```
