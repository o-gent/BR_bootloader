# BR_bootloader

PlatformIO + Arduino-framework, AP_Periph compaible, bootloader for Beyond Robotix STM32 boards.
Lets Arduino-DroneCAN apps update themselves over the CAN bus.

If you're developing an ArduinoDroneCAN app, you don't need this repo, the bootloader binaries are managed within the platformio.ini of [ArduinoDroneCAN](https://github.com/BeyondRobotix/Arduino-DroneCAN)

Supported boards: **MicroNode** (STM32L431), **MicroNodePlus** (STM32H723),
**CoreNode** (STM32H743).

## Building from source (bootloader developers)

To flash a freshly-built bootloader directly to a board:

```
cd BR_bootloader
pio run -e <board>-Bootloader -t upload
```
