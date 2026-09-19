# Pico W Project

This is a minimal Raspberry Pi Pico W project based on the core files from `pico-examples`.

## Requirements

- CMake
- Ninja
- ARM GNU toolchain
- Raspberry Pi Pico SDK

On Windows, install CMake, Ninja, and the Arm GNU toolchain, then ensure all three
commands are available in the VS Code terminal's `PATH`.

The included CMake preset uses the SDK installed at:

`C:\Users\Jason Lai\.pico-sdk\sdk\2.3.0`

To use another SDK location, set `PICO_SDK_PATH` or change `CMakePresets.json`.

## Build

From this folder in PowerShell:

```powershell
cmake --preset pico-w
cmake --build --preset pico-w
```

The Pico W firmware is generated at:

`build\pico_w_project.uf2`

## Flash

1. Hold the Pico W `BOOTSEL` button while connecting it over USB.
2. Copy `build\pico_w_project.uf2` to the `RPI-RP2` drive.
3. The board reboots and starts blinking its onboard LED.

Alternatively, after holding `BOOTSEL` while connecting the board, run the VS Code
task `Flash Pico W`. It finds the mounted `RPI-RP2` drive and copies the UF2 automatically.
