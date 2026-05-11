# VROS_2025

Firmware for a Mega 2560-based VROS drawing machine / hanging plotter.

## Overview

This project drives a two-motor carriage system using an Arduino Mega 2560. The firmware:

- controls left and right stepper motors
- stores and restores carriage position with EEPROM
- reads drawing instructions from an SD card
- accepts serial commands for movement, calibration, and drawing control
- includes a built-in drawing library for generating patterns on the SD card

The active PlatformIO environment is defined in [platformio.ini](platformio.ini) for `megaatmega2560`.

## Hardware Assumptions

The current firmware is configured for a specific physical machine setup:

- Arduino Mega 2560
- two stepper drivers and motors
- SD card module
- rotary inputs on `A14` and `A15`
- toggle switches and status LEDs on the pins defined in [`src/VROS_2.5.3_coilCalibration.ino`](src/VROS_2.5.3_coilCalibration.ino)

Machine geometry and calibration values are hard-coded in the main sketch, including:

- motor spacing
- scan/feed offsets
- home position
- steps-per-centimeter conversion
- left/right coil compensation

## Project Structure

- [`src/VROS_2.5.3_coilCalibration.ino`](src/VROS_2.5.3_coilCalibration.ino): main entry point, setup, loop, state machine, geometry, SD and EEPROM helpers
- [`src/controller.ino`](src/controller.ino): serial command parser
- [`src/movePenSegmented.ino`](src/movePenSegmented.ino): segmented motion algorithm
- [`src/movePen.ino`](src/movePen.ino): alternate direct motion algorithm
- [`src/drawingLibrary.ino`](src/drawingLibrary.ino): built-in pattern generators
- [`src/shapes.ino`](src/shapes.ino): drawing primitives
- [`src/serialPlot.ino`](src/serialPlot.ino): serial plotting/debug helper

## Build And Upload

This is a PlatformIO project.

```bash
pio run
pio run --target upload
pio device monitor
```

If you do not have `pio` installed globally, use the PlatformIO extension in VS Code instead.

## Serial Commands

Examples of supported commands:

```text
drawFromFile,<file>
writeToFile,<drawing>,<file>
move,<scan>,<feed>
stepL,<amount>
stepR,<amount>
setSpeed,<delay>
returnToOrigin
returnToHome
resetHome
position
abort
monitoring on
monitoring off
```

## Notes

- The source is currently organized as multiple `.ino` files. That is intentional and works with Arduino/PlatformIO preprocessing.
- External Arduino libraries may be expected in your local library folder because `platformio.ini` uses `lib_extra_dirs`.
- The firmware appears tailored to one physical VROS machine, so calibration constants should be reviewed before running it on different hardware.
