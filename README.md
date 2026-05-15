# VROS_2025

Firmware for a Mega 2560-based VROS drawing machine / hanging plotter.

Primary project documentation lives in the Obsidian vault `VBOT Vault` at `~/Documents/VBOT BRAIN/VBOT Vault`; future agents working on this repository should use that vault as the main reference and keep it updated alongside code changes.

## Overview

This project drives a two-motor carriage system using an Arduino Mega 2560. The firmware:

- controls left and right stepper motors
- stores and restores carriage position with EEPROM
- reads drawing instructions from an SD card
- accepts serial commands for movement, calibration, and drawing control

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
- [`src/shapes.ino`](src/shapes.ino): drawing primitives and geometry helpers used by shape-style paths
- [`src/serialPlot.ino`](src/serialPlot.ino): serial plotting/debug helper
- [`drawings/`](drawings): repo-local drawing files that can be streamed over serial
- [`scripts/stream_drawing.py`](scripts/stream_drawing.py): host-side serial streaming tool

## Build And Upload

This is a PlatformIO project.

```bash
~/.platformio/penv/bin/pio run
~/.platformio/penv/bin/pio run --target upload
~/.platformio/penv/bin/pio device monitor
```

If you do not have `pio` installed globally, use the PlatformIO extension in VS Code instead.

The default monitor settings are configured in [platformio.ini](platformio.ini):

- `115200` baud
- local echo enabled
- `LF` line endings so commands match the firmware parser
- `send_on_enter` so pressing Enter sends the command

## State Machine

The firmware uses one top-level handler per machine state:

- `idle`: manual control, file selection, and draw start
- `drawing`: processes one SD instruction per main-loop iteration
- `pausing`: paused drawing that can resume or abort
- `aborting`: one-pass cleanup after abort, completion, or draw error
- `launchpad`: post-home blinking state used after `resetHome`
- `noSD`: degraded mode without an SD card, with periodic SD re-detection

Allowed high-level transitions:

- `setup -> noSD -> idle` when SD initialization succeeds
- `idle -> drawing` via toggle or `drawFromFile,<file>`
- `drawing -> pausing` via toggle or `pause`
- `pausing -> drawing` via toggle or `continue`
- `drawing|pausing -> aborting` via toggle or `abort`
- `aborting -> idle` after file cleanup and any completion handling
- `idle|noSD -> launchpad` via `resetHome`
- `launchpad -> idle` via toggle 4
- `noSD -> idle` when an SD card is detected later

Behavioral notes:

- Serial commands are parsed in one place and queued for the state handlers.
- `drawing` no longer owns nested serial loops.
- `abort` no longer reports completion or forces a return to origin.
- `noSD` is recoverable; inserting an SD card later can move the machine back to `idle`.

## Serial Commands

Examples of supported commands:

```text
drawFromFile,<file>
move,<scan>,<feed>
type,<absolute|relative>
mode,<segmented|movePen>
adjustment,<none|largeSin|complexSin|noise>
stepL,<amount>
stepR,<amount>
setSpeed,<delay>
outlineCanvas
returnToOrigin
returnToHome
resetHome
position
abort
pause
continue
retrySD
monitoring on
monitoring off
```

State-sensitive commands:

- `drawFromFile,<file>` is only accepted from `idle` or `noSD`
- `pause` is only accepted while `drawing`
- `continue` is only accepted while `pausing`
- `abort` is only accepted while `drawing` or `pausing`
- manual commands such as `move`, `stepL`, `stepR`, `returnToOrigin`, and `position` are accepted in non-drawing states

## Streaming Drawings From The Repo

You can keep drawing files in [`drawings/`](drawings) and stream them directly over serial without an SD card.

The recommended file format matches the SD instruction format:

```text
type	absolute
mode	segmented
adjustment	none
move	21,31
move	25,31
move	25,35
move	21,35
move	21,31
```

Use the host-side streamer:

```bash
~/.platformio/penv/bin/python scripts/stream_drawing.py drawings/example.txt --port /dev/cu.usbmodem14401
```

How it works:

- the script opens the serial port, waits for the board to boot, and sends one drawing instruction at a time
- the firmware accepts streamed `move`, `type`, `mode`, and `adjustment` instructions over serial
- after each instruction completes, the firmware replies with `ok`
- the script waits for `ok` before sending the next instruction, so long moves do not overflow the serial buffer

Optional:

- add `--return-to-origin` to send `returnToOrigin` after the streamed file finishes
- add `--repeat 20` to run the same drawing twenty times in a row
- lines starting with `#` and blank lines are ignored by the script

## Interactive Serial Console

If you want a serial-console workflow instead of calling the one-shot streamer directly, use:

```bash
~/.platformio/penv/bin/python scripts/serial_console.py --port /dev/cu.usbmodem14401
```

Inside that console:

- type normal firmware commands such as `position`, `returnToOrigin`, or `drawFromFile,3`
- type `drawFromStream,drawing.txt` to stream [`drawings/drawing.txt`](drawings/drawing.txt)
- type `drawFromStream,drawing.txt,20` to repeat the same repo-local drawing twenty times

Important limitation:

- `drawFromFile,...` is handled by the Arduino and reads from the SD card
- `drawFromStream,...` is handled by the host console and reads from the repo `drawings/` folder
- the Arduino firmware cannot directly open files from your computer, so `drawFromStream` must stay a host-side command

## Notes

- The source is currently organized as multiple `.ino` files. That is intentional and works with Arduino/PlatformIO preprocessing.
- External Arduino libraries may be expected in your local library folder because `platformio.ini` uses `lib_extra_dirs`.
- The firmware appears tailored to one physical VROS machine, so calibration constants should be reviewed before running it on different hardware.
