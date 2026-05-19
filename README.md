# VROS_2025

Firmware for a Mega 2560-based VROS drawing machine / hanging plotter.

Primary project documentation lives in the Obsidian vault `VBOT Vault` at `~/Documents/03_VAULTS/VBOT Vault`; future agents working on this repository should use that vault as the main reference and keep it updated alongside code changes.

Workspace mapping for future agents lives in [`PROJECT_CONTEXT.md`](PROJECT_CONTEXT.md) and [`project-context.json`](project-context.json).

## Overview

This project drives a two-motor carriage system using an Arduino Mega 2560. The firmware:

- controls left and right stepper motors
- stores and restores carriage position with EEPROM
- stores robot-specific geometry and calibration values in EEPROM
- reads drawing instructions from an SD card
- accepts serial commands for movement, calibration, and drawing control
- exposes a Control Station-oriented serial protocol for robot setup, EEPROM status, and diagnostics

The active PlatformIO environment is defined in [platformio.ini](platformio.ini) for `megaatmega2560`.

## Important Changes

This firmware now supports a desktop-driven robot setup workflow instead of relying purely on hard-coded machine geometry.

Major additions made in this iteration:

- versioned robot-setup EEPROM storage with CRC validation
- separate versioned carriage-position EEPROM storage
- serial commands for reading, writing, loading, defaulting, and clearing EEPROM-backed robot setup
- explicit reporting of robot setup EEPROM validity over serial
- Control Station compatibility that now depends on an exact firmware version match

This is an important protocol contract:

- when firmware behavior changes in a way the desktop app depends on, the firmware version string must be incremented
- the desktop app is expected to require that exact firmware version
- this prevents the Control Station from silently talking to a robot that has an older or incompatible protocol

## Hardware Assumptions

The current firmware is configured for a specific physical machine setup:

- Arduino Mega 2560
- two stepper drivers and motors
- SD card module
- rotary inputs on `A14` and `A15`
- toggle switches and status LEDs on the pins defined in [`src/VROS_2.5.3_coilCalibration.ino`](src/VROS_2.5.3_coilCalibration.ino)

The firmware still contains a compiled default robot profile, but those values are now also writable to EEPROM and can be managed from the Control Station. The robot setup includes:

- motor spacing
- scan/feed offsets
- canvas height
- line resolution
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

## Versioning And Compatibility

The firmware boot banner is the compatibility identifier used by the Control Station.

Current firmware banner:

```text
VROS_2.5.4_caseController
```

Important rule:

- any firmware change that affects EEPROM layout, serial protocol, Control Station handshake, robot setup workflow, or machine-state reporting must increment this version string

Why this matters:

- the Control Station now treats the firmware version as a compatibility contract
- legacy firmware is rejected
- mismatched newer/older firmware is also rejected
- the operator is told to flash the required version instead of getting partial or misleading behavior

This should be treated as an operational rule, not as optional cleanup.

## Robot Setup EEPROM

The firmware now maintains two EEPROM concepts:

1. Robot setup block
2. Position block

Robot setup block:

- stores machine geometry and calibration values
- is versioned
- is CRC-protected
- is read by the Control Station after connection

Position block:

- stores the last known carriage position
- is versioned
- is CRC-protected
- still supports migration from the previous legacy position addresses

Robot setup fields now persisted in EEPROM:

- `motorDistance`
- `scanOffset`
- `feedOffset`
- `height`
- `lineResolution`
- `homePosition`
- `leftCoilFeed`
- `rightCoilFeed`
- `stepsToCm`

Derived values that are not stored directly:

- `width`
- runtime-adjusted `stepsToCm`
- `stepLength`

## Invalid Robot Setup EEPROM Behavior

This behavior changed deliberately during this chat and is important.

Current behavior for an invalid or missing robot setup block:

- the firmware does **not** automatically repair EEPROM
- the firmware loads the compiled default setup into RAM so the machine can still boot
- the firmware reports:

```text
robotSetupStatus	invalid
```

- the Control Station is expected to prompt the operator to burn a valid robot setup

This is intentional because silent auto-repair hides configuration mistakes and makes it hard to know whether a robot is actually configured.

Position EEPROM behavior is different:

- the firmware still attempts to migrate legacy carriage position values into the new position block
- if that fails, position falls back to origin-like defaults

## Control Station Handshake

The Control Station now expects the following connection model:

1. robot boots and prints the exact required firmware banner
2. Control Station verifies the banner
3. Control Station requests `robotSetupGet`
4. firmware returns `robotSetupStatus` and the active robot setup fields
5. Control Station decides whether the EEPROM is valid, invalid, or the firmware is incompatible

Failure cases:

- legacy firmware without robot setup commands: connection refused
- wrong firmware version: connection refused
- invalid robot setup EEPROM: connection allowed, operator prompted to burn setup

## Robot Setup Commands

These commands were added or formalized for Control Station support:

```text
robotSetupGet
robotSetupWrite	<motorDistance>,<scanOffset>,<feedOffset>,<height>,<lineResolution>,<homePosition>,<leftCoilFeed>,<rightCoilFeed>,<stepsToCm>
robotSetupLoad
robotSetupDefaults
clearEEPROM
```

Command intent:

- `robotSetupGet`: report EEPROM validity and the active runtime setup
- `robotSetupWrite`: apply a full setup payload and persist it
- `robotSetupLoad`: reload the EEPROM setup block into runtime state
- `robotSetupDefaults`: write compiled defaults into EEPROM and make them active
- `clearEEPROM`: wipe the full EEPROM, including robot setup and saved carriage position

After setup-changing commands, the firmware also reports current position so the Control Station preview can update immediately.

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
robotSetupGet
robotSetupWrite	62,10,20,50,0.5,82,1,0.997,35
robotSetupLoad
robotSetupDefaults
clearEEPROM
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
- The firmware appears tailored to one physical VROS machine, but robot-specific setup is now expected to be managed through EEPROM and the Control Station instead of source edits alone.
- If you change firmware behavior that the Control Station depends on, increment the firmware version banner and update the Control Station required version in lockstep.
