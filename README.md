# VROS_2025

Firmware for a Mega 2560-based VROS drawing machine, covering both the hanging two-motor V-bot and the flat four-motor tension quad.

Primary project documentation lives in the Obsidian vault `VBOT Vault` at `~/Documents/03_VAULTS/VBOT Vault`; future agents working on this repository should use that vault as the main reference and keep it updated alongside code changes.

Workspace mapping for future agents lives in [`PROJECT_CONTEXT.md`](PROJECT_CONTEXT.md) and [`project-context.json`](project-context.json).

## Overview

This project drives a robot carriage system using an Arduino Mega 2560. The firmware:

- controls either two or four stepper motors depending on the active robot kind
- stores and restores carriage position with EEPROM
- stores robot-specific geometry and calibration values in EEPROM
- reads drawing instructions from an SD card
- accepts serial commands for movement, calibration, and drawing control
- exposes a Control Station-oriented serial protocol for robot setup, EEPROM status, 3D quad telemetry, and diagnostics

The active PlatformIO environment is defined in [platformio.ini](platformio.ini) for `megaatmega2560`.

## Important Changes

This firmware now supports a desktop-driven robot setup workflow instead of relying purely on hard-coded machine geometry.

Major additions made in this iteration:

- versioned robot-setup EEPROM storage with CRC validation
- separate versioned carriage-position EEPROM storage
- serial commands for reading, writing, loading, defaulting, and clearing EEPROM-backed robot setup
- explicit reporting of robot setup EEPROM validity over serial
- Control Station compatibility that now depends on an exact firmware version match
- robot-kind-aware setup storage and reporting for both `hanging_vbot` and `flat_quad_tension`
- quad telemetry reporting for `robotKind`, `position` / `positionZ`, `target` / `targetZ`, and four `cableLengths`
- a single Bresenham-based motion engine that replaces the older `segmented` and `movePen` implementations
- a Timer1-driven pulse backend shared by coordinated Bresenham motion and manual `stepL` / `stepR` commands

This is an important protocol contract:

- when firmware behavior changes in a way the desktop app depends on, the firmware version string must be incremented
- the desktop app is expected to require that exact firmware version
- this prevents the Control Station from silently talking to a robot that has an older or incompatible protocol

## Hardware Assumptions

The current firmware is configured for a specific physical machine setup:

- Arduino Mega 2560
- stepper drivers and motors for either the hanging two-axis path or the flat quad four-axis path
- SD card module
- rotary inputs on `A14` and `A15`
- toggle switches and status LEDs on the pins defined in [`src/VROS_2.5.3_coilCalibration.ino`](src/VROS_2.5.3_coilCalibration.ino)

The firmware still contains a compiled default robot profile, but those values are now also writable to EEPROM and can be managed from the Control Station. The robot setup includes:

- motor spacing
- scan/feed offsets
- canvas height
- line resolution used for geometric resampling before Bresenham step scheduling
- home position
- steps-per-centimeter conversion
- RAMPS microstep scaling
- left/right coil compensation

For the flat quad path, the setup also carries the quad home point, per-cable feed compensation, and a shared `quadMotorHeight` that defines the logical `z` range from the table (`z=0`) to the motor plane.

## Project Structure

- [`src/VROS_2.5.3_coilCalibration.ino`](src/VROS_2.5.3_coilCalibration.ino): main entry point, setup, loop, state machine, geometry, SD and EEPROM helpers
- [`src/controller.ino`](src/controller.ino): serial command parser
- [`src/movePenBresenham.ino`](src/movePenBresenham.ino): Bresenham-based motion engine, Timer1 pulse scheduling, motion-mode normalization, and per-segment step planning
- [`src/shapes.ino`](src/shapes.ino): drawing primitives and geometry helpers used by shape-style paths
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

The firmware now exposes a structured `vros` compatibility handshake for the Control Station.

Current firmware banner:

```text
VROS_2.5.17_caseController
```

Current host protocol version:

```text
1
```

Important rule:

- any firmware change that affects EEPROM layout, serial protocol, Control Station handshake, robot setup workflow, or machine-state reporting must increment this version string

Why this matters:

- the Control Station now treats the firmware version as a compatibility contract
- legacy firmware is rejected
- mismatched newer/older firmware is also rejected
- the operator is told to flash the required version instead of getting partial or misleading behavior

This should be treated as an operational rule, not as optional cleanup.

## Host Status Protocol

Host-facing telemetry now uses a structured line format:

```text
vros	<section>	<field>	<value>
```

Current sections:

- `compat` for protocol and firmware identity
- `status` for live machine state
- `config` for robot-setup and EEPROM-backed values
- `event` for one-shot completions and errors

Representative lines:

```text
vros	compat	protocolVersion	1
vros	compat	firmwareVersion	VROS_2.5.17_caseController
vros	status	state	idle
vros	status	position	21.0000,31.0000
vros	config	robotSetupStatus	valid
vros	event	moveComplete	21.0000,31.0000
```

Important compatibility rule:

- host commands stay unchanged
- streamed-job acknowledgements still use raw `ok`
- human-oriented debug text may still appear, but the Control Station should depend on `vros` records for machine state and configuration

## Robot Setup EEPROM

The firmware now maintains two EEPROM concepts:

1. Robot setup block
2. Speed settings block
3. Position block

Robot setup block:

- stores machine geometry and calibration values
- is versioned
- is CRC-protected
- is read by the Control Station after connection

Speed settings block:

- stores the active step pulse and delay values used by `setSpeed`
- is versioned
- is CRC-protected
- is loaded during boot before the machine enters `idle` or `noSD`

Position block:

- stores the last known carriage position
- is versioned
- is CRC-protected
- still supports migration from the previous legacy position addresses

Robot setup fields now persisted in EEPROM:

- `robotKind`
- `motorDistance`
- `scanOffset`
- `feedOffset`
- `width`
- `height`
- `lineResolution`
- `homePosition`
- `leftCoilFeed`
- `rightCoilFeed`
- `stepsToCm`
- `microstepResolution`
- `quadHomeScan`
- `quadHomeFeed`
- `quadCableAFeed`
- `quadCableBFeed`
- `quadCableCFeed`
- `quadCableDFeed`
- `quadMotorHeight`

Derived/runtime values that are not stored directly:

- runtime-adjusted `stepsToCm`
- `stepLength`

## Invalid Robot Setup EEPROM Behavior

This behavior changed deliberately during this chat and is important.

Current behavior for an invalid or missing robot setup block:

- the firmware does **not** automatically repair EEPROM
- the firmware loads the compiled default setup into RAM so the machine can still boot
- the firmware reports:
 
```text
vros	config	robotSetupStatus	invalid
```

- the Control Station is expected to prompt the operator to burn a valid robot setup

This is intentional because silent auto-repair hides configuration mistakes and makes it hard to know whether a robot is actually configured.

Position EEPROM behavior is different:

- the firmware still attempts to migrate legacy carriage position values into the new position block
- if that fails, position falls back to origin-like defaults

## Control Station Handshake

The Control Station now expects the following connection model:

1. robot boots and emits `vros	compat	protocolVersion	...`
2. robot emits `vros	compat	firmwareVersion	...`
3. Control Station verifies both values
3. Control Station requests `robotSetupGet`
4. firmware returns `vros	config	robotSetupStatus	...` plus the active robot setup fields
5. Control Station decides whether the EEPROM is valid, invalid, or the firmware is incompatible

Failure cases:

- legacy firmware without robot setup commands: connection refused
- wrong firmware version: connection refused
- invalid robot setup EEPROM: connection allowed, operator prompted to burn setup

## Robot Setup Commands

These commands were added or formalized for Control Station support:

```text
robotSetupGet
robotSetupWrite	<motorDistance>,<scanOffset>,<feedOffset>,<height>,<lineResolution>,<homePosition>,<leftCoilFeed>,<rightCoilFeed>,<stepsToCm>[,<microstepResolution>]
robotSetupWrite	robotKind=hanging_vbot,motorDistance=62,scanOffset=10,feedOffset=20,height=50,lineResolution=0.5,homePosition=82,leftCoilFeed=1,rightCoilFeed=0.997,stepsToCm=35,microstepResolution=0.0625
robotSetupWrite	robotKind=flat_quad_tension,scanOffset=0,feedOffset=0,width=42,height=50,lineResolution=0.5,stepsToCm=35,microstepResolution=0.0625,quadHomeScan=21,quadHomeFeed=25,quadCableAFeed=1,quadCableBFeed=1,quadCableCFeed=1,quadCableDFeed=1,quadMotorHeight=10
robotSetupLoad
robotSetupDefaults
clearEEPROM
contact	draw|travel
```

Command intent:

- `robotSetupGet`: report EEPROM validity and the active runtime setup
- `robotSetupWrite`: apply a full setup payload and persist it; both the legacy V-bot CSV payload and the newer schema-aware `key=value` payload are accepted, and both can now carry `microstepResolution`
- `robotSetupLoad`: reload the EEPROM setup block into runtime state
- `robotSetupDefaults`: write compiled defaults into EEPROM and make them active
- `clearEEPROM`: wipe the full EEPROM, including robot setup and saved carriage position
- `contact`: legacy compatibility command accepted as a no-op so older drawings and console workflows still parse cleanly

After setup-changing commands, the firmware also reports current position so the Control Station preview can update immediately.

## Quad Status

The firmware now has a first implementation pass for the flat quad robot:

- schema-aware setup read/write support
- quad home, motor-height, and 3D position persistence in EEPROM
- quad scan/feed offsets plus `z` in cable geometry
- `robotKind`, `position` / `positionZ`, `target` / `targetZ`, and four-cable telemetry reporting
- `motionSupport` and `motionBackend` reporting so the desktop can decide whether quad streaming is available
- solved `x/y/z` reporting through the same position protocol used by the Control Station
- a generic multi-axis pulse planner that now drives both the hanging two-axis backend and the flat-quad four-axis backend
- optional absolute `z` on `move,<scan>,<feed>[,<z>]` plus `moveZ,<deltaZ>` for debug jogging on the flat quad path

Important current limit:

- the current default build assumes a RAMPS 1.4 map that routes quad cables A/B/C/D through the X/Y/E0/E1 driver sockets
- per-motor direction inversion may still need tuning on the real machine
- `resetHome`, `returnToHome`, and `returnToOrigin` all use `z=0` on the flat quad path, where `z=0` means the carriage is at the table and `z=quadMotorHeight` is the motor plane
- quad `contact` no longer drives or reports a physical draw/travel actuator; legacy contact commands are metadata only

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
- `launchpad -> idle` via toggle 4 or serial `returnToOrigin`
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
move,<scan>,<feed>[,<z>]
moveZ,<deltaZ>
moveX,<deltaScan>
moveY,<deltaFeed>
moveLeft,<distance>
moveRight,<distance>
moveUp,<distance>
moveDown,<distance>
type,<absolute|relative>
mode,<bresenham>
adjustment,<none|largeSin|complexSin|noise>
stepA,<amount>
stepB,<amount>
stepC,<amount>
stepD,<amount>
stepAll,<amount>
stepL,<amount>
stepR,<amount>
motors on
motors off
setSpeed,<delay>
getSpeed
saveSpeed
outlineCanvas
returnToOrigin
returnToHome
feedToHome
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
- manual commands such as `move`, `moveZ`, `moveX`, `moveY`, `moveLeft`, `moveRight`, `moveUp`, `moveDown`, `stepA`, `stepB`, `stepC`, `stepD`, `stepAll`, `stepL`, `stepR`, `motors on`, `motors off`, `returnToOrigin`, `returnToHome`, `feedToHome`, `resetHome`, and `position` are accepted in non-drawing states

Jog and debug notes:

- `move,<scan>,<feed>,<z>` is available on the flat quad path; omitting `z` keeps the current logical `z`
- `moveZ,<deltaZ>` is a flat-quad-only relative jog along the logical `z` axis
- `moveX` and `moveY` are relative jogs in centimeters from the current logical position
- `moveLeft`, `moveRight`, `moveUp`, and `moveDown` are directional jog aliases; in this coordinate system, `up` means negative feed and `down` means positive feed
- `stepL` and `stepR` remain compatibility aliases for `stepA` and `stepB`
- `stepAll` runs the same signed raw step amount on every active motion axis at once; that means both axes on the hanging V-bot path and all four axes on the flat quad path
- `setSpeed,<delay>` updates the active step pulse/delay timing directly; on the flat quad path this is the authoritative operator speed control because the legacy panel speed knob is not used there
- `getSpeed` reports the current active speed delay value as `vros	status	speedDelayMs	...`
- `saveSpeed` writes the current active speed delay/pulse values to EEPROM so they are restored after power-up; on the hanging V-bot path the A15 speed knob still overrides that live value once the idle/drawing loop polls it again
- `feedToHome` is a flat-quad setup helper for the fully-wound starting condition; it feeds the configured home cable lengths using the same global step pulse/delay timing as other coordinated motion, then leaves the logical position unchanged so the operator can attach/place the carriage and finish with `resetHome`
- `motors on` and `motors off` control stepper enable pins for debugging when the active robot exposes configurable motor enables; the current RAMPS-backed flat quad path does, while the hanging V-bot path does not
- the next stepped or coordinated move automatically re-enables those motors before pulsing, so `motors off` is mainly a temporary release/debug action
- raw `step*` commands and `feedToHome` do not update the logical scan/feed/z position model, so return the carriage to a known pose and run `resetHome` before trusting `position` or later absolute moves

## Streaming Drawings From The Repo

You can keep drawing files in [`drawings/`](drawings) and stream them directly over serial without an SD card.

The recommended file format matches the SD instruction format:

```text
type	absolute
mode	bresenham
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
- the firmware accepts streamed `move`, `type`, `mode`, `adjustment`, and legacy `contact` instructions over serial
- flat-quad `move` instructions can optionally carry `z` as `move<TAB>scan,feed,z`
- `mode<TAB>segmented` and `mode<TAB>movePen` are still accepted as legacy aliases and normalized to `mode<TAB>bresenham`
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
