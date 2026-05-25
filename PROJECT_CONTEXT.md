# Project Context

This repository is the firmware repo for the VROS project.

## Canonical Documentation

- Primary docs vault: `/Users/u0127995/Documents/03_VAULTS/VBOT Vault`
- Vault home note: `Welcome.md`

## Related Workspaces

- Firmware repo: `/Users/u0127995/Documents/PlatformIO/Projects/260511-111720-megaatmega2560`
- Desktop app repo: `/Users/u0127995/Documents/02_DEVELOPER/VROS Control Station`

## What This Repo Owns

- Arduino Mega 2560 firmware
- PlatformIO build configuration
- Repo-local drawing files used for host streaming
- Host-side serial helper scripts used during firmware work
- the firmware side of the Control Station compatibility contract
- the EEPROM layout for robot setup and carriage position

## Suggested Starting Points

Read these first in the vault:

1. `Welcome.md`
2. `Firmware Documentation/Project Overview.md`
3. `Firmware Documentation/Firmware Architecture.md`
4. `Firmware Documentation/Serial And Drawing Workflow.md`
5. `Worklog/2026-05-15.md`

Then inspect these local files:

- `README.md`
- `platformio.ini`
- `src/VROS_2.5.3_coilCalibration.ino`
- `src/controller.ino`

For the current Control Station integration work, pay special attention to:

1. the firmware version banner
2. `robotSetupStatus`
3. `robotSetupGet` / `robotSetupWrite` / `clearEEPROM`
4. the rule that firmware behavior changes which affect the desktop protocol must increment the firmware version

## Working Rules

- Treat the vault as the primary documentation source.
- Keep vault notes in sync with meaningful code changes.
- For every meaningful code change, update the relevant vault documentation and add or update a dated entry under `Worklog/`.
- If the vault and the repo code/current direction diverge, treat that as documentation drift and fix the vault before relying on the stale note.
- Do not assume the desktop app repo is the current working directory.
- If a task mentions `VROS Control Station`, switch attention to the desktop app repo.
- Treat firmware versioning as part of the wire protocol, not as cosmetic metadata.
- Do not change EEPROM or serial setup behavior without also checking the required firmware version in the Control Station repo.
