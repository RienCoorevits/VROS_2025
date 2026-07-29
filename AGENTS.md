# VROS Firmware Repo Guidance

This repository is the firmware repo for the VROS drawing machine. The primary documentation source is the Obsidian vault `VBOT Vault` at `/Users/u0127995/Documents/03_VAULTS/VBOT Vault`.

## Workspace Map

- Firmware repo: `/Users/u0127995/Documents/PlatformIO/Projects/260511-111720-megaatmega2560`
- Control Station repo: `/Users/u0127995/Documents/02_DEVELOPER/VROS Control Station`
- Obsidian vault: `/Users/u0127995/Documents/03_VAULTS/VBOT Vault`

## Start Here

For VROS tasks, read the vault before making assumptions. Use the Obsidian CLI when available.

Read these notes first:

1. `Welcome.md`
2. `Firmware Documentation/Project Overview.md`
3. `Firmware Documentation/Firmware Architecture.md`
4. `Firmware Documentation/Serial And Drawing Workflow.md`

Then inspect local repo context:

- `README.md`
- `PROJECT_CONTEXT.md`
- `platformio.ini`
- `src/VROS_2.5.3_coilCalibration.ino`
- `src/controller.ino`

If the task mentions the desktop app, operator workflow, serial host tooling, firmware compatibility, robot setup, or protocol changes, also read:

- `Control Station Documentation/Control Station Overview.md`
- the Control Station repo `README.md`
- the Control Station repo `PROJECT_CONTEXT.md`

## Working Rules

- Treat the vault as the canonical project documentation.
- If vault notes and code disagree, fix the vault before relying on stale notes.
- Keep meaningful code changes in sync with the relevant vault notes.
- For meaningful VROS changes, add or update a dated note under `Worklog/`.
- Do not assume the Control Station repo is the current working directory; use its exact path.
- Preserve SD-card playback support. Control Station work extends the workflow but must not break the firmware SD path.
- Treat the firmware banner and Control Station required firmware version as a protocol contract.
- If firmware behavior changes in a way the Control Station depends on, update the firmware version string and check the required firmware version in the Control Station repo.
- Do not change EEPROM layout, robot setup behavior, or serial protocol casually; verify the corresponding desktop expectations.

## Verification

- Firmware changes: `~/.platformio/penv/bin/pio run`
- Firmware upload when requested: `~/.platformio/penv/bin/pio run --target upload`
- Serial monitor when needed: `~/.platformio/penv/bin/pio device monitor`
- If you edit the Control Station repo as part of the same task, run its relevant tests there before finishing.

## Documentation Workflow

- Prefer the Obsidian vault over repo READMEs for current VROS project understanding.
- Use repo READMEs and `PROJECT_CONTEXT.md` files as quick entry points and path maps.
- When updating vault notes, preserve existing note names and folder structure unless the task requires a deliberate reorganization.
