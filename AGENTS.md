# Firmware Agent Notes

Purpose: onboard coding agents quickly and safely for this firmware repo.

## Project layout
- `app/`: application logic (control, logging, telem, etc.).
- `app/logging/`: blackbox logging + dump-to-SD.
- `app/drivers/`: MCU + peripheral drivers (QSPI, storage, watchdog, etc.).
- `common/`: shared protocol definitions.
- `STM32H723XG_FLASH.ld`: linker script (memory sections).
- `build/Debug/`: CMake build output (CubeMX/Cube-IDE toolchain).

## Build
- Primary build command (Debug):
  ```bash
  PATH="/Users/emmanuelcarjat/.vscode/extensions/stmicroelectronics.stm32cube-ide-core-1.1.0/resources/binaries/darwin/aarch64:/Users/emmanuelcarjat/.vscode/extensions/stmicroelectronics.stm32cube-ide-build-cmake-1.43.0/resources/cube-cmake/darwin:$PATH" /Users/emmanuelcarjat/.vscode/extensions/stmicroelectronics.stm32cube-ide-build-cmake-1.43.0/resources/cube-cmake/darwin/cube-cmake --build /Users/emmanuelcarjat/git/robot2Wheel/stm32Controller/firmware/build/Debug --target all --
  ```
- No unit test runner is wired up in this repo. If you add tests, document how to run them.

## DMA and cache rules (H7)
- DMA buffers must live in `.dma_buffer` (RAM_D1). Do not place DMA TX/RX buffers in `.bss`/DTCMRAM.
- Declare DMA buffers with:
  `__attribute__((section(".dma_buffer"), aligned(32)))`
- Clean/invalidate D-cache appropriately when DMA reads/writes those buffers.

## Logging / blackbox notes
- Record layout and sizes are defined in `app/logging/blackbox_format.h`.
- `LOG_RECORD_SIZE` must match `sizeof(LogRecord)` (keep struct packed and size checks intact).
- QSPI write path is async; do not block the control loop with flash operations.
- Dump path reads from QSPI and writes to SD with DMA buffers; follow cache maintenance patterns.

## Coding conventions
- Favor existing patterns in surrounding code (C-style in `app/`, minimal C++ in control).
- Use `rg` for searching; avoid large refactors without confirming with the user.
- Keep changes scoped; avoid touching CubeMX-generated files unless necessary.

## Workflow (Issue Fixes)
- Create a new branch for each issue (e.g., `fix/issue-XX-short-title`).
- Implement the fix on that branch only.
- Open a PR from the branch and reference the issue in the PR (e.g., `Closes #XX`).
- Keep main clean; only merge via PR.

## Gitea repository
Main repo:
ecarjat/robotFirmware

Common repo:
ecarjat/robotCommon

esp32-arduino repo
ecarjat/robotESP32-Arduino
