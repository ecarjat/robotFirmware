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
- Host-unit tests are wired up under `tests/` using Catch2 + CMake/CTest.
- Build + run tests (host): `cmake --build tests/build` then `ctest --test-dir tests/build -V`.
- After any code change, run the firmware build and host tests before reporting done.

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
- Prefer Serena MCP for symbol-aware tasks: cross-file refactors/renames, API changes touching many files, and when needing precise symbol lookup/edit to avoid brittle text edits. Use shell tools for quick greps or single-file changes.

## Tests (host/unit)
- Place unit tests in `tests/` and register them in `tests/CMakeLists.txt`.
- Prefer link-time fakes over stub headers:
  - Keep production headers in includes.
  - Provide fake implementations in `tests/fakes/*.c/.cpp` for the small set of symbols needed.
- Avoid including `app_config.h` in unit-tested modules when a narrower include is sufficient.
  - Use `app/utils/app_log_macros.h` for logging macros instead of pulling HAL headers.
- If a module depends on HAL or MCU registers, isolate those dependencies behind a tiny wrapper or interface before writing tests.
- Always add a test task that rebuilds before running CTest (see `tests/.vscode/tasks.json`).

## Workflow (Issue Fixes)
- Create a new branch for each issue (e.g., `fix/issue-XX-short-title`).
- Implement the fix on that branch only.
- Open a PR from the branch and reference the issue in the PR (e.g., `Closes #XX`).
- Keep main clean; only merge via PR.
- Issue comments often include the intended fix; read them before coding.

## Gitea repository
Main repo:
ecarjat/robotFirmware

Common repo:
ecarjat/robotCommon

esp32-arduino repo
ecarjat/robotESP32-Arduino
