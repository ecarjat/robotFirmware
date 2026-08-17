# MIGRATION_AGENT.md — Codex-ready Migration Agent (ESP32 → STM32H723 + ESP32 Coprocessor)

> Historical migration plan. Its UART motor topology and IMU bus assignments
> are superseded by `Pinmap.md`, `STM32_External_Module_Pinmap.md`, and
> `docs/SteadywinCan.md`.

Version: 1.0  
Audience: Codex (code generation + refactor assistant)  
Primary goal: Port an existing ESP32 codebase (motors + sensors + logging + estimator/control) to STM32H723, leaving ESP32 as BT + Wi-Fi portal coprocessor.

---

## 0) Context & Target Architecture

### 0.1 Current state (source)
- You have a **substantial ESP32 codebase** that already does:
  - motor communication (defined in the README below)
  - estimator + PID controller (already built)
  - telemetry/logging infrastructure (at least partially)
- Motor protocol reference:
  - https://github.com/ecarjat/T-Storm32NT-simpleFoc/blob/main/README.md

### 0.2 Target state (destination)
- **STM32H723 (WeAct board)** is the real-time robot brain:
  - IMUs (BMI270 on I2C1, ICM-42688 on SPI6, BMM150 on I2C2)
  - TFmini Plus LiDARs (UART)
  - motor nodes (UART to each driver board)
  - on-board SDMMC + TFT + QSPI
  - estimator + PID control loop
  - telemetry/logging production
  - USB CDC device (to host / Raspberry Pi future)
- **ESP32 (bare module)** becomes a coprocessor:
  - Xbox controller (BT)
  - Wi-Fi portal (download logs, firmware, telemetry)
  - OTA for itself
  - acts as a network bridge to STM32 via UART

### 0.3 STM32 ↔ ESP32 link (fixed)
- Transport: **UART @ 921600**, optional RTS/CTS
- Protocol: **framed binary (COBS or SLIP) + CRC32**
- Multiplex channels: **CMD, TELEM, FILE, RPC**
- STM32 is authoritative for robot state/control; ESP32 is UI/portal.

### 0.4 Repo baseline, CubeMX, and legacy sources
- This migration agent extends the Migration spec described in `Migration.md`; if anything conflicts, follow that document plus the clarifications here.
- The existing `firmware/` directory already contains a CubeMX-generated STM32 project (`stm32Controller.ioc`, `Core/`, `Drivers/`, `Middlewares/`, `USB_DEVICE/`, `cmake/`, linker scripts, startup files). Reuse these files — do not create a parallel STM32 project elsewhere. All HAL initialization changes must go through CubeMX and keep these files as the source of truth.
- `Core/Src/main.c` (owned by CubeMX) must include `app/app_main.h` and call a Codex-maintained `app_main()` entry point so that we can regenerate from CubeMX safely.
- Legacy controller, estimator, UI, logging, and motor comm code that must be ported lives under `legacy/src/`, notably:
  - `legacy/src/control/MotionController.{cpp,h}` and `legacy/src/estimation/StateEstimator.{cpp,h}` for the PID controller + estimator glue.
  - `legacy/src/ekf/BalancerEKF.*` for the EKF, `legacy/src/drivers/motors_binaryIO.*` for the motor protocol, `legacy/src/drivers/tfmini.*` for LiDAR parsing, and `legacy/src/util/*` for telemetry/logging helpers.
  Treat these files as the reference while recreating the functionality on STM32 + ESP32.

### 0.5 Bootloader + app placement
- `bootloader/` hosts the USB CDC bootloader described in `bootloader/README.md` and `bootloader/AGENT_BOOTLOADER.md`. It occupies flash `0x08000000–0x0800FFFF` and leaves a 256-byte metadata block at `0x08010000`.
- The STM32 application produced by this migration **must link its vector table at `0x08010100` (APP_VECTOR_BASE)** and never overwrite the reserved bootloader + metadata regions. CubeMX linker scripts and `CMakeLists.txt` must reflect this so that every build generates binaries suitable for KVBL updates.
- The bootloader validates metadata (length + CRC32 + flags) before jumping; the migration work must keep that metadata layout intact and provide an update path via the RPC/FILE channels or host tooling that ultimately calls into the bootloader.

### 0.6 Pin/peripheral mapping reference
- `Pinmap.md` is the single source of truth for all STM32 pin assignments and peripheral selections. Do not restate pin numbers elsewhere; refer to that document whenever hardware wiring is referenced.

---

## 1) Migration Agent Role

You (Codex) are a migration agent. Your output must:
1) Introduce a **new STM32 firmware skeleton** that compiles, runs, and matches the bring-up plan.
2) Port logic from ESP32 code to STM32 in a controlled sequence.
3) Keep ESP32 firmware minimal and stable: BT+WiFi+bridge only.
4) Keep real-time safety: control loop deterministic; avoid blocking I/O.

---

## 2) Constraints & Non-goals

### 2.1 Hard constraints
- Do not change the agreed UART protocol shape: COBS/SLIP + CRC32 + channels.
- STM32 firmware uses CubeMX/HAL initialization (do not rewrite init).
- All heavy control runs on STM32; ESP32 must not do control math.
- UI on STM32 TFT via u8g2 + MUI (wizard system already specified).

### 2.2 Non-goals (for now)
- No autonomous driving stack on Pi yet (only keep a clean path via USB CDC / future host protocol).
- No camera support.
- No fancy QSPI filesystem: only basic QSPI R/W + optional simple key-value store.
- No deep estimator rewrite; only adapt interfaces.

---

## 3) Inputs Required from Repo (Agent must locate)
Agent must scan the repository for:
- ESP32 motor link / protocol module(s)
- estimator module(s) (EKF/TinyEKF or equivalent)
- PID controller module(s)
- telemetry/logging modules
- configuration system and parameter storage
- transport framing code (COBS/SLIP) and CRC function
- tasking model (FreeRTOS tasks on ESP32)

These implementations already live under `legacy/src/`:
- `legacy/src/control/MotionController.{cpp,h}` and `legacy/src/estimation/StateEstimator.{cpp,h}` for the controller + estimator interface.
- `legacy/src/ekf/BalancerEKF.*` for the EKF core.
- `legacy/src/drivers/motors_binaryIO.*`, `legacy/src/drivers/tfmini.*`, and `legacy/src/drivers/xbox_controller.*` for motor comms and sensor drivers.
- `legacy/src/util/flight_log.*`, `legacy/src/util/log.*`, and `legacy/src/util/perf_stats.*` for telemetry/logging infrastructure.
- `legacy/src/main.cpp` for the existing tasking model / scheduler skeleton.

If parts are missing, stub them with placeholders but preserve interfaces.

---

## 4) Deliverables (Files to Generate)

### 4.1 Project layout (destination)

**common/** (shared deliverable from `Migration.md`)
- Repository root folder for code shared between STM32 and ESP32 builds: protocol schemas, framing/CRC utilities, transport mux helpers, config serialization, and any other platform-agnostic logic.
- `Pinmap.md` lives alongside these docs; reference it instead of duplicating pin tables.

```
common/
  shared_protocol/
    robot_protocol.h
    PROTOCOL.md
  framing/
    framing_cobs.c
    framing_cobs.h
    crc32.c
    crc32.h
  mux/
    mux_channels.c
    mux_channels.h
  transport/
    file_channel.c
    file_channel.h
    telem_channel.c
    telem_channel.h
    cmd_channel.c
    cmd_channel.h
```

**STM32 (CubeMX project inside `firmware/`, i.e., the `stm32/` deliverable from `Migration.md`)**
- Keep the existing CubeMX-generated assets: `stm32Controller.ioc`, `Core/Inc`, `Core/Src`, `Drivers/`, `USB_DEVICE/`, `Middlewares/`, `cmake/`, linker/startup scripts, and `CMakeLists.txt`. Any HAL or peripheral init tweaks happen through CubeMX so these files remain authoritative.
- Add new higher-level modules under dedicated folders at the root of `firmware/` so CubeMX regenerations never overwrite them. Pull the new folders into the build via the existing `CMakeLists.txt`.
- `Core/Src/main.c` includes `app/app_main.h` and calls `app_main()` once HAL is initialized.

```
firmware/
  app/
    app_main.c
    app_main.h
    app_config.h
  comms/
    link_uart_dma.c
    link_uart_dma.h
    framing_cobs.c
    framing_cobs.h
    crc32.c
    crc32.h
    mux_channels.c
    mux_channels.h
    rpc.c
    rpc.h
    file_channel.c
    file_channel.h
    telem_channel.c
    telem_channel.h
    cmd_channel.c
    cmd_channel.h
  control/
    estimator.c
    estimator.h
    pid_controller.c
    pid_controller.h
    motion_modes.c
    motion_modes.h
  drivers/
    imu/
      imu_bmi270.c
      imu_bmi270.h
      imu_icm42688.c
      imu_icm42688.h
      mag_bmm150.c
      mag_bmm150.h
    lidar/
      tfmini_parser.c
      tfmini_parser.h
    motors/
      motor_link.c
      motor_link.h
      motor_protocol.h
  ui/
    ui_main.c
    ui_main.h
    ui_wizard.c
    ui_wizard.h
    ui_screens.c
    ui_screens.h
  storage/
    sd_logger.c
    sd_logger.h
    qspi_w25q64.c
    qspi_w25q64.h
    settings_store.c
    settings_store.h
  bringup/
    (bring-up tests from BRINGUP_TEST_PLAN.md)
  third_party/
    u8g2/
    (sensor vendor libs if needed)
```

Document build/flash/debug steps in `firmware/README.md` and keep bring-up instructions next to the code (e.g., `bringup/`).

**ESP32 gateway (`esp32/`, i.e., the `esp32/` deliverable from `Migration.md`)**
- Houses the coprocessor firmware. Mirrors the STM32 protocol stack but links against ESP-IDF/PlatformIO.
- Links against the shared code in `common/` (protocol headers, framing/mux helpers).

```
esp32/
  README.md
  main/
    app_main.cpp
    bt_xbox.cpp
    bt_xbox.h
    wifi_portal.cpp
    wifi_portal.h
    stm32_bridge_uart.cpp
    stm32_bridge_uart.h
    (reuses framing/mux/protocol from ../common)
  components/
    (optional)
```

### 4.2 Shared protocol header
Live under `common/shared_protocol/`. Create `robot_protocol.h` used by both firmwares:
- packet header
- channel IDs
- message types
- RPC IDs
- file transfer message formats
- telemetry message formats
- command message formats
Keep it C-compatible (`extern "C"` guards).

---

## 5) Migration Plan (Phased, must implement in order)

### Phase 0 — Foundations (compile + run)
1) Create STM32 project scaffolding that compiles under CubeIDE:
   - `app_main()` called from `main()`
   - a logger function `LOGI/LOGE` that writes USB CDC
2) Implement STM32 link layer:
   - UART DMA RX ring + IDLE detect + TX DMA queue
   - COBS (or SLIP) framing + CRC32
   - channel mux/demux

Acceptance:
- STM32 prints boot log and echoes framed packets.
- ESP32 bridge can connect and exchange pings.

### Phase 1 — ESP32 bridge minimal firmware
Implement on ESP32:
- BT Xbox input → emits CMD channel messages to STM32
- Wi-Fi portal:
  - minimal HTTP endpoints:
    - `/telem` websocket or long-poll (optional)
    - `/files/list`
    - `/files/read?name=...&offset=...&len=...`
    - `/fw/upload` (store and forward to STM32 via FILE/RPC)
- STM32 bridge over UART:
  - reconnect on failure
  - rate limit

Acceptance:
- Xbox A/B + joystick up/down messages arrive at STM32.
- ESP32 can request file list + read chunks via FILE channel.

### Phase 2 — Sensors on STM32
Implement:
- IMU drivers (BMI270 on I2C1; ICM-42688 on SPI6)
- Mag driver (BMM150)
- TFmini UART parser for 3 LiDARs

Acceptance:
- Sensor IDs validated
- IMU sample stream at target rate (e.g. 200–400Hz) with timestamps
- LiDAR distances update reliably

### Phase 3 — Motor communication on STM32
Port the motor protocol logic from ESP32 to STM32:
- per-motor UART link
- request/response frames
- telemetry acquisition
- command outputs (torque/velocity/current setpoints) as defined by the existing motor firmware protocol

Acceptance:
- Can ping each motor node
- Can read encoder/velocity/current telemetry
- Can command low torque safely (in calibration mode)

### Phase 4 — Estimator + PID (port interfaces, not math)
Port estimator + PID controller modules:
- preserve math functions, refactor hardware I/O:
  - estimator reads IMU data from STM32 sensor layer
  - controller outputs motor setpoints via motor layer
- implement motion modes:
  - DISARMED, CALIBRATION, BALANCING, FAULT
  - joystick commands → desired velocity/turn rate → reference shaping

Acceptance:
- In DISARMED, no motor torque
- In CALIBRATION, explicit tests can move motors
- In BALANCING, controller loop runs at 1kHz and produces bounded outputs

### Phase 5 — Logging + UI
- Implement SD logger (binary run files + free-space rotation policy already specified elsewhere)
- Implement TFT UI using u8g2 + MUI:
  - main dashboard values
  - the calibration wizards (IMU 6-face, axis invert, motor calib, motor dir, zero upright)
- Expose settings persist/load.

Acceptance:
- SD logging stable under load
- UI responsive (10–20Hz) and not impacting control loop

---

## 6) STM32 Task/Loop Model (no RTOS required, RTOS optional)
Codex must implement a deterministic scheduler model:

- TIM 1kHz ISR: sets `control_tick_flag`
- Main loop:
  - `poll_uart_links()` (DMA-driven)
  - `poll_sensors()` (SPI/I2C reads scheduled; ICM-42688 via SPI DMA on DRDY)
  - `if(control_tick_flag) run_control_tick_1khz()`
  - `run_ui_tick_20hz()`
  - `run_logger_tick()` (non-blocking)
  - `run_usb_cdc_tick()` (if enabled)

Rules:
- Control tick must be bounded and non-blocking.
- SD writes must not occur in the 1kHz tick; buffer and flush asynchronously.

---

## 7) Protocol Definition Requirements (must implement)
### 7.1 Frame
- Start/end implicit by COBS/SLIP
- Payload includes:
  - version
  - channel
  - message type
  - seq
  - length
  - CRC32

### 7.2 Channels
- CMD: teleop commands, UI inputs (Xbox)
- TELEM: status + sensor summaries
- FILE: list/read/write chunks, run rotation
- RPC: request/response (ping, get info, set config, start calibration)

### 7.3 Minimum RPC set
- PING
- GET_SYSTEM_INFO (fw versions, build hash, uptime)
- GET_SETTINGS / SET_SETTINGS
- SET_MODE (DISARMED/CALIB/BAL)
- START_WIZARD / WIZARD_ACTION (optional)
- SD_LIST_FILES / SD_READ_CHUNK
- QSPI_JEDEC / QSPI_TEST (optional)

---

## 8) Acceptance Tests (must be runnable)
Codex must provide a runnable test runner on STM32:
- `bringup_run_all()` or CLI command via USB CDC:
  - `test usb`
  - `test sd`
  - `test qspi`
  - `test i2c`
  - `test spi_imu`
  - `test motors`
  - `test lidar`

ESP32 must provide a minimal host interaction path:
- prints UART bridge status
- exposes HTTP endpoints for file listing and reading

---

## 9) Documentation to Generate
Codex must generate:
- `firmware/README.md`:
  - build steps
  - flashing steps
  - how to run bring-up suite
  - how to connect ESP32
- `esp32/README.md`:
  - wiring + UART settings
  - BT pairing instructions
  - portal endpoints
- `common/shared_protocol/PROTOCOL.md`:
  - channels, message formats, CRC/framing details

---

## 10) Implementation Notes (important)
- Use `uint32_t` monotonic timestamps (ms) from SysTick.
- For UART DMA RX ring, implement:
  - IDLE interrupt: compute new bytes, push into frame decoder
- For QSPI W25Q64:
  - JEDEC ID read + QE bit handling required before quad I/O
- For SD logger:
  - implement your free-space enforcement + run rotation policy at init
- For u8g2 UI:
  - SPI write uses DMA where possible
  - UI tick at 10–20Hz, never in control tick

---

## 11) “Done” Definition
Migration is considered successful when:
- STM32 runs full bring-up suite PASS
- ESP32 can:
  - send Xbox inputs
  - fetch logs over Wi-Fi via FILE channel
- Motors respond to commands safely in CALIB mode
- Estimator + PID run at 1kHz without missed ticks during SD logging and UI updates

End of migration agent spec.
