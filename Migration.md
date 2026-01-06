# Migration Spec — ESP32 → STM32H723 Robot Brain + ESP32 Gateway

## 1) Objective
Migrate the robot “brain” from an ESP32-centered architecture to:

- **STM32H723VGT6 (WeAct board)** as the primary controller:
  - LQR control loop (already built)
  - motion control (already built)
  - state estimator (already built)
  - all sensor IO (LiDAR, IMUs, magnetometer)
  - all motor-node communication (SimpleFOC nodes)
  - logging to SD (on-board)
  - TFT dashboard (on-board)

- **ESP32** retained as a **gateway/coproc only**:
  - Xbox controller over Bluetooth (Bluepad32)
  - Wi-Fi portal for robot interaction:
    - STM32 firmware upload
    - logs download
    - telemetry streaming

Everything except BT+Wi-Fi portal moves to STM32.

---

## 2) Inputs / Existing Assets
### 2.1 Motor communication protocol source of truth
Motor communication is defined here and must be preserved:
- https://github.com/ecarjat/T-Storm32NT-simpleFoc/blob/main/README.md

**Requirement:** migration must not “invent a new protocol.” It may refactor code, but behavior and framing must remain compatible with existing motor-driver firmware.

### 2.2 Control stack already exists
- LQR controller exists
- motion control exists
- estimator exists

**Requirement:** migration must integrate these modules into the STM32 bare-metal schedule, not redesign them.

---

## 3) Hardware Scope & Buses
### 3.1 External devices (STM32 side)
- 3× TFmini Plus LiDAR: front, back, down (UART)
- 2× IMU + 1× magnetometer
  - ICM-42688 (IMU, SPI6)
  - BMI270 (IMU, I2C1)
  - BMM150 (mag, I2C2)
- 4× motor drivers (SimpleFOC nodes, UART)
- dual-color status LED (2 GPIO)
- buttons (define set; see §8)
- ESP32 module (UART link)

### 3.2 On-board devices (already wired on WeAct board)
- SD card
- TFT screen
- USB

**Constraint:** do not break the on-board wiring; treat those pins/peripherals as reserved.

---

## 4) Target Architecture
### 4.1 Three-layer split
1) **common/** (platform-agnostic):
   - transport framing + CRC
   - packet schemas
   - parsing/encoding utilities
   - shared state machines (file transfer, RPC, telemetry schema)
   - *optional*: shared motor packet structs (if identical on both sides)

2) **stm32/** (robot brain):
   - UART DMA drivers for all UART links
   - SPI/I2C drivers for IMU/mag
   - motor comm manager (port of existing ESP32 motor comm logic)
   - LiDAR drivers (TFmini)
   - logging service (SD)
   - UI service (TFT)
   - bare-metal scheduler: control tick + cooperative tasks

3) **esp32/** (gateway/coproc):
   - Bluepad32 gamepad acquisition
   - Wi-Fi portal HTTP server
   - STM32 link transport (UART)
   - request/response routing: CMD/TELEM/FILE/RPC multiplexing

---

## 5) STM32 ↔ ESP32 Link (Agreed Spec)
### 5.1 Physical link
- **UART @ 921600**
- Optional hardware flow control: **RTS/CTS** (enable if routed and needed)
- Full duplex

### 5.2 Framing + integrity
- **COBS or SLIP** framing (choose one; default: COBS)
- **CRC32** for payload integrity (CRC covers: channel + type + payload)
- Multiplex channels:
  - **CMD**  (teleop commands, mode changes)
  - **TELEM** (telemetry stream)
  - **FILE** (log listing + chunk download)
  - **RPC**  (request/response actions: calibrate, arm/disarm, config, etc.)

### 5.3 Link behavior requirements
- Non-blocking on STM32: UART RX uses DMA circular + IDLE interrupt; parsing in main loop.
- Sequence numbers for CMD + FILE + RPC messages.
- Heartbeat mechanism:
  - ESP32 sends `CMD_HEARTBEAT` only when no other frames are being sent.
  - STM32 marks BT/Wi-Fi link stale if no frames > 250 ms (configurable).

---

## 6) STM32 Firmware Responsibilities
### 6.1 Real-time schedule (bare metal)
- **1 kHz control tick** driven by hardware timer ISR:
  - sample “latest” sensors (already acquired)
  - run estimator + LQR
  - enforce safety + fault logic
  - publish motor commands into per-motor TX queues
  - publish telemetry snapshot to TELEM ring buffer (optional decimation)

- Main loop cooperative tasks (priority order):
  1) IMU service (read on DRDY / scheduled)
  2) UART parsers (motors, lidars, esp32 link)
  3) motor TX DMA service
  4) slow sensors (mag)
  5) health monitor + watchdog
  6) log flush step (chunked, non-blocking)
  7) TFT update step (rate limited)

### 6.2 Motor communication
- Port existing ESP32 motor comm code to STM32.
- Maintain compatibility with protocol defined in the README link.
- Support 4 motor nodes concurrently.
- Use per-motor UART, each with:
  - DMA RX circular + IDLE interrupt
  - TX DMA queue
- Include timeouts per node:
  - if node stale > threshold => fault/disarm policy

### 6.3 LiDAR (TFmini Plus)
- 3 UART streams, DMA RX circular + IDLE
- parse TFmini frames, update latest distance + timestamp
- provide:
  - obstacle gating (front/back)
  - cliff/height gating (down)

### 6.4 IMUs + magnetometer
- I2C1: IMU bus (BMI270)
- I2C2: magnetometer bus (BMM150)
- SPI6: IMU bus (ICM-42688)
- interrupt lines from IMUs preferred (DRDY -> EXTI)
- publish fused or redundant IMU data to estimator inputs (existing estimator decides usage)

### 6.5 Logging
- SD card logging with ring buffer:
  - producer: control tick + parsers
  - consumer: main loop flush chunks (e.g., 4–16KB)
- Expose file service to ESP32 over FILE channel:
  - list logs
  - read chunks with offsets

### 6.6 TFT UI
- low-rate refresh (10–20 Hz)
- shows:
  - mode (DISARMED/ARMED/BALANCING/FAULT)
  - tilt angle, command, battery (if available)
  - link status (ESP32 connected)
  - fault code

---

## 7) ESP32 Firmware Responsibilities
### 7.1 Bluetooth Xbox controller
- Use Bluepad32 (BT Classic where required)
- Map controller inputs to `CMD` messages to STM32:
  - forward/back velocity command
  - turn/yaw command
  - mode buttons (arm/disarm, mode cycle)
  - e-stop/disarm
- Send CMD at fixed rate (e.g., 100 Hz). Heartbeat only when idle.

### 7.2 Wi-Fi portal
Provide HTTP (or WebSocket/SSE) portal, proxying requests to STM32 via UART link.

#### Required endpoints (minimum viable)
- `GET /health`
  - returns ESP32 status + whether STM32 link is up
- `GET /telemetry` (SSE or WebSocket)
  - streams TELEM frames received from STM32
- `GET /logs/list`
  - proxies FILE_LIST to STM32, returns JSON
- `GET /logs/download?name=...`
  - proxies FILE_READ chunking from STM32, streams to client
- `POST /firmware/stm32`
  - accepts STM32 firmware image upload and pushes it to STM32 update protocol (RPC/FILE channel)
- Optional:
  - `POST /firmware/esp32` for ESP32 OTA

### 7.3 STM32 update transport
ESP32 provides transport only. STM32 owns flashing logic (bootloader or in-app updater).
- recommended: staged transfer with CRC validation + explicit commit
- supports resume/retry if link drops mid-transfer

---

## 8) Buttons & LED Policy
### 8.1 Buttons (recommended set = 4)
- BTN1: Arm / Enable balancing
- BTN2: Mode cycle (PID/LQR/CAL/DIAG) — even if LQR is primary, keep mode channel for debug/cal
- BTN3: IMU calibrate / zero (only when disarmed)
- BTN4: E-Stop / Disarm (immediate torque=0)

### 8.2 Dual-color LED
- Green solid: armed/ready
- Green blink: balancing active
- Red solid: fault latched
- Red blink codes: fault ID

---

## 9) Protocol Spec (STM32↔ESP32)
### 9.1 Frame format (proposed)
- Outer: COBS framing with 0x00 delimiter
- Payload:
  - `u8  channel`   (CMD=1, TELEM=2, FILE=3, RPC=4)
  - `u8  msgType`
  - `u16 payloadLen`
  - `u32 seq`
  - `payload[payloadLen]`
  - `u32 crc32` (over channel..payload)

### 9.2 Required message types
**CMD**
- `CMD_TELEOP` (forward, turn, flags)
- `CMD_MODE` (requested mode)
- `CMD_ARM` / `CMD_DISARM`
- `CMD_HEARTBEAT`

**TELEM**
- `TELEM_FRAME` (fixed layout, versioned)
- `TELEM_EVENT` (faults, transitions)

**FILE**
- `FILE_LIST_REQ` / `FILE_LIST_RESP`
- `FILE_READ_REQ` (name, offset, length)
- `FILE_READ_RESP` (offset, data)
- `FILE_ERR`

**RPC**
- `RPC_REQ` / `RPC_RESP`
- required RPC methods:
  - `CAL_IMU`
  - `ZERO_ESTIMATOR`
  - `SET_PARAM` (optional)
  - `GET_PARAM` (optional)
  - `GET_STATUS`

---

## 10) Phased Migration Plan (Codex-friendly)
### Phase 0 — Refactor & freeze interfaces
- Extract packet structs and parsing utilities into `common/`
- Define `Transport` API: `send(channel,type,payload)` and `poll()` callbacks
- Freeze motor protocol as-is (from README)

### Phase 1 — STM32 bring-up minimal
- UART DMA RX + IDLE + TX DMA working on one port
- timebase + 1kHz timer tick
- LED + buttons
- “hello telemetry” over TELEM to ESP32 link (loopback if needed)

### Phase 2 — Motor comm port
- Port motor comm manager from ESP32 to STM32
- Get 1 motor node stable, then scale to 4
- Add motor status telemetry

### Phase 3 — Sensors
- Add IMU SPI/I2C reads + timestamps
- Add LiDAR UART parsing
- Feed existing estimator inputs

### Phase 4 — Logging + file service
- SD logging ring buffer + flush
- implement FILE list/read chunk API over link

### Phase 5 — ESP32 gateway
- Bluepad32 teleop → CMD
- Wi-Fi portal endpoints proxy to STM32 (telemetry + logs)
- STM32 firmware upload path (staged transfer)

---

## 11) Acceptance Tests (Definition of Done)
### Link / transport
- CRC32 rejects corrupted frames
- sustained TELEM stream at 50–200 Hz without dropping control loop deadlines
- Link timeout (no frames) triggers neutral commands within 250 ms

### Motor comm
- 4 motor nodes simultaneously:
  - command at control rate (1 kHz or decimated as designed)
  - status received at expected rate
  - timeout fault triggers disarm

### LiDAR
- 3 TFmini streams decoded concurrently with no cross-talk
- down LiDAR triggers safety action (configurable)

### Logging + portal
- ESP32 `/logs/list` returns current SD log list
- `/logs/download` downloads a multi-MB log without corruption (CRC/size check)
- `/telemetry` streams continuously while motors are running

### Firmware update
- upload STM32 firmware via portal, verify:
  - full image transfer OK (CRC match)
  - STM32 flashes and boots new image
  - rollback behavior defined if CRC fails (at minimum: do not commit)

---

## 12) Deliverables (What Codex must produce)
1) New repo layout: `common/`, `stm32/`, `esp32/`
2) `Protocol.md` + `Transport.md` documenting the agreed protocol
3) STM32 drivers:
   - `uart_dma_idle.*` (multi-instance)
   - `i2c_service.*`
   - `lidar_tfmini.*`
   - `sd_logger.*`
   - `tft_ui.*` (minimal)
4) STM32 app:
   - `scheduler_baremetal.*` (task layout per spec)
   - integration points for existing estimator + LQR modules
5) ESP32 gateway:
   - `bt_bluepad32.*`
   - `wifi_portal.*` endpoints above
   - `stm32_link_transport.*`
6) Minimal test tools:
   - host-side unit test for COBS+CRC32 framing
   - file chunking correctness test (offset/length/CRC)

---

## 13) Constraints / Non-goals
- No RTOS required on STM32 (bare-metal cooperative model)
- No Wi-Fi / BT stack on STM32
- Do not redesign motor protocol; only port/refactor it
- Avoid dynamic allocation in STM32 control path
- ISR work must be minimal; parsing and heavy logic in main loop

---
