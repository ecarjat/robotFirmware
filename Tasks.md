# Bare-metal “Task Layout” (STM32H723 Robot Brain)

This defines a **bare-metal (no RTOS)** execution model for the robot brain. The goal is:
- deterministic **1 kHz** control,
- **DMA-driven** I/O,
- zero blocking in time-critical paths,
- clean separation between “hard real-time” and “background” work.

---

## 0) Core Ideas

1) **One hard real-time control loop** driven by a hardware timer (e.g., TIMx @ 1 kHz).  
2) Everything I/O heavy uses **DMA + short ISRs**:
   - UART RX: DMA circular + **IDLE line interrupt** to signal “new bytes”
   - I2C: DMA or interrupt-driven transfers; keep each transfer short
3) The `while(1)` main loop runs **cooperative background tasks** in priority order.
4) All ISRs do **minimum work**: timestamp + copy pointers/length + set flags.
5) Use **ring buffers** for: UART RX parsing, log queue, motor commands, UI updates.

---

## 1) Timing & “Task Classes”

### 1.1 Hard Real-Time (must meet deadline, no blocking)
- Control loop (EKF predict/update, LQR, safety gating)
- Motor command output scheduling

### 1.2 Soft Real-Time (bounded latency; can slip occasionally)
- UART frame parsing (motors, LiDAR, ESP32)
- Sensor sampling aggregation (IMU reading scheduling)

### 1.3 Background (best effort)
- SD logging flush
- TFT UI refresh
- Diagnostics, stats, config writes

---

## 2) Top-Level Main Loop Structure

Pseudo-flow:

- Init clocks, GPIO, DMA, USARTs, I2C, SD, TFT
- Start DMA RX on all UARTs
- Start control timer interrupt at 1 kHz
- `while(1)`:
  - run pending parsers (motor/LiDAR/BT)
  - run slow sensor reads (magnetometer)
  - run log flush step (non-blocking chunked)
  - run UI update step (rate-limited)
  - run health monitoring step
  - optional low-power wait-for-interrupt when idle

---

## 3) Interrupts & Their Responsibilities

### 3.1 Control Timer ISR (TIMx @ 1 kHz) — “CONTROL_TICK”
Responsibilities:
- Snapshot “latest sensor values” (already decoded / last known)
- Run safety checks (timeouts, faults)
- Run EKF step(s)
- Compute control (LQR/PID)
- Write motor command targets to TX queues
- Trigger motor UART TX DMA if idle

Rules:
- No dynamic allocation
- No SD writes
- No display updates
- No I2C transactions inside ISR (unless extremely short and proven safe)

Outputs:
- `controlTickCount++`
- `motorCmdQueue.push(...)` for each motor node

### 3.2 UART IDLE ISR (per UART)
Triggered when RX line goes idle:
- Determine number of new bytes in DMA circular buffer
- Update per-UART “rx write index”
- Set a flag: `uartX_rxPending = true`
- Optionally timestamp last-received

Parsing is NOT done in ISR.

### 3.3 DMA TX Complete ISR (per UART)
- Mark TX channel idle
- If TX queue not empty, start next DMA TX block

### 3.4 EXTI ISR (IMU interrupt lines, buttons)
- Set flags for “new IMU sample ready”
- Debounce buttons via timestamp or defer to main loop

---

## 4) Data Flow by Subsystem

### 4.1 IMU Sampling (ICM-45686 + BMI270)
Preferred model:
- IMU provides data-ready interrupt at fixed rate (e.g., 1 kHz)
- EXTI ISR sets `imu1_drdy = true` / `imu2_drdy = true`
- Main loop executes the I2C read as soon as possible (fast path), stores into `latestImu{1,2}` + timestamp
- Control tick uses the latest available IMU data (with freshness checks)

If I2C reads are too slow:
- Reduce IMU ODR to 500 Hz
- Or switch to SPI later (not required initially)

### 4.2 Magnetometer (BMM150)
- Read at 25–100 Hz (timer in main loop)
- Store `latestMag` + timestamp

### 4.3 LiDAR (3× TFmini Plus)
- Each LiDAR UART RX via DMA
- Main loop parses frames from ring buffer when `uart_lidar*_rxPending`
- Update `latestLidarFront/Back/Down` + timestamp

### 4.4 Motor Nodes (4× UART)
- RX: DMA + IDLE; parse small status packets
- TX: command packets (torque/vel/pos) scheduled by control tick

Motor nodes should include:
- status: encoder position/velocity, current/voltage (if available), fault flags
- ack or sequence id optional

### 4.5 ESP32 BT Coprocessor
- RX: DMA + IDLE; parse `BtPacket` (forward/turn/buttons/connected/seq)
- Control tick reads the latest `btCmd` (freshness check)
- If stale/disconnected -> command neutral (0 forward/turn) and/or disarm

### 4.6 Logging (SD card)
- Log producer: control tick (and parsers) push small log records into a RAM ring buffer
- Log consumer: main loop flushes in chunks (e.g., 4–16 KB) to SD
- Never block control tick; if buffer near full, drop oldest or reduce log detail

### 4.7 TFT UI
- Update rate-limited (10–20 Hz)
- Pull snapshot of telemetry (angle, mode, battery, faults)
- Render minimal UI; use DMA for SPI where possible
- If rendering is heavy, split into incremental steps (one widget per loop)

---

## 5) Cooperative “Bare-Metal Tasks” in Main Loop

These are not threads—just functions called repeatedly if their “work pending” flag is set.
Order is from most to least time-sensitive.

### 5.1 Task: Parse UARTs (High Priority)
Runs every loop iteration (or when flagged).
- `Task_ParseMotorUarts()`
- `Task_ParseLidarUarts()`
- `Task_ParseBtUart()`

Each task:
- consumes bytes from DMA ring
- finds frames
- validates CRC/checksum if present
- updates `latest*` structs
- updates “lastSeen” timestamps

### 5.2 Task: IMU Read Service (High Priority)
- If `imu*_drdy` flag set:
  - clear flag
  - kick I2C DMA read (non-blocking) OR do short polled read
  - on completion callback, update `latestImu*`

### 5.3 Task: Slow Sensors (Medium)
- Magnetometer periodic read (25–100 Hz)
- Battery monitor periodic read (if present)

### 5.4 Task: Motor TX Service (Medium)
If you do not fully handle TX in ISR:
- Check per-motor TX idle
- Pop queued packets
- Start DMA TX

### 5.5 Task: Logging Flush (Low)
- If log buffer has >= chunk threshold or periodic timer elapsed:
  - write one chunk to SD
  - return immediately (do not loop until empty)

### 5.6 Task: UI Refresh (Low)
- If `uiNextUpdateDue`:
  - update one “page” or a small set of widgets
  - never block long

### 5.7 Task: Health Monitor (Low)
- Watchdog feed
- Compute loop timing stats
- Check timeouts:
  - IMU stale
  - motor nodes stale
  - LiDAR stale
  - BT stale
- Set fault flags / disarm conditions

### 5.8 Idle
- If nothing pending: `__WFI()` (wait for interrupt)

---

## 6) Priority & Latency Targets

### 6.1 Control Loop
- 1 kHz target (1 ms budget)
- Aim for < 200–400 µs typical execution

### 6.2 IMU freshness
- Use last timestamp
- If older than 3–5 ms at 1 kHz, treat as stale and reduce trust or fault

### 6.3 BT command freshness
- BT packets at 50–200 Hz
- If stale > 200 ms: set commands to neutral and optionally disarm

### 6.4 Logging
- Accept occasional backlog
- Never block control tick

---

## 7) Suggested State Machines

### 7.1 Robot Mode
- BOOT
- DISARMED
- ARMED (ready)
- BALANCING
- FAULT (latched)
- CALIBRATION

Buttons + BT can request transitions; control tick enforces safety.

### 7.2 Fault Handling
Fault sources:
- IMU missing/stale
- Motor node missing/stale
- Over-tilt
- Over-current (if available)
- Watchdog / internal error

Fault behavior:
- immediately command torque = 0 to all motors
- latch fault until BTN reset or power cycle (configurable)

---

## 8) Minimal Public “Task API” (Function List)

Main loop calls (in this order):

1) `Task_ServiceImuReads();`
2) `Task_ParseBtUart();`
3) `Task_ParseMotorUarts();`
4) `Task_ParseLidarUarts();`
5) `Task_ServiceMotorTx();`
6) `Task_ServiceSlowSensors();`
7) `Task_HealthMonitor();`
8) `Task_LogFlushStep();`
9) `Task_UiUpdateStep();`
10) `Idle_WFI_IfNoWork();`

ISRs:
- `ISR_ControlTick_TIMx();`
- `ISR_UartIdle_USARTx();` (x = all used UARTs)
- `ISR_DmaTxComplete_USARTx();`
- `ISR_EXTI_ImuDrdy();`
- `ISR_EXTI_Buttons();`

---

## 9) Notes / Guardrails

- Avoid printf in ISR.
- Prefer fixed-size buffers and structs; no malloc.
- Keep a global `now_us()` monotonic timebase (DWT cycle counter or TIM).
- Use sequence numbers for motor and BT packets to detect drops.
- Consider CRC8 on all packets for robustness.

---
