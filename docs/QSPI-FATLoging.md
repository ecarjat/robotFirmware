# QSPI “Blackbox” Flight Recorder Spec (QSPI → SD dump)
Target: STM32H723 (WeAct H723VG) with on-board QSPI NOR flash **W25Q64 (8 MB)**.  
Goal: Always-on, low-latency, real-time-safe logging of control data (IMU raw, EKF, wheel velocity, PID), with the ability to **dump a time window to SD** on demand or on fault.

This spec is **Agent-ready**: defines on-flash layout, record format, tasks, APIs, triggers, SD export format, and **full app integration**.

---

## 0) Why QSPI-first
- SD can stall unpredictably (filesystem + wear leveling).  
- QSPI NOR is deterministic if we control erase/write patterns.  
- We want “the crash” logs guaranteed even if SD/USB/ESP32 are busy.

---

## 1) Requirements
### 1.1 Functional
- Log at **control tick rate** (default **500 Hz**, configurable).
- Record must include:
  - IMU raw (active IMU used by EKF this tick)
  - dual-IMU health metrics (gyro_diff, acc_angle_diff, vib, active_imu, accel_gated)
  - EKF fields (theta, thetaDot, biases, optional yaw/yawRate)
  - wheel velocities (wL, wR), derived v
  - PID internals (theta_ref, error, P/I/D, u_common/u_turn, uL/uR, saturation flags)
- Maintain a **circular buffer** storing last **N seconds** (N derived from flash capacity and record size).
- On trigger, dump a selected time window (e.g. last 10–60 s) to **microSD** as a file.
- Support “dump by host” over USB/ESP32 FILE channel (optional).

### 1.2 Non-functional
- Control loop must never block on flash erase.
- Logging must be bounded-time per tick.
- Corruption tolerance: power loss at any moment must not brick logging; recovery must find last valid records.
- Wear minimized: avoid erasing the same sector too frequently.

---


## 2) Flash characteristics (W25Q64 assumptions)
(Verify exact erase sizes from datasheet used in your BSP; these are typical.)
- Total size: **8 MiB**
- Page program: **256 B pages**
- Erase: **4 KiB sectors**, optionally **32 KiB** and **64 KiB blocks**
- Write: can only change bits 1→0; erase resets to 0xFF.

Design must:
- write sequentially in pages
- erase sectors in background before they are needed

## 2.1 OCTOSPI / CubeMX configuration (required)
This project uses the on-board **W25Q64 (8 MB)** over OCTOSPI.

### 2.1.1 Clocking
- OCTOSPI kernel clock: **200 MHz** (as configured in RCC).
- OCTOSPI serial clock is derived as:
  - `f_qspi = f_octospi / (prescaler + 1)`

### 2.1.2 Memory configuration
CubeMX parameters MUST match W25Q64:
- **Memory Type:** set to **Micron** (Winbond-compatible command set in Cube/HAL).
- **Device Size:** **23** (because `2^23 = 8 MB`).
  - NOTE: device size 22 would incorrectly limit addressing to 4 MB and corrupt the ring.
- **Dual Quad mode:** Disabled.
- **Clock Mode:** Low (Mode 0).
- **Sample Shifting:** Half cycle.

### 2.1.3 Cache coherency (mandatory)
Because H7 has DCache and MDMA bypasses it:
- If buffers are in cacheable RAM:
  - Before RAM→QSPI program: `SCB_CleanDCache_by_Addr(buf, len)`
  - After QSPI→RAM read: `SCB_InvalidateDCache_by_Addr(buf, len)`
- Preferred: place staging buffers (`qspi_stage_buf`, `dump_buf`) in an MPU **non-cacheable** region to avoid manual maintenance.

---

## 3) Memory map (partitioning)
Reserve QSPI for logs only (simplest). If you need other use later, carve partitions.

### 3.1 Partitions
- **[0x000000 .. 0x000FFF] 4 KiB**  : `LOG_META_SLOT0` (metadata, config, pointers, recovery info)
- **[0x001000 .. 0x001FFF] 4 KiB**  : `LOG_META_SLOT1` (independent recovery copy)
- **[0x002000 .. 0x7FFFFF] ~8184 KiB**: `LOG_RING` (circular record storage)

All addresses are offsets from QSPI base.

---

## 4) Log data model

### 4.0 Log field selection (runtime bitmask)
Logging fields MUST be runtime-configurable via a bitmask stored in `robot_params_t`.

- Add to `robot_params_t`:
  - `uint32_t log_fields_mask;`
- The logger uses this mask to decide which groups of fields are populated in each record.
- The record format on flash remains **fixed-size** for deterministic writes; when a field group is disabled, the corresponding bytes are written as **zero** (or a defined NaN pattern for floats), and the record CRC still covers the full record.
- The mask is recorded in `LogMeta` snapshot at dump time (and optionally periodically) so offline tools know what was enabled.

**Bit assignments (stage-1):**
- `LOGF_IMU_RAW       = (1u << 0)`  // acc_raw + gyro_raw of active IMU used by EKF
- `LOGF_IMU2_HEALTH    = (1u << 1)`  // gyro_diff, acc_angle_diff, vib, active_imu, accel_gated
- `LOGF_EKF            = (1u << 2)`  // theta, thetaDot, biases (+ optional yaw/yawRate if present)
- `LOGF_WHEELS         = (1u << 3)`  // wL, wR, v, yawRate_enc
- `LOGF_PID            = (1u << 4)`  // theta_ref, error, P/I/D, u_common/u_turn, uL/uR
- `LOGF_POWER          = (1u << 5)`  // Vbat, brownout flags, torque derate factor (if available)
- `LOGF_DIAG           = (1u << 6)`  // dropped counters, active state, fault codes (if desired)

**Defaults:**
- Default `log_fields_mask = LOGF_IMU_RAW | LOGF_IMU2_HEALTH | LOGF_EKF | LOGF_WHEELS | LOGF_PID`.

**Implementation note:**
- Control loop should only compute/format what is enabled to reduce CPU load (but must still write a full fixed-size record).

### 4.1 Record cadence
- One record per control tick.
- If control is 400 Hz, use 400 Hz; record includes timestamp so offline tools can resample.

### 4.2 Record format overview
- Fixed-size **binary record**, padded to 4 bytes.
- Record size chosen to fit within a small number of pages (prefer 128B, 160B, 192B, or 256B).
- Each record includes a CRC to validate partial writes.

### 4.3 Header + Record structs
#### 4.3.1 `LogMeta` (stored in LOG_META)
Stores the ring write pointer and generation count.

Fields (little-endian):
- `magic[8] = "R2WLOG1\0"`
- `version_u16`
- `record_size_u16`
- `rate_hz_u16`
- `log_fields_mask_u32` (copy of `robot_params_t.log_fields_mask`)
- `reserved_u16`
- `ring_start_u32` (= 0x002000)
- `ring_size_u32` (= total bytes in LOG_RING)
- `write_addr_u32` (next write position within LOG_RING)
- `wrap_count_u32` (increments each wrap)
- `boot_count_u32`
- `last_dump_id_u32`
- `meta_crc32_u32` (CRC of all fields except this one)

Update policy:
- Do NOT rewrite LOG_META every tick.
- Update on:
  - boot (boot_count++)
  - periodically (e.g., 1 Hz)
  - on dump event
- Use **two-slot meta** with sequence numbers to survive power loss:
  - `LOG_META_SLOT0` and `LOG_META_SLOT1` each occupy their own 4 KiB erase sector.
  - Erase only the inactive slot, program the candidate, and read it back with
    CRC validation before treating it as committed; the previously verified
    slot remains intact throughout the update.

#### 4.3.2 `LogRecord` (stored in LOG_RING)
Recommended to keep as compact as practical.

Suggested fields:
- `magic_u16 = 0xA55A`
- `version_u8`
- `flags_u8` bitmask:
  - bit0 accel_gated
  - bit1 imu_fallback_active
  - bit2 uL_sat
  - bit3 uR_sat
  - bit4 fallen
  - bit5 armed
- `seq_u32` (increments each tick)
- `t_us_u32` (microseconds since boot; wraps OK)
- `active_imu_u8` (1=BMI270, 2=ICM42688)
- `reserved3_u8[3]`

IMU raw (active IMU used in EKF this tick):
- `acc_raw_i16[3]`  (sensor units; store scale in LogMeta or constants)
- `gyro_raw_i16[3]`

Dual-IMU health (float or fixed):
- `gyro_diff_dps_f32`
- `acc_angle_deg_f32`
- `vib_grms_f32`

EKF outputs:
- `theta_rad_f32`
- `thetaDot_rads_f32`
- `gyro_bias_pitch_rads_f32` (or bias xyz if available)
- optional: `yaw_rad_f32`, `yawRate_rads_f32` (if you compute them)

Wheels / motion:
- `wL_rads_f32`
- `wR_rads_f32`
- `v_mps_f32`
- `yawRate_enc_rads_f32` (optional)

PID internals (balance + speed bias + steering):
- `theta_ref_rad_f32`
- `e_theta_rad_f32`
- `P_f32`, `I_f32`, `D_f32`
- `u_common_f32`
- `u_turn_f32`
- `uL_cmd_f32`, `uR_cmd_f32`  (command in “torque proxy” units; define clearly)

Trailer:
- `crc32_u32` (CRC of entire record excluding crc field)

Record size target:
- If too large, drop optional yaw fields or store some values as i16 fixed-point.

---

## 5) Ring buffer mechanics
### 5.1 Write pointer
- `write_addr` always points to next write location in LOG_RING.
- After writing a record, advance by `record_size`.
- When `write_addr` reaches `ring_start + ring_size`, wrap to `ring_start` and increment `wrap_count`.

### 5.2 Erase strategy (critical)
NOR requires erase before write. We must never erase in the control tick.

Plan:
- Divide LOG_RING into **erase units** = 4 KiB sectors.
- Maintain a background “pre-erase” task that keeps **K sectors erased ahead** of `write_addr`.

Parameters:
- `ERASE_UNIT = 4 KiB`
- `PREERASE_AHEAD_SECTORS = 8` (32 KiB ahead) stage-1 default

Algorithm:
- Compute `next_needed_sector` = sector containing `write_addr`.
- Ensure sectors in `[next_needed_sector .. next_needed_sector + PREERASE_AHEAD_SECTORS]` are erased (0xFF).
- The erase task runs at low priority and can be suspended when CPU is busy, but should keep up on average.

### 5.3 Atomicity & power loss
On power loss:
- Last record may be partially written.
- Recovery scans forward from `ring_start` (or from meta pointer) and validates by:
  - magic
  - crc32
  - monotonic seq (optional)
Stop at first invalid record; that is the end-of-log.
If wrap_count>0, also scan to find most recent valid region.

### 5.4 Finding “last N seconds” window
We want to dump last T seconds. To do that robustly:
- Use `seq` and/or `t_us` to locate boundary.
- Approach:
  1) Determine `latest_record_addr` during runtime as the last successfully written record.
  2) For dump, walk backwards by `N_records = T * rate_hz`:
     - compute `start_addr = latest - N_records*record_size`, wrapping as needed.
  3) If records can be dropped, use CRC validation during dump and skip invalid.

---

## 6) Runtime tasks & ISR interaction
### 6.1 Control tick path (hard real-time)
- Control loop creates a `LogRecord` in RAM and enqueues to a lock-free ring queue.
- It must be O(1) and never block.

### 6.2 Logger task (high priority but below control)
- Drains RAM queue and writes to QSPI in batches.
- Writes must be page-aligned where possible:
  - coalesce records into 256B pages or larger contiguous writes.

- `QSPI writes MUST be performed using OCTOSPI indirect write + MDMA (CubeMX MDMA settings).`
- `Logger should coalesce records into page-aligned chunks (>=256 B, preferably 4 KiB) before starting an MDMA transfer.`
- `If buffers are cacheable, perform DCache clean/invalidate as specified in §2.1.5.`

### 6.3 Erase task (low priority)
- Pre-erases sectors ahead of write pointer.

### 6.4 Dump task (on trigger)
- Copies a time window from QSPI to SD file.
- Must not run in control context; run as a separate task.
- During dump, logging continues (preferred) or can be temporarily paused (allowed if simpler).

- `Dump reads SHOULD use OCTOSPI indirect read + MDMA into a RAM chunk buffer (e.g., 8–32 KiB), then write to SD in the same chunk size.`
- `If buffers are cacheable, perform DCache clean/invalidate as specified in §2.1.5.`

---

## 7) Dump-to-SD behavior
### 7.1 Trigger sources
- User action: Xbox menu “Dump last 30s”
- User action: Teleop X button (single-press) triggers `log_dump_last_seconds(DUMP_SECONDS_DEFAULT)`; require a rising-edge + 1s debounce to prevent repeated dumps while held.
- Fault: `FALLEN` state
- Watchdog imminent reset (if detectable)
- Health event: IMU switch / large gyro_diff sustained (optional)

### 7.2 Dump file format
Write a binary file to SD:
- filename: `LOGDUMP_YYYYMMDD_HHMMSS.bin` (or sequential `DUMP0001.bin`)
- contents:
  - `LogMeta` snapshot (header) + constants
  - contiguous records from start_addr..end_addr (wrapping handled by two writes)
- Optionally add a simple index trailer:
  - start/end timestamps
  - count
  - CRC of full dump

### 7.3 SD write strategy
- Write in large chunks (>= 8 KiB).
- Use FatFS `f_write` buffered; call `f_sync` at end.
- Do not call `f_sync` per chunk.

---

## 8) APIs (firmware interfaces)
### 8.1 Initialization
- `log_init(const robot_params_t* params)`:
  - init QSPI driver
  - load `LogMeta` from meta slots (choose newest valid by sequence + crc)
  - if invalid, reset the metadata and begin a fresh ring (pre-erase in background)
  - set `write_addr`
  - start tasks (logger, erase)

### 8.2 Logging (called from control)
- `log_push_record(const LogRecord* rec)`:
  - copy into RAM queue (ring buffer)
  - increments dropped counter if queue full (do not block)

### 8.3 Dump
- `log_dump_last_seconds(uint32_t seconds)`:
  - schedules dump task
  - returns immediately

### 8.4 Stats
- `log_get_stats()`:
  - dropped_records
  - qspi_write_errors
  - erase_lag (sectors not erased ahead)
  - current fill seconds

---

## 9) Parameter defaults (stage-1)
- `LOG_RATE_HZ = 500`
- `LOG_FIELDS_MASK_DEFAULT = LOGF_IMU_RAW | LOGF_IMU2_HEALTH | LOGF_EKF | LOGF_WHEELS | LOGF_PID`
- `RECORD_SIZE = 160 bytes` (target; adjust after struct finalization)
- `RAM_QUEUE_BYTES = 256 KiB` (≈ 3–6 seconds of buffering depending on record size)
- `PREERASE_AHEAD_SECTORS = 8` (32 KiB)
- `DUMP_SECONDS_DEFAULT = 30`
- `META_UPDATE_PERIOD_MS = 1000` (update meta pointer once per second, alternating slots)

---

## 10) Capacity estimate (rule of thumb)
Capacity seconds:
- `seconds = LOG_RING_BYTES / (record_size * rate_hz)`

Example with ~8,188 KiB ring (~8,388,608 - 4096 meta - 4096 alignment ≈ 8.38MB, adjust):
- record 160B @ 500Hz → 80,000 B/s → ~100 s
- record 128B @ 500Hz → 64,000 B/s → ~130 s
- record 192B @ 500Hz → 96,000 B/s → ~85 s

Stage-1 aim: **~60–120 seconds** blackbox.

---

## 11) Integration points
- Logging fields sourced from:
  - IMU driver: raw acc/gyro used this tick
  - EKF: theta/thetaDot/bias
  - Encoders: wL/wR
  - Controller: PID terms + outputs
  - Dual-IMU logic: gyro_diff/acc_angle_diff/vib/active_imu/accel_gated

- Triggers:
  - On FALLEN → `log_dump_last_seconds(30)` then keep logging
  - On Teleop X button rising edge → `log_dump_last_seconds(DUMP_SECONDS_DEFAULT)` (debounced).
  - On user menu “Dump” → choose seconds

---

## 12) Validation tests (bring-up)
1) Format + write test:
   - write 10k dummy records, power cycle mid-run, verify recovery finds last valid record.
2) Pre-erase lag test:
   - slow erase task artificially, verify logger reports erase_lag and drops gracefully.
3) Dump test:
   - fill ring, trigger dump, verify SD file created and parseable.
4) Throughput test:
   - run at 500Hz with full record, confirm no control tick overruns and no queue drops.

---

## 13) Deliverables (full integration, not just modules)
Agent MUST implement the blackbox end-to-end, including integration into the main app and parameter system.

### 13.1 New source files
Create these files under the firmware tree (paths may be adjusted to match repo conventions):
- `app/logging/blackbox_format.h` (record + meta structs, bitmask enums)
- `app/logging/blackbox.h`
- `app/logging/blackbox.c`
- `app/logging/blackbox_dump.h`
- `app/logging/blackbox_dump.c`

### 13.2 Modified existing files (integration points)
Agent MUST patch existing application code to wire logging into the system.

#### 13.2.1 `robot_params_t` (parameter system)
- Locate the definition of `robot_params_t` and add:
  - `uint32_t log_fields_mask;`
- Ensure defaults are set on cold boot / param reset:
  - `log_fields_mask = LOG_FIELDS_MASK_DEFAULT`

#### 13.2.2 Main app initialization
Agent MUST add logging initialization to the main application startup sequence.

Call order requirements:
1) HAL + clocks
2) QSPI peripheral init (and memory-mapped mode if used)
3) SD/FatFS init
4) Parameter load (`robot_params_t`)
5) `log_init(&robot_params)`
6) Start scheduler/tasks

`log_init()` MUST:
- read/validate meta slots
- start background erase task
- start logger writer task
- expose stats counters

#### 13.2.3 Control loop integration (record population)
Agent MUST integrate `log_push_record()` into the control loop tick on STM32H723. Ensure that `log_push_record()` has a minimal impact of the loop by leveraging all acceleration techniques available.

At each control tick (400–500 Hz):
- construct a `LogRecord rec;`
- `memset(&rec, 0, sizeof(rec));`
- populate always-on minimal fields (magic/version/seq/timestamp/flags)
- populate optional field groups based on `robot_params.log_fields_mask`
- compute CRC32 reusing existing method
- call `log_push_record(&rec)`

Fields must be sourced from existing modules:
- IMU driver (raw)
- dual-IMU health logic (gyro_diff, acc_angle_diff, vib, accel_gated, active_imu)
- EKF outputs
- wheel velocities
- PID internals and outputs

The control loop MUST NOT call flash/SD APIs directly.

#### 13.2.4 Teleop button integration (X button dump)
Agent MUST wire the teleop input path so that:
- On **X button rising edge** (debounced 1s): call `log_dump_last_seconds(robot_params.dump_seconds_default)`.
- Ensure the dump runs in the dump task context.

#### 13.2.6 Communication / download integration (optional)
- Add a FILE RPC to request the latest dump file list and download a dump from SD.

### 13.3 Build system updates
Agent MUST ensure the new sources are compiled and linked:
- Update `CMakeLists.txt` (or equivalent) to include `app/logging/*.c`
- Ensure QSPI, FatFS, and CRC dependencies are included

### 13.4 Host tooling
Provide a minimal host parser:
- `tools/parse_blackbox.py`
  - reads `LOGDUMP_*.bin`
  - decodes LogMeta + records
  - exports to CSV and/or plots key signals (theta, theta_ref, P/I/D, uL/uR, wL/wR, gyro_diff, vib)

### 13.5 Acceptance criteria
- Logging runs continuously with control loop enabled and does not cause missed control ticks.
- Power loss during logging does not brick the system; on reboot, logger recovers and continues.
- Teleop X button generates a dump file on SD within a bounded time.
- Dump file is parseable and contains valid CRC’d records.

END.
