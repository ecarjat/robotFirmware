# Steadywin DD8015 CAN Integration Spec (Draft)

Date: 2026-01-27

Source: `docs/CAN_DOC.pdf` (Custom CAN Protocol v3.07b0, 2025-05-18)

## 1. Scope and goals
- Integrate Steadywin DD8015 motors over CAN while keeping the current `motor_link` API stable.
- Compile-time selection only (no runtime switching).
- Preserve safety behavior: zero torque on disable and telemetry-based motor timeout detection.

## 2. CAN protocol basics (from CAN_DOC.pdf)
- CAN bit rate default 1 Mbps; supported: 1M, 500k, 250k, 125k, 100k.
- Standard 11-bit ID frames; data frames only; little-endian payloads.
- Device address `Dev_addr` default 0x01; valid range 1..254.
- 0x00 is broadcast (no response). 0xFF is public address (all devices respond).
- Slave responds with StdID = `Dev_addr`, even if host used `(0x100 | Dev_addr)`.
- Commands can be sent to `Dev_addr` or `(0x100 | Dev_addr)`.
- MIT control frames use StdID with bit10 set: `0x400 | Dev_addr` (and `0x400 | 0x100 | Dev_addr`).
- CAN bus access uses the STM32H7 FDCAN peripheral (no UART adapter).
- A CAN transceiver (e.g., TJA1050) is required between FDCAN TX/RX pins and CANH/CANL.
- Requests use StdID = `0x100 | Dev_addr` (direction separation), responses are always StdID = `Dev_addr`.

## 3. Data types and conversions
- 4s = signed 32-bit little-endian; 2s = signed 16-bit little-endian.
- 2u/4u = unsigned little-endian.
- Current: value * 0.001 A.
- Speed: value * 0.01 rpm; rad/s = rpm * 2*pi / 60.
- Position (counts): 1 rev = 16384 counts; deg = counts * (360 / 16384).

## 4. Command summary (relevant subset)

### System and status
- 0xA0: Read boot/app/hw/CAN protocol versions.
- 0xAE: Read bus voltage/current, temperature, run mode, fault bits.
- 0xAF: Clear faults; returns current fault byte.
- 0xCF: Disable motor output (free state). Response mirrors 0xAE payload.

### Control
- 0xC0: Q-axis current control (torque proxy).
- 0xC1: Speed control (rpm * 0.01).
- 0xC2: Absolute position control (counts).
- 0xC3: Relative position control (counts).

### Optional MIT mode
- 0xF0: Configure Pos_Max/Vel_Max/T_Max.
- 0xF1: Read MIT mode state (pos/vel/torque/status).
- MIT control command: data-only frame with StdID bit10 set.

## 5. Mapping to current firmware API

### 5.1 motor_link_init()
- Configure FDCAN for classic CAN (11-bit IDs), 1 Mbps nominal bit rate, no FD/BRS.
- Use FDCAN kernel clock = 100 MHz (PLL2) with nominal timing: prescaler 5, seg1 15, seg2 4, sjw 1.
- Configure message RAM, RX FIFO0, and standard ID filters for the motor IDs.
- User code must configure the actual filter entries and activate RX/bus-off notifications (CubeMX does not create filters or enable interrupts).
- Optionally query 0xA0 and 0xB0 for diagnostics.
- Use left motor address `0x01` and right motor address `0x02` on the CAN bus.

### 5.2 motor_link_enable(on)
- If `on`:
  - Send 0xAF (clear fault).
  - Send zero command in the active control mode.
- If `off`:
  - Send 0xCF (disable output/free state).
  - Response is same payload as 0xAE.

### 5.3 motor_link_set_control_mode(mode)
- `MOTOR_CONTROL_TORQUE` -> use 0xC0 (Q-axis current).
- `MOTOR_CONTROL_VELOCITY` -> use 0xC1 (speed).
- `MOTOR_CONTROL_ANGLE` -> use 0xC2 (absolute position) or 0xC3 (relative) if needed.
- Open-loop modes are not supported. Do not map or silently fall back.

### 5.4 motor_link_set_wheel_Iq(left_A, right_A, max_A)
- Use 0xC0 per motor.
- Payload: 4s current in 0.001 A units, little-endian.
- Apply motor direction sign before encoding; clamp to `max_A`.
- Response payload is the same as 0xA1 (measured current).

### 5.5 motor_link_set_wheel_torques(left_Nm, right_Nm, max_Nm)
- Convert torque to current using Kt (Nm/A), then send via 0xC0.

### 5.6 motor_link_get_wheel_velocities(left, right)
- Request compact telemetry with 0xA4 for each motor; response includes speed as 2s in 0.01 rpm units.
- Convert speed to rad/s. (0xA4 also carries temp, Q current, and single-turn angle.)
- Mark stale if no response within `MOTOR_LINK_TELEM_STALE_MS`.
- Schedule requests at `MOTOR_LINK_TELEM_RATE_HZ` (default 500 Hz) to match current UART telemetry cadence.

### 5.7 Status and fault handling
- 0xAE: returns bus voltage/current, temperature, run mode, and fault bits.
- Fault auto-report: device sends 0xAE every 200 ms on fault. Accept unsolicited frames.
- 0xAF: clears faults and returns current fault byte.
- Prefer compact telemetry reads with 0xA4 (temp, Q current, speed, single-turn angle) to reduce bus load.
- Bus-off: stop scheduling TX, mark motor link invalid, set fault flag, and deassert motor enable GPIO (if present).

## 6. Compile-time integration plan
1) Add a backend interface and facade: `app/drivers/motors/motor_backend.{h,c}` and keep `motor_link.c` as the facade.
2) Add a Steadywin CAN backend: `app/drivers/motors/motor_backend_steadywin_can.c` (impl in `.inc`).
3) Keep the UART backend as a separate backend: `app/drivers/motors/motor_backend_robust_uart.c` (impl in `.inc`).
4) Introduce a compile-time switch (e.g., `MOTOR_BACKEND_STEADYWIN_CAN`) in `app_config.h` or build flags.
5) Configure FDCAN (STM32H7) for standard IDs, 1 Mbps, filters for left/right IDs, and enable RX FIFO0 + bus-off interrupts.
6) Add user-code filter entries (StdID 0x01/0x02) and activate FDCAN notifications (RX FIFO0 new message, bus-off).
7) Implement TX helpers for 0xC0/0xC1/0xC2/0xC3/0xA4/0xAE/0xAF.
8) Implement RX parsing for responses; update telemetry timestamps and fault status.
9) Implement a periodic request scheduler in `motor_link_poll` (interleave left/right 0xA4) using `MOTOR_LINK_TELEM_RATE_HZ`.
10) Wire `motor_link_enable(false)` to 0xCF and ensure zero torque is sent before disable.
11) Add parameters for left/right `Dev_addr` (and bitrate if needed) to param storage/config.
12) Bench-test with single motor, then dual motor; verify telemetry timing and fault handling.

## 7. Notes and open items
- Use `MOTOR_LINK_TELEM_RATE_HZ` (default 500 Hz); adjust only if CAN bus load requires it.
- Open-loop modes are explicitly unsupported for this backend.
- Address provisioning is out of scope; assume the drives are already configured with the target `Dev_addr` values.
