# Motor Control (Phase 3) — Historical UART Driver Spec

> This document describes the retired STM32F103/SimpleFOC UART architecture.
> The deployed hardware uses GIM8108-8 motors with GDS68 drivers on CAN Simple.
> Use `docs/SteadywinCan.md` for the active wheel protocol and
> `docs/SteadyWinUsbProvisioning.md` for commissioning.

This document defines how to port the legacy motor protocol
(`legacy/src/drivers/motors_binaryIO.{cpp,h}`) to STM32 for Motor 1 and 2, per
`MIGRATION_AGENT.md`. The STM32 implementation must preserve protocol behavior
and telemetry semantics while using CubeMX/HAL UART + DMA.

## Scope
- Bring up **Motor 1** and **Motor 2** only.
- Port the SimpleFOC BinaryIO protocol usage from legacy.
- Provide telemetry (velocity, status) and command outputs (Iq, torque).
- Keep control-loop safe: non-blocking, bounded per tick.

## Source of Truth
- Pin/USART mapping and baud rate are taken from `stm32Controller.ioc`.
- Wiring overview is reflected in `Pinmap.md`.

## Hardware / UART Mapping (CubeMX)
Motor links are dedicated UARTs with DMA:
- **Motor 1**: USART1 — PB14 (TX), PB15 (RX), **460800 baud**
- **Motor 2**: USART6 — PC6 (TX), PC7 (RX), **460800 baud**

DMA (from `.ioc`):
- RX: circular DMA on both USART1/USART6
- TX: normal DMA on both USART1/USART6

## Protocol Summary (legacy behavior)
Legacy uses `SimpleFocBinaryComms` to read/write SimpleFOC registers.
The STM32 driver must replicate the following register operations:

Registers (from legacy, via SimpleFOCRegister):
- `REG_TARGET` (float) — command target
- `REG_ENABLE` (u8) — 0/1 enable
- `REG_CONTROL_MODE` (u8) — control mode
- `REG_TORQUE_MODE` (u8) — torque mode selection (if required by driver)
- `REG_MODULATION_MODE` (u8) — PWM modulation (set to 1)
- `REG_VELOCITY` (telemetry)
- `REG_STATUS` (telemetry)
- `REG_TELEM_DOWNSAMPLE` (u32) — throttle telemetry during config

Control modes (`ControlMode` enum values are sent directly):
- `TORQUE = 0x00`
- `VELOCITY = 0x01`
- `ANGLE = 0x02`
- `VELOCITY_OPENLOOP = 0x03`
- `ANGLE_OPENLOOP = 0x04`

Drivers are configured to operate in **TORQUE** mode.

Telemetry rate:
- base telemetry: 1000 Hz
- target telemetry: 500 Hz
- ack timeout: 200 ms

Telemetry payload layout:
- `velocity` float (LE, rad/s)
- `status` u8

## Driver API (STM32)
Provide a C API equivalent to legacy (names may be adjusted for STM32):
- `motor_link_init()`
- `motor_link_enable(bool on)`
- `motor_link_set_control_mode(ControlMode mode)`
- `motor_link_set_wheel_Iq(float left_A, float right_A, float max_A)`
- `motor_link_set_wheel_torques(float left_Nm, float right_Nm, float max_Nm)`
- `motor_link_get_wheel_velocities(float *left, float *right)`
- diagnostics: parser drops, telemetry stale/late counters

## Initialization Sequence (STM32)
1) Configure UARTs (USART1/USART6) + DMA as per CubeMX.
2) Start RX DMA circular for both UARTs.
3) Wait briefly for any UART traffic (best-effort liveness check).
4) Throttle telemetry during config:
   - `REG_TELEM_DOWNSAMPLE = 10000` (best-effort, no ACK wait).
5) Set telemetry registers (blocking until ACK):
   - register list: `{REG_VELOCITY, REG_STATUS}`
6) Configure driver:
   - `REG_CONTROL_MODE = TORQUE`
   - `REG_MODULATION_MODE = 1` (SpaceVectorPWM)
   - `REG_ENABLE = 0`
7) Restore telemetry rate:
   - `setTelemetryRateHz(500)` (blocking until ACK)
8) Mark driver initialized.

Notes:
- Legacy blocks boot until telemetry register setup succeeds; STM32 should do
  the same for bring-up (Phase 3 acceptance).
- Direction signs default to +1/+1 until persistence is added (Phase 5).

## Runtime Behavior
### Enable / Disable
`motor_link_enable(on)`:
- Write `REG_ENABLE` on both drivers (best-effort, no ACK wait).
- Drive a **hardware enable pin** (to be defined in CubeMX) high/low.
- Keep local `enable_applied` state for gating commands.

### Command Outputs
Per-cycle commands must be non-blocking:
- If a config or enable write is in progress, skip the command for this cycle.
- If motors are disabled, send zero targets.
- Clamp to:
  - torque: `PARAM_MAX_WHEEL_TORQUE`
  - current: `Iq_max` (set by motor limits / safety)
- Apply per-wheel direction sign (±1).
- `Iq_ref` sign convention: positive = forward torque.

Torque vs current:
- **Torque mode target is amps (Iq).**
- `motor_link_set_wheel_Iq()` sends Iq targets directly (A).
- `motor_link_set_wheel_torques()` converts torque→current using
  `docs/MainControl.md`:
  - `Iq_ref ≈ tau / Kt`
  - `Kt ≈ 60 / (2π * KV_rpm_per_V)` when only KV is known.

### Telemetry Parsing
RX DMA feeds a binary frame parser that:
- tracks parser drops
- decodes the latest telemetry frame
- updates timestamp per field (velocity/status)

Telemetry validity:
- `TELEMETRY_WARN_MS` => log once when late
- `TELEMETRY_STALE_MS` => mark stale; return last known values + false

## Concurrency / Timing
STM32 is bare-metal; no FreeRTOS in Phase 3.
Use a simple lock:
- `config_in_progress` flag for init and control-mode switches
- `tx_busy` flag for the UART TX DMA
- control tick uses a try-lock; if busy, skip output to preserve bounded timing

## Diagnostics
Expose counters:
- parser drops (left/right)
- telemetry stale counts (left/right)
- telemetry late counts (left/right)
- last ACK failure counts

## Pass-through (Optional)
Support a passthrough mode for direct motor console access:
- suspend RX parser
- switch host UART baud to selected motor
- on release: reapply telemetry registers + rate and restore control mode

## Acceptance Checklist (Phase 3)
- Motor 1 and Motor 2 UARTs initialize at 460800.
- Telemetry registers acknowledged on both motors.
- Read velocity and status at 500 Hz without stale/late flags.
- Enable/disable is immediate and safe (zero torque when disabled).
- Can command low Iq/torque in calibration mode without blocking control tick.

## Implementation Notes
- Port or re-implement the minimum subset of the SimpleFOC BinaryIO protocol
  used in legacy (`writeRegU8/U32/F32`, `waitAck`, telemetry frames).
- Keep register IDs aligned with the motor firmware (see the protocol README in
  `MIGRATION_AGENT.md`).
- Do not block inside the 1 kHz control tick.
- Configure and keep drivers in **TORQUE** control mode.
