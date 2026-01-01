# Motion Control Phase 4 Plan

This plan implements Phase 4 from `MIGRATION_AGENT.md`: port the estimator +
PID controller interfaces to STM32 while preserving the math, adapting only
hardware I/O and scheduling. Because the legacy modules are C++, Phase 4
uses C++ on STM32 for a low-friction port and exposes a small C-facing shim.

## Scope (Phase 4)
- Port `legacy/src/control/MotionController.*` and
  `legacy/src/estimation/StateEstimator.*` (and their dependencies) to STM32.
- Keep math/filters/gains intact; refactor sensor + motor I/O to use STM32
  drivers and protocol layer.
- Add motion modes: DISARMED, CALIBRATION, BALANCING, FAULT.
- Run control loop at 1 kHz, non-blocking.

Non-goals:
- UI, SD logging, or configuration persistence (Phase 5).
- Reworking estimator math or controller gains.

## Inputs / References
- `legacy/src/control/MotionController.*`
- `legacy/src/estimation/StateEstimator.*`
- `legacy/src/ekf/BalancerEKF.*`
- `legacy/src/util/perf_stats.*` (timing/overrun metrics)
- STM32 sensor layer: `Drivers/imu/*` + scheduler
- STM32 motor layer (Phase 3): `drivers/motors/*`

## Target Modules (STM32)
Create/extend these under `firmware/control/` (C++ unless noted):
- `MotionController.h/.cpp`: port of legacy controller math
- `StateEstimator.h/.cpp`: port of legacy estimator glue
- `ekf/BalancerEKF.h/.cpp`: port of legacy EKF core
- `motion_modes.h/.cpp`: mode state machine + safety gating
- `motion_control.h/.cpp`: **C-facing shim** called from `app_main.c`
- `types.h`, `StateEstimate.h`, `RobotParams.h`, `config_control.h`: shared types/constants

Support interfaces in `firmware/drivers/motors/` and sensor drivers are already
in place from Phases 2-3.

## Data Flow (runtime)
1) Sensor layer updates latest-only samples (IMU, mag, lidar).
2) Control tick (1 kHz):
   - Build `estimator_input_t` from latest sensor samples + timestamps.
   - Run estimator update.
   - Run controller update (PID) to compute motor setpoints.
   - Gate outputs based on motion mode and safety checks.
   - Send commands to motor layer.
3) Lower-rate telemetry/logging consumes estimator + controller outputs.

## Scheduling Model
- A hardware timer ISR sets `control_tick_flag` at 1 kHz.
- Main loop calls `control_run_1khz()` when the flag is set.
- `control_run_1khz()` is bounded and does no blocking I/O.

## Detailed Implementation Steps

### 1) Inventory legacy APIs and define STM32 equivalents
- Identify public methods and data structures in:
  - `MotionController.{cpp,h}`
  - `StateEstimator.{cpp,h}`
  - `BalancerEKF.*`
- Create a mapping table in `firmware/control/README` (or comments in headers)
  that lists legacy functions and their STM32 wrappers.
- Decide which functions stay verbatim (math) vs. get adapted (I/O).

### 2) Define core types and boundaries (headers first)
Mirror legacy headers in STM32 `control/`:
- `types.h`: `ImuReading`, `MotionCommand`, etc. (C++-friendly, no Arduino)
- `StateEstimate.h`: EKF output struct
- `RobotParams.h` + `config_control.h`: constants and robot params
- `motion_modes.h`: `motion_mode_t` enum + transition API

Add a C interface in `motion_control.h` so `app_main.c` can drive the loop
without pulling C++ symbols directly.

### 3) Port estimator math (no I/O inside)
- Move EKF + estimator math into `StateEstimator.cpp` + `BalancerEKF.cpp`
  with the same equations and constants as legacy.
- Keep all filters/gains numerically identical; only replace Arduino timing
  with STM32 time sources or explicit `dt` injection.

### 4) Implement sensor adapter for estimator input
Create a small adapter (can live inside `estimator.c` or `control/estimator_io.c`):
- Read latest samples via `imu_*_try_get_latest()` and timestamp them.
- Handle missing samples:
  - Use last-known sample + `valid=false` flag.
  - Allow estimator to skip/weight invalid inputs.
- Use monotonic `HAL_GetTick()` timebase; compute `dt` in ms.
- Ensure IMU units match legacy expectations (scale if needed).

### 5) Port controller math and output shaping
In `MotionController.cpp`:
- Port logic as-is.
- Inputs: `StateEstimate`, teleop setpoints, `dt`.
- Outputs: per-wheel velocity/torque setpoints.
- Apply saturation and safety limits (from legacy config).

### 6) Implement motion mode state machine
In `motion_modes.c`:
- Default to DISARMED.
- Transition rules:
  - DISARMED -> CALIBRATION via RPC/command.
  - CALIBRATION -> DISARMED or BALANCING on user command.
  - BALANCING -> FAULT on estimator failure or motor fault.
  - FAULT -> DISARMED only via explicit reset command.
- Gate output:
  - DISARMED: zero torque.
  - CALIBRATION: allow explicit test outputs.
  - BALANCING: allow closed-loop outputs within limits.
  - FAULT: zero torque + log reason.

### 7) Control loop integration
In `app/app_main.c` via `motion_control.h`:
- Add `motion_control_init()` to initialize estimator/controller/mode.
- In the 1 kHz tick:
  - Build input from sensors in `motion_control.cpp`
  - `StateEstimator::update()`
  - `motion_mode_step()` (handles transitions / gating)
  - `MotionController::computeControl()` if mode allows
  - `motor_link_send()` to motors
- Record timing via `perf_stats` to confirm 1 kHz budget.

### 8) Telemetry hooks
Expose low-rate (e.g. 50-100 Hz) telemetry:
- estimator state (angles, rates, bias)
- controller outputs (setpoints, saturations)
- motion mode + fault codes
Use the existing mux/TELEM channel.

### 9) Dynamic tuning aids
- Add logging hooks for key signals (e.g. `IqL_ref`, `IqR_ref`, `thetaRef`, `vRef`).
- Provide runtime tuning via serial or RPC interface.
- Integrate with existing perf_stats for timing and overrun detection.
- Log Iq targets, controller outputs, and error signals for offline analysis.

## 12) Already in place (current algorithm snapshot)
This section summarizes what the **current MotionControl implementation** already does today (baseline), so we can clearly see what changes are required.

- **Porting approach**: Estimator + PID controller math is already ported from legacy C++ modules with minimal math changes; hardware I/O is adapted at the boundaries (sensor adapters + motor link).
- **Scheduling model**: A hardware timer drives a deterministic control tick (currently targeted at **1 kHz**), and the control tick is designed to be bounded and non-blocking.
- **Motion modes**: A mode state machine exists (or is planned) with at least DISARMED / CALIBRATION / BALANCING / FAULT, gating outputs to ensure safety.
- **Sensor adapter boundary**: IMU readings are acquired and converted into the estimator’s expected units at a single adapter boundary; missing samples are handled via valid flags / last-known sample strategy.
- **Motor link abstraction**: Motor commands are sent via a dedicated motor layer/protocol (UART) so that the main control loop never blocks on motor I/O.
- **Telemetry plumbing**: A low-rate telemetry path exists (or is planned) to publish estimator/controller outputs without stalling the control loop.

## 13) TODO to reach the desired final state (this document)
This section lists what must be amended so the implementation matches the **MainControl** target design (torque via estimated current, dual-IMU Pattern A, and PID-based balance + speed + steering).

### Control-rate alignment
- Decide and implement the final **outer-loop rate** (this doc assumes **400–500 Hz**) versus the current MotionControl target (**1 kHz**). Update the timer tick, perf budget checks, and all filters/LPFs accordingly.
- Ensure telemetry at **500 Hz** does not block control (use queue/ring-buffer + separate task).

### Motor command semantics (Iq targets)
- Update the STM32H723 → motor-driver protocol so the outer loop sends `{IqL_ref, IqR_ref}` (Amps) as the primary command.
- On each STM32F103 motor driver, ensure SimpleFOC is configured with:
  - `motor.phase_resistance` and `motor.KV_rating`
  - voltage torque mode + estimated current + Back‑EMF compensation
  - a clear clamp (`IqMax`) and consistent sign convention
- Add (optional but recommended) telemetry from motor drivers for: `estimated_Uq`, `bus_voltage`, `shaft_velocity`, and saturation flags so we can validate current→voltage conversion and limits.

### Dual-IMU integration (Pattern A)
- Implement the Pattern A health metrics and gating in code:
  - `gyro_diff = ||gyro1 - gyro2||`
  - `acc_angle_diff = angle(acc1, acc2)`
  - `accel_vib_index = variance(acc1 window)` (and optionally acc2)
- Implement robust **fault detection** and **fallback switching**:
  - detect IMU1 stalls (missing DRDY) and FIFO overflows
  - switch EKF input to IMU2 when IMU1 is unhealthy
  - revert to IMU1 only after a clean recovery window
- Implement accel gating and/or dynamic measurement noise `R` adjustment based on vibration index.

### Time alignment and dataflow
- Ensure both IMU samples are timestamped at DRDY edge and stored in a ring buffer with timestamps.
- Define a deterministic rule for which sample pair a given EKF/control tick consumes (e.g., latest IMU1 sample, plus nearest-in-time IMU2 sample for validation).

### Control law implementation details
- Implement the speed→tilt reference path (`vRef → thetaRef`) with anti-windup.
- Confirm derivative term uses `thetaDot` from gyro/EKF (not numerical diff).
- Implement steering split (`uTurn`) with optional yaw damping.

### Safety + modes
- Integrate the tilt kill-switch (`thetaKill`) into the motion mode state machine.
- Define behavior for: FALLEN recovery, IMU fault, motor link timeout, and saturation.

## 14) Outstanding questions / design decisions
These must be resolved (or explicitly frozen as defaults) before final tuning.

### Rates and filter constants
- Should the outer control loop run at **400–500 Hz** (as in this doc) or remain at **1 kHz** (as in MotionControl)? If 1 kHz, do we still keep EKF at a lower rate (e.g., 400 Hz) with predict-only steps in between?
- What are the final IMU ODRs (gyro and accel) and FIFO/DRDY strategy per IMU?

### Velocity and state estimation
- Is forward velocity `v` derived purely from wheel encoders, or do we also estimate wheel slip / integrate acceleration in the EKF?
- Do we fuse wheel-derived velocity into the EKF state (recommended if drift matters), or keep EKF as tilt/bias only and let the controller use wheel velocity separately?

### Dual-IMU thresholds and recovery
- What thresholds should we use for:
  - `GYRO_DISAGREE_THRESH`
  - `ACC_ANGLE_DISAGREE_THRESH`
  - vibration gating (`VIBRATION_HIGH_THRESH`)
- How long must IMU2 be healthy before switching, and how long must IMU1 be healthy before switching back?
- Should we gate accel updates based on **IMU1 vibration only**, or the max/combined vibration across both IMUs?

### Frame alignment
- Are IMU1 and IMU2 mounted with identical axes? If not, what is the fixed rotation `R_imu_to_body` for each IMU, and where is it applied (driver vs estimator input adapter)?

### Motor command units and saturation
- What is the authoritative sign convention for `Iq_ref` (positive = forward torque?) across H723 → protocol → F103 → SimpleFOC? Document and test with a direction test.
- What should `IqMax` be relative to motor heating and driver limits, given we do not have true current sensing?
- Do we need battery-voltage-aware clamps (reduce `IqMax` / detect brownout) to prevent undervoltage resets under aggressive correction?

### Steering model
- Do we steer using torque differential only, or also bias `vRef` per wheel (e.g., curvature control)?
- Is yaw damping derived from IMU gyro Z, wheel differential, or both?

End of dual‑IMU fusion integration spec.
