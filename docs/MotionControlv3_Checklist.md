# MotionControlv3 Implementation Checklist

This checklist maps MotionControlv3 sections to code and tracks implementation status.

Legend: ✅ implemented, 🟡 partial, ❌ missing.

## 1–2 Purpose & Design Goals
- ✅ Documented only (no code).

## 3 Control hierarchy (multi-rate)
- ✅ Hip loop rate gating: `app/control/hip_control.c` (HIP_CMD_PERIOD_MS)
- ✅ Coordinator/state machine (slow loop): `app/control/hip_behavior.c` + `motion_control.cpp`
- ✅ Wheel loop existing: `app/control/motion_control.cpp`, `MotionController.cpp`

## 4 Decoupling fast balance from posture
- ✅ Hip loop decoupled via rate gating; optional feedforward in `motion_control.cpp` (gain = 0 by default).

## 5 Hip control objectives
- ✅ Posture/height: `hip_control.c` (height ref + impedance)
- ✅ Ground-following stiffness modulation: `hip_behavior.c` (ground/flight/landing gains)
- ✅ Jump/stair climb state machine: `hip_behavior.c`

## 6 Control structure (block view)
- ✅ Wheel loop: existing control path
- ✅ Hip loop inputs/outputs implemented; constraints enforced (slew/limits/clamps)
- ✅ Coordinator/state machine: `hip_behavior.c`

## 7 Interactions and feedforward
- ✅ Optional wheel feedforward from hip height rate (configurable gain)

## 8 Logging and diagnostics
- ✅ Hip mode transitions logged via APP_LOG.
- ✅ Hip telemetry + fault flags surfaced in `robot_telem_v3_t` and `app_telem.c`.

## 9 Integration points
- ✅ Hip commands integrated in `motion_control.cpp` (tick/init)
- ✅ Behavior state machine integrated (slow coordinator in motion_control)
- ✅ Fault logic: hip faults stop commands + bus-off disables outputs

## 10 Hip actuator protocol
- ✅ CAN addressing/commands: `hip_control.c`

## 11 Hip kinematics solver
- ✅ Forward kinematics: `hip_kinematics.c`
- ✅ Inverse kinematics (LUT + interp): `hip_kinematics.c`
- ✅ Jacobian mapping: `hip_control.c`
- ✅ Geometry constants: `hip_kinematics.c`

## 12 Next steps (from spec)
- ✅ Feedforward coupling into wheel loop
- ✅ Logging fields and fault flags

## 13 Spec v1 (implementation-ready)
### 13.1 Control interfaces / ownership
- ✅ Coordinator state machine + interfaces

### 13.2 Data structures (HipState/Target/Command)
- ✅ Implemented in `hip_control.h`

### 13.3 Units/conversions
- ✅ Encoder rev/rad conversions: `hip_control.c`
- ✅ Sign convention + params: `hip_control.c`, `param_storage.c`

### 13.4 Limits and safety
- ✅ Theta min/max + height range clamped from LUT
- ✅ Velocity/torque clamps
- ✅ Height slew limit
- ✅ Bus-off handling sets fault and disables outputs (GPIO optional)
- ✅ Kinematics failure sets fault and holds height
- ✅ Stall detection fault

### 13.5 Limit switches
- ✅ Debounce + limit behavior: `hip_limits.c`, `hip_control.c`

### 13.6 Calibration/zeroing
- ✅ Zero offset param and usage: `param_storage.c`, `hip_control.c`
- ✅ Calibration RPC: `ROBOT_RPC_METHOD_HIP_CALIB_ZERO` in `app_rpc.c`

### 13.7 Hip loop control (position + impedance)
- ✅ Position + impedance in `hip_control.c`

### 13.8 Behavior state machine
- ✅ Implemented in `hip_behavior.c`

### 13.9 Telemetry/logging
- ✅ Hip telemetry fields in `robot_telem_v3_t` + `app_telem.c`
- ✅ Hip fault flags surfaced in telemetry

### 13.10 CAN configuration
- ✅ Implemented (IDs, commands, heartbeat, scheduling)

### 13.11 Test plan
- ✅ `test_hip_kinematics.cpp`
- ✅ `test_hip_limits.cpp`
- ✅ `test_hip_control.cpp`

### 13.12 Defaults table
- ✅ Implemented via `app_config.h` defaults (partial in code)
