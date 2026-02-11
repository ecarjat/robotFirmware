# ControlOutput Torque Units Spec

Goal: make `MotionController::ControlOutput` represent **torque (N·m)** instead of motor current (A), and perform torque→iq conversion only in the motor backend via `motor_link_torque_to_iq`.

## Scope
- Applies to wheel control outputs (`ControlOutput` and downstream usage).
- LQR + PID internal calculations can remain in their current domains, but **final outputs** must be **N·m**.
- Hip control paths are unaffected by this spec.

## Required behavior

### 1) ControlOutput units
- `MotionController::ControlOutput` fields (`iqLeft`, `iqRight`) are renamed or re‑documented as torque:
  - **Preferred**: rename to `torqueLeftNm` / `torqueRightNm`.
  - **Minimum**: keep names but update comments and all call sites to treat as N·m.
- Any diagnostics or logs using these fields must document N·m units.

### 2) MotionController output path
- `computeControl()` returns **torque in N·m**.
- The mixing of `uSum` and `uTurn` produces torque outputs, not iq.
- If PID operates in iq internally, convert to torque before output:
  ```
  uSumPidNm = uSumPidIq * motorKt
  uSumLqrNm = computeLqrUSumNm(...)
  uSumNm = (1-alpha)*uSumPidNm + alpha*uSumLqrNm
  uTurnNm = uTurnIq * motorKt
  out.torqueLeftNm  = uSumNm - uTurnNm
  out.torqueRightNm = uSumNm + uTurnNm
  ```
- Saturation limits must be expressed in **N·m** at the output stage.
  - `lqr.u_limit` and `lqr.du_limit` are defined in **N·m** / **N·m/s**.

### 3) Motor backend conversion
- Motor command dispatch uses **torque** API:
  - `motor_link_set_wheel_torques(left_Nm, right_Nm, max_Nm)`
- Backends convert torque to iq via `motor_link_torque_to_iq`.
- No torque→iq conversion anywhere else in control code.

### 4) Diagnostics
- `InnerCtrlDiag` fields must clearly label units:
  - `u_sum_cmd` **N·m**
  - `u_sum_pid` **N·m** (after conversion if needed)
  - `u_sum_lqr` **N·m** (already torque-domain)
  - `u_diff_cmd` **N·m**
- Log/telemetry fields for wheel outputs must be updated to N·m.

### 5) Config impact
- `motorKt` remains the only conversion factor.
- `lqr.K` remains torque-domain (already true).
- PID gains remain unchanged; conversion happens at output boundary.

## Implementation steps (high-level)
1) Rename or re‑document `ControlOutput` to N·m.
2) Update `MotionController::computeControl()` to return N·m outputs.
3) Update callers to use `motor_link_set_wheel_torques` (not `set_wheel_Iq`).
4) Update diagnostics/telemetry field units.
5) Tests:
   - Verify torque outputs are correct for known K and motorKt.
   - Verify backend conversion uses `motor_link_torque_to_iq`.

## Spec gap report
None (this document is complete for implementation planning).
