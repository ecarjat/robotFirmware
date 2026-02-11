# MotionControl v3 (Wheels + Hip Actuation)

Date: 2026-01-28

## 1. Purpose
This document describes the updated control structure for a robot with:
- Two wheel motors for balance and locomotion.
- Two hip motors that control posture/leg-length/ground-following and provide jumping capability (e.g., stair climbing).

The hip mechanism is a three-bar linkage that adjusts the overall height of the top structure (where the IMUs are mounted). The linkage preserves the center of gravity alignment as height changes.

## 2. Design goals
- Preserve the stability of the existing wheel-based balance loop.
- Add hip control without creating fast, destabilizing coupling into the balance loop.
- Support terrain adaptation (ground-following) and dynamic behaviors (jump/step-up).
- Keep the architecture modular and diagnosable (clear logging and fault modes).

## 3. Control hierarchy (multi-rate)

### Level 0: Wheel stabilization (fast)
- Rate: 500-1000 Hz (existing loop).
- Inputs: IMU pitch/roll, wheel speed, body rate.
- Outputs: wheel torque (Iq).
- Goal: maintain balance and commanded velocity.

### Level 1: Hip posture/height (medium)
- Rate: 100-200 Hz.
- Inputs: hip angle/length, IMU pitch, estimated height, ground contact cues (if available).
- Outputs: hip torque or position setpoints.
- Goal: maintain body height, compensate terrain, provide posture bias.

### Level 2: Behavior state machine (slow)
- Rate: 10-50 Hz.
- Modes: normal, crouch, jump, landing, stair-climb assist.
- Outputs: bias targets and constraints for Level 1 and Level 0.

## 4. Key concept: decouple fast balance from slower posture
The wheel loop remains the primary stabilizer. Hip control is treated as a slower posture/height actuator that:
- Adjusts the IMU height (and thus the body pivot) without rapidly exciting pitch.
- Applies smooth bias terms (not rapid oscillations).
- Uses damping for landing and uneven terrain.

If hip motion changes the body pitch, a feedforward compensation term should be injected into the wheel loop to avoid transient destabilization.

## 5. Hip control objectives

### 5.1 Posture and height control
- Command a target body height (h_ref).
- Track with a smooth position/impedance controller on the hip actuators.
- Maintain CoG alignment through the three-bar geometry (mechanical alignment helps, controller enforces smoothness).

### 5.2 Ground following
- Use impedance control (stiffness + damping) to absorb terrain variation.
- Optionally modulate stiffness based on speed or terrain roughness.
- Avoid direct feedback into the wheel loop at high rate.

### 5.3 Jump and stair climb
- Use a state machine with phases:
  1) Prep/Crouch (lower height, preload)
  2) Impulse (rapid hip extension)
  3) Flight/Unload (reduce wheel torque, limit wheel control authority)
  4) Landing (high damping, conservative wheel torque)
- The wheel loop remains enabled but is constrained in impulse/flight phases.

## 6. Control structure (block view)

```text
            +--------------------+
            |  Motion Modes /    |
            |  Hip Behavior SM   |
            +---------+----------+
                      |
                      v
            +--------------------+
IMU + Hip   |  Hip Control Loop  |  Hip encoder + limits
state ----> |  (height + impedance) -----> Hip CAN commands
            +---------+----------+
                      |
                      v
            +--------------------+
            |  Wheel Controller  |  Wheel IMU + speed
            |  (balance + FF)    |-----> Wheel CAN commands
            +--------------------+
```

### Wheel loop (existing)
- Inputs: IMU pitch, pitch rate, wheel speed
- Output: wheel torque
- Constraints: torque saturation, safety limits

### Hip loop (new)
- Inputs: hip angle/length (sensors), IMU pitch bias, optional contact estimate
- Output: hip torque or position target
- Constraints: max hip torque, max speed, soft limits

### Coordinator / State Machine
- Provides height targets, stiffness/damping schedules, and wheel torque limits.
- Ensures smooth transitions between modes.

## 7. Interactions and feedforward
- Hip motion changes body inertia and pitch dynamics.
- Use feedforward to wheel loop:
  - Predicted pitch offset from hip height change.
  - Optional compensation torque during rapid hip movements.
- Use wheel loop saturation flags to prevent hip-induced instability.

## 8. Logging and diagnostics
Log at minimum:
- Wheel torque command, wheel velocity, balance state.
- Hip torque/position command, hip measured position/velocity.
- Body height estimate, IMU pitch.
- State machine mode and phase transitions.
- Jump phase progress (0-100%).

Fault conditions:
- Hip motor communication loss.
- Hip actuator saturation or stalled motion.
- IMU out of range during jump/landing.

## 9. Integration points
- Add hip motor commands alongside existing wheel commands.
- Insert hip loop in the control scheduler at medium rate.
- Add behavior state machine in the high-level control layer (motion modes).
- Ensure fault logic disables hip actuation independently if needed.

## 10. Hip actuator hardware and protocol (GIM8108-8 / GDS68 / 2ES68)
The hip actuator is a Steadywin GIM8108-8 with a GDS68 driver and an output encoder (2ES68 secondary encoder). The driver is controlled via CAN using the CAN Simple protocol (per `docs/SteadyWin GIM6010-8 Motor Manual_rev2.2.pdf`).

### 10.1 CAN Simple addressing
- Standard CAN frame, 11-bit ID, 8-byte data, little-endian payload.
- CAN ID format: **CAN_ID = (node_id << 5) | cmd_id**.
- Choose unique node_id per hip (e.g., `hip_left = 0x03`, `hip_right = 0x04`).

### 10.2 Required control mode
- Hip actuators run in **position control**.
- Use `Set_Controller_Mode` (CMD ID `0x00B`) with:
  - `Control_Mode = 3` (position control)
  - `Input_Mode = 3` (position filter) for smooth tracking
- Enter closed-loop using `Set_Axis_State` (CMD ID `0x007`) with `Axis_Requested_State = 8`.

### 10.3 Position command
- Use `Set_Input_Pos` (CMD ID `0x00C`):
  - `Input_Pos` float32 **rev** (output shaft)
  - `Vel_FF` int16 **0.001 rev/s** (optional)
  - `Torque_FF` int16 **0.001 Nm** (optional)
- Command rate: 100-200 Hz (hip loop rate).

### 10.4 Feedback and telemetry
- `Get_Encoder_Estimates` (CMD ID `0x009`):
  - `Pos_Estimate` float32 rev
  - `Vel_Estimate` float32 rev/s
- `Get_Torques` (CMD ID `0x01C`):
  - `Torque_Setpoint` float32 Nm
  - `Torque` float32 Nm
- Use `Heartbeat` (CMD ID `0x001`) for health and axis state.

### 10.5 Zero/reference handling
- Positions are based on the absolute encoder zero; define a **user zero** during commissioning.
- If needed, use limit switches or manual offset configuration to align encoder zero to mechanical neutral.

## 11. Incorporating hip kinematics solver
You already have a linkage solver and the positions of the linkage points. Integrate it as follows:

### 11.1 Forward kinematics (measurement)
- Input: actuator output angle from encoder (rev -> rad).
- Output: body height, hip joint angle, and other linkage points as needed.
- Use this for state estimation and to derive height/leg-length feedback.

### 11.2 Inverse kinematics (command)
- Input: desired body height / posture target.
- Output: actuator angle (rev) for `Set_Input_Pos`.
- Apply smooth rate limiting and position filtering in the hip loop.

### 11.3 Jacobian mapping (force/torque)
- Compute the Jacobian from actuator angle to vertical force at the body.
- Use it to map desired vertical force (from impedance control) into `Torque_FF` (Nm).
- This allows ground-following and landing damping without destabilizing the wheel loop.

### 11.4 Solver geometry (implementation constants)
The current solver implementation lives in `app/control/hip_kinematics.c` and uses
the following geometry (meters) derived from the selected linkage dimensions:
- Upper leg H-K: `0.39609`
- Lower leg K-W: `0.37996`
- Link Bc-C: `0.41762`
- Inner offset K-C: `0.07506`
- Pin joint (Bc) position: `X = 0.09893`, `Y = -0.11576`

Coordinate system:
- Solver coordinates use +Y down.
- The pin joint Y is negative because the solver's reference is above the pin.

Angle domain:
- Theta (actuator angle) nominal range: `23.95 deg` to `61.04 deg`.
- Implementation expects radians (convert from encoder rev to radians).

Branch selection:
- Circle intersection yields two candidate C points.
- The current implementation selects the W point (wheel) with the larger Y
  (greater downward height) to stay on the physically valid branch.

## 12. Next steps
- Define hip kinematics (three-bar linkage) mapping between motor angles and body height.
- Identify hip sensor inputs (angle/length/torque) and ranges.
- Select hip actuator control mode (torque vs position).
- Define the feedforward coupling terms into the wheel loop.
- Add new logging fields and fault flags.

## 13. Spec v1 (implementation-ready)
This section enumerates concrete interfaces, limits, and behaviors required to
implement hip control in firmware.

### 13.1 Control interfaces and ownership
Scheduler and ownership:
- Wheel loop: existing fast loop (500-1000 Hz), remains the primary balance controller.
- Hip loop: new medium-rate loop (100-200 Hz) running in the app control scheduler.
- Coordinator/state machine: slow loop (10-50 Hz) providing targets and constraints.

Function call order per cycle:
1) Coordinator update (slow): updates `HipTarget`, `WheelConstraints`.
2) Hip loop update (medium): computes `hip_pos_cmd_rev`, `hip_vel_ff`, `hip_torque_ff`.
3) Wheel loop update (fast): uses IMU + wheel state and optional hip feedforward.

### 13.2 Data structures
Add or extend a shared control struct:
- `HipState`:
  - `theta_rad`, `theta_dot_rad_s`
  - `height_m`, `height_dot_m_s`
  - `torque_nm` (from telemetry)
  - `valid` (bool)
- `HipTarget`:
  - `height_ref_m`
  - `height_rate_ref_m_s`
  - `stiffness_n_m` (for impedance)
  - `damping_n_s_m`
  - `mode` (enum: `HIP_MODE_HOLD`, `HIP_MODE_GROUND_FOLLOW`, `HIP_MODE_JUMP`)
- `HipCommand`:
  - `pos_cmd_rev`
  - `vel_ff_rev_s`
  - `torque_ff_nm`

Add to telemetry (robot protocol v3):
- `hip_phase_progress_pct` (0-100) indicating jump phase completion.

### 13.3 Units and conversions
- Encoder input:
  - Use output shaft encoder (2ES68) in revolutions.
  - Convert to radians: `theta_rad = rev * 2*pi`.
- CAN command:
  - `Set_Input_Pos` expects `Input_Pos` in revolutions.
- Kinematics:
  - `hip_kinematics_height_from_theta(theta_rad, &height_m)`
  - Inverse uses LUT + interpolation: `hip_kinematics_theta_from_height(height_m, &theta_rad)`
  - Height output uses solver coordinates (+Y down); treat as positive height.
- **Actuator sign convention**: positive command increases height. Add a runtime
  parameter `hip_dir_sign` (`+1` or `-1`) so installs with inverted wiring/assembly
  can be corrected without code changes. Apply as:
  - `pos_cmd_rev = hip_dir_sign * pos_cmd_rev`
  - `vel_ff_rev_s = hip_dir_sign * vel_ff_rev_s`

### 13.4 Limits and safety
Initial limits (conservative defaults):
- `theta_min = 23.95 deg`, `theta_max = 61.04 deg`
- `height_min_m` and `height_max_m` derived from kinematics at limits
- `hip_vel_max_rev_s = 0.5` (configurable)
- `hip_torque_max_nm = 20` (configurable)
- Rate limit: `d(height_ref)/dt <= 0.2 m/s`
 - Max travel is an encoder angle range: `theta_range = theta_max - theta_min`
   (from `HIP_THETA_MIN_DEG` and `HIP_THETA_MAX_DEG`).

Safety behavior:
- Hip CAN comm loss or timeout: disable hip commands and freeze height at current estimate.
- Bus-off (FDCAN): disable hip motor enable GPIO and assert fault flag.
- If kinematics solver fails: do not update height; enter safe hold mode.

### 13.5 Limit switches (hip travel bounds)
Hardware: 4 limit switches, NC to GND with pull-ups. Triggered state = logic low.

Pins (CubeMX generated names in `Core/Inc/main.h`):
- LeftHipUpperLimit: `GPIOA` / `GPIO_PIN_8`
- LeftHipLowerLimit: `GPIOA` / `GPIO_PIN_10`
- RightHipUpperLimit: `GPIOD` / `GPIO_PIN_3`
- RightHipLowerLimit: `GPIOD` / `GPIO_PIN_5`

Behavior:
- Upper switch active: block further extension (increase in height).
- Lower switch active: block further retraction (decrease in height).
- If a limit is active at boot, only allow slow motion **away** from that limit until it clears.
- If the opposite limit does not trigger, stop after the maximum allowed motion range (do not continue driving).
- Switches are **polled** in the hip loop (no interrupts). Use software debounce:
  - Require N consecutive samples (e.g., 3–5) before changing the latched state.
  - Apply the same rule for clearing to avoid chatter.

### 13.6 Calibration and zeroing
- Store `hip_zero_offset_rev` in nonvolatile config.
- On boot:
  - Read zero offset.
  - Apply `theta_rad = (rev - hip_zero_offset_rev) * 2*pi`.
- Mechanical neutral is a calibrated reference, not assumed.
- Calibration procedure:
  - Place the robot on a flat surface.
  - Set hip motors to **0 torque** (free‑hold).
  - Read hip encoder angle and store it as the mechanical neutral offset.

### 13.7 Hip loop control (position + impedance)
Mode: position control with impedance overlay.
- Position target:
  - `height_error = height_ref_m - height_m`
  - `height_rate_error = height_rate_ref_m_s - height_dot_m_s`
  - `delta_height = Kp_h * height_error + Kd_h * height_rate_error`
  - Convert `height_ref_m` to `pos_cmd_rev` via inverse kinematics.
- Torque feedforward:
  - `torque_ff_nm = stiffness_n_m * height_error + damping_n_s_m * height_rate_error`
  - Map through Jacobian `tau = J^T * Fz`.

Initial gains (placeholder):
- `Kp_h = 50`, `Kd_h = 5`
- `stiffness_n_m = 800`, `damping_n_s_m = 80`

### 13.8 Behavior state machine (jump/landing)
Define states and transitions:
- `NORMAL`: tracks `height_ref_m` with low stiffness.
- `CROUCH`: ramp down height at `0.15 m/s`, increase stiffness.
- `IMPULSE`: rapid extension, enable torque feedforward.
- `FLIGHT`: reduce wheel torque, hold hip position.
- `LANDING`: high damping, clamp hip velocity, then return to `NORMAL`.

Transitions:
- `NORMAL -> CROUCH` on command.
- `CROUCH -> IMPULSE` after `t_crouch >= 150 ms`.
- `IMPULSE -> FLIGHT` when height_dot > threshold or time exceeded.
- `FLIGHT -> LANDING` on IMU vertical accel or contact detection.
- `LANDING -> NORMAL` after settling (e.g., 200 ms).

### 13.9 Tuning guide (impedance + jump)
Start with conservative stiffness/damping and increase until the body tracks
height without oscillation:
- Increase `stiffness_n_m` until tracking error is acceptable.
- Increase `damping_n_s_m` to suppress oscillation on landing.
- If jump impulse feels sluggish, increase `HIP_IMPULSE_RATE_MPS` gradually.
- If landing feels harsh, increase `HIP_LANDING_DAMPING_N_S_M` before raising
  `HIP_LANDING_STIFFNESS_N_M`.
- Keep wheel loop limits conservative during impulse/flight and relax only
  after reliable landing detection.

### 13.10 Telemetry and logging
Telemetry fields required:
- `hip_pos_rev`, `hip_vel_rev_s`, `hip_torque_nm`
- `height_m`, `height_dot_m_s`
- `hip_mode`, `hip_fault_flags`
- `can_rx_age_ms` per hip
- `hip_fault_flags` includes per-motor stall flags.

Logging rate:
- Hip loop: 100-200 Hz
- State changes: log event stamps

### 13.11 CAN configuration
- Node IDs:
  - `hip_left = 0x03`, `hip_right = 0x04`
- CAN Simple ID:
  - `CAN_ID = (node_id << 5) | cmd_id`
- Required commands:
  - `Set_Controller_Mode (0x00B)` with position control
  - `Set_Axis_State (0x007)` to closed-loop
  - `Set_Input_Pos (0x00C)`
  - Telemetry: `Get_Encoder_Estimates (0x009)`, `Get_Torques (0x01C)`, `Heartbeat (0x001)`

Timeouts:
- If `Heartbeat` not received for 200 ms, flag CAN fault and disable hip.

CAN scheduling (shared with wheels):
- Priority: wheel control IDs highest, hip position next, telemetry lowest.
- Hip position request rate: **100 Hz** (per hip).
- Hip telemetry (encoder + torque): **50 Hz** (per hip, stagger if needed).
- Heartbeat poll: **5–10 Hz**.
- Request timeout: **50 ms** for position command ack (if required by driver).
- Telemetry timeout: **200 ms** (aligns with heartbeat fault).

Startup sequence (defaults):
1) Boot: initialize FDCAN, filters, and enable notifications.
2) Read `hip_zero_offset_rev` and `hip_dir_sign` from parameters.
3) Read limit switches (debounced). If any is active, enter **limit‑recovery** mode.
4) Send `Set_Controller_Mode` (position + input filter) to both hips.
5) Send `Set_Axis_State` to closed‑loop.
6) Start hip command loop at **100 Hz** with:
   - If in limit‑recovery, command slow motion away from the active limit.
   - Else hold current height using `Set_Input_Pos`.
7) Start telemetry polling at **50 Hz**; start heartbeat at **5–10 Hz**.

### 13.12 Test plan (minimum)
Unit tests:
- `hip_kinematics_height_from_theta` at limits and midpoints.
- Inverse kinematics (when implemented) round-trip consistency.

Integration tests:
- CAN command encoding/decoding.
- Simulator loop with synthetic theta to height trace.

### 13.12 Defaults table
| Item | Default | Notes |
| --- | --- | --- |
| Hip command rate | 100 Hz | `Set_Input_Pos` per hip |
| Hip telemetry rate | 50 Hz | Encoder + torque per hip |
| Heartbeat rate | 5 Hz | CAN Simple heartbeat |
| Heartbeat timeout | 200 ms | Fault if exceeded |
| Position ack timeout | 50 ms | If driver requires ack |
| Limit switch debounce | 3 samples | At 100–200 Hz (15–50 ms) |
| Max height slew | 0.2 m/s | `d(height_ref)/dt` |
| Hip vel max | 0.5 rev/s | Configurable |
| Hip torque max | 20 Nm | Configurable |
| Theta range | 23.95–61.04 deg | `HIP_THETA_MIN/MAX_DEG` |
| Hip dir sign | +1 | Invert if assembly reversed |
