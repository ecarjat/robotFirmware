# Hip Terrain Adaptation Specification

## 1 Aim

Enable the robot to maintain body stability (level IMU) when traversing uneven terrain by independently adjusting each hip leg height based on ground contact feedback and body orientation.

**Current limitation:** Both legs track a single shared height reference (`s_height_ref_m`). When one wheel encounters a bump or depression, the body tilts because both legs maintain the same commanded height.

**Goal:** Each leg should independently adjust its height to keep the body level, using IMU roll feedback and per-leg force/contact sensing.

---

## 2 Requirements

### 2.1 Functional Requirements

| ID | Requirement | Priority |
|----|-------------|----------|
| F1 | Each leg shall have an independent height target | Must |
| F2 | Body roll angle (EKF-filtered) shall be used in closed-loop control for differential leg height adjustments | Must |
| F3 | Ground contact shall be detected per-leg using torque feedback | Must |
| F4 | System shall blend between position control and force/compliance control based on contact state | Should |
| F5 | Terrain adaptation shall be disabled during jump and stairs sequences | Must |
| F6 | Maximum differential height shall be limited to prevent mechanical damage | Must |
| F7 | Terrain adaptation parameters shall be configurable | Should |
| F8 | Wheel torque shall be reduced on retracting leg side to prevent yaw disturbance | Must |
| F9 | Robot shall support stair climb mode with sequential leg lifting | Must |
| F10 | Robot shall support stair descend mode with controlled leg extension | Must |
| F11 | Step edge shall be detected using force discontinuity | Must |
| F12 | Stairs mode shall abort safely if ground not detected within timeout | Must |
| F13 | Simultaneous step edge detection shall be supported for two-wheeled descent | Must |
| F14 | Robot shall perform coordinated symmetric descent when both edges detected simultaneously | Must |

### 2.2 Performance Requirements

| ID | Requirement | Target |
|----|-------------|--------|
| P1 | Body roll deviation on 5cm step | < 5 deg |
| P2 | Settling time after step encounter | < 300 ms |
| P3 | Control loop latency | < 20 ms |
| P4 | No oscillation or hunting on flat ground | Stable |
| P5 | Single stair step climb time | < 2 sec |
| P6 | Single stair step descend time | < 2 sec |
| P7 | Body roll during stair climb | < 10 deg |
| P8 | Step edge detection latency | < 50 ms |

### 2.3 Safety Requirements

| ID | Requirement |
|----|-------------|
| S1 | Differential height shall be limited to mechanical range |
| S2 | Loss of ground contact on one leg shall trigger safe response |
| S3 | IMU failure shall revert to symmetric height control |
| S4 | Stairs descend shall abort if ground not detected within timeout |
| S5 | Stairs mode shall abort on excessive roll during transfer phase |
| S6 | Stairs mode shall abort on motor stall fault |
| S7 | Simultaneous descent shall abort if leg heights diverge |
| S8 | Simultaneous descent shall abort on roll exceeding threshold (no anchor leg) |

---

## 3 Design Overview

### 3.1 Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Roll source | **EKF-filtered** | Smoother signal, reduced noise; latency acceptable at 20 Hz coordinator rate |
| Control type | **Closed-loop** | Feedback on roll angle + roll rate for stability and disturbance rejection |
| Wheel coordination | **Yes** | Reduce wheel torque on retracting leg side to prevent yaw disturbance |

### 3.2 Control Architecture

```
                    ┌─────────────────────────────────────────────────────────────┐
                    │                  Terrain Coordinator (20 Hz)                 │
                    │                                                              │
  EKF roll ────────►│  ┌────────────────────┐    ┌──────────────────────────┐     │
  EKF roll_dot ────►│  │  Roll Closed-Loop  │───►│ Per-Leg Height Targets   │     │
                    │  │  Controller (PD)   │    │  left_h_ref, right_h_ref │     │
  Nominal height ──►│  └────────────────────┘    └──────────────────────────┘     │
                    │            ▲                           │                     │
                    │            │                           ▼                     │
  Left torque ─────►│  ┌────────────────────┐    ┌──────────────────────────┐     │
  Right torque ────►│  │  Contact Detector  │───►│ Impedance Gain Scheduler │     │
                    │  └────────────────────┘    └──────────────────────────┘     │
                    │                                        │                     │
                    │            ┌───────────────────────────┤                     │
                    │            ▼                           ▼                     │
                    │  ┌────────────────────┐    ┌──────────────────────────┐     │
                    │  │ Wheel Torque Scale │    │  Hip Target Output       │     │
                    │  │  (yaw prevention)  │    │                          │     │
                    │  └────────────────────┘    └──────────────────────────┘     │
                    │            │                           │                     │
                    └────────────┼───────────────────────────┼─────────────────────┘
                                 │                           │
                                 ▼                           ▼
                    ┌────────────────────────┐  ┌──────────────────────────────────┐
                    │  Wheel Control         │  │  Hip Control (per-leg, 100 Hz)   │
                    │  (motion_control.cpp)  │  │                                  │
                    │                        │  │  Left:  height_ref ──► pos_cmd   │
                    │  iq_left  *= scale_L   │  │         stiffness ──► torque_ff  │
                    │  iq_right *= scale_R   │  │  Right: height_ref ──► pos_cmd   │
                    │                        │  │         stiffness ──► torque_ff  │
                    └────────────────────────┘  └──────────────────────────────────┘
```

### 3.3 Key Components

1. **Roll Closed-Loop Controller** - PD controller on EKF roll angle and rate to compute differential leg heights
2. **Contact Detector** - Determines ground contact state per-leg from torque feedback
3. **Impedance Gain Scheduler** - Adjusts stiffness/damping based on contact state
4. **Wheel Torque Scaler** - Reduces wheel torque on retracting leg side to prevent yaw
5. **Per-Leg Height Controller** - Existing `hip_control.c` extended for independent targets
6. **Step Edge Detector** - Detects stair edges via force rate discontinuity
7. **Stairs State Machine** - Manages climb/descend sequences with phase-specific control

---

## 4 Detailed Design

### 4.1 Data Structures

#### New: Per-leg target in `hip_control.h`
```c
typedef struct {
    float height_ref_m;         /* Base height reference (shared) */
    float left_height_adj_m;    /* Left leg adjustment from roll */
    float right_height_adj_m;   /* Right leg adjustment from roll */
    float height_rate_ref_m_s;
    float left_stiffness_n_m;   /* Per-leg stiffness */
    float right_stiffness_n_m;
    float left_damping_n_s_m;   /* Per-leg damping */
    float right_damping_n_s_m;
    hip_mode_t mode;
    bool enabled;
    bool terrain_adapt_enabled; /* New: enable/disable terrain adaptation */
} hip_target_t;
```

#### New: Contact state per-leg
```c
typedef enum {
    HIP_CONTACT_UNKNOWN = 0,
    HIP_CONTACT_GROUND = 1,     /* Leg in firm ground contact */
    HIP_CONTACT_LIGHT = 2,      /* Light contact (transitioning) */
    HIP_CONTACT_AIR = 3         /* No ground contact */
} hip_contact_state_t;

typedef struct {
    hip_contact_state_t left;
    hip_contact_state_t right;
    float left_force_n;         /* Estimated vertical force */
    float right_force_n;
} hip_contact_t;
```

### 4.2 Roll Closed-Loop Controller

**Geometry:**
```
         wheel_base_m
    ◄──────────────────►

    ●──────────────────●  ← Body (tilted by roll_rad)
    │                  │
    │ h_left           │ h_right
    │                  │
    ○                  ○  ← Wheels on uneven ground
    ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
         Ground (bump on right)
```

**Closed-loop PD controller on EKF roll:**

The controller uses EKF-filtered roll angle and roll rate for smooth, stable terrain adaptation.

```c
/* Get EKF state */
float roll_rad = ekf_get_roll();           /* From state estimator */
float roll_dot_rad_s = ekf_get_roll_rate(); /* From state estimator */

/* PD controller: drive roll error to zero */
float roll_error = 0.0f - roll_rad;         /* Target: level body (roll = 0) */
float roll_rate_error = 0.0f - roll_dot_rad_s;

/* Compute differential height command */
float half_base = wheel_base_m * 0.5f;
float diff_height_cmd = TERRAIN_ROLL_KP * roll_error +
                        TERRAIN_ROLL_KD * roll_rate_error;

/* Convert to per-leg adjustments */
float left_adj_m  = -half_base * diff_height_cmd;
float right_adj_m = +half_base * diff_height_cmd;

/* Slew rate limit for smooth transitions */
static float left_adj_prev = 0.0f;
static float right_adj_prev = 0.0f;
float max_step = TERRAIN_ADAPT_RATE_MPS * dt_s;

left_adj_m  = slew_limit(left_adj_m, left_adj_prev, max_step);
right_adj_m = slew_limit(right_adj_m, right_adj_prev, max_step);
left_adj_prev = left_adj_m;
right_adj_prev = right_adj_m;

/* Clamp to mechanical limits */
left_adj_m  = clampf(left_adj_m, -MAX_DIFF_HEIGHT_M, MAX_DIFF_HEIGHT_M);
right_adj_m = clampf(right_adj_m, -MAX_DIFF_HEIGHT_M, MAX_DIFF_HEIGHT_M);
```

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `TERRAIN_ROLL_KP` | 2.0 | Proportional gain on roll error |
| `TERRAIN_ROLL_KD` | 0.1 | Derivative gain on roll rate |
| `MAX_DIFF_HEIGHT_M` | 0.05 | Maximum differential adjustment per leg |
| `TERRAIN_ADAPT_RATE_MPS` | 0.1 | Slew rate limit for height adjustments |

**Tuning notes:**
- `KP` controls how aggressively the system corrects roll errors
- `KD` provides damping to prevent overshoot and oscillation
- Start with low gains and increase until response is adequate without oscillation

### 4.3 Ground Contact Detection

**Method:** Use torque feedback to estimate ground reaction force.

```c
/* Estimate vertical force from motor torque via Jacobian */
float jacobian = hip_compute_jacobian(hip->theta_rad);
float force_n = (fabsf(jacobian) > 1e-6f) ?
                (hip->torque_nm / jacobian) : 0.0f;

/* Classify contact state */
hip_contact_state_t contact;
if (force_n > CONTACT_FORCE_GROUND_N) {
    contact = HIP_CONTACT_GROUND;
} else if (force_n > CONTACT_FORCE_LIGHT_N) {
    contact = HIP_CONTACT_LIGHT;
} else {
    contact = HIP_CONTACT_AIR;
}
```

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `CONTACT_FORCE_GROUND_N` | 50.0 | Force threshold for firm ground contact |
| `CONTACT_FORCE_LIGHT_N` | 10.0 | Force threshold for light contact |
| `CONTACT_DEBOUNCE_MS` | 20 | Debounce time for contact transitions |

### 4.4 Impedance Gain Scheduling

Adjust stiffness and damping based on contact state:

| Contact State | Stiffness | Damping | Rationale |
|---------------|-----------|---------|-----------|
| GROUND | High (800 N/m) | High (80 Ns/m) | Firm stance, resist perturbations |
| LIGHT | Medium (400 N/m) | Medium (40 Ns/m) | Compliant during transition |
| AIR | Low (200 N/m) | Low (20 Ns/m) | Soft for landing preparation |

### 4.5 Wheel Torque Coordination

**Problem:** When one leg retracts (e.g., right leg going over a bump), the wheel on that side has less ground contact force. If full torque is applied, it can cause:
- Wheel slip on the retracting side
- Unwanted yaw moment (robot turns)
- Reduced balance stability

**Solution:** Scale wheel torque proportionally to leg height adjustment.

```c
/* Compute wheel torque scale factors based on leg height adjustments */
float left_height_adj = target.left_height_adj_m;
float right_height_adj = target.right_height_adj_m;

/* Leg retracting (negative adj) = reduce torque on that wheel */
/* Leg extending (positive adj) = maintain full torque */
float scale_left = 1.0f;
float scale_right = 1.0f;

if (left_height_adj < 0.0f) {
    /* Left leg retracting - scale down left wheel */
    scale_left = 1.0f + (left_height_adj / MAX_DIFF_HEIGHT_M) * WHEEL_TORQUE_REDUCTION;
    scale_left = clampf(scale_left, WHEEL_TORQUE_MIN_SCALE, 1.0f);
}
if (right_height_adj < 0.0f) {
    /* Right leg retracting - scale down right wheel */
    scale_right = 1.0f + (right_height_adj / MAX_DIFF_HEIGHT_M) * WHEEL_TORQUE_REDUCTION;
    scale_right = clampf(scale_right, WHEEL_TORQUE_MIN_SCALE, 1.0f);
}

/* Apply to wheel commands in motion_control.cpp */
cmd.iq.iqLeft  *= scale_left;
cmd.iq.iqRight *= scale_right;
```

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `WHEEL_TORQUE_REDUCTION` | 0.5 | Max reduction factor (0.5 = reduce by 50% at max retraction) |
| `WHEEL_TORQUE_MIN_SCALE` | 0.3 | Minimum wheel torque scale (never go below 30%) |

**Example:** Right leg retracting 3cm (adj = -0.03m), MAX_DIFF = 0.05m:
- `scale_right = 1.0 + (-0.03 / 0.05) * 0.5 = 1.0 - 0.3 = 0.7`
- Right wheel torque reduced to 70%

### 4.6 Integration with Hip Behavior

**During jump sequences:** Terrain adaptation shall be disabled.

```c
void hip_behavior_tick(...) {
    ...
    /* Disable terrain adaptation during non-normal modes */
    target_out->terrain_adapt_enabled =
        (s_behavior.mode == HIP_BEHAVIOR_NORMAL) &&
        (motion_mode == MOTION_MODE_BALANCING);
    ...
}
```

### 4.6 Failure Modes

| Condition | Response |
|-----------|----------|
| EKF roll invalid | Revert to symmetric height (left_adj = right_adj = 0), wheel scales = 1.0 |
| One leg loses contact | Hold last known height, reduce stiffness, reduce wheel torque on that side |
| Both legs lose contact | Prepare for landing (reduce stiffness), maintain wheel torque for balance |
| Torque feedback timeout | Revert to position-only control, wheel scales = 1.0 |
| Hip height adjustment saturated | Wheel torque scaled to minimum, log warning |

---

## 5 Implementation Plan

### Phase 1: Per-Leg Height Targets
- [ ] Extend `hip_target_t` with per-leg fields
- [ ] Modify `hip_send_commands()` to use per-leg targets
- [ ] Update `hip_control_set_target()` API
- [ ] Add unit tests for per-leg control

### Phase 2: Roll Closed-Loop Controller
- [ ] Add EKF roll/roll_rate input to terrain coordinator
- [ ] Implement PD controller for roll-to-height mapping
- [ ] Add slew rate limiting for smooth transitions
- [ ] Add configurable parameters to `app_config.h`
- [ ] Add unit tests for closed-loop roll controller

### Phase 3: Contact Detection
- [ ] Implement force estimation from torque
- [ ] Add contact state classifier with debounce
- [ ] Surface contact state in telemetry
- [ ] Add unit tests for contact detection

### Phase 4: Impedance Scheduling
- [ ] Implement gain scheduler based on contact state
- [ ] Integrate with per-leg control
- [ ] Add unit tests for gain scheduling

### Phase 5: Wheel Torque Coordination
- [ ] Compute wheel torque scale factors from leg height adjustments
- [ ] Integrate scaling into `motion_control.cpp` wheel command path
- [ ] Add configurable reduction and minimum scale parameters
- [ ] Add unit tests for wheel coordination

### Phase 6: Step Edge Detection
- [ ] Implement force rate detector
- [ ] Add edge type classification (step-up/step-down)
- [ ] Add debouncing for edge detection
- [ ] Add unit tests for edge detection

### Phase 7: Stairs Climb Mode
- [ ] Add `STAIRS_CLIMB_*` states to hip_behavior
- [ ] Implement climb state machine (approach → lift → transfer → pull → level)
- [ ] Add `hip_behavior_request_stairs_climb()` API
- [ ] Integrate with wheel torque scaling
- [ ] Add safety abort conditions
- [ ] Add unit tests for climb sequence

### Phase 8: Stairs Descend Mode (Sequential)
- [ ] Add `STAIRS_DESCEND_*` states to hip_behavior
- [ ] Implement descend state machine (approach → extend → transfer → lower → level)
- [ ] Add `hip_behavior_request_stairs_descend()` API
- [ ] Add extend timeout safety (no ground detection)
- [ ] Add unit tests for descend sequence

### Phase 8b: Stairs Descend Mode (Simultaneous)
- [ ] Add dual edge detector with time window
- [ ] Add `STAIRS_DESCEND_BOTH_*` states
- [ ] Implement simultaneous descent (both extend → both settle → level)
- [ ] Add symmetric motion constraints (roll limit, height diff limit)
- [ ] Integrate decision logic: sequential vs simultaneous based on edge timing
- [ ] Add unit tests for simultaneous descent
- [ ] Add unit tests for edge timing classification

### Phase 9: Integration & Tuning
- [ ] Integrate all components
- [ ] Add terrain adaptation enable/disable
- [ ] Tune parameters on hardware
- [ ] Validate performance requirements
- [ ] Test stair climb/descend on actual stairs

---

## 6 Configuration Defaults

Add to `app_config.h`:

```c
/* Terrain adaptation - closed-loop roll controller */
#ifndef HIP_TERRAIN_ADAPT_ENABLE
#define HIP_TERRAIN_ADAPT_ENABLE 1
#endif
#ifndef HIP_TERRAIN_ROLL_KP
#define HIP_TERRAIN_ROLL_KP 2.0f
#endif
#ifndef HIP_TERRAIN_ROLL_KD
#define HIP_TERRAIN_ROLL_KD 0.1f
#endif
#ifndef HIP_TERRAIN_MAX_DIFF_M
#define HIP_TERRAIN_MAX_DIFF_M 0.05f
#endif
#ifndef HIP_TERRAIN_ADAPT_RATE_MPS
#define HIP_TERRAIN_ADAPT_RATE_MPS 0.1f
#endif

/* Wheel torque coordination */
#ifndef HIP_WHEEL_TORQUE_REDUCTION
#define HIP_WHEEL_TORQUE_REDUCTION 0.5f
#endif
#ifndef HIP_WHEEL_TORQUE_MIN_SCALE
#define HIP_WHEEL_TORQUE_MIN_SCALE 0.3f
#endif

/* Contact detection */
#ifndef HIP_CONTACT_FORCE_GROUND_N
#define HIP_CONTACT_FORCE_GROUND_N 50.0f
#endif
#ifndef HIP_CONTACT_FORCE_LIGHT_N
#define HIP_CONTACT_FORCE_LIGHT_N 10.0f
#endif
#ifndef HIP_CONTACT_DEBOUNCE_MS
#define HIP_CONTACT_DEBOUNCE_MS 20U
#endif

/* Stairs mode */
#ifndef STAIR_STEP_HEIGHT_M
#define STAIR_STEP_HEIGHT_M 0.18f
#endif
#ifndef STEP_EDGE_FORCE_THRESHOLD_N
#define STEP_EDGE_FORCE_THRESHOLD_N 20.0f
#endif
#ifndef STEP_EDGE_FORCE_RATE_THRESHOLD
#define STEP_EDGE_FORCE_RATE_THRESHOLD -100.0f
#endif
#ifndef STAIRS_WHEEL_TORQUE_SCALE
#define STAIRS_WHEEL_TORQUE_SCALE 0.4f
#endif
#ifndef STAIRS_EXTEND_TIMEOUT_MS
#define STAIRS_EXTEND_TIMEOUT_MS 1000U
#endif

/* Per-contact-state impedance */
#ifndef HIP_GROUND_STIFFNESS_N_M
#define HIP_GROUND_STIFFNESS_N_M 800.0f
#endif
#ifndef HIP_LIGHT_STIFFNESS_N_M
#define HIP_LIGHT_STIFFNESS_N_M 400.0f
#endif
#ifndef HIP_AIR_STIFFNESS_N_M
#define HIP_AIR_STIFFNESS_N_M 200.0f
#endif
```

---

## 7 Telemetry Extensions

Add to `robot_telem_v3_t`:

```c
/* Per-leg contact state */
uint8_t hip_left_contact;      /* hip_contact_state_t */
uint8_t hip_right_contact;
float hip_left_force_n;        /* Estimated vertical force */
float hip_right_force_n;

/* Terrain adaptation state */
float hip_left_height_adj_m;   /* Current left adjustment */
float hip_right_height_adj_m;  /* Current right adjustment */
uint8_t hip_terrain_adapt_active;

/* Wheel coordination */
float wheel_scale_left;        /* Wheel torque scale factor (0.3-1.0) */
float wheel_scale_right;       /* Wheel torque scale factor (0.3-1.0) */
```

---

## 8 Design Decisions (Resolved)

| Question | Decision | Section |
|----------|----------|---------|
| Roll source | EKF-filtered roll (smoother, acceptable latency) | §3.1, §4.2 |
| Control type | Closed-loop PD on roll angle + rate | §4.2 |
| Wheel coordination | Yes, reduce torque on retracting side | §4.5 |

## 9 Stairs Mode

### 9.1 Overview

Stairs mode provides coordinated leg control for ascending and descending staircases. Unlike continuous terrain adaptation (which reacts to roll), stairs mode actively manages leg placement through each step.

```
Stair Climb:                          Stair Descend:

    ┌───┐                                 ●═══●  Robot
    │   │ ●═══●  Robot                ┌───┤   │
┌───┤   │ │   │                   ┌───┤   └───┘
│   └───┘ └───┘               ┌───┤   └───────┘
└─────────────────            └───────────────────
```

### 9.2 Stairs Behavior State Machine

```c
typedef enum {
    STAIRS_IDLE = 0,           /* Not in stairs mode */
    STAIRS_CLIMB_APPROACH,     /* Approaching step, detecting edge */
    STAIRS_CLIMB_LIFT,         /* Lifting front leg onto step */
    STAIRS_CLIMB_TRANSFER,     /* Weight transfer to front leg */
    STAIRS_CLIMB_PULL,         /* Pulling rear leg up */
    STAIRS_CLIMB_LEVEL,        /* Leveling on step, prepare for next */
    STAIRS_DESCEND_APPROACH,   /* Approaching step edge */
    STAIRS_DESCEND_EXTEND,     /* Extending lead leg down (sequential) */
    STAIRS_DESCEND_TRANSFER,   /* Weight transfer to lower leg */
    STAIRS_DESCEND_LOWER,      /* Lowering trail leg */
    STAIRS_DESCEND_LEVEL,      /* Leveling, prepare for next */
    STAIRS_DESCEND_BOTH_EXTEND, /* Both legs extending simultaneously */
    STAIRS_DESCEND_BOTH_SETTLE  /* Both legs landed, stabilizing */
} stairs_state_t;
```

### 9.3 Stairs Mode API

```c
/* Enter/exit stairs mode */
void hip_behavior_request_stairs_climb(void);
void hip_behavior_request_stairs_descend(void);
void hip_behavior_cancel_stairs(void);

/* Query state */
stairs_state_t hip_behavior_get_stairs_state(void);
uint8_t hip_behavior_get_stairs_step_count(void);  /* Steps completed */
```

### 9.4 Stair Climb Sequence

| Phase | Action | Height (Lead) | Height (Trail) | Stiffness | Wheel Torque |
|-------|--------|---------------|----------------|-----------|--------------|
| APPROACH | Detect step edge via force drop | nominal | nominal | high | normal |
| LIFT | Raise lead leg to clear step | +step_height | nominal | medium | reduced |
| TRANSFER | Shift weight to lead leg | +step_height | nominal | high | reduced |
| PULL | Raise trail leg | +step_height | +step_height | medium | reduced |
| LEVEL | Both legs at new height | +step_height | +step_height | high | normal |

```c
void stairs_climb_tick(uint32_t now_ms, ...) {
    switch (s_stairs.state) {
    case STAIRS_CLIMB_APPROACH:
        /* Detect step: lead leg force drops as it goes over edge */
        if (lead_force_n < STEP_EDGE_FORCE_THRESHOLD_N) {
            s_stairs.state = STAIRS_CLIMB_LIFT;
            s_stairs.step_edge_detected_ms = now_ms;
        }
        break;

    case STAIRS_CLIMB_LIFT:
        /* Raise lead leg by step height */
        lead_height_target = nominal_height + STAIR_STEP_HEIGHT_M;
        trail_height_target = nominal_height;
        if (lead_height_reached && (now_ms - s_stairs.phase_start_ms) > MIN_LIFT_MS) {
            s_stairs.state = STAIRS_CLIMB_TRANSFER;
        }
        break;

    case STAIRS_CLIMB_TRANSFER:
        /* Wait for weight shift - lead force increases */
        if (lead_force_n > WEIGHT_TRANSFER_FORCE_N) {
            s_stairs.state = STAIRS_CLIMB_PULL;
        }
        break;

    case STAIRS_CLIMB_PULL:
        /* Raise trail leg to match */
        trail_height_target = nominal_height + STAIR_STEP_HEIGHT_M;
        if (trail_height_reached) {
            s_stairs.state = STAIRS_CLIMB_LEVEL;
            s_stairs.step_count++;
        }
        break;

    case STAIRS_CLIMB_LEVEL:
        /* Stabilize, then look for next step or exit */
        if (stable && no_more_steps_detected) {
            s_stairs.state = STAIRS_IDLE;
        } else if (stable) {
            s_stairs.state = STAIRS_CLIMB_APPROACH;
        }
        break;
    }
}
```

### 9.5 Stair Descend Sequence

| Phase | Action | Height (Lead) | Height (Trail) | Stiffness | Wheel Torque |
|-------|--------|---------------|----------------|-----------|--------------|
| APPROACH | Detect step edge (lead leg force drops) | nominal | nominal | high | reduced |
| EXTEND | Extend lead leg down to find next step | -step_height | nominal | low | reduced |
| TRANSFER | Shift weight to lower leg | -step_height | nominal | high | reduced |
| LOWER | Lower trail leg to match | -step_height | -step_height | medium | reduced |
| LEVEL | Both legs at new height | -step_height | -step_height | high | normal |

```c
void stairs_descend_tick(uint32_t now_ms, ...) {
    switch (s_stairs.state) {
    case STAIRS_DESCEND_APPROACH:
        /* Slow approach - detect edge when lead force drops */
        wheel_torque_scale = 0.5f;  /* Slow down */
        if (lead_force_n < STEP_EDGE_FORCE_THRESHOLD_N) {
            s_stairs.state = STAIRS_DESCEND_EXTEND;
        }
        break;

    case STAIRS_DESCEND_EXTEND:
        /* Extend lead leg down, soft stiffness to feel for ground */
        lead_height_target = nominal_height - STAIR_STEP_HEIGHT_M;
        lead_stiffness = HIP_AIR_STIFFNESS_N_M;  /* Soft */

        /* Detect ground contact */
        if (lead_force_n > GROUND_CONTACT_FORCE_N) {
            s_stairs.state = STAIRS_DESCEND_TRANSFER;
        }
        /* Timeout safety - didn't find ground */
        if ((now_ms - s_stairs.phase_start_ms) > EXTEND_TIMEOUT_MS) {
            /* Abort - retract and stop */
            hip_behavior_cancel_stairs();
            set_fault(STAIRS_FAULT_NO_GROUND);
        }
        break;

    case STAIRS_DESCEND_TRANSFER:
        /* Shift weight to lower leg */
        if (lead_force_n > WEIGHT_TRANSFER_FORCE_N) {
            s_stairs.state = STAIRS_DESCEND_LOWER;
        }
        break;

    case STAIRS_DESCEND_LOWER:
        /* Lower trail leg */
        trail_height_target = nominal_height - STAIR_STEP_HEIGHT_M;
        if (trail_height_reached && trail_force_n > GROUND_CONTACT_FORCE_N) {
            s_stairs.state = STAIRS_DESCEND_LEVEL;
            s_stairs.step_count++;
        }
        break;

    case STAIRS_DESCEND_LEVEL:
        /* Stabilize */
        if (stable && no_more_steps_detected) {
            s_stairs.state = STAIRS_IDLE;
        } else if (stable) {
            s_stairs.state = STAIRS_DESCEND_APPROACH;
        }
        break;
    }
}
```

### 9.6 Step Edge Detection

Detect step edges using force discontinuity:

```c
typedef struct {
    float force_prev_n;
    float force_rate_n_s;
    uint32_t last_update_ms;
    uint8_t edge_detected;
    uint8_t edge_type;  /* 0=none, 1=step-up, 2=step-down */
} step_edge_detector_t;

void step_edge_update(step_edge_detector_t *det, float force_n, uint32_t now_ms) {
    float dt_s = 0.001f * (float)(now_ms - det->last_update_ms);
    if (dt_s > 0.0f) {
        det->force_rate_n_s = (force_n - det->force_prev_n) / dt_s;
    }
    det->force_prev_n = force_n;
    det->last_update_ms = now_ms;

    /* Detect sudden force drop (step edge) */
    if (det->force_rate_n_s < -STEP_EDGE_FORCE_RATE_THRESHOLD) {
        det->edge_detected = 1U;
        det->edge_type = (force_n < STEP_EDGE_FORCE_MIN_N) ? 2U : 1U;
    } else {
        det->edge_detected = 0U;
        det->edge_type = 0U;
    }
}
```

### 9.7 Stairs Parameters

```c
/* Stairs geometry */
#ifndef STAIR_STEP_HEIGHT_M
#define STAIR_STEP_HEIGHT_M 0.18f           /* Standard stair rise: 18cm */
#endif
#ifndef STAIR_STEP_HEIGHT_TOLERANCE_M
#define STAIR_STEP_HEIGHT_TOLERANCE_M 0.03f /* ±3cm tolerance */
#endif

/* Edge detection */
#ifndef STEP_EDGE_FORCE_THRESHOLD_N
#define STEP_EDGE_FORCE_THRESHOLD_N 20.0f   /* Force drop indicating edge */
#endif
#ifndef STEP_EDGE_FORCE_RATE_THRESHOLD
#define STEP_EDGE_FORCE_RATE_THRESHOLD -100.0f  /* N/s - sudden drop rate */
#endif

/* Phase timing */
#ifndef STAIRS_LIFT_MIN_MS
#define STAIRS_LIFT_MIN_MS 200U
#endif
#ifndef STAIRS_TRANSFER_TIMEOUT_MS
#define STAIRS_TRANSFER_TIMEOUT_MS 500U
#endif
#ifndef STAIRS_EXTEND_TIMEOUT_MS
#define STAIRS_EXTEND_TIMEOUT_MS 1000U      /* Safety timeout for descend */
#endif
#ifndef STAIRS_LEVEL_SETTLE_MS
#define STAIRS_LEVEL_SETTLE_MS 100U
#endif

/* Wheel control during stairs */
#ifndef STAIRS_WHEEL_TORQUE_SCALE
#define STAIRS_WHEEL_TORQUE_SCALE 0.4f      /* Reduced torque during stairs */
#endif
#ifndef STAIRS_APPROACH_SPEED_SCALE
#define STAIRS_APPROACH_SPEED_SCALE 0.3f    /* Slow approach to edge */
#endif

/* Simultaneous descent (two-wheeled robot) */
#ifndef SIMULTANEOUS_EDGE_WINDOW_MS
#define SIMULTANEOUS_EDGE_WINDOW_MS 50U     /* Time window to consider edges simultaneous */
#endif
#ifndef STAIRS_BOTH_MAX_ROLL_RAD
#define STAIRS_BOTH_MAX_ROLL_RAD 0.15f      /* ~8.5 deg abort threshold during both-extend */
#endif
#ifndef STAIRS_BOTH_MAX_HEIGHT_DIFF_M
#define STAIRS_BOTH_MAX_HEIGHT_DIFF_M 0.02f /* 2cm - abort if legs diverge */
#endif
#ifndef STAIRS_BOTH_EXTEND_TORQUE_SCALE
#define STAIRS_BOTH_EXTEND_TORQUE_SCALE 0.1f /* Very low wheel torque during free descent */
#endif
```

### 9.8 Safety Considerations

| Hazard | Detection | Response |
|--------|-----------|----------|
| Missing step (descend) | Extend timeout without ground contact | Retract leg, abort stairs, fault |
| Step too high | Height delta > max | Abort, fault |
| Loss of balance | Roll > threshold during transfer | Abort, emergency level |
| Stall during lift | Motor stall fault | Hold position, abort |
| Slip during transfer | Sudden force drop on planted leg | Increase stiffness, slow down |

```c
/* Emergency abort */
void stairs_abort(stairs_fault_t fault) {
    s_stairs.state = STAIRS_IDLE;
    s_stairs.fault = fault;

    /* Return legs to nominal height */
    left_height_target = nominal_height;
    right_height_target = nominal_height;

    /* High stiffness for stability */
    stiffness = HIP_LANDING_STIFFNESS_N_M;

    /* Reduce wheel torque */
    wheel_torque_scale = STAIRS_WHEEL_TORQUE_SCALE;

    APP_LOG_WARN("Stairs abort: fault=%u", (unsigned)fault);
}
```

### 9.9 Integration with Hip Behavior

Stairs mode is mutually exclusive with jump mode:

```c
typedef enum {
    HIP_BEHAVIOR_NORMAL = 0,
    HIP_BEHAVIOR_CROUCH = 1,
    HIP_BEHAVIOR_IMPULSE = 2,
    HIP_BEHAVIOR_FLIGHT = 3,
    HIP_BEHAVIOR_LANDING = 4,
    HIP_BEHAVIOR_STAIRS_CLIMB = 5,   /* New */
    HIP_BEHAVIOR_STAIRS_DESCEND = 6  /* New */
} hip_behavior_mode_t;

void hip_behavior_tick(...) {
    ...
    /* Disable terrain roll adaptation during stairs (legs controlled sequentially) */
    target_out->terrain_adapt_enabled =
        (s_behavior.mode == HIP_BEHAVIOR_NORMAL) &&
        (motion_mode == MOTION_MODE_BALANCING);

    /* Stairs modes */
    if (s_behavior.mode == HIP_BEHAVIOR_STAIRS_CLIMB) {
        stairs_climb_tick(now_ms, left, right, target_out);
    } else if (s_behavior.mode == HIP_BEHAVIOR_STAIRS_DESCEND) {
        stairs_descend_tick(now_ms, left, right, target_out);
    }
    ...
}
```

### 9.10 Telemetry Extensions for Stairs

```c
/* Add to robot_telem_v3_t */
uint8_t stairs_state;           /* stairs_state_t */
uint8_t stairs_step_count;      /* Steps completed in current sequence */
uint8_t stairs_fault;           /* stairs_fault_t if any */
float stairs_lead_height_m;     /* Current lead leg height target */
float stairs_trail_height_m;    /* Current trail leg height target */
```

### 9.11 Simultaneous Step Detection (Two-Wheeled Descent)

For a two-wheeled robot going straight down stairs, both wheels reach the step edge at the same time. The sequential lead/trail approach doesn't apply—both legs must descend together.

**Detection:** Both legs show force drop simultaneously (within a small time window):

```c
#define SIMULTANEOUS_EDGE_WINDOW_MS 50U  /* Max time between leg edge detections */

typedef struct {
    uint8_t left_edge_detected;
    uint8_t right_edge_detected;
    uint32_t left_edge_ms;
    uint32_t right_edge_ms;
} dual_edge_detector_t;

uint8_t is_simultaneous_edge(dual_edge_detector_t *det, uint32_t now_ms) {
    if (!det->left_edge_detected || !det->right_edge_detected) {
        return 0U;
    }
    uint32_t delta = (det->left_edge_ms > det->right_edge_ms) ?
                     (det->left_edge_ms - det->right_edge_ms) :
                     (det->right_edge_ms - det->left_edge_ms);
    return (delta <= SIMULTANEOUS_EDGE_WINDOW_MS) ? 1U : 0U;
}
```

**New States:** Add coordinated descent states:

```c
typedef enum {
    /* ... existing states ... */
    STAIRS_DESCEND_BOTH_EXTEND,   /* Both legs extending down simultaneously */
    STAIRS_DESCEND_BOTH_SETTLE    /* Both legs settling on lower step */
} stairs_state_t;
```

**Coordinated Descent Sequence:**

| Phase | Action | Left Height | Right Height | Stiffness | Wheel Torque |
|-------|--------|-------------|--------------|-----------|--------------|
| APPROACH | Detect simultaneous edge | nominal | nominal | high | reduced |
| BOTH_EXTEND | Lower both legs together | -step_height | -step_height | low | minimal |
| BOTH_SETTLE | Both legs land, stabilize | -step_height | -step_height | high | normal |
| LEVEL | Ready for next step | -step_height | -step_height | high | normal |

**Implementation:**

```c
void stairs_descend_simultaneous_tick(uint32_t now_ms,
                                       const hip_state_t *left,
                                       const hip_state_t *right,
                                       hip_target_t *target) {
    switch (s_stairs.state) {
    case STAIRS_DESCEND_APPROACH:
        /* Check for simultaneous edge detection */
        step_edge_update(&s_edge_left, left->force_n, now_ms);
        step_edge_update(&s_edge_right, right->force_n, now_ms);

        if (s_edge_left.edge_detected) {
            s_dual_edge.left_edge_detected = 1U;
            s_dual_edge.left_edge_ms = now_ms;
        }
        if (s_edge_right.edge_detected) {
            s_dual_edge.right_edge_detected = 1U;
            s_dual_edge.right_edge_ms = now_ms;
        }

        if (is_simultaneous_edge(&s_dual_edge, now_ms)) {
            /* Both wheels at edge - coordinated descent */
            s_stairs.state = STAIRS_DESCEND_BOTH_EXTEND;
            s_stairs.phase_start_ms = now_ms;
        } else if (s_dual_edge.left_edge_detected && !s_dual_edge.right_edge_detected &&
                   (now_ms - s_dual_edge.left_edge_ms) > SIMULTANEOUS_EDGE_WINDOW_MS) {
            /* Only left detected - sequential descent (left leads) */
            s_stairs.lead_leg = LEG_LEFT;
            s_stairs.state = STAIRS_DESCEND_EXTEND;
        } else if (s_dual_edge.right_edge_detected && !s_dual_edge.left_edge_detected &&
                   (now_ms - s_dual_edge.right_edge_ms) > SIMULTANEOUS_EDGE_WINDOW_MS) {
            /* Only right detected - sequential descent (right leads) */
            s_stairs.lead_leg = LEG_RIGHT;
            s_stairs.state = STAIRS_DESCEND_EXTEND;
        }
        break;

    case STAIRS_DESCEND_BOTH_EXTEND:
        /* Lower both legs together with matched rate */
        target->left_height_adj_m = -STAIR_STEP_HEIGHT_M;
        target->right_height_adj_m = -STAIR_STEP_HEIGHT_M;
        target->left_stiffness_n_m = HIP_AIR_STIFFNESS_N_M;   /* Soft for landing */
        target->right_stiffness_n_m = HIP_AIR_STIFFNESS_N_M;

        /* Minimal wheel torque - we're "falling" onto next step */
        wheel_torque_scale = 0.1f;

        /* Check for ground contact on BOTH legs */
        uint8_t left_ground = (left->force_n > GROUND_CONTACT_FORCE_N);
        uint8_t right_ground = (right->force_n > GROUND_CONTACT_FORCE_N);

        if (left_ground && right_ground) {
            s_stairs.state = STAIRS_DESCEND_BOTH_SETTLE;
            s_stairs.phase_start_ms = now_ms;
        }

        /* Timeout safety */
        if ((now_ms - s_stairs.phase_start_ms) > STAIRS_EXTEND_TIMEOUT_MS) {
            stairs_abort(STAIRS_FAULT_NO_GROUND);
        }
        break;

    case STAIRS_DESCEND_BOTH_SETTLE:
        /* High stiffness to absorb landing and stabilize */
        target->left_stiffness_n_m = HIP_LANDING_STIFFNESS_N_M;
        target->right_stiffness_n_m = HIP_LANDING_STIFFNESS_N_M;
        target->left_damping_n_s_m = HIP_LANDING_DAMPING_N_S_M;
        target->right_damping_n_s_m = HIP_LANDING_DAMPING_N_S_M;

        /* Restore wheel torque gradually */
        wheel_torque_scale = 0.5f;

        if ((now_ms - s_stairs.phase_start_ms) > STAIRS_LEVEL_SETTLE_MS) {
            s_stairs.state = STAIRS_DESCEND_LEVEL;
            s_stairs.step_count++;
        }
        break;
    }
}
```

**Critical Differences from Sequential Descent:**

| Aspect | Sequential | Simultaneous |
|--------|------------|--------------|
| Edge detection | Wait for lead leg only | Wait for both legs (with window) |
| Weight transfer | Shift to lead before trailing | No weight transfer needed |
| Extension | One leg at a time | Both legs together |
| Ground detection | Wait for lead, then trail | Require both before settling |
| Wheel torque | Reduced during sequence | Minimal (near-free-fall) |
| Roll control | Active (compensating) | Disabled (symmetric motion) |
| Stability | Lead leg provides anchor | Rely on balanced symmetric descent |

**Safety for Simultaneous Descent:**

Since there's no "planted" leg during extension, simultaneous descent is inherently less stable:

```c
/* Additional safety checks for simultaneous descent */
if (s_stairs.state == STAIRS_DESCEND_BOTH_EXTEND) {
    /* Roll must stay small - no anchor leg */
    float roll_rad = ekf_get_roll();
    if (fabsf(roll_rad) > STAIRS_BOTH_MAX_ROLL_RAD) {
        stairs_abort(STAIRS_FAULT_ROLL_EXCEEDED);
    }

    /* Height tracking must stay symmetric */
    float height_diff = fabsf(left->height_m - right->height_m);
    if (height_diff > STAIRS_BOTH_MAX_HEIGHT_DIFF_M) {
        stairs_abort(STAIRS_FAULT_ASYMMETRIC);
    }
}
```

**Parameters for Simultaneous Descent:**

```c
#ifndef SIMULTANEOUS_EDGE_WINDOW_MS
#define SIMULTANEOUS_EDGE_WINDOW_MS 50U    /* Time window to consider edges simultaneous */
#endif
#ifndef STAIRS_BOTH_MAX_ROLL_RAD
#define STAIRS_BOTH_MAX_ROLL_RAD 0.15f     /* ~8.5 deg - abort threshold during both-extend */
#endif
#ifndef STAIRS_BOTH_MAX_HEIGHT_DIFF_M
#define STAIRS_BOTH_MAX_HEIGHT_DIFF_M 0.02f /* 2cm - abort if legs diverge */
#endif
#ifndef STAIRS_BOTH_EXTEND_TORQUE_SCALE
#define STAIRS_BOTH_EXTEND_TORQUE_SCALE 0.1f  /* Very low wheel torque during free descent */
#endif
```

**Decision Flow:**

```
                        ┌────────────────────┐
                        │  DESCEND_APPROACH  │
                        └────────┬───────────┘
                                 │
                    ┌────────────┴────────────┐
                    │  Edge detected on       │
                    │  left, right, or both?  │
                    └─────────┬───────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
        ▼                     ▼                     ▼
   Left only             Both within            Right only
   (wait window)          window               (wait window)
        │                     │                     │
        ▼                     ▼                     ▼
┌───────────────┐   ┌─────────────────┐   ┌────────────────┐
│ DESCEND_EXTEND│   │DESCEND_BOTH_EXT │   │ DESCEND_EXTEND │
│ (left leads)  │   │(symmetric lower)│   │ (right leads)  │
└───────────────┘   └─────────────────┘   └────────────────┘
```

---

## 10 Open Questions

1. **Which leg leads?** (Partially resolved)
   - For simultaneous edge detection (§9.11): Both legs descend together, no lead
   - For sequential descent: Lead is auto-selected based on which leg detects edge first
   - **Remaining question for climb:** Should lead leg be:
     - Fixed (always left or right)?
     - Direction-based (uphill leg leads)?
     - Configurable?

2. **Continuous stair climbing:** After completing one step, should robot:
   - Pause and wait for command to continue?
   - Automatically detect and climb next step?
   - Require explicit "continue" command?

3. **Speed during stairs:** How fast should wheel torque be during stair phases?
   - Configurable per-phase?
   - Adaptive based on stability?

---

## 11 References

- Existing hip control: `app/control/hip_control.c`
- Hip behavior state machine: `app/control/hip_behavior.c`
- Hip kinematics: `app/control/hip_kinematics.c`
- Motion control integration: `app/control/motion_control.cpp`
