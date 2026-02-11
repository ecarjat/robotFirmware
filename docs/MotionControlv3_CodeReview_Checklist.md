# MotionControlv3 Core Review Checklist

Code review findings for the hip control system implementation.

Legend: ✅ fixed, 🟡 acknowledged, ❌ to address, ⏳ deferred.

## 1 Code Issues

### 1.1 Critical
- ✅ Duplicated `UNIT_TEST` estimate override block in `motion_control.cpp` removed
- ✅ Indentation issue in `motion_control.cpp` fixed

### 1.2 Minor
- ✅ NaN comparison pattern replaced with `isnan()` checks (`hip_control.c`, `hip_behavior.c`)
- ✅ Recovery mode now updates `s_height_ref_m` to avoid snap-back

## 2 Architectural Risks

| ID | Risk | Impact | Status |
|----|------|--------|--------|
| R1 | CAN bus congestion (hip + wheel commands share bus) | Commands delayed | 🟡 Separate filters added; monitor queue depth |
| R2 | Stall timeout false positives during transients | Hip disabled unexpectedly | 🟡 500ms + 0.02 rev threshold; tune if needed |
| R3 | Height slew rate limiting constrains jump dynamics | Slow jump response | 🟡 `HIP_HEIGHT_SLEW_MPS=0.2` may need tuning |
| R4 | Recovery mode could get stuck if limit switches fail | Robot stuck at limit | ✅ Recovery timeout escape + block until limit clears |
| R5 | Shared stall fault flag for both motors | Premature fault clear | ✅ Per-motor stall flags added |

## 3 Suggestions

### 3.1 Code Clarity
- ✅ `APP_CONFIG_HOST` dummy typedefs moved to `tests/fakes/app_config_host.h`
- ✅ NaN self-comparison removed in favor of `isnan()`
- ✅ CAN message format documented in `hip_control.c` header comment

### 3.2 Configuration
- ✅ Wheel scaling factors configurable in `app_config.h`:
  - `HIP_WHEEL_SCALE_IMPULSE`
  - `HIP_WHEEL_SCALE_FLIGHT`
  - `HIP_WHEEL_SCALE_LANDING`

### 3.3 Safety
- ✅ Per-motor stall fault flags (`HIP_FAULT_STALL_LEFT`, `HIP_FAULT_STALL_RIGHT`) added
- ✅ Height-based jump phase transitions implemented (crouch/impulse guards)
- ✅ IMU feedback used for liftoff/landing detection

## 4 Improvements

### 4.1 Hip Behavior State Machine
- ✅ Add height-based transition triggers (crouch/impulse guards)
- ✅ Add IMU-based liftoff/landing detection for more robust jump phases
- ✅ Add abort/cancel jump capability

### 4.2 Hip Control
- ✅ Split stall fault into per-motor flags
- ✅ Recovery mode timeout escape (with block until limit clears)
- ✅ Update `s_height_ref_m` during recovery mode to avoid snap-back

### 4.3 Telemetry
- ✅ Per-motor stall status already exposed via `hip_faults` bitfield
- ✅ Add jump phase progress indicator (% complete)

## 5 Additional Tests

### 5.1 Missing Unit Tests
- ✅ `test_hip_control.cpp`: Stall detection scenarios (single motor stall, both motors stall, recovery from stall)
- ✅ `test_hip_control.cpp`: Recovery mode behavior (enter on limit, exit when clear, theta range escape, timeout)
- ✅ `test_hip_control.cpp`: Concurrent fault conditions (bus-off + encoder timeout)
- ✅ `test_hip_behavior.cpp`: Height reference tracking during jump phases
- ✅ `test_motion_control.cpp`: Wheel scaling during all jump phases (crouch baseline, impulse, flight, landing)

### 5.2 Integration Tests
- ✅ Full jump sequence with simulated motor feedback
- ✅ Limit switch recovery with encoder feedback
- ✅ CAN bus-off recovery sequence

### 5.3 Edge Cases
- ✅ `test_hip_kinematics.cpp`: Out-of-range height rejection + min/max acceptance
- ✅ `test_hip_limits.cpp`: Rapid switch bouncing (faster than debounce period)
- ✅ `test_hip_control.cpp`: Zero offset at extreme values

## 6 Documentation

- ✅ Document CAN message IDs and payload formats
- ✅ Add block diagram showing hip control data flow
- ✅ Add tuning guide for impedance/stiffness parameters

---

## Summary

**Blockers:** Resolved ✅  
**High priority:** Resolved ✅  
**Deferred (track for future):** None.
