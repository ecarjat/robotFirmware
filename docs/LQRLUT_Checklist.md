# LQR LUT Integration Checklist

Spec: `docs/LQRLUT.md`

Legend: ✅ done, ❌ missing, 🟡 partial.

## Checklist

### 1. LUT module with interpolation
- ✅ `lqr_lut.c/.h` created with LUT data and `lqr_lut_eval` interpolation
- ✅ Clamps to endpoints when hip angle is out of range

### 2. Hip angle source
- ✅ Uses average of left/right hip angles from `hip_control_get_state` (`app/control/motion_control.cpp`)

### 3. LQR gain update scheduling
- ✅ Updates gains at 20 ms cadence (50 Hz) (`app/control/motion_control.cpp`)
- ✅ Keeps `u_limit` / `du_limit` and other params unchanged (copies from `g_robot_params.lqr`)

### 4. Units
- ✅ LUT gains used for torque-domain LQR (`computeLqrUSumNm`)

### 5. Tests
- ✅ `test_lqr_lut` covers endpoint clamp + interpolation

## Spec gap report

No gaps found. All sections in `docs/LQRLUT.md` are implemented.
