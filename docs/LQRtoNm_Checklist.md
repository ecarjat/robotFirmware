# LQRtoNm Implementation Checklist

Spec: `docs/LQRtoNm.md`

Legend: ✅ done, ❌ missing, 🟡 partial.

## Checklist

### 1. LQR path output in torque (Nm)
- ✅ `MotionController::computeLqrUSumNm` returns Nm (`app/control/MotionController.cpp`)
- ✅ Rename exposed test helper to Nm (`app/control/MotionController.h`, `tests/test_motion_controller.cpp`)

### 2. Blend in iq after converting LQR Nm → iq
- ✅ Convert LQR Nm → iq before blend (`app/control/MotionController.cpp`)
- ✅ Keep PID terms in iq (no changes required)
- ✅ Keep yaw differential in iq (no changes required)

### 3. Diagnostics (units clarified)
- ✅ `InnerCtrlDiag::u_sum_lqr` documented as Nm (`app/control/MotionController.h`)
- ✅ `u_sum_cmd` remains iq (A) (`app/control/MotionController.h`)

### 4. LQR limit semantics (torque)
- ✅ `lqr.u_limit` / `lqr.du_limit` applied in torque (`app/control/MotionController.cpp`)

### 5. Backend torque→iq helper
- ✅ Add `motor_link_torque_to_iq` API (`app/drivers/motors/motor_link.h/.c`)
- ✅ Backend implementations reuse helper logic (steadywin + robust UART)
  - `app/drivers/motors/motor_backend_steadywin_can_impl.inc`
  - `app/drivers/motors/motor_backend_robust_uart_impl.inc`
- ✅ Hook helper into backend ops (`app/drivers/motors/motor_backend_steadywin_can.c`,
  `app/drivers/motors/motor_backend_robust_uart.c`)
- ✅ Host fake implements helper (`tests/fakes/motor_link_fake.c`)

### 6. Tests
- ✅ Unit test for LQR Nm computation (`tests/test_motion_controller.cpp`)

## Spec gap report

No gaps found. All sections in `docs/LQRtoNm.md` are implemented.
