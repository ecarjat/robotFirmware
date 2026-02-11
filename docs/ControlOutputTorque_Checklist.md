# ControlOutput Torque Implementation Checklist

Spec: `docs/ControlOutputTorqueSpec.md`

Legend: ✅ done, ❌ missing, 🟡 partial.

## Checklist

### 1) ControlOutput units
- ✅ `MotionController::ControlOutput` fields renamed to torque (`app/control/MotionController.h`)
- ✅ Command wrapper uses torque (`app/control/MotionController.h/.cpp`)
- ✅ Motion control output struct renamed to torque fields (`app/control/motion_control.h`)

### 2) MotionController output path
- ✅ `computeControl` returns torque (Nm) and blends in torque (`app/control/MotionController.cpp`)
- ✅ PID path converted to torque at output boundary using motorKt (`app/control/MotionController.cpp`)
- ✅ Yaw differential converted to torque before mixing (`app/control/MotionController.cpp`)
- ✅ Output saturation applied in torque units (`app/control/MotionController.cpp`)
- ✅ `lqr.u_limit` / `lqr.du_limit` interpreted as Nm / Nm/s (`app/control/MotionController.cpp`)

### 3) Motor backend conversion
- ✅ Wheel commands now use `motor_link_set_wheel_torques` (`app/control/motion_control.cpp`)
- ✅ Torque limit derived from `IqMax * PARAM_MOTOR_KT` before dispatch (`app/control/motion_control.cpp`)

### 4) Diagnostics / telemetry
- ✅ Diagnostics unit comments updated to Nm (`app/control/MotionController.h`)
- ✅ Blackbox comments updated to Nm (`app/logging/blackbox_format.h`)
- ✅ Telemetry output populated with torque values (`app/app_telem.c`)
- ✅ Protocol fields renamed to torque and version bumped (`common/shared_protocol/robot_protocol.h`)

### 5) Tests
- ✅ MotionController slew test updated for torque fields (`tests/test_motion_controller.cpp`)
- ✅ Motion control scaling test updated to use torque fake (`tests/test_motion_control.cpp`)
- ✅ Motor link fake captures torque outputs (`tests/fakes/motor_link_fake.c`)

## Spec gap report

No gaps found. All sections in `docs/ControlOutputTorqueSpec.md` are implemented.
