# MotionControl v3 Test Checklist

## Hip kinematics
- [x] Height from theta at min/max bounds.
- [x] Height increases monotonically across operating range.
- [x] Theta from height inverts correctly.
- [x] Height range helper returns valid bounds.

## Hip limit switches
- [x] Debounce requires consecutive samples.
- [x] Limit clamp prevents motion further into the active limit.
- [x] Recovery mode on boot when a limit is active (debounced before init).

## Hip control
- [x] Startup sequence (ctrl mode -> axis state).
- [x] Sends position commands after encoder telemetry.
- [x] Bus-off disables commands and sets fault.
- [x] Telemetry timeout disables commands.
- [x] Stall fault triggers when command is not followed.
- [x] Target height clamps to kinematic range.
- [x] Height slew rate limits step size.
- [x] Direction sign inversion flips command.
- [x] Velocity and torque feedforward clamp to configured limits.
- [x] Heartbeat timeout raises fault.
- [x] Recovery motion stops after max travel if opposing limit never triggers.

## Hip behavior
- [x] Normal mode holds measured height.
- [x] Jump sequence advances through phases.
- [x] Non-balancing mode disables outputs.

## Telemetry and RPC
- [x] Telemetry v3 includes hip fields and faults.
- [x] Hip calibration RPC returns NOT_READY when invalid.
- [x] Hip calibration RPC writes zero offsets and saves when requested.

## Motion control integration
- [x] Hip behavior -> hip_control target wiring in motion_control coordinator.
- [x] Wheel feedforward adjustment from hip velocity during jump phases.
