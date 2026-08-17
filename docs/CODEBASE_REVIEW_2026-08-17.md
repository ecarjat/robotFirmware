# Firmware codebase review — 2026-08-17

## Scope and validation

This assessment covers the application, control, transport, storage/logging,
motor, build, and test code in the current worktree. It excludes third-party
libraries and CubeMX/HAL internals except where their use affects application
behaviour. The worktree already has substantial uncommitted changes; this
report is assessment-only and intentionally does not alter them.

Review work included code-path tracing, linker-map inspection, a host-test
build/run, and a build of the configured Debug directory.

- Host tests: 15/15 CTest targets passed.
- Firmware: cmake --build build/Debug --target all passed; the build was up to date.
- The documented CubeIDE cube-cmake wrapper is absent at its configured path,
  so the configured CMake build was used as the fallback.

This is not a hardware-in-the-loop or fault-injection qualification. Timing,
CAN behaviour, SD latency, QSPI power-loss recovery, and physical interlocks
still require target testing.

## Priority summary

| Priority | Status | Finding | Consequence |
|---|---|---|---|
| P0 | Fixed | Blackbox dumping could run while balancing | Storage operations could block the foreground control loop. |
| P0 | Fixed | Raw RPC parameters were not semantically validated | A valid frame could apply NaN, infinity, or unsafe limits/gains. |
| P1 | Fixed | Redundant blackbox metadata shared one erase sector | Independent sectors preserve a verified recovery copy through an update. |
| P1 | Open | Normal reboot command is a no-op | Reboot payload 0 does not enter the normal reboot path. |
| P1 | Open | Any valid frame refreshes the control heartbeat | Non-control traffic can keep an armed robot alive. |
| P1 | Open | CAN torque sends are change-driven, not watchdog-driven | Constant torque may not be refreshed after the first transmission. |
| P2 | Open | Telemetry v3 is absent from channel mapping | v3 uses sequence counter 0 rather than telemetry. |
| P2 | Open | File-list pagination cannot retrieve a second page | The response reports more entries without a usable cursor. |
| P2 | Open | Deadline accounting collapses missed timer releases | Overload is underreported and has no defined fault policy. |
| P2 | Open | RAM_D1 is 84% occupied | Only about 52 KiB of DMA-safe RAM headroom remains. |
| P3 | Open | CMake source globbing can miss newly added files | An incremental build can succeed without compiling a new source. |

## Findings

### P0 — Blackbox dump work can block control while armed

Status: fixed. A dump request now establishes an asynchronous logger capture
watermark; it does not synchronously flush QSPI or perform filesystem work.
The capture state remains safe while balancing, while QSPI reads, directory
scans, SD writes, and finalization are deferred until the control mode is no
longer `BALANCING`. The retained capture window is protected from ring
overwrite during the deferred export. Host tests cover a wrapped snapshot,
post-request exclusion, failure cleanup, and the no-export-while-balancing
gate.

Before this fix, app/app_cmd.c accepted a teleop dump edge without checking the
motion mode. The same teleop packet could then arm, while the dump continued
unconditionally from app_idle_tick.

The idle-budget check only verifies that time remains before entering
app_idle_tick; it cannot preempt that work. The dump performs polling QSPI
reads of up to 8 KiB (app/logging/blackbox_dump.c:159-205) and synchronous
FatFs writes (lines 209-235). Starting the dump also calls
log_flush_pending(200U) synchronously (lines 75-77). The control timer ISR
only sets a pending flag, and control work is performed by the same foreground
loop.

Impact: a dump requested while armed, or a dump and arm edge in the same
teleop packet, can delay one or more control cycles and cause a fall or unsafe
actuator behaviour.

Remaining qualification:

1. Run target timing tests for dump-then-arm ordering and deliberately slow SD
   writes.
2. Measure and bound the non-blocking capture service work against the control
   deadline on hardware.

### P0 — RPC can install unsafe live parameters

Status: fixed. `SET_PARAM` now accepts only declared parameter fields, validates
the full candidate (finite values, physical limits, calibration matrices, and
cross-field constraints), and rejects parameter writes while balancing or
manual motor mode is active. `control_timer_set_rate_hz()` also falls back for
non-finite input.

Before this fix, app/app_rpc.c accepted a byte offset and length, copied
arbitrary bytes into a candidate robot_params_t, and applied it after only
bounds checks. No schema-level validation occurred before
motion_control_apply_params or control_timer_set_rate_hz.

For example, a NaN control_rate_hz bypasses the rate_hz <= 1e-3f fallback in
app/control/control_timer.c:87-127. Converting a NaN-derived period to an
unsigned timer value is not a safe configuration path. The same mechanism can
install non-finite gains, invalid matrices, negative limits, or incompatible
geometry while the controller is live.

Recommended actions:

1. Replace raw offset writes with typed field IDs, or centrally validate every
   writable byte range.
2. Validate the full candidate for finite values, physical ranges, valid
   matrices, and cross-field constraints before committing it.
3. Forbid changes to timing, motor limits, and calibration while balancing or
   manual motor mode is active; persist only after dependent reconfiguration
   succeeds.
4. Add tests for NaN, infinity, zero/negative limits, extreme rates, malformed
   matrices, and unsafe mode transitions.

### P1 — Metadata is power-loss redundant

Status: fixed. `LOG_META_SLOT0` now occupies `0x000000-0x000fff` and
`LOG_META_SLOT1` occupies `0x001000-0x001fff`; each is a separate 4 KiB erase
sector. The ring begins at `0x002000`, reducing capacity by 4 KiB.

The metadata save state machine erases only the inactive slot, programs the
candidate, then reads it back and validates its CRC and full contents before
committing the sequence. A failed operation retains the last verified sequence
and retries the same inactive sector; newest-slot selection is wrap-safe across
`UINT32_MAX` to zero. Host tests emulate NOR programming and power loss during
target erase/program, plus failed readback verification and sequence rollover.

This layout intentionally starts a fresh blackbox ring after the firmware
upgrade: legacy metadata has the old ring geometry and is rejected. Record CRC
validation in `log_validate_ring_tail` remains a separate follow-up.

### P1 — Normal reboot is never requested

The protocol defines reboot payload 0 as normal and 1 as bootloader
(common/shared_protocol/robot_protocol.h:43). app/app_cmd.c:81-84 writes zero
to g_reboot_request for normal reboot, but app/app_main.c:178-187 acts only
when that variable is nonzero. Normal reboot is therefore a no-op.

Use explicit values such as NONE=0, BOOTLOADER=1, and NORMAL=2, then add host
tests for both supported payloads and invalid payloads.

### P1 — Transport liveness is conflated with control heartbeat

app/app_main.c describes the timeout as no commands received, but
app/app_link.c:651-673 refreshes s_last_cmd_ms for every valid decoded frame.
ACKs, file requests, RPC responses, and other non-control traffic consequently
keep a balancing robot armed.

Refresh the control deadline only for deliberate, authorized control heartbeat
or teleop frames. Track transport liveness separately. If the USB/UART link is
not physically trusted, add authorization and replay protection for arm,
motor, parameter, and reboot actions.

### P1 — Constant CAN torque is not periodically refreshed

motor_link_set_wheel_torque_commands sends only when a target changes
(app/drivers/motors/motor_backend_steadywin_can_impl.inc:651-682). A constant
non-zero torque is sent once; MOTOR_LINK_TARGET_PERIOD_MS is only a minimum
spacing, not a periodic keepalive. A controller freeze or CAN-path failure
therefore relies on unverified external drive watchdog behaviour.

The diagnostics named left/right ack timeouts increment when adding a frame to
the transmit FIFO fails (lines 664-679), not when an acknowledgement times out.

Define the drive watchdog contract and test it. If it is not independently
fail-safe, transmit command/zero-torque frames at a fixed watchdog rate, disarm
on transmit or telemetry freshness failure, and distinguish queue/TX errors
from actual acknowledgement failures.

### P2 — Telemetry v3 has the wrong sequence channel

robot_channel_from_type maps telemetry v1 and v2 but omits
ROBOT_MSG_TELEM_FRAME_V3 (common/shared_protocol/robot_protocol.c:25-48).
app_link_send therefore increments s_seq_counters[0] for v3 instead of the
telemetry counter (app/app_link.c:231-260). Frames remain valid, but host loss
and ordering accounting is inconsistent.

Map v3 to ROBOT_CHANNEL_TELEM and add a unit test covering every defined
message type and expected channel.

### P2 — File-list pagination is incomplete

app/app_file.c:69-105 sets more=1 when a directory does not fit in one frame,
but app_file_handle_list ignores its request payload and has no cursor
(lines 49-52). Repeating the request always returns the first page.

Add a cursor or opaque continuation token, define directory ordering, and test
directories larger than one response. Alternatively remove the more flag.

### P2 — Deadline diagnostics do not count missed releases

The timer callback stores a timestamp and sets a Boolean
(app/control/control_timer.c:254-260). Multiple timer periods before the main
loop runs collapse into one event. The diagnostic compares only the duration of
the eventual cycle (lines 192-209), so it does not record missed releases.

Use a bounded tick counter or record missed releases in the ISR; publish
deadline-miss and latency metrics; and define a disarm/fault policy for
sustained overload. Shared-state helpers should restore the previous PRIMASK
rather than unconditionally enabling interrupts.

### P2 — DMA-safe RAM headroom is low

The reviewed Debug map reports RAM_D1 use of 0x43340 of 0x50000 bytes (84%).
The DMA buffer region alone is 0x43340 bytes; the 256,000-byte blackbox queue
uses 0x3f834 of it. Roughly 52 KiB remains.

Add per-region map budgets to CI, make log queue depth a measured configuration,
and retain map history. Do not relocate DMA buffers to DTCMRAM; only move
objects after confirming the relevant bus/DMA accessibility.

### P3 — New sources require an explicit CMake reconfigure

CMakeLists.txt uses file(GLOB_RECURSE ...) for application and common sources
without CONFIGURE_DEPENDS (lines 49-58). A new source can be absent from an
incremental build until CMake is rerun. hip_behavior.c and lqr_lut.c are also
manually appended despite already matching the glob.

Prefer explicit target_sources lists for safety-critical firmware, or use
CONFIGURE_DEPENDS as an interim measure; remove redundant additions; and run a
clean configure/build in CI.

## Recommended implementation order

1. Fix normal reboot and qualify the P0 dump/RPC fixes on target hardware.
2. Redesign metadata persistence and establish CAN watchdog/freshness rules.
3. Separate control heartbeat from transport liveness and add overload metrics.
4. Repair v3 sequence mapping, file pagination, and source discovery.
5. Run deadline load tests, metadata power-cut tests, CAN
   bus-off/watchdog tests, and parameter fuzzing.

## Test gaps

The host suite now covers RPC semantic validation and deferred dump export,
alongside control, kinematics, telemetry, and motor-backend behaviour. It does
not provide target timing qualification for dump-plus-arm ordering or SD/QSPI
load, and it still lacks normal-reboot, metadata power-loss, heartbeat
classification, and CAN-keepalive coverage.
