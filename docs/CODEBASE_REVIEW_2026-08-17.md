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

| Priority | Finding | Consequence |
|---|---|---|
| P0 | Blackbox dumping can run while balancing | Storage operations can block the foreground control loop. |
| P0 | Raw RPC parameters are not semantically validated | A valid frame can apply NaN, infinity, or unsafe limits/gains. |
| P1 | Redundant blackbox metadata shares one erase sector | Power loss can invalidate both copies and lose the ring pointer. |
| P1 | Normal reboot command is a no-op | Reboot payload 0 does not enter the normal reboot path. |
| P1 | Any valid frame refreshes the control heartbeat | Non-control traffic can keep an armed robot alive. |
| P1 | CAN torque sends are change-driven, not watchdog-driven | Constant torque may not be refreshed after the first transmission. |
| P2 | Telemetry v3 is absent from channel mapping | v3 uses sequence counter 0 rather than telemetry. |
| P2 | File-list pagination cannot retrieve a second page | The response reports more entries without a usable cursor. |
| P2 | Deadline accounting collapses missed timer releases | Overload is underreported and has no defined fault policy. |
| P2 | RAM_D1 is 84% occupied | Only about 52 KiB of DMA-safe RAM headroom remains. |
| P3 | CMake source globbing can miss newly added files | An incremental build can succeed without compiling a new source. |

## Findings

### P0 — Blackbox dump work can block control while armed

app/app_cmd.c accepts a teleop dump edge at lines 37-45 without checking the
motion mode. The same teleop packet can then arm at lines 56-60. A dump
continues unconditionally from app_idle_tick in app/app_main.c at lines
300-303.

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

Recommended actions:

1. Permit dumps only while disarmed and manual motor mode is off; reject arming
   while a dump is active.
2. Abort or pause a dump before arming, and do not execute SD/QSPI dump work in
   the balancing path.
3. Bound every background operation by the remaining deadline, or run storage
   work in a task that cannot delay control.
4. Add target timing tests for dump-then-arm ordering and an SD card with
   deliberately slow writes.

### P0 — RPC can install unsafe live parameters

Status: fixed. `SET_PARAM` now accepts only declared parameter fields, validates
the full candidate (finite values, physical limits, calibration matrices, and
cross-field constraints), and rejects parameter writes while balancing or
manual motor mode is active. `control_timer_set_rate_hz()` also falls back for
non-finite input.

app/app_rpc.c:243-263 accepts a byte offset and length, copies arbitrary bytes
into a candidate robot_params_t, and applies it after only bounds checks. No
schema-level validation occurs before motion_control_apply_params or
control_timer_set_rate_hz.

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

### P1 — Metadata is not power-loss redundant

The two claimed metadata slots are both in one 4 KiB erase sector:
LOG_META_SLOT0 is 0x000000-0x0007ff and LOG_META_SLOT1 is
0x000800-0x000fff (app/logging/blackbox_format.h:17-20). On alternating
metadata saves, log_meta_tick erases LOG_META_START before writing the selected
slot (app/logging/blackbox.c:606-614). That erase removes the old slot too.

A reset during erase/program can leave no valid metadata. The next boot calls
log_format_meta when metadata cannot be loaded (app/logging/blackbox.c:697-709)
and resets the write pointer, so later logs may overwrite existing ring data.

Recommended actions:

- Put each copy in a separate 4 KiB sector, or use an append-only journal with
  a final commit marker.
- Verify a new record before erasing the obsolete sector.
- Make sequence comparison wrap-safe and inject power loss at every erase and
  program step in tests.
- Validate record CRC in log_validate_ring_tail, not just magic/version.

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

1. Block dumps and all long storage work while armed; fix normal reboot; add
   parameter validation.
2. Redesign metadata persistence and establish CAN watchdog/freshness rules.
3. Separate control heartbeat from transport liveness and add overload metrics.
4. Repair v3 sequence mapping, file pagination, and source discovery.
5. Qualify on hardware: deadline load tests, metadata power-cut tests, CAN
   bus-off/watchdog tests, and parameter fuzzing.

## Test gaps

The passing host suite covers several control, kinematics, RPC, telemetry, and
motor-backend behaviours. It does not cover the critical paths above:
dump-plus-arm ordering, normal reboot, non-finite RPC writes, metadata
power-loss recovery, heartbeat classification, CAN keepalive behaviour, or
hardware deadline behaviour under SD/QSPI load.
