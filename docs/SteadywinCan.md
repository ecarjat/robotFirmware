# SteadyWin GIM8108-8 / GDS68 CAN Integration

The robot uses GIM8108-8 motors with GDS68 drivers and the CAN Simple
protocol. This supersedes the older DD8015 custom-CAN integration note.

Source: `docs/SteadyWin GIM6010-8 Motor Manual_rev2.2.pdf`, CAN Simple
section. The GDS68 protocol is ODrive-compatible.

## Node allocation

| Motor | Node ID | CAN Simple base ID |
|---|---:|---:|
| Left wheel | `1` | `0x020` |
| Right wheel | `2` | `0x040` |
| Left hip | `3` | `0x060` |
| Right hip | `4` | `0x080` |

All CAN traffic uses standard 11-bit frames and little-endian payloads:

```text
CAN_ID = (node_id << 5) | command_id
```

For example, the left-wheel torque command is `(1 << 5) | 0x00E = 0x02E`.

## Wheel backend behavior

`app/drivers/motors/motor_backend_steadywin_can_impl.inc` keeps the existing
`motor_link` API but emits CAN Simple frames:

- `motor_link_enable(true)` clears errors (`0x018`), selects torque/direct
  control (`0x00B`, values `1/1`), sends zero torque (`0x00E`), then requests
  closed-loop state (`0x007`, state `8`) for each wheel.
- `motor_link_set_wheel_torques()` sends `Set_Input_Torque` (`0x00E`) with a
  float32 torque in Nm. The supplied torque limit is enforced before sending.
- `motor_link_enable(false)` sends zero torque, requests Idle (`0x007`, state
  `1`), and clears the local enabled state.
- Encoder velocity arrives through periodic `Get_Encoder_Estimates` messages
  (`0x009`): position and velocity are float32 values in rev and rev/s. The
  backend converts velocity to rad/s for the control loop.

`motor_link_set_wheel_Iq()` remains available for callers that command q-axis
current, but it needs `MOTOR_LINK_KT_NM_PER_A` or `MOTOR_LINK_KV_RPM_PER_V` to
convert current to the CAN Simple torque unit (Nm). The normal control path
uses `motor_link_set_wheel_torques()` and does not depend on that conversion.

## Required drive provisioning

Program the four node IDs with `tools/steadywin_usb_set_id.py` before placing
the motors on the same bus. See `docs/SteadyWinUsbProvisioning.md`.

The driver transmits encoder estimates periodically; configure and save a rate
of 10 ms or faster for each wheel over USB:

```python
odrv0.axis0.config.can.encoder_rate_ms = 10
odrv0.save_configuration()
```

The firmware considers wheel telemetry stale after 30 ms. Do not use a rate
slower than that. The default CAN bitrate is 500 kbit/s; ensure the STM32 FDCAN
configuration uses the same bitrate.

## Bench bring-up

1. Power and connect one drive over USB-C.
2. Assign its node ID and set `encoder_rate_ms`, saving after each change.
3. Disconnect USB, connect CANH/CANL, and ensure the bus has exactly two 120
   ohm terminators.
4. Power the STM32 with all motor outputs disabled. Confirm periodic `0x029`
   and `0x049` encoder frames from the wheel nodes.
5. Enable motors only with the wheels clear of the ground. Confirm each wheel
   receives the zero-torque command before entering closed-loop operation.
