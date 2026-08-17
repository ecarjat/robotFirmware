# SteadyWin GDS68 USB-C CAN ID Provisioning

This tool sets the CAN Simple node ID of one GIM8108-8/GDS68 motor directly
through its USB-C port. It is intended for bench provisioning before the motor
is connected to the robot CAN bus.

## Requirements

- The motor must be powered from its normal main supply. USB-C does not power
  the GDS68 driver or the motor.
- Use a USB-C data cable. Charge-only cables do not expose the driver.
- Connect only one unprovisioned motor at a time. New GDS68 motors default to
  CAN node ID `0`, so multiple defaults on the same CAN bus would conflict.
- Install the ODrive host package once:

  ```bash
  python3 -m pip install --user odrive
  ```

## Set the node ID

From the firmware directory:

```bash
python3 tools/steadywin_usb_set_id.py 1
```

Node IDs are `0..63`; decimal and `0x`-prefixed hexadecimal input are both
accepted. The tool reads the current ID, writes the new ID, restores CAN mode,
then saves the configuration. Saving reboots the motor.

For multiple USB-connected motors, select the target by its USB serial number:

```bash
python3 tools/steadywin_usb_set_id.py 2 --serial-number 123456789
```

Normally the tool enables CAN A, puts axis 0 in Idle, and selects the CAN
communication interface where the driver exposes that setting. Use
`--leave-interface-unchanged` only when deliberately retaining an existing USB
interface configuration.

## Configure Wheel Encoder Telemetry

Wheel velocity is supplied to the STM32 by periodic CAN Simple
`Get_Encoder_Estimates` messages. After assigning the ID to each wheel motor,
use `odrivetool` over USB-C to configure a 10 ms period and save it:

```python
odrv0.axis0.config.can.encoder_rate_ms = 10
odrv0.save_configuration()
```

The STM32 firmware considers wheel encoder telemetry stale after 30 ms. Do not
set a slower reporting period. The motor will reboot after saving; then verify
its traffic on the 500 kbit/s CAN bus before enabling motor output.

## Verify on CAN

After the motor has rebooted and USB is disconnected, connect it to the CAN
bus and use the assigned node ID. CAN Simple arbitration IDs are formed as:

```text
(node_id << 5) | command_id
```

For example, node ID `1` uses `0x020` for command `0`; node ID `2` uses
`0x040`.

The same ID can also be set through the STM32 provisioning RPC with:

```text
MOTOR SET-ID <current-id> <new-id> SAVE
```
