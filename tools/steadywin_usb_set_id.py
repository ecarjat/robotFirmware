#!/usr/bin/env python3
"""Set a SteadyWin GDS68 CAN Simple node ID over the motor USB-C port."""

from __future__ import annotations

import argparse
import sys


CAN_SIMPLE_NODE_ID_MAX = 63
AXIS_STATE_IDLE = 1
COMM_INTF_MUX_CAN = 0


def parse_node_id(value: str) -> int:
    try:
        node_id = int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"{value!r} is not a decimal or hexadecimal node ID"
        ) from exc

    if not 0 <= node_id <= CAN_SIMPLE_NODE_ID_MAX:
        raise argparse.ArgumentTypeError(
            f"node ID must be in the range 0..{CAN_SIMPLE_NODE_ID_MAX}"
        )
    return node_id


def find_motor(odrive_module, serial_number: str | None, timeout: float):
    kwargs = {"timeout": timeout}
    if serial_number is not None:
        kwargs["serial_number"] = serial_number

    try:
        return odrive_module.find_any(**kwargs)
    except TypeError as exc:
        if serial_number is None:
            raise
        raise RuntimeError(
            "The installed odrive package does not support --serial-number. "
            "Upgrade it with: python3 -m pip install --upgrade odrive"
        ) from exc


def configure_can_interface(odrive_device) -> None:
    """Restore the vendor-recommended CAN settings before saving."""
    odrive_device.config.enable_can_a = True
    odrive_device.axis0.requested_state = AXIS_STATE_IDLE

    # GDS68 hardware version 3.10 and later exposes this communication mux.
    # Earlier hardware does not, but enable_can_a above remains applicable.
    try:
        odrive_device.config.comm_intf_mux = COMM_INTF_MUX_CAN
    except AttributeError:
        print("CAN interface mux is not exposed by this driver; leaving it unchanged.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Set a GDS68 motor CAN Simple node ID through its USB-C data port. "
            "The motor must be powered from its normal supply."
        )
    )
    parser.add_argument(
        "node_id",
        type=parse_node_id,
        help="new CAN node ID (0..63; decimal or 0x-prefixed hexadecimal)",
    )
    parser.add_argument(
        "--serial-number",
        help="USB serial number when more than one ODrive-compatible motor is connected",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="USB discovery timeout in seconds (default: 10)",
    )
    parser.add_argument(
        "--leave-interface-unchanged",
        action="store_true",
        help="only write the ID; do not explicitly restore CAN mode and idle state",
    )
    args = parser.parse_args()

    if args.timeout <= 0.0:
        parser.error("--timeout must be greater than zero")

    try:
        import odrive
    except ImportError:
        print(
            "Missing Python dependency 'odrive'. Install it with:\n"
            "  python3 -m pip install --user odrive",
            file=sys.stderr,
        )
        return 2

    print("Waiting for a powered GDS68 motor on USB-C...")
    try:
        motor = find_motor(odrive, args.serial_number, args.timeout)
    except Exception as exc:
        print(f"Could not find a motor over USB: {exc}", file=sys.stderr)
        return 1

    try:
        current_id = int(motor.axis0.config.can.node_id)
        print(f"Connected. Current CAN node ID: {current_id}")
        motor.axis0.config.can.node_id = args.node_id

        if not args.leave_interface_unchanged:
            configure_can_interface(motor)

        print(f"Saving CAN node ID {args.node_id}; the motor will reboot.")
        motor.save_configuration()
    except Exception as exc:
        print(f"Programming failed: {exc}", file=sys.stderr)
        return 1

    print(
        f"Done. After the reboot, use CAN Simple node ID {args.node_id} "
        f"(base arbitration ID 0x{args.node_id << 5:03X} for command 0)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
