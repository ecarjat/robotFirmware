#!/usr/bin/env python3
"""
Motor init diagnostic using the PySFOC BinaryPacketCommander client.

Sends the same register writes as motor_link_init and logs responses.
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path


def _resolve_pysfoc_path() -> Path:
    return Path(__file__).resolve().parent / "pysfoc"


def _log_line(log_file, message: str) -> None:
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    line = f"[{ts}] {message}"
    print(line)
    log_file.write(line + "\n")
    log_file.flush()


def _write_reg_u8(client, reg: int, value: int, timeout: float):
    payload = bytes([reg & 0xFF, value & 0xFF])
    client._send_packet(client.PACKET_REGISTER, payload) 
    return client._wait_for_register_response(reg, timeout=timeout)


def main() -> int:
    parser = argparse.ArgumentParser(description="Motor init command test")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=460800, help="Baud rate")
    parser.add_argument("--log", default="tools/motor_init_test.log", help="Log file path")
    parser.add_argument("--timeout", type=float, default=0.6, help="Response timeout (s)")
    parser.add_argument("--config-min-elapsed-us", type=int, default=10_000_000,
                        help="Initial telemetry min elapsed time (us)")
    parser.add_argument("--rate-hz", type=float, default=500.0,
                        help="Final telemetry rate (Hz)")
    args = parser.parse_args()

    pysfoc_path = _resolve_pysfoc_path()
    sys.path.insert(0, str(pysfoc_path))

    try:
        from pysfoc.packet_commander import BinaryPacketCommanderClient
        from pysfoc.constants import (
            REG_CONTROL_MODE,
            REG_ENABLE,
            REG_STATUS,
            REG_TELEMETRY_MIN_ELAPSED,
            REG_TELEMETRY_REG,
            REG_VELOCITY,
        )
    except ImportError as exc:
        raise SystemExit(f"Failed to import pysfoc from {pysfoc_path}: {exc}") from exc

    log_path = Path(args.log)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    with log_path.open("w", encoding="utf-8") as log_file:
        _log_line(log_file, f"Opening port {args.port} @ {args.baud} baud")
        client = BinaryPacketCommanderClient(
            port=args.port,
            baud=args.baud,
            timeout=args.timeout,
            debug=True,
            log_packets=True,
            logger=lambda msg: _log_line(log_file, msg),
        )

        # 1) Slow down telemetry during config (min elapsed)
        _log_line(log_file, f"WRITE telemetry_min_elapsed={args.config_min_elapsed_us}")
        resp = client.write_reg(REG_TELEMETRY_MIN_ELAPSED, int(args.config_min_elapsed_us))
        _log_line(log_file, f"RESP telemetry_min_elapsed={resp}")
        time.sleep(0.4)

        # 2) Set telemetry registers: velocity + status
        _log_line(log_file, "WRITE telemetry_reg=[velocity,status]")
        try:
            client.set_telemetry_registers([REG_VELOCITY, REG_STATUS], motor=0)
            _log_line(log_file, "RESP telemetry_reg=OK")
        except Exception as exc:
            _log_line(log_file, f"RESP telemetry_reg=ERROR {exc}")

        # 3) Control mode = torque (0)
        _log_line(log_file, "WRITE control_mode=0")
        resp = client.write_reg(REG_CONTROL_MODE, 0)
        _log_line(log_file, f"RESP control_mode={resp}")

        # 4) Modulation mode (reg 0x07) = 1 (u8)
        _log_line(log_file, "WRITE modulation_mode=1 (reg 0x07)")
        resp = _write_reg_u8(client, 0x07, 1, args.timeout)
        _log_line(log_file, f"RESP modulation_mode={resp}")

        # 5) Enable = 0
        _log_line(log_file, "WRITE enable=0")
        resp = client.write_reg(REG_ENABLE, 0)
        _log_line(log_file, f"RESP enable={resp}")

        # 6) Final telemetry rate using min_elapsed
        if args.rate_hz > 0.0:
            period_us = int(1_000_000 / args.rate_hz)
        else:
            period_us = 0
        if period_us < 1:
            period_us = 1
        _log_line(log_file, f"WRITE telemetry_min_elapsed={period_us} (rate {args.rate_hz} Hz)")
        resp = client.write_reg(REG_TELEMETRY_MIN_ELAPSED, period_us)
        _log_line(log_file, f"RESP telemetry_min_elapsed={resp}")

        _log_line(log_file, "Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
