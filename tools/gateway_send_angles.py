#!/usr/bin/env python3
"""
FOC_eye gateway UART sender — batch 6 motor angles to node 0.

Command RX pins (115200 8N1, either works):
  - PB11 (USART3 RX)
  - PB7  (USART1 RX)

Wire format (8 bytes, binary, no newline):
  [0xC6, seq, ang0, ang1, ang2, ang3, ang4, ang5]
  - angN: signed int8, mechanical angle in degrees (1 deg resolution, -128..127)

Gateway forwards the same 6 angles on CAN ID 0x300 in one classic CAN frame:
  [seq, 0x01, ang0, ang1, ang2, ang3, ang4, ang5]

Default serial: 115200 8N1 (connect to PB11 or PB7 RX).

Requires: pip install pyserial

Examples:
  python3 tools/gateway_send_angles.py --angles -15 -20 -15 -15 -15 -52
  python3 tools/gateway_send_angles.py --port /dev/ttyACM0 --blink
  python3 tools/gateway_send_angles.py --port /dev/ttyUSB0 --angles ...

Interactive console (debug UART PB6/PB7):
  python3 tools/foc_console.py
  python3 tools/foc_console.py --port /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial not installed. Run: pip install pyserial", file=sys.stderr)
    sys.exit(1)

from foc_protocol import (
    BLINK_CLOSED,
    BLINK_OPEN,
    DEFAULT_BAUD,
    DEFAULT_SERIAL_PORT,
    MOTOR_COUNT,
    send_batch,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send 6 motor angle commands to FOC_eye gateway."
    )
    parser.add_argument(
        "--port", "-p",
        default=DEFAULT_SERIAL_PORT,
        help=f"Serial port (default: {DEFAULT_SERIAL_PORT})",
    )
    parser.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD)
    parser.add_argument(
        "--angles", "-a", nargs=MOTOR_COUNT, type=float, metavar="DEG",
        help=f"{MOTOR_COUNT} target angles in degrees (motor 0..5)",
    )
    parser.add_argument("--blink", action="store_true", help="One blink preset")
    parser.add_argument("--seq", type=int, default=0)
    parser.add_argument("--repeat", "-r", type=int, default=1)
    parser.add_argument("--interval", "-i", type=float, default=0.0)
    args = parser.parse_args()

    if args.blink and args.angles is not None:
        parser.error("use either --blink or --angles, not both")
    if not args.blink and args.angles is None:
        parser.error("specify --angles or --blink")

    seq = args.seq & 0xFF

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        if args.blink:
            steps = [
                ("open", BLINK_OPEN, 0.0),
                ("closed", BLINK_CLOSED, 0.035),
                ("open", BLINK_OPEN, 0.35),
            ]
            for name, angles, delay in steps:
                payload = send_batch(ser, seq, [float(x) for x in angles])
                print(f"seq={seq} step={name} bytes={payload.hex()} angles={angles}")
                seq = (seq + 1) & 0xFF
                if delay > 0:
                    time.sleep(delay)
        else:
            angles = [float(x) for x in args.angles]
            for n in range(args.repeat):
                payload = send_batch(ser, seq, angles)
                print(f"seq={seq} sent bytes={payload.hex()} angles={angles}")
                seq = (seq + 1) & 0xFF
                if n + 1 < args.repeat and args.interval > 0:
                    time.sleep(args.interval)

    return 0


if __name__ == "__main__":
    sys.exit(main())
