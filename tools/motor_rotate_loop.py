#!/usr/bin/env python3
"""
Every N seconds, add the same delta to all 6 motor angles and send a batch packet.
At int8 limits (-128..127) direction reverses automatically (ping-pong).

Requires gateway firmware on node 0. Uses binary UART batch (0xC6).

Examples:
  python3 tools/motor_rotate_loop.py
  python3 tools/motor_rotate_loop.py --port /dev/ttyUSB0 --step 10 --interval 0.5
  python3 tools/motor_rotate_loop.py --start -15 -15 -15 -15 -15 -15 --step -10 --count 20
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
    DEFAULT_BAUD,
    DEFAULT_SERIAL_PORT,
    MOTOR_COUNT,
    clamp_angle_deg,
    send_batch,
)


def parse_start_angles(values: list[str] | None) -> list[float]:
    if values is None:
        return [0.0] * MOTOR_COUNT
    if len(values) != MOTOR_COUNT:
        raise argparse.ArgumentTypeError(
            f"--start requires {MOTOR_COUNT} values, got {len(values)}"
        )
    return [float(v) for v in values]


def next_angles(angles: list[float], direction: float, step_mag: float) -> tuple[list[float], float]:
    """Advance all motors; flip direction at int8 limits."""
    updated = [a + direction * step_mag for a in angles]
    if direction > 0 and max(updated) >= 127.0:
        updated = [127.0] * MOTOR_COUNT
        return updated, -1.0
    if direction < 0 and min(updated) <= -128.0:
        updated = [-128.0] * MOTOR_COUNT
        return updated, 1.0
    return updated, direction


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Periodically increment all motor angles and send to gateway."
    )
    parser.add_argument(
        "--port", "-p",
        default=DEFAULT_SERIAL_PORT,
        help=f"Serial port (default: {DEFAULT_SERIAL_PORT})",
    )
    parser.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD)
    parser.add_argument(
        "--interval", "-i", type=float, default=0.5,
        help="Seconds between updates (default: 0.5)",
    )
    parser.add_argument(
        "--step", "-s", type=float, default=10.0,
        help="Degrees added to every motor each step (default: +10)",
    )
    parser.add_argument(
        "--start", nargs=MOTOR_COUNT, type=float, metavar="DEG",
        help=f"Initial angles for motors 0..{MOTOR_COUNT - 1} (default: all 0)",
    )
    parser.add_argument(
        "--count", "-n", type=int, default=0,
        help="Stop after N steps (0 = run until Ctrl+C)",
    )
    args = parser.parse_args()

    if args.interval <= 0:
        parser.error("--interval must be > 0")

    angles = parse_start_angles(args.start)
    step_mag = abs(args.step)
    direction = 1.0 if args.step >= 0 else -1.0
    seq = 0
    step_num = 0

    print(
        f"motor_rotate_loop: port={args.port} interval={args.interval}s "
        f"step={step_mag:.0f}deg ping-pong start={angles}",
        flush=True,
    )
    print("Ctrl+C to stop.", flush=True)

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        time.sleep(0.1)
        ser.reset_input_buffer()

        try:
            while True:
                payload = send_batch(ser, seq, angles)
                sent = [clamp_angle_deg(a) for a in angles]
                print(
                    f"step={step_num} seq={seq} dir={'+' if direction > 0 else '-'} "
                    f"angles={sent} hex={payload.hex()}",
                    flush=True,
                )
                seq = (seq + 1) & 0xFF
                step_num += 1

                angles, direction = next_angles(angles, direction, step_mag)

                if args.count > 0 and step_num >= args.count:
                    break

                time.sleep(args.interval)
        except KeyboardInterrupt:
            print("\nstopped.", flush=True)

    return 0


if __name__ == "__main__":
    sys.exit(main())
