#!/usr/bin/env python3
"""
Interactive terminal for FOC_eye — all control via text commands (115200 8N1).

Wire USB-serial to PB6 (MCU TX / printf) + PB7 or PB11 (MCU RX):
  python3 tools/foc_console.py
  python3 tools/foc_console.py --port /dev/ttyUSB0
  python tools/foc_console.py --port COM3

Six-motor batches use text 'all' (same as VOFA), not binary 0xC6.
"""

from __future__ import annotations

import argparse
import sys
import threading
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
    send_all_angles,
    send_text_line,
)

PRINT_LOCK = threading.Lock()


def safe_print(*args, **kwargs) -> None:
    with PRINT_LOCK:
        print(*args, **kwargs)
        sys.stdout.flush()


def reader_loop(ser: serial.Serial, stop: threading.Event) -> None:
    while not stop.is_set():
        try:
            line = ser.readline()
        except serial.SerialException as exc:
            safe_print(f"[serial read error] {exc}")
            break
        if not line:
            continue
        text = line.decode(errors="replace").rstrip("\r\n")
        if text:
            safe_print(f"[mcu] {text}")


def print_help() -> None:
    safe_print(
        "Commands (all text, CAN motor index 0..5):\n"
        "  help\n"
        "  all   a0 a1 a2 a3 a4 a5     6-motor angles, e.g. all 0 -15 -15 -15 0 -15\n"
        "  batch a0 .. a5              alias of 'all'\n"
        "  open                        preset: all -15 x6\n"
        "  closed                      preset: all -15 -20 -15 -15 -15 -52\n"
        "  blink                       local blink on this node\n"
        "  set DEG                     set local motor angle\n"
        "  pos DEG                     add to local motor angle\n"
        "  enc                         read encoder (local node)\n"
        "  cal align|save|show         rotor offset cal\n"
        "  quit / exit\n"
        f"Angles: int8 degrees (-128..127). Motor order: CAN0..CAN5."
    )


def parse_six_floats(tokens: list[str]) -> list[float]:
    if len(tokens) != MOTOR_COUNT:
        raise ValueError(f"need {MOTOR_COUNT} angles, got {len(tokens)}")
    return [float(x) for x in tokens]


def send_all_or_error(ser: serial.Serial, angles: list[float]) -> None:
    line = send_all_angles(ser, angles)
    safe_print(f"[tx] {line}")


def handle_command(ser: serial.Serial, line: str) -> bool:
    """Return False to quit."""
    line = line.strip()
    if not line:
        return True

    parts = line.split()
    cmd = parts[0].lower()

    if cmd in ("quit", "exit", "q"):
        return False

    if cmd == "help":
        print_help()
        return True

    if cmd in ("all", "batch"):
        try:
            angles = parse_six_floats(parts[1:])
        except (ValueError, IndexError) as exc:
            safe_print(f"[error] {exc}")
            return True
        send_all_or_error(ser, angles)
        return True

    if cmd == "open":
        send_all_or_error(ser, [float(x) for x in BLINK_OPEN])
        return True

    if cmd == "closed":
        send_all_or_error(ser, [float(x) for x in BLINK_CLOSED])
        return True

    if cmd in ("blink", "blink1", "set", "pos", "enc"):
        send_text_line(ser, line)
        safe_print(f"[tx] {line}")
        return True

    if parts[0] == "cal":
        send_text_line(ser, line)
        safe_print(f"[tx] {line}")
        return True

    safe_print(f"[error] unknown command: {cmd!r} (type help)")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="FOC_eye interactive console (text protocol only)"
    )
    parser.add_argument(
        "--port", "-p",
        default=DEFAULT_SERIAL_PORT,
        help=f"Serial port (default: {DEFAULT_SERIAL_PORT})",
    )
    parser.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD)
    args = parser.parse_args()

    stop = threading.Event()

    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        time.sleep(0.1)
        ser.reset_input_buffer()

        t = threading.Thread(target=reader_loop, args=(ser, stop), daemon=True)
        t.start()

        safe_print(
            f"FOC_eye text console on {args.port} @ {args.baud}. Type 'help'."
        )
        try:
            while True:
                try:
                    line = input("foc> ")
                except EOFError:
                    break
                if not handle_command(ser, line):
                    break
        except KeyboardInterrupt:
            safe_print("\n[console] interrupted")
        finally:
            stop.set()
            t.join(timeout=0.5)

    return 0


if __name__ == "__main__":
    sys.exit(main())
