#!/usr/bin/env python3
"""
Interactive terminal for FOC_eye gateway / debug UART.

One USB-serial adapter on USART1 (PB6 TX / PB7 RX), or ST-Link path to PB6/PB7:
  - MCU -> host: debug prints on PB6
  - host -> MCU: commands on PB7

115200 8N1. Requires: pip install pyserial

Usage:
  python3 tools/foc_console.py
  python3 tools/foc_console.py --port /dev/ttyACM0
  python3 tools/foc_console.py --port /dev/ttyUSB0

Commands (type help in console):
  batch -15 -20 -15 -15 -15 -52   binary 6-motor packet
  all   -15 -20 -15 -15 -15 -52   text 6-motor (gateway firmware)
  open / closed / blink             presets
  set -15 / pos 5                   local motor on this node
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
    send_batch,
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
        "Commands:\n"
        "  help\n"
        "  batch a0 a1 a2 a3 a4 a5   send 6-motor binary packet (gateway)\n"
        "  all   a0 a1 a2 a3 a4 a5   send 6-motor text cmd (gateway firmware)\n"
        "  open                      preset: all -15 deg\n"
        "  closed                    preset: blink closed angles\n"
        "  blink                     run firmware blink (local motor only)\n"
        "  set DEG                   set local motor angle\n"
        "  pos DEG                   add to local motor angle\n"
        "  cal align|save|show       rotor offset cal in flash\n"
        "  quit / exit\n"
        f"Motors are indexed 0..{MOTOR_COUNT - 1}. Angles are int8 degrees (-128..127)."
    )


def parse_six_floats(tokens: list[str]) -> list[float]:
    if len(tokens) != MOTOR_COUNT:
        raise ValueError(f"need {MOTOR_COUNT} angles, got {len(tokens)}")
    return [float(x) for x in tokens]


def handle_command(ser: serial.Serial, seq: int, line: str) -> int:
    line = line.strip()
    if not line:
        return seq

    parts = line.split()
    cmd = parts[0].lower()

    if cmd in ("quit", "exit", "q"):
        return -1

    if cmd == "help":
        print_help()
        return seq

    if cmd == "batch":
        try:
            angles = parse_six_floats(parts[1:])
        except (ValueError, IndexError) as exc:
            safe_print(f"[error] {exc}")
            return seq
        payload = send_batch(ser, seq, angles)
        safe_print(f"[tx] batch seq={seq} hex={payload.hex()} angles={angles}")
        return (seq + 1) & 0xFF

    if cmd == "all":
        try:
            angles = parse_six_floats(parts[1:])
        except (ValueError, IndexError) as exc:
            safe_print(f"[error] {exc}")
            return seq
        text = "all " + " ".join(str(int(round(a))) for a in angles)
        send_text_line(ser, text)
        safe_print(f"[tx] text: {text}")
        return (seq + 1) & 0xFF

    if cmd == "open":
        payload = send_batch(ser, seq, [float(x) for x in BLINK_OPEN])
        safe_print(f"[tx] open seq={seq} hex={payload.hex()}")
        return (seq + 1) & 0xFF

    if cmd == "closed":
        payload = send_batch(ser, seq, [float(x) for x in BLINK_CLOSED])
        safe_print(f"[tx] closed seq={seq} hex={payload.hex()}")
        return (seq + 1) & 0xFF

    if cmd in ("blink", "blink1", "set", "pos") or cmd.startswith("set") or cmd.startswith("pos"):
        send_text_line(ser, line)
        safe_print(f"[tx] text: {line}")
        return seq

    if cmd.startswith("cal "):
        send_text_line(ser, line)
        safe_print(f"[tx] text: {line}")
        return seq

    safe_print(f"[error] unknown command: {cmd!r} (type help)")
    return seq


def main() -> int:
    parser = argparse.ArgumentParser(description="FOC_eye interactive debug console")
    parser.add_argument(
        "--port", "-p",
        default=DEFAULT_SERIAL_PORT,
        help=f"Serial port (default: {DEFAULT_SERIAL_PORT})",
    )
    parser.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD)
    args = parser.parse_args()

    stop = threading.Event()
    seq = 0

    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        time.sleep(0.1)
        ser.reset_input_buffer()

        t = threading.Thread(target=reader_loop, args=(ser, stop), daemon=True)
        t.start()

        safe_print(f"FOC_eye console on {args.port} @ {args.baud}. Type 'help'.")
        try:
            while True:
                try:
                    line = input("foc> ")
                except EOFError:
                    break
                seq = handle_command(ser, seq, line)
                if seq < 0:
                    break
        except KeyboardInterrupt:
            safe_print("\n[console] interrupted")
        finally:
            stop.set()
            t.join(timeout=0.5)

    return 0


if __name__ == "__main__":
    sys.exit(main())
