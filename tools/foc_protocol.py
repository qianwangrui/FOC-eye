"""Shared UART/CAN angle batch protocol for FOC_eye gateway tools."""

from __future__ import annotations

import struct

UART_BATCH_MAGIC = 0xC6
MOTOR_COUNT = 6
DEFAULT_BAUD = 115200
DEFAULT_SERIAL_PORT = "/dev/ttyACM0"  # NUCLEO ST-Link VCP; use --port for others

BLINK_OPEN = [-15, -15, -15, -15, -15, -15]
BLINK_CLOSED = [-15, -20, -15, -15, -15, -52]


def clamp_angle_deg(deg: float) -> int:
    if deg > 127:
        return 127
    if deg < -128:
        return -128
    return int(round(deg))


def pack_uart_batch(seq: int, angles_deg: list[float]) -> bytes:
    if len(angles_deg) != MOTOR_COUNT:
        raise ValueError(f"need exactly {MOTOR_COUNT} angles, got {len(angles_deg)}")
    ints = [clamp_angle_deg(a) for a in angles_deg]
    return struct.pack("<BB6b", UART_BATCH_MAGIC, seq & 0xFF, *ints)


def send_batch(ser, seq: int, angles_deg: list[float]) -> bytes:
    payload = pack_uart_batch(seq, angles_deg)
    ser.write(payload)
    ser.flush()
    return payload


def send_text_line(ser, line: str) -> None:
    ser.write((line.rstrip("\r\n") + "\n").encode("ascii"))
    ser.flush()
