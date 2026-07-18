#!/usr/bin/env python3
"""
左眼缓慢眨眼 — 文本 all 协议，只动 CAN1(左下) 与 CAN2(左上)。

角度约定（与机械一致）:
  CAN2 左上眼皮: -90° 睁眼 … -45° 闭眼
  CAN1 左下眼皮:  25° 睁眼 …   0° 闭眼

其它 CAN 轴保持固定（默认与 GUI「全开」一致）。

用法:
  python3 tools/left_eye_blink.py --port /dev/ttyUSB0
  python3 tools/left_eye_blink.py --port COM3 --count 3
  python3 tools/left_eye_blink.py --port /dev/ttyUSB0 --speed 2
  python3 tools/left_eye_blink.py --speed 0.5 --pause-open 3
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("请先安装: pip install pyserial", file=sys.stderr)
    sys.exit(1)

from foc_protocol import DEFAULT_BAUD, DEFAULT_SERIAL_PORT, MOTOR_COUNT, send_all_angles

# CAN index
CAN_LR = 0
CAN_LEFT_LOWER = 1
CAN_LEFT_UPPER = 2
CAN_RIGHT_LOWER = 3
CAN_EYE_UD = 4
CAN_RIGHT_UPPER = 5

# 左眼眼皮行程
UPPER_OPEN = -90.0
UPPER_CLOSED = -45.0
LOWER_OPEN = 25.0
LOWER_CLOSED = 0.0

# 默认眨眼动作时长（可被 --speed 缩放）
BASE_CLOSE_S = 0.35
BASE_OPEN_S = 0.45
BASE_PAUSE_CLOSED_S = 0.08

# 其它轴默认保持（CAN0..5，与 motor_eye_gui 全开预设一致）
DEFAULT_HOLD = [0.0, LOWER_OPEN, UPPER_OPEN, -15.0, 0.0, -15.0]


def blink_durations(
    speed: float, close_s: float, open_s: float, pause_closed: float
) -> tuple[float, float, float]:
    """speed 越大，闭眼/睁眼动作越快（时长按 1/speed 缩短）。"""
    scale = 1.0 / max(speed, 0.01)
    return close_s * scale, open_s * scale, pause_closed * scale


def list_serial_ports() -> list[str]:
    found = sorted({p.device for p in serial.tools.list_ports.comports()})
    preferred: list[str] = []
    for pat in ("ttyUSB", "ttyACM", "cu.usb", "cu.wchusb", "COM"):
        preferred.extend([d for d in found if pat in d])
    rest = [d for d in found if d not in preferred]
    return preferred + rest


def resolve_port(hint: str | None) -> str:
    if hint:
        return hint
    ports = list_serial_ports()
    for name in ports:
        if "ttyUSB0" in name or name.endswith("ttyUSB0"):
            return name
    if ports:
        return ports[0]
    return DEFAULT_SERIAL_PORT


def lerp(a: float, b: float, t: float) -> float:
    if t <= 0.0:
        return a
    if t >= 1.0:
        return b
    return a + (b - a) * t


def build_angles(
    hold: list[float],
    upper: float,
    lower: float,
) -> list[float]:
    angles = list(hold)
    angles[CAN_LEFT_LOWER] = lower
    angles[CAN_LEFT_UPPER] = upper
    return angles


def ramp(
    ser: serial.Serial,
    hold: list[float],
    upper_from: float,
    upper_to: float,
    lower_from: float,
    lower_to: float,
    duration_s: float,
    hz: float,
) -> None:
    if duration_s <= 0.0:
        send_all_angles(ser, build_angles(hold, upper_to, lower_to))
        return
    steps = max(1, int(round(duration_s * hz)))
    dt = duration_s / steps
    for i in range(1, steps + 1):
        t = i / steps
        upper = lerp(upper_from, upper_to, t)
        lower = lerp(lower_from, lower_to, t)
        send_all_angles(ser, build_angles(hold, upper, lower))
        time.sleep(dt)


def one_blink(
    ser: serial.Serial,
    hold: list[float],
    close_s: float,
    hold_closed_s: float,
    open_s: float,
    hold_open_s: float,
    hz: float,
) -> None:
    ramp(
        ser, hold,
        UPPER_OPEN, UPPER_CLOSED,
        LOWER_OPEN, LOWER_CLOSED,
        close_s, hz,
    )
    if hold_closed_s > 0.0:
        time.sleep(hold_closed_s)

    ramp(
        ser, hold,
        UPPER_CLOSED, UPPER_OPEN,
        LOWER_CLOSED, LOWER_OPEN,
        open_s, hz,
    )
    if hold_open_s > 0.0:
        time.sleep(hold_open_s)


def parse_hold(values: list[str] | None) -> list[float]:
    if values is None:
        return list(DEFAULT_HOLD)
    if len(values) != MOTOR_COUNT:
        raise argparse.ArgumentTypeError(
            f"--hold 需要 {MOTOR_COUNT} 个角度 (CAN0..CAN5)，收到 {len(values)} 个"
        )
    return [float(v) for v in values]


def main() -> int:
    parser = argparse.ArgumentParser(description="左眼缓慢眨眼 (text all)")
    parser.add_argument("--port", "-p", default=None, help="串口 (默认自动选 ttyUSB0 等)")
    parser.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD)
    parser.add_argument(
        "--count", "-n", type=int, default=0,
        help="眨眼次数，0=无限循环 (默认 0)",
    )
    parser.add_argument(
        "--speed", "-s", type=float, default=1.0,
        help="眨眼速度倍率：1=默认慢眨，2=快一倍，0.5=慢一倍 (默认 1)",
    )
    parser.add_argument(
        "--close-s", type=float, default=BASE_CLOSE_S,
        help=f"闭眼基准耗时秒，再按 speed 缩放 (默认 {BASE_CLOSE_S})",
    )
    parser.add_argument(
        "--open-s", type=float, default=BASE_OPEN_S,
        help=f"睁眼基准耗时秒，再按 speed 缩放 (默认 {BASE_OPEN_S})",
    )
    parser.add_argument(
        "--pause-closed", type=float, default=BASE_PAUSE_CLOSED_S,
        help=f"闭眼保持秒，再按 speed 缩放 (默认 {BASE_PAUSE_CLOSED_S})",
    )
    parser.add_argument(
        "--pause-open", type=float, default=2.5,
        help="睁眼保持秒，两次眨眼间隔 (默认 2.5)",
    )
    parser.add_argument(
        "--hz", type=float, default=25.0,
        help="插值发送帧率 (默认 25)",
    )
    parser.add_argument(
        "--hold",
        nargs=MOTOR_COUNT,
        type=float,
        metavar=("CAN0", "CAN1", "CAN2", "CAN3", "CAN4", "CAN5"),
        help="非左眼轴保持角度，默认全开预设",
    )
    args = parser.parse_args()

    close_s, open_s, pause_closed = blink_durations(
        args.speed, args.close_s, args.open_s, args.pause_closed
    )

    port = resolve_port(args.port)
    hold = parse_hold(args.hold)
    # 眨眼从睁眼姿态开始
    hold[CAN_LEFT_LOWER] = LOWER_OPEN
    hold[CAN_LEFT_UPPER] = UPPER_OPEN

    print(
        f"左眼眨眼: 左上 {UPPER_OPEN}°↔{UPPER_CLOSED}°, "
        f"左下 {LOWER_OPEN}°↔{LOWER_CLOSED}°"
    )
    print(
        f"串口 {port} @ {args.baud}, speed={args.speed}x, "
        f"close={close_s:.2f}s open={open_s:.2f}s pause_open={args.pause_open}s"
    )

    ser: serial.Serial | None = None
    try:
        ser = serial.Serial(port, args.baud, timeout=0.05)
        time.sleep(0.1)
        ser.reset_input_buffer()
        send_all_angles(ser, hold)
        time.sleep(0.2)

        n = 0
        while args.count == 0 or n < args.count:
            one_blink(
                ser,
                hold,
                close_s,
                pause_closed,
                open_s,
                args.pause_open,
                args.hz,
            )
            n += 1
            if args.count > 0:
                print(f"blink {n}/{args.count}")
    except serial.SerialException as exc:
        print(f"串口错误: {exc}", file=sys.stderr)
        if "No such file" in str(exc) or "could not open port" in str(exc):
            ports = list_serial_ports()
            if ports:
                print(f"可用串口: {', '.join(ports)}", file=sys.stderr)
            print("请指定: --port /dev/ttyUSB0", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\n已停止")
        if ser and ser.is_open:
            try:
                send_all_angles(ser, build_angles(hold, UPPER_OPEN, LOWER_OPEN))
            except serial.SerialException:
                pass
    finally:
        if ser and ser.is_open:
            ser.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
