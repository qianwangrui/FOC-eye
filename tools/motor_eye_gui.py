#!/usr/bin/env python3
"""
FOC_eye 六电机可视化调角 — 滑块 -127..+127，经 PB11 (USART3) 发 0xC6 批量包。

依赖: pip install pyserial

用法:
  python3 tools/motor_eye_gui.py
  python3 tools/motor_eye_gui.py --port /dev/ttyUSB0   # Linux
  python tools/motor_eye_gui.py --port COM3            # Windows

打包 Windows exe (在 Windows 上双击或命令行):
  tools\\build_gui_exe.bat
  输出: tools\\dist\\FOC_eye_gui.exe

或在 GitHub Actions「Build FOC_eye GUI」workflow 下载产物。
"""

from __future__ import annotations

import argparse
import sys
import tkinter as tk
from pathlib import Path
from tkinter import ttk, messagebox

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("请先安装: pip install pyserial", file=sys.stderr)
    sys.exit(1)

# 复用仓库内协议
sys.path.insert(0, str(Path(__file__).resolve().parent))
from foc_protocol import DEFAULT_BAUD, MOTOR_COUNT, pack_all_text, pack_uart_batch  # noqa: E402

# 界面滑块顺序（右眼上下眼皮相邻）；括号内为 CAN 帧 motor 下标
UI_LABELS = [
    "1 眼睛左右  [CAN0]",
    "2 左眼下眼皮 [CAN1]",
    "3 左眼上眼皮 [CAN2]",
    "4 右眼下眼皮 [CAN3]",
    "5 右眼上眼皮 [CAN5]",
    "6 眼睛上下  [CAN4]",
]

# 界面第 i 路 → CAN 帧 motor 下标 (payload[2+id])
# CAN3=右眼下眼皮, CAN4=眼睛上下, CAN5=右眼上眼皮
UI_TO_CAN = [0, 1, 2, 3, 5, 4]

# 预设按 CAN motor 0..5 顺序
CAN_PRESETS = {
    "全开": [0, -15, -15, -15, 0, -15],
    "全闭": [0, -52, -30, -52, 0, -30],
}


def can_to_ui(can_angles: list[int]) -> list[int]:
    return [can_angles[UI_TO_CAN[i]] for i in range(MOTOR_COUNT)]


def ui_to_can(ui_angles: list[int]) -> list[int]:
    can = [0] * MOTOR_COUNT
    for ui_i, can_i in enumerate(UI_TO_CAN):
        can[can_i] = ui_angles[ui_i]
    return can


def list_serial_ports() -> list[str]:
    found = sorted({p.device for p in serial.tools.list_ports.comports()})
    # Linux 常见设备优先
    preferred = []
    for pat in ("ttyUSB", "ttyACM", "cu.usb", "cu.wchusb"):
        preferred.extend([d for d in found if pat in d])
    rest = [d for d in found if d not in preferred]
    return preferred + rest


def pick_default_port(ports: list[str], hint: str | None) -> str:
    if hint and hint in ports:
        return hint
    for name in ports:
        if "ttyUSB0" in name or name.endswith("ttyUSB0"):
            return name
    return ports[0] if ports else ""


class MotorEyeGui:
    def __init__(self, root: tk.Tk, default_port: str | None) -> None:
        self.root = root
        self.root.title("FOC_eye 六电机调角")
        self.root.minsize(520, 520)

        self._ser: serial.Serial | None = None
        self._seq = 0
        self._debounce_id: str | None = None
        self._default_port_hint = default_port

        self.live_var = tk.BooleanVar(value=True)
        self.solo_var = tk.BooleanVar(value=False)
        self.binary_var = tk.BooleanVar(value=False)  # default: text all (same as VOFA)
        self._last_ui_idx = 0
        self.angle_vars = [
            tk.IntVar(value=can_to_ui(CAN_PRESETS["全开"])[i]) for i in range(MOTOR_COUNT)
        ]
        self._scales: list[ttk.Scale] = []

        self._build_ui()
        self.refresh_ports()

    def _build_ui(self) -> None:
        pad = {"padx": 8, "pady": 4}

        top = ttk.Frame(self.root)
        top.pack(fill=tk.X, **pad)

        ttk.Label(top, text="串口:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            top, textvariable=self.port_var, width=22, state="readonly"
        )
        self.port_combo.pack(side=tk.LEFT, padx=(4, 4))
        ttk.Button(top, text="刷新", command=self.refresh_ports).pack(side=tk.LEFT)
        ttk.Button(top, text="连接", command=self.toggle_connect).pack(side=tk.LEFT, padx=(8, 0))

        ttk.Label(top, text=f"  {DEFAULT_BAUD} 8N1").pack(side=tk.LEFT, padx=(8, 0))

        opts = ttk.Frame(self.root)
        opts.pack(fill=tk.X, **pad)
        ttk.Checkbutton(
            opts, text="拖动滑块时实时发送", variable=self.live_var
        ).pack(side=tk.LEFT)
        ttk.Checkbutton(
            opts, text="单路测试(其余 CAN 发 0)", variable=self.solo_var
        ).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Checkbutton(
            opts, text="二进制 0xC6 (需新固件)", variable=self.binary_var
        ).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(opts, text="立即发送", command=self.send_now).pack(side=tk.LEFT, padx=(12, 0))
        ttk.Button(opts, text="全部归零", command=self.reset_zero).pack(side=tk.LEFT, padx=(8, 0))

        preset_row = ttk.Frame(self.root)
        preset_row.pack(fill=tk.X, **pad)
        ttk.Label(preset_row, text="预设:").pack(side=tk.LEFT)
        for name in CAN_PRESETS:
            ttk.Button(
                preset_row, text=name, command=lambda n=name: self.apply_preset(n)
            ).pack(side=tk.LEFT, padx=(4, 0))

        body = ttk.LabelFrame(self.root, text="电机角度 (-127 .. +127)")
        body.pack(fill=tk.BOTH, expand=True, **pad)

        for i in range(MOTOR_COUNT):
            row = ttk.Frame(body)
            row.pack(fill=tk.X, padx=8, pady=6)

            ttk.Label(row, text=UI_LABELS[i], width=22).pack(side=tk.LEFT)
            val_lbl = ttk.Label(row, textvariable=self.angle_vars[i], width=5)
            val_lbl.pack(side=tk.RIGHT)

            scale = ttk.Scale(
                row,
                from_=-127,
                to=127,
                orient=tk.HORIZONTAL,
                command=lambda v, idx=i: self._on_scale(idx, float(v)),
            )
            scale.set(self.angle_vars[i].get())
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 8))
            self._scales.append(scale)

        self.status_var = tk.StringVar(value="未连接")
        ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN).pack(
            fill=tk.X, padx=8, pady=(0, 8)
        )

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def refresh_ports(self) -> None:
        ports = list_serial_ports()
        self.port_combo["values"] = ports
        cur = self.port_var.get()
        if cur not in ports:
            self.port_var.set(pick_default_port(ports, self._default_port_hint))
        if not ports:
            self.status_var.set("未发现串口设备")

    def toggle_connect(self) -> None:
        if self._ser and self._ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("串口", "请先选择串口")
            return
        try:
            self._ser = serial.Serial(port, DEFAULT_BAUD, timeout=0.05)
            self._ser.reset_input_buffer()
        except serial.SerialException as exc:
            messagebox.showerror("连接失败", str(exc))
            self._ser = None
            return
        self.status_var.set(f"已连接 {port} @ {DEFAULT_BAUD}")

    def disconnect(self) -> None:
        if self._ser:
            try:
                self._ser.close()
            except serial.SerialException:
                pass
            self._ser = None
        self.status_var.set("未连接")

    def _on_scale(self, idx: int, value: float) -> None:
        v = int(round(value))
        v = max(-127, min(127, v))
        self.angle_vars[idx].set(v)
        self._last_ui_idx = idx
        if self.live_var.get():
            self._schedule_send()

    def _schedule_send(self) -> None:
        if self._debounce_id is not None:
            self.root.after_cancel(self._debounce_id)
        self._debounce_id = self.root.after(60, self._debounce_send)

    def _debounce_send(self) -> None:
        self._debounce_id = None
        self.send_now()

    def get_angles(self) -> list[int]:
        return [self.angle_vars[i].get() for i in range(MOTOR_COUNT)]

    def _build_can_angles(self, ui_angles: list[int]) -> list[int]:
        if self.solo_var.get():
            can = [0] * MOTOR_COUNT
            can_i = UI_TO_CAN[self._last_ui_idx]
            can[can_i] = ui_angles[self._last_ui_idx]
            return can
        return ui_to_can(ui_angles)

    def send_now(self) -> None:
        if not self._ser or not self._ser.is_open:
            self.status_var.set("发送失败: 未连接串口")
            return
        ui_angles = self.get_angles()
        can_angles = self._build_can_angles(ui_angles)
        try:
            if self.binary_var.get():
                payload = pack_uart_batch(self._seq, [float(a) for a in can_angles])
                proto = f"0xC6 seq={self._seq}"
                self._seq = (self._seq + 1) & 0xFF
            else:
                payload = pack_all_text(can_angles)
                proto = "all text"
            self._ser.write(payload)
            self._ser.flush()
        except serial.SerialException as exc:
            self.status_var.set(f"发送失败: {exc}")
            return
        can_i = UI_TO_CAN[self._last_ui_idx]
        self.status_var.set(
            f"已发送 {proto}  "
            f"ui[{self._last_ui_idx}]={ui_angles[self._last_ui_idx]}→CAN{can_i}  "
            f"can={can_angles}"
        )

    def reset_zero(self) -> None:
        for i in range(MOTOR_COUNT):
            self.angle_vars[i].set(0)
        self._sync_scales_from_vars()
        self.send_now()

    def apply_preset(self, name: str) -> None:
        vals = can_to_ui(CAN_PRESETS[name])
        for i, v in enumerate(vals):
            self.angle_vars[i].set(v)
        self._sync_scales_from_vars()
        self.send_now()

    def _sync_scales_from_vars(self) -> None:
        for i, scale in enumerate(self._scales):
            scale.set(float(self.angle_vars[i].get()))

    def on_close(self) -> None:
        self.disconnect()
        self.root.destroy()


def main() -> int:
    parser = argparse.ArgumentParser(description="FOC_eye 六电机滑块 GUI")
    parser.add_argument(
        "--port", "-p", default=None, help="默认串口 (如 /dev/ttyUSB0)"
    )
    args = parser.parse_args()

    root = tk.Tk()
    style = ttk.Style()
    if "clam" in style.theme_names():
        style.theme_use("clam")

    app = MotorEyeGui(root, args.port)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
