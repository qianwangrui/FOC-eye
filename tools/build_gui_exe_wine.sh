#!/usr/bin/env bash
# 在 Linux 上通过 Wine + Windows Python 交叉打包 exe（开发机用）
# 原生 Windows 请直接运行 build_gui_exe.bat
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
TMP="${TMPDIR:-/tmp}/foc_eye_winbuild"
WINEPREFIX="$TMP/wineprefix"
PY="$WINEPREFIX/drive_c/Python311/python.exe"
export WINEDEBUG=-all
export WINEPREFIX

mkdir -p "$TMP"
cd "$TMP"
if [[ ! -f python-3.11.9-amd64.exe ]]; then
  echo "下载 Windows Python 3.11.9 ..."
  wget -q -O python-3.11.9-amd64.exe \
    "https://www.python.org/ftp/python/3.11.9/python-3.11.9-amd64.exe"
fi

if [[ ! -f "$PY" ]]; then
  echo "Wine 安装 Python ..."
  wineboot -i 2>/dev/null || true
  wine python-3.11.9-amd64.exe /quiet InstallAllUsers=0 TargetDir=C:\\Python311 \
    Include_pip=1 Include_test=0 Shortcuts=0
fi

echo "安装 PyInstaller / pyserial ..."
wine "$PY" -m pip install --upgrade pip pyinstaller pyserial -q

cd "$ROOT"
rm -rf build dist
echo "打包 FOC_eye_gui.exe ..."
wine "$PY" -m PyInstaller --noconfirm --clean motor_eye_gui.spec

echo ""
echo "完成: $ROOT/dist/FOC_eye_gui.exe"
file "$ROOT/dist/FOC_eye_gui.exe"
