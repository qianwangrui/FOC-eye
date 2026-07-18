@echo off
setlocal
cd /d "%~dp0"

echo === FOC_eye GUI Windows 打包 ===

where python >nul 2>&1
if errorlevel 1 (
    echo 未找到 python，请先安装 Python 3 并加入 PATH
    exit /b 1
)

python -m pip install --upgrade pip pyinstaller pyserial
if errorlevel 1 exit /b 1

if exist build rmdir /s /q build
if exist dist rmdir /s /q dist

python -m PyInstaller --noconfirm --clean motor_eye_gui.spec
if errorlevel 1 exit /b 1

echo.
echo 完成: dist\FOC_eye_gui.exe
echo 可将 dist\FOC_eye_gui.exe 单独拷贝到任意 Windows 电脑使用（无需安装 Python）
endlocal
