@echo off
REM Build IGNI.exe with PyInstaller
echo === IGNI - building .exe ===

REM Install dependencies
pip install pyserial matplotlib pyinstaller

REM Single-file exe, no console window
pyinstaller --onefile --windowed ^
  --name IGNI ^
  --collect-all matplotlib ^
  app\peltier_control.py

echo.
echo === DONE ===
echo File: dist\IGNI.exe
pause
