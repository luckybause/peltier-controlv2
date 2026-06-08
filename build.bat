@echo off
REM Build PeltierControl.exe przez PyInstaller
echo === PeltierControl - budowanie .exe ===

REM Instalacja zaleznosci
pip install pyserial matplotlib pyinstaller

REM Budowa jednego pliku exe bez konsoli
pyinstaller --onefile --windowed ^
  --name PeltierControl ^
  --collect-all matplotlib ^
  app\peltier_control.py

echo.
echo === GOTOWE ===
echo Plik: dist\PeltierControl.exe
pause
