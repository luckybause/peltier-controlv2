name: Build EXE

"on":
  push:
    tags:
      - 'v*'
  workflow_dispatch: {}

permissions:
  contents: write

jobs:
  build-windows:
    runs-on: windows-latest

    steps:
      - name: Checkout kodu
        uses: actions/checkout@v4

      - name: Setup Python
        uses: actions/setup-python@v5
        with:
          python-version: '3.11'

      - name: Instalacja zaleznosci
        run: |
          python -m pip install --upgrade pip
          pip install pyserial matplotlib pyinstaller

      - name: Budowa EXE
        run: pyinstaller --onefile --windowed --name PeltierControl --collect-all matplotlib app/peltier_control.py

      - name: Upload artefaktu
        uses: actions/upload-artifact@v4
        with:
          name: PeltierControl-windows
          path: dist/PeltierControl.exe

      - name: Publikacja w Release
        if: startsWith(github.ref, 'refs/tags/')
        uses: softprops/action-gh-release@v2
        with:
          files: dist/PeltierControl.exe
