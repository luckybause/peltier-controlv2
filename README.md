# PeltierControl

Sterownik temperatury PID dla modułu Peltiera z aplikacją desktopową w stylu brutalist.
Pełna kontrola z PC: setpoint, rampa, strojenie PID, kalibracja termopary i profile wieloetapowe.

![wersja](https://img.shields.io/badge/wersja-6.0-d4452e)
![python](https://img.shields.io/badge/python-3.8%2B-4d9fff)
![platforma](https://img.shields.io/badge/MCU-ItsyBitsy%20M0-5fc77f)

## Architektura

```
Aplikacja PC (Python/tkinter)  <--USB/Serial 115200-->  ItsyBitsy M0
  - panel sterowania                                      - regulator PID
  - wykres na zywo                                        - sterowanie PWM (Cytron MDD3A)
  - zapis CSV                                             - odczyt temperatury (MAX31856)
  - profile wieloetapowe                                  - pamiec Flash (profile)
```

Komunikacja jest dwukierunkowa: aplikacja wysyła komendy tekstowe, urządzenie odsyła telemetrię CSV oraz aktualne nastawy (auto-synchronizacja suwaków).

## Funkcje

- **Sterowanie z aplikacji** - zero fizycznych potencjometrów i przycisków
- **Suwak + pole liczbowe** dla każdego parametru (wpisz wartość lub przeciągnij)
- **AUTO grzanie/chłodzenie** - PID sam dobiera kierunek na podstawie setpointu
- **Wykres na żywo** - temperatura, setpoint, moc PWM
- **Self-Tune** - szybkie strojenie PID dla bieżącego punktu pracy
- **Auto-kalibracja** - pełne strojenie 36 profili (9 temperatur x 4 rampy)
- **Kalibracja termopary** - offset z podglądem surowy/skalibrowany
- **Profile wieloetapowe** - sekwencja etapów temperatura/rampa/czas
- **Zapis CSV** - każdy cykl archiwizowany automatycznie
- **Pamięć Flash** - profile i nastawy zapamiętane w urządzeniu

## Hardware

| Element | Model |
|---------|-------|
| Mikrokontroler | Adafruit ItsyBitsy M0 (ATSAMD21G18) |
| Sterownik silnika | Cytron MDD3A |
| Czujnik temperatury | MAX31856 + termopara K |
| Wyświetlacz (opcjonalny) | OLED SH1106 1.3" I2C |
| Element wykonawczy | Moduł Peltiera |

### Połączenia

```
MAX31856  CS   -> pin 9
Cytron    M1A  -> pin 11 (PWM)
Cytron    M1B  -> pin 10 (PWM)
OLED      SDA/SCL -> I2C (opcjonalnie)
```

## Instalacja

### Opcja 1: Gotowy plik .exe (Windows)

Pobierz najnowszy `PeltierControl.exe` z [Releases](../../releases) i uruchom. Nie wymaga instalacji Pythona.

### Opcja 2: Uruchomienie ze źródeł

```bash
git clone https://github.com/luckybause/peltier-control.git
cd peltier-control
pip install -r requirements.txt
python app/peltier_control.py
```

### Firmware

1. Otwórz `firmware/peltier_pid_kontroler.ino` w Arduino IDE
2. Zainstaluj biblioteki: `Adafruit MAX31856`, `Adafruit BusIO`, `U8g2`, `FlashStorage_SAMD`
3. Wybierz płytkę: Adafruit ItsyBitsy M0
4. Wgraj na urządzenie

## Użycie

1. Podłącz ItsyBitsy przez USB
2. Uruchom aplikację, przejdź do zakładki **POŁĄCZENIE**
3. Wybierz port COM i kliknij **POŁĄCZ** - suwaki zsynchronizują się automatycznie
4. Ustaw temperaturę docelową i parametry w panelu **STEROWANIE**
5. Kliknij **START** - regulacja rusza, wykres pokazuje przebieg na żywo

## Protokół komend (Serial 115200)

| Komenda | Opis |
|---------|------|
| `SP:25.5` | Setpoint [°C] |
| `RU:2.0` / `RD:1.5` | Rampa grzania / chłodzenia [°C/min] |
| `TMAX:80` | Maksymalna temperatura (zabezpieczenie) |
| `KP:10` / `KI:0.3` / `KD:0.8` | Parametry PID |
| `OFFSET:-5.0` | Offset kalibracji termopary |
| `START` / `STOP` / `ESTOP` | Sterowanie pracą |
| `SELFTUNE` | Auto-strojenie bieżącego punktu |
| `AUTOCAL` | Pełna kalibracja 36 profili |
| `SAVE` / `LOAD` / `RESET` | Pamięć Flash |
| `GET` | Odeślij aktualne nastawy |

Telemetria zwracana jako CSV: `czas_s,temp_C,setpoint_akt,setpoint_cel,PWM,Kp,Ki,Kd,stan`

## Struktura projektu

```
peltier-control/
├── app/
│   └── peltier_control.py      # aplikacja desktopowa
├── firmware/
│   └── peltier_pid_kontroler.ino  # firmware ItsyBitsy M0
├── .github/workflows/
│   └── build.yml               # automatyczny build .exe
├── requirements.txt
├── build.bat                   # lokalny build .exe
└── README.md
```

## Licencja

MIT
