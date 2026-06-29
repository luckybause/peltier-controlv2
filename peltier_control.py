#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PeltierControl v6.0 - BRUTALIST
Pelny panel sterowania PID Peltiera z dwukierunkowa komunikacja.
Sterowanie z aplikacji: setpoint, rampa, PID, kalibracja, profile.
Wymaga firmware v19 (PC MODE) na ItsyBitsy M0.
"""

import sys, os, time, csv, json, threading, queue
from datetime import datetime
from pathlib import Path

try:
    import serial, serial.tools.list_ports
except ImportError:
    print("pip install pyserial"); input(); sys.exit(1)
try:
    import tkinter as tk
    from tkinter import ttk, messagebox
except ImportError:
    print("brak tkinter"); input(); sys.exit(1)
try:
    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
except ImportError as e:
    print(f"pip install matplotlib numpy\n{e}"); input(); sys.exit(1)

# ════════════════════════════════════════════════════════
#  MOTYW BRUTALIST - beton, stal, surowe krawedzie
# ════════════════════════════════════════════════════════
C = {
    'bg':       '#3a3d42',   # beton glowny
    'bg2':      '#2b2d31',   # stal ciemna (paski, pola)
    'panel':    '#33363b',   # karty
    'panel2':   '#2b2d31',   # elementy wewnetrzne
    'panel3':   '#42454a',   # hover
    'border':   '#4a4d52',   # ramki
    'border2':  '#5a5d63',   # ramki jasniejsze
    'text':     '#f0f0f0',   # tekst glowny
    'dim':      '#b0b3b8',   # tekst przygaszony
    'dim2':     '#6a6d72',   # tekst bardzo przygaszony
    'blue':     '#4d9fff',   # temperatura
    'orange':   '#e8a33d',   # setpoint
    'yellow':   '#e8c63d',   # tempo / rampa grzania
    'green':    '#5fc77f',   # pwm / ok / start
    'red':      '#d4452e',   # stop / grzanie / alarm (sowiecka czerwien)
    'cyan':     '#4db8d4',   # pid / chlodzenie
    'purple':   '#a87dd4',   # kalibracja / profile
    'rec':      '#d4452e',   # nagrywanie
    'grid':     '#42454a',   # siatka wykresu
}

# Fonty - monospace dla brutalist
FONT      = 'Consolas'
FONT_UI   = 'Roboto Mono'   # fallback do Consolas jesli brak

# Globalny mnoznik rozmiaru fontow (ustawiany na starcie wg DPI)
FS = 1.0
def fsz(n):
    """Skaluje rozmiar fontu wg globalnego DPI."""
    return max(6, int(round(n * FS)))

def _font(size, weight='normal'):
    """Zwraca tuple fontu z fallbackiem"""
    return (FONT, size, weight) if weight != 'normal' else (FONT, size)

def _lighten(hex_color, amount=0.15):
    """Rozjasnia kolor hex o zadana ilosc"""
    hex_color = hex_color.lstrip('#')
    r = int(hex_color[0:2], 16)
    g = int(hex_color[2:4], 16)
    b = int(hex_color[4:6], 16)
    r = min(255, int(r + (255 - r) * amount))
    g = min(255, int(g + (255 - g) * amount))
    b = min(255, int(b + (255 - b) * amount))
    return f'#{r:02x}{g:02x}{b:02x}'

def mk_btn(parent, text, cmd, bg=None, fg='#1a1c1f', **kw):
    """Brutalist button - ostre krawedzie, monospace"""
    bg = bg or C['green']
    b = tk.Button(parent, text=text, command=cmd, bg=bg, fg=fg,
                  font=(FONT, fsz(10), 'bold'), padx=16, pady=8,
                  relief='flat', cursor='hand2', bd=0,
                  activebackground=_lighten(bg, 0.15), activeforeground=fg, **kw)
    def on_enter(e):
        if b['state'] != 'disabled': b.config(bg=_lighten(bg, 0.15))
    def on_leave(e):
        if b['state'] != 'disabled': b.config(bg=bg)
    b.bind('<Enter>', on_enter)
    b.bind('<Leave>', on_leave)
    return b

def mk_btn_outline(parent, text, cmd, color, **kw):
    """Button z obramowaniem (outline) zamiast wypelnienia"""
    b = tk.Button(parent, text=text, command=cmd, bg=C['bg2'], fg=color,
                  font=(FONT, fsz(10), 'bold'), padx=14, pady=7,
                  relief='flat', cursor='hand2', bd=0,
                  highlightthickness=2, highlightbackground=color,
                  highlightcolor=color,
                  activebackground=C['panel3'], activeforeground=color, **kw)
    return b


# ════════════════════════════════════════════════════════
#  WIDGET: Suwak + pole liczbowe (kluczowy element panelu)
# ════════════════════════════════════════════════════════
class SliderField:
    """Suwak + pole liczbowe obok. Wpisanie wartosci lub przeciagniecie suwaka.
       on_change(value) wywolywany przy zmianie (debounced)."""
    def __init__(self, parent, label, vmin, vmax, vinit, color,
                 unit='', decimals=1, on_change=None, width=170):
        self.vmin = vmin; self.vmax = vmax
        self.color = color; self.decimals = decimals
        self.on_change = on_change
        self._last_sent = None
        self._after_id = None

        # Kontener
        self.frame = tk.Frame(parent, bg=C['bg2'])
        self.frame.pack(fill='x', pady=(0, 14))

        # Etykieta + jednostka
        top = tk.Frame(self.frame, bg=C['bg2'])
        top.pack(fill='x')
        tk.Label(top, text=label, bg=C['bg2'], fg=C['dim'],
                 font=(FONT, fsz(9)), anchor='w').pack(side='left')
        if unit:
            tk.Label(top, text=unit, bg=C['bg2'], fg=C['dim2'],
                     font=(FONT, fsz(8)), anchor='e').pack(side='right')

        # Wiersz: suwak + pole
        row = tk.Frame(self.frame, bg=C['bg2'])
        row.pack(fill='x', pady=(4, 0))

        # Pole liczbowe (Entry) - po prawej
        self.entry = tk.Entry(row, width=7, bg=C['panel'], fg=color,
                              font=(FONT, fsz(12), 'bold'), justify='center',
                              relief='flat', bd=0,
                              highlightthickness=1.5, highlightbackground=color,
                              highlightcolor=_lighten(color, 0.2),
                              insertbackground=color)
        self.entry.pack(side='right', ipady=4, padx=(8, 0))
        self.entry.bind('<Return>', self._on_entry)
        self.entry.bind('<FocusOut>', self._on_entry)

        # Suwak (Scale) - wypelnia reszte
        self.var = tk.DoubleVar(value=vinit)
        self.scale = tk.Scale(row, from_=vmin, to=vmax, resolution=10**(-decimals),
                             orient='horizontal', variable=self.var,
                             showvalue=False, bg=C['bg2'], fg=color,
                             troughcolor=C['panel'], highlightthickness=0,
                             bd=0, sliderrelief='flat', sliderlength=18,
                             activebackground=color, length=width,
                             command=self._on_slide)
        self.scale.pack(side='right', fill='x', expand=True)

        self._set_entry(vinit)

    def _set_entry(self, v):
        self.entry.delete(0, 'end')
        self.entry.insert(0, f"{v:.{self.decimals}f}")

    def _on_slide(self, val):
        v = float(val)
        self._set_entry(v)
        self._debounced(v)

    def _on_entry(self, evt=None):
        try:
            v = float(self.entry.get().replace(',', '.'))
            v = max(self.vmin, min(self.vmax, v))
            self.var.set(v)
            self._set_entry(v)
            self._debounced(v)
        except ValueError:
            self._set_entry(self.var.get())

    def _debounced(self, v):
        """Wysylaj zmiane z opoznieniem 150ms zeby nie zalac serialu"""
        if self._after_id:
            self.frame.after_cancel(self._after_id)
        self._after_id = self.frame.after(150, lambda: self._emit(v))

    def _emit(self, v):
        if self.on_change and v != self._last_sent:
            self._last_sent = v
            self.on_change(v)

    def get(self):
        return self.var.get()

    def set(self, v, silent=True):
        """Ustaw wartosc. silent=True nie wywoluje on_change (sync z urzadzenia)."""
        v = max(self.vmin, min(self.vmax, v))
        if silent:
            self._last_sent = v
        self.var.set(v)
        self._set_entry(v)

    def set_enabled(self, en):
        st = 'normal' if en else 'disabled'
        self.scale.config(state=st)
        self.entry.config(state=st)


# ════════════════════════════════════════════════════════
#  APLIKACJA GLOWNA
# ════════════════════════════════════════════════════════
class PeltierControl:
    def __init__(self, root):
        self.root = root
        self.root.title("PeltierControl v6.0 - BRUTALIST")
        self.root.configure(bg=C['bg'])
        self.root.geometry("1280x800")
        self.root.minsize(1100, 720)

        # Serial
        self.ser = None
        self.port_name = None
        self.baud = 115200
        self.running = False
        self.connected = False

        # Dane pomiarowe (bufory)
        self.maxlen = 3000
        self.t = []; self.temp = []; self.spt = []; self.spa = []
        self.pwm = []; self.kp = []; self.ki = []; self.kd = []; self.states = []
        self.t0 = None
        self.data_queue = queue.Queue()
        self.last_state = 'MAN'
        self.cur_state = 'MAN'

        # Sledzenie dotarcia do setpointu (statystyki)
        self.reach_start_t = None    # czas startu dojscia (s)
        self.reach_start_temp = None # temp na starcie
        self.reach_target = None     # docelowa temp
        self.reach_done = False      # czy osiagnieto
        self.reach_time = None       # ile trwalo dotarcie [s]
        self.reach_avg_rate = None   # srednia rampa [C/min]
        self.last_setpoint_target = None

        # Polaryzacja i zakres kalibracji (z urzadzenia)
        self.dev_pol_swapped = False
        self.dev_pol_set = False
        self.dev_cal_min = 50.0
        self.dev_cal_max = 100.0

        # Sterowanie wykresem live
        self.chart_paused = False      # pauza przewijania (do zoomu)
        self.chart_window = 0          # 0 = caly przebieg, >0 = ostatnie N sekund

        # CSV cyklu
        self.log_dir = Path.home() / "PeltierLogi"
        self.log_dir.mkdir(exist_ok=True)
        self.cyc_on = False; self.cyc_file = None; self.cyc_wr = None
        self.cyc_t0 = None; self.cyc_fn = None

        # Profile (lista etapow: dict temp/ramp/time)
        self.profile_steps = []

        # Status synchronizacji z urzadzeniem
        self.dev_cal = False       # czy urzadzenie ma kalibracje
        self.last_cfg_time = 0

        # Stan kalibracji
        self.cal_plan = []         # lista (temp, ramp) wszystkich krokow
        self.cal_total = 0         # liczba krokow
        self.cal_current = 0       # aktualny krok (1-based)
        self.cal_cur_temp = None
        self.cal_cur_ramp = None
        self.cal_phase = None      # faza biezacego kroku: 'heating'/'stabil'/'relay'
        self.cal_running = False
        self.cal_t0 = None         # czas startu kalibracji
        self.cal_step_times = []   # czasy ukonczenia krokow (do ETA)
        self.cal_win = None        # okno postepu kalibracji

        # Zapis kalibracji na dysku PC
        self.cal_file = self.log_dir / "kalibracja.json"
        self.presets_file = self.log_dir / "presety.json"
        self._caldump_buf = []     # bufor odbieranych profili
        self._caldump_active = False
        self._caldump_purpose = None  # 'save' lub None
        self._pending_offset = None   # offset do zapisania z dumpem

        # Pulsowanie statusu
        self._pulse_state = 0

        self._build_styles()
        self._build_ui()
        self._pulse()
        self.tick()
        # Auto-polaczenie: sprobuj polaczyc z urzadzeniem po starcie
        self.root.after(800, self._auto_connect)

    def _auto_connect(self):
        """Automatyczne polaczenie - wykryj i polacz z ItsyBitsy"""
        if self.connected:
            return
        try:
            ports = list(serial.tools.list_ports.comports())
        except Exception:
            return
        if not ports:
            return
        # Priorytet: porty z opisem pasujacym do ItsyBitsy/Adafruit/USB
        def score(p):
            d = (p.description or '').lower()
            m = (p.manufacturer or '').lower() if hasattr(p, 'manufacturer') else ''
            s = 0
            for kw in ['itsybitsy', 'adafruit', 'usb serial', 'usb-serial', 'circuitpython']:
                if kw in d or kw in m: s += 10
            # ItsyBitsy M0 VID = 0x239A (Adafruit)
            if hasattr(p, 'vid') and p.vid == 0x239A: s += 20
            return s
        best = max(ports, key=score)
        # Polacz tylko jesli cos sensownego (jakikolwiek port jesli jeden)
        if score(best) > 0 or len(ports) == 1:
            self.connect(best.device)

    def _build_styles(self):
        st = ttk.Style()
        try: st.theme_use('clam')
        except: pass
        st.configure('TNotebook', background=C['bg2'], borderwidth=0, tabmargins=[0,0,0,0])
        st.configure('TNotebook.Tab', background=C['bg2'], foreground=C['dim'],
                     padding=[20, 10], font=(FONT, fsz(10), 'bold'), borderwidth=0)
        st.map('TNotebook.Tab',
               background=[('selected', C['bg'])],
               foreground=[('selected', C['text'])])

    # ────────────────────────────────────────────────────
    #  KOMUNIKACJA SERIAL
    # ────────────────────────────────────────────────────
    def send(self, cmd):
        """Wyslij komende do urzadzenia"""
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + '\n').encode())
            except Exception as e:
                print(f"send err: {e}")

    def connect(self, port):
        try:
            self.ser = serial.Serial(port, self.baud, timeout=0.5)
            self.port_name = port
            self.clear_buf()
            self._cfg_synced = False  # pozwol na jednorazowa synchronizacje suwakow
            self.set_status(True, f"{port} - 115200")
            self.running = True
            threading.Thread(target=self.reader, daemon=True).start()
            # Pobierz konfiguracje startowa
            self.root.after(1500, lambda: self.send("GET"))
            # Auto-wczytaj zapisana kalibracje z PC (jesli istnieje)
            self.root.after(2200, self._auto_load_calibration)
        except Exception as e:
            messagebox.showerror("Error", f"{port}:\n{e}")
            self.set_status(False, "")

    def _auto_load_calibration(self):
        """Przy polaczeniu - automatycznie wgraj zapisana kalibracje"""
        if not self.connected:
            return
        if self.cal_file.exists():
            ok = self.load_calibration_from_pc()
            if ok:
                print("Auto-wgrano kalibracje z PC przy polaczeniu")

    def disconnect(self):
        self.running = False
        if self.cyc_on: self.cyc_stop("Rozlaczono")
        if self.ser:
            try: self.ser.close()
            except: pass
            self.ser = None
        self.set_status(False, "")

    def clear_buf(self):
        for a in [self.t, self.temp, self.spt, self.spa,
                  self.pwm, self.kp, self.ki, self.kd, self.states]:
            a.clear()
        self.t0 = None

    def reader(self):
        """Watek czytajacy serial - parsuje CSV i CFG"""
        if self.ser and self.ser.is_open:
            self.ser.reset_input_buffer()
        while self.running:
            try:
                if not self.ser or not self.ser.is_open: break
                raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not raw: continue

                # Linia konfiguracji CFG:SP=...,RU=...
                if raw.startswith("CFG:"):
                    self._parse_cfg(raw[4:])
                    continue

                # Plan kalibracji CALPLAN:24,temps=50/60/70,ramps=2/5/10/20
                if raw.startswith("CALPLAN:"):
                    self._parse_calplan(raw[8:])
                    continue

                # Dump kalibracji - poczatek
                if raw.startswith("CALDUMP:"):
                    self._caldump_buf = []
                    self._caldump_active = True
                    continue
                # Pojedynczy profil PROF:idx,KpH,...
                if raw.startswith("PROF:") and self._caldump_active:
                    self._caldump_buf.append(raw[5:])
                    continue
                # Koniec dumpu
                if raw == "CALDUMPEND":
                    self._caldump_active = False
                    self.root.after(0, self._finish_caldump_save)
                    continue

                # Status kalibracji CALSTAT:5/24,T=40,R=2
                if raw.startswith("CALSTAT:"):
                    self._parse_calstat(raw[8:])
                    continue

                # Linia danych CSV (9 pol + opcjonalne temp2 jako 10.)
                p = raw.split(',')
                if len(p) < 9: continue
                try: float(p[0])
                except ValueError: continue
                try:
                    d = dict(temp=float(p[1]), sa=float(p[2]), st=float(p[3]),
                             pwm=int(p[4]), kp=float(p[5]), ki=float(p[6]),
                             kd=float(p[7]), state=p[8].strip())
                except: continue
                # temp2 - druga termopara (10. pole, jesli obecne)
                d['temp2'] = None
                if len(p) >= 10:
                    try:
                        v2 = float(p[9])
                        d['temp2'] = v2 if v2 != 0 else None  # 0 = brak/blad
                    except: pass
                self._latest_temp2 = d['temp2']  # do wyswietlenia na karcie

                # Czas z FIRMWARE (p[0] = czas_s) - dokladny, niezalezny od
                # opoznien aplikacji/buforowania kolejki. Zegar komputera (time.time)
                # rozjezdzal sie przy buforowaniu i zanizal AVG RATE.
                try:
                    fw_time = float(p[0])
                except:
                    fw_time = 0
                if self.t0 is None:
                    self.t0 = fw_time  # pierwszy czas firmware = punkt zero
                now = fw_time - self.t0
                state = d['state']

                if self.cyc_on and state in ('AUTO', 'COOLDOWN', 'FREEZE', 'FREEZE_READY'):
                    self.cyc_log(time.time() - self.cyc_t0 if self.cyc_t0 else 0,
                                d['temp'], d['sa'], d['st'],
                                d['pwm'], d['kp'], d['ki'], d['kd'], state,
                                d.get('temp2'))

                prev = self.last_state
                self.last_state = state
                self.cur_state = state
                # SELF-TUNE: gdy stan to ST-..., self-tune zmienia PID na zywo.
                # Przepisz nowe Kp/Ki/Kd na suwaki, zeby tabela sie aktualizowala.
                if state.startswith('ST') or state.startswith('CAL'):
                    self._st_pid_update = (d['kp'], d['ki'], d['kd'])
                # Wykryj koniec kalibracji (CAL/CAL-N -> MAN)
                if self.cal_running and 'CAL' in prev and state == 'MAN':
                    self.cal_running = False
                    self.cal_current = self.cal_total  # ukoncz pasek
                    self.root.after(0, self._cal_finished)
                self.data_queue.put((now, d['temp'], d['st'], d['sa'],
                                    d['pwm']*100/255, d['kp'], d['ki'],
                                    d['kd'], state, prev))

            except serial.SerialException:
                self.running = False
                self.root.after(0, lambda: self.set_status(False, "Utracono polaczenie"))
                break
            except Exception as e:
                if self.running: print(f"reader err: {e}")
                time.sleep(0.3)

    def _parse_cfg(self, cfg):
        """Parsuje CFG:SP=25.5,RU=2.0,... i synchronizuje suwaki"""
        d = {}
        for part in cfg.split(','):
            if '=' in part:
                k, v = part.split('=', 1)
                d[k.strip()] = v.strip()
        # Synchronizuj suwaki (silent - bez wysylania z powrotem)
        self.root.after(0, lambda: self._apply_cfg(d))

    def _apply_cfg(self, d):
        try:
            # Suwaki synchronizuj TYLKO przy pierwszym CFG po polaczeniu.
            # Potem nastawy uzytkownika maja zostawac (nie nadpisuj po STOP itp.)
            if not getattr(self, '_cfg_synced', False):
                if 'SP' in d and hasattr(self, 'sl_sp'):    self.sl_sp.set(float(d['SP']))
                if 'RU' in d and hasattr(self, 'sl_ru'):    self.sl_ru.set(float(d['RU']))
                if 'RD' in d and hasattr(self, 'sl_rd'):    self.sl_rd.set(float(d['RD']))
                if 'TMAX' in d and hasattr(self, 'sl_tmax'): self.sl_tmax.set(float(d['TMAX']))
                if 'KP' in d and hasattr(self, 'sl_kp'):    self.sl_kp.set(float(d['KP']))
                if 'KI' in d and hasattr(self, 'sl_ki'):    self.sl_ki.set(float(d['KI']))
                if 'KD' in d and hasattr(self, 'sl_kd'):    self.sl_kd.set(float(d['KD']))
                if 'OFFSET' in d and hasattr(self, 'sl_off'): self.sl_off.set(float(d['OFFSET']))
                if 'KFFH' in d and hasattr(self, 'sl_kffh'): self.sl_kffh.set(float(d['KFFH']))
                if 'KFFR' in d and hasattr(self, 'sl_kffr'): self.sl_kffr.set(float(d['KFFR']))
                self._cfg_synced = True
            if 'CAL' in d:
                self.dev_cal = (d['CAL'] == '1')
            if 'STATE' in d:
                self.cur_state = d['STATE']
            # Polaryzacja
            if 'POL' in d:
                self.dev_pol_swapped = (d['POL'] == '1')
            if 'POLSET' in d:
                self.dev_pol_set = (d['POLSET'] == '1')
            # Zakres kalibracji
            if 'CALMIN' in d:
                self.dev_cal_min = float(d['CALMIN'])
            if 'CALMAX' in d:
                self.dev_cal_max = float(d['CALMAX'])
            # Stan wentylatorow
            if 'FAN' in d:
                fan_val = int(float(d['FAN']))
                self.fan_on = (fan_val > 0)
                if hasattr(self, 'sl_fan') and fan_val > 0:
                    self.sl_fan.set(fan_val, silent=True)
                if hasattr(self, 'btn_fan'):
                    if fan_val > 0:
                        self.btn_fan.config(text="● ON", fg=C['green'],
                                           highlightbackground=C['green'])
                    else:
                        self.btn_fan.config(text="○ OFF", fg=C['dim2'],
                                           highlightbackground=C['dim'])
            # Zaktualizuj wskaznik polaryzacji w UI jesli istnieje
            if hasattr(self, '_update_pol_indicator'):
                self._update_pol_indicator()
        except Exception as e:
            print(f"apply_cfg err: {e}")

    def _parse_calplan(self, txt):
        """CALPLAN:9,temps=20/30/.../90,ramps=relay - buduj liste krokow.
        Relay: jeden test na temperature (ramps=relay), nie siatka temp×rampa."""
        try:
            d = {}
            parts = txt.split(',')
            total = int(parts[0])
            temps, ramps, relay_mode = [], [], False
            for part in parts[1:]:
                if part.startswith('temps='):
                    temps = [float(x) for x in part[6:].split('/') if x]
                elif part.startswith('ramps='):
                    rv = part[6:]
                    if rv.strip() == 'relay':
                        relay_mode = True
                    else:
                        ramps = [float(x) for x in rv.split('/') if x]
            # Buduj plan
            plan = []
            if relay_mode:
                # Relay: jeden krok na temperature
                for t in temps:
                    plan.append((t, 'relay'))
            else:
                for t in temps:
                    for r in ramps:
                        plan.append((t, r))
            self.cal_plan = plan
            self.cal_total = total or len(plan)
            self.cal_current = 0
            self.cal_phase = None
            self.cal_running = True
            self.cal_t0 = time.time()
            self.cal_step_times = []
            self.root.after(0, self._refresh_cal_view)
        except Exception as e:
            print(f"calplan err: {e}")

    def _parse_calstat(self, txt):
        """CALSTAT:5/24,T=40,R=2 - aktualizuj postep"""
        try:
            d = {}
            parts = txt.split(',')
            # parts[0] = "5/24"
            cur, tot = parts[0].split('/')
            new_current = int(cur)
            self.cal_total = int(tot)
            for part in parts[1:]:
                if part.startswith('T='):
                    self.cal_cur_temp = float(part[2:])
                elif part.startswith('R='):
                    rv = part[2:].strip()
                    # Relay: R= to FAZA kroku (heating/stabil/relay), nie rampa.
                    if rv in ('heating', 'stabil', 'relay'):
                        self.cal_phase = rv
                        self.cal_cur_ramp = 'relay'
                    else:
                        self.cal_phase = None
                        try: self.cal_cur_ramp = float(rv)
                        except: self.cal_cur_ramp = rv
            # Jesli zmienil sie krok - zapisz czas (do ETA)
            if new_current != self.cal_current:
                if self.cal_t0:
                    self.cal_step_times.append(time.time())
                self.cal_current = new_current
            self.cal_running = True
            self.root.after(0, self._refresh_cal_view)
        except Exception as e:
            print(f"calstat err: {e}")

    def _cal_eta(self):
        """Szacowany pozostaly czas kalibracji [s]"""
        if not self.cal_t0 or self.cal_current < 1 or self.cal_total < 1:
            return None
        elapsed = time.time() - self.cal_t0
        if self.cal_current == 0:
            return None
        per_step = elapsed / self.cal_current
        remaining = (self.cal_total - self.cal_current) * per_step
        return max(0, remaining)

    def _cal_finished(self):
        """Kalibracja zakonczona"""
        self._refresh_cal_view()
        if hasattr(self, 'cal_status'):
            self.cal_status.config(text="✓ Calibration done - saving to PC...")
        self.dev_cal = True
        # Pobierz zaktualizowane nastawy
        self.send("GET")
        # Automatycznie pobierz profile i zapisz na dysk PC
        self.root.after(800, lambda: self.dump_calibration_to_pc(silent=False))

    # ────────────────────────────────────────────────────
    #  KALIBRACJA - ZAPIS/ODCZYT NA DYSKU PC
    # ────────────────────────────────────────────────────
    def _manual_load_cal(self):
        """Reczne wgranie kalibracji z PC (z potwierdzeniem)"""
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        if not self.cal_file.exists():
            messagebox.showinfo("No calibration",
                "No saved calibration found on PC.\n"
                "Run calibration first, or save it with\n"
                "the 'SAVE CAL TO PC' button.")
            return
        # Pokaz date zapisu
        try:
            with open(self.cal_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            saved = data.get('saved', '?')
            nvalid = sum(1 for p in data.get('profiles', []) if p.get('valid'))
        except:
            saved = '?'; nvalid = 0
        if messagebox.askyesno("Load calibration from PC",
                f"Load saved calibration to the device?\n\n"
                f"Saved: {saved}\n"
                f"Profiles: {nvalid}\n\n"
                "This will overwrite the current calibration."):
            self.load_calibration_from_pc()

    def show_cal_table(self):
        """Pobierz profile z urzadzenia i pokaz tabele Kp/Ki/Kd"""
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        self._caldump_buf = []
        self._caldump_active = False
        self._caldump_purpose = 'view'
        self.send("DUMPCAL")
        print("Pobieranie tabeli kalibracji...")

    def _show_cal_table_window(self, profiles):
        """Okno z tabela skalibrowanych PID (temp x rampa)"""
        # Siatka jak w firmware (PR_N=8)
        PT = [20, 30, 40, 50, 60, 70, 80, 90, 100]
        PR = [2, 5, 10, 20, 30, 40, 60, 80]
        win = tk.Toplevel(self.root)
        win.title("Calibration Table")
        win.configure(bg=C['bg'])
        win.geometry("720x520")
        tk.Label(win, text="CALIBRATION TABLE — heating PID (Kp / Ki / Kd)",
                 bg=C['bg'], fg=C['purple'], font=(FONT, fsz(12), 'bold')).pack(
                 anchor='w', padx=16, pady=(14, 4))
        n_valid = sum(1 for p in profiles if p['valid'])
        tk.Label(win, text=f"{n_valid} of {len(profiles)} grid points calibrated.  "
                 "Empty = not calibrated (uses defaults 10/0.3/0.8).",
                 bg=C['bg'], fg=C['dim'], font=(FONT, fsz(9))).pack(anchor='w', padx=16)
        # Mapa idx -> profil
        pmap = {p['idx']: p for p in profiles}
        # Tabela przewijalna
        frame = tk.Frame(win, bg=C['bg'])
        frame.pack(fill='both', expand=True, padx=16, pady=12)
        canvas = tk.Canvas(frame, bg=C['bg2'], highlightthickness=0)
        sb = tk.Scrollbar(frame, orient='vertical', command=canvas.yview)
        inner = tk.Frame(canvas, bg=C['bg2'])
        canvas.create_window((0, 0), window=inner, anchor='nw')
        inner.bind('<Configure>', lambda e: canvas.config(scrollregion=canvas.bbox('all')))
        canvas.config(yscrollcommand=sb.set)
        canvas.pack(side='left', fill='both', expand=True)
        sb.pack(side='right', fill='y')
        # Naglowek: rampy
        tk.Label(inner, text="Temp\\Ramp", bg=C['panel'], fg=C['cyan'],
                 font=(FONT, fsz(9), 'bold'), width=10, anchor='w').grid(
                 row=0, column=0, sticky='nsew', padx=1, pady=1)
        for ci, r in enumerate(PR):
            tk.Label(inner, text=f"{r}°C/min", bg=C['panel'], fg=C['cyan'],
                     font=(FONT, fsz(9), 'bold'), width=16).grid(
                     row=0, column=ci+1, sticky='nsew', padx=1, pady=1)
        # Wiersze: temperatury
        for ri, t in enumerate(PT):
            tk.Label(inner, text=f"{t}°C", bg=C['panel'], fg=C['orange'],
                     font=(FONT, fsz(9), 'bold'), width=10, anchor='w').grid(
                     row=ri+1, column=0, sticky='nsew', padx=1, pady=1)
            for ci, r in enumerate(PR):
                idx = ri * len(PR) + ci  # pi_(ti,ri) = ti*PR_N+ri
                p = pmap.get(idx)
                if p and p['valid']:
                    txt = f"{p['KpH']:.1f} / {p['KiH']:.2f} / {p['KdH']:.2f}"
                    fg = C['text']; bg = C['bg2']
                else:
                    txt = "—"
                    fg = C['dim2']; bg = C['panel2']
                tk.Label(inner, text=txt, bg=bg, fg=fg,
                         font=(FONT, fsz(8)), width=16).grid(
                         row=ri+1, column=ci+1, sticky='nsew', padx=1, pady=1)
        # Stopka
        tk.Label(win, text="Each cell: Kp / Ki / Kd for that temperature and ramp rate.\n"
                 "On START, the app interpolates between the 4 nearest points automatically.",
                 bg=C['bg'], fg=C['dim'], font=(FONT, fsz(8)), justify='left').pack(
                 anchor='w', padx=16, pady=(0, 12))

    def dump_calibration_to_pc(self, silent=True):
        """Poprosi urzadzenie o profile i offset, zapisze do JSON"""
        if not self.connected:
            return
        self._caldump_purpose = 'save'
        # Zapamietaj offset z aktualnego suwaka
        try:
            self._pending_offset = self.sl_off.get()
        except:
            self._pending_offset = 0.0
        self.send("DUMPCAL")
        if not silent:
            print("Pobieranie profili z urzadzenia...")

    def _finish_caldump_save(self):
        """Po odebraniu wszystkich profili - zapisz do pliku JSON"""
        try:
            profiles = []
            for line in self._caldump_buf:
                parts = line.split(',')
                if len(parts) >= 8:
                    profiles.append({
                        'idx': int(parts[0]),
                        'KpH': float(parts[1]), 'KiH': float(parts[2]), 'KdH': float(parts[3]),
                        'KpC': float(parts[4]), 'KiC': float(parts[5]), 'KdC': float(parts[6]),
                        'valid': parts[7].strip() == '1',
                    })
            data = {
                'version': 1,
                'saved': datetime.now().isoformat(timespec='seconds'),
                'offset': self._pending_offset if self._pending_offset is not None else 0.0,
                'profiles': profiles,
            }
            with open(self.cal_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
            n_valid = sum(1 for p in profiles if p['valid'])
            print(f"Kalibracja zapisana: {self.cal_file.name} ({n_valid}/{len(profiles)} profili)")
            if hasattr(self, 'cal_status'):
                self.cal_status.config(text=f"✓ Calibration saved to PC ({n_valid} profiles)")
            if self._caldump_purpose == 'save':
                try:
                    messagebox.showinfo("Calibration saved",
                        f"PID profiles + offset saved to disk:\n{self.cal_file}\n\n"
                        f"Saved {n_valid} calibrated profiles.\n"
                        "They will be auto-loaded on next connection.")
                except: pass
            elif self._caldump_purpose == 'view':
                # Pokaz tabele w oknie
                self._show_cal_table_window(profiles)
        except Exception as e:
            print(f"Blad zapisu kalibracji: {e}")
        self._caldump_purpose = None

    def load_calibration_from_pc(self):
        """Wczytaj kalibracje z pliku JSON i wyslij do urzadzenia"""
        if not self.cal_file.exists():
            return False
        try:
            with open(self.cal_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            profiles = data.get('profiles', [])
            offset = data.get('offset', 0.0)
            if not profiles:
                return False
            # Wyslij offset
            self.send(f"OFFSET:{offset:.1f}")
            # Wyslij kazdy profil (z malym odstepem zeby nie zalac bufora)
            def send_profiles(i=0):
                if i >= len(profiles):
                    # Po wszystkich - oznacz kalibracje jako gotowa
                    self.send("SETCALDONE:1")
                    self.dev_cal = True
                    if hasattr(self, 'cal_status'):
                        self.cal_status.config(
                            text=f"✓ Loaded calibration from PC ({len(profiles)} profiles)")
                    print(f"Wgrano {len(profiles)} profili z PC do urzadzenia")
                    return
                p = profiles[i]
                self.send(f"SETPROF:{p['idx']},{p['KpH']:.3f},{p['KiH']:.4f},"
                         f"{p['KdH']:.3f},{p['KpC']:.3f},{p['KiC']:.4f},"
                         f"{p['KdC']:.3f},{1 if p['valid'] else 0}")
                # Nastepny profil za 40ms
                self.root.after(40, lambda: send_profiles(i + 1))
            send_profiles(0)
            saved = data.get('saved', '?')
            print(f"Ladowanie kalibracji z PC (zapisana: {saved})")
            return True
        except Exception as e:
            print(f"Blad ladowania kalibracji: {e}")
            return False


    # ────────────────────────────────────────────────────
    #  BUDOWA UI
    # ────────────────────────────────────────────────────
    def _build_ui(self):
        # Pasek tytulowy z lampka statusu
        top = tk.Frame(self.root, bg=C['bg2'], height=44)
        top.pack(fill='x'); top.pack_propagate(False)
        tk.Frame(top, bg=C['red'], width=6).pack(side='left', fill='y')
        tk.Label(top, text="  PELTIER CONTROL", bg=C['bg2'], fg=C['text'],
                 font=(FONT, fsz(13), 'bold')).pack(side='left', padx=(8, 0))
        tk.Label(top, text="v6.0", bg=C['bg2'], fg=C['dim2'],
                 font=(FONT, fsz(9))).pack(side='left', padx=8)

        # Status po prawej
        sf = tk.Frame(top, bg=C['bg2'])
        sf.pack(side='right', padx=16)
        self.s_dot = tk.Canvas(sf, width=14, height=14, bg=C['bg2'], highlightthickness=0)
        self.s_dot.pack(side='left', padx=(0, 8))
        self._draw_dot(C['dim2'], glow=False)
        self.s_lbl = tk.Label(sf, text="DISCONNECTED", bg=C['bg2'], fg=C['dim'],
                              font=(FONT, fsz(10)))
        self.s_lbl.pack(side='left')

        # Notebook
        nb = ttk.Notebook(self.root)
        nb.pack(fill='both', expand=True, padx=0, pady=0)
        t1 = tk.Frame(nb, bg=C['bg']); nb.add(t1, text='CONTROL')
        t2 = tk.Frame(nb, bg=C['bg']); nb.add(t2, text='ADVANCED')
        t3 = tk.Frame(nb, bg=C['bg']); nb.add(t3, text='ARCHIVE')
        t4 = tk.Frame(nb, bg=C['bg']); nb.add(t4, text='CONNECTION')
        self.build_live(t1)
        self.build_advanced(t2)
        self.build_arch(t3)
        self.build_conn(t4)

    def _draw_dot(self, color, glow=True):
        self.s_dot.delete('all')
        if glow:
            self.s_dot.create_oval(0, 0, 14, 14, fill='', outline=color, width=1)
        self.s_dot.create_rectangle(3, 3, 11, 11, fill=color, outline='')

    def _pulse(self):
        if self.connected:
            self._pulse_state = (self._pulse_state + 1) % 20
            phase = abs(self._pulse_state - 10) / 10.0
            col = _lighten(C['green'], phase * 0.4)
            self._draw_dot(col)
        self.root.after(80, self._pulse)

    def set_status(self, connected, msg):
        self.connected = connected
        if connected:
            self._draw_dot(C['green'])
            self.s_lbl.config(text=msg or "CONNECTED", fg=C['green'])
        else:
            self._draw_dot(C['dim2'], glow=False)
            self.s_lbl.config(text=msg or "DISCONNECTED", fg=C['dim'])
        # Aktywuj/dezaktywuj panel
        if hasattr(self, 'btn_run'):
            self._set_panel_enabled(connected)

    # ────────────────────────────────────────────────────
    #  EKRAN LIVE: wykres (lewo) + panel sterowania (prawo)
    # ────────────────────────────────────────────────────
    def build_live(self, parent):
        # Gorny pasek: kompaktowe karty statystyk + przyciski START/STOP
        topbar = tk.Frame(parent, bg=C['bg'])
        topbar.pack(fill='x', padx=16, pady=(10, 6))

        # Karty (lewa czesc, rozciagane)
        cards = tk.Frame(topbar, bg=C['bg'])
        cards.pack(side='left', fill='x', expand=True)
        self.cards = {}
        self.cards['temp'] = self._stat_card(cards, "TEMP", "°C", C['blue'])
        self.cards['temp2'] = self._stat_card(cards, "TEMP 2", "°C", C['cyan'])
        self.cards['sp']   = self._stat_card(cards, "SETPOINT", "°C", C['orange'])
        self.cards['rate'] = self._stat_card(cards, "AVG RATE", "°C/min", C['yellow'])
        self.cards['pwm']  = self._stat_card(cards, "PWM", "%", C['green'])

        # Przyciski START/STOP/E-STOP (prawa czesc paska) - zawsze widoczne
        ctrl = tk.Frame(topbar, bg=C['bg'])
        ctrl.pack(side='right', padx=(8, 0))
        self.is_running = False  # stan: czy cykl trwa
        self.btn_run = tk.Button(ctrl, text="▶ START", command=self.toggle_run,
                                 bg=C['green'], fg='#1a1c1f', font=(FONT, fsz(12), 'bold'),
                                 relief='flat', cursor='hand2', bd=0, padx=16, pady=12,
                                 activebackground=_lighten(C['green'], 0.15))
        self.btn_run.pack(side='left', padx=(0, 4), fill='y')
        # FREEZE - zamroz gal do wymiany probki
        self.btn_freeze = tk.Button(ctrl, text="❄ FREEZE", command=self.do_freeze,
                                    bg=C['bg2'], fg=C['cyan'], font=(FONT, fsz(12), 'bold'),
                                    relief='flat', cursor='hand2', bd=0, padx=12, pady=12,
                                    highlightthickness=2, highlightbackground=C['cyan'],
                                    activebackground=C['panel3'])
        self.btn_freeze.pack(side='left', padx=(0, 4), fill='y')
        self.btn_estop = tk.Button(ctrl, text="⛔", command=self.do_estop,
                                   bg=C['red'], fg='#fff', font=(FONT, fsz(14), 'bold'),
                                   relief='flat', cursor='hand2', bd=0, padx=12, pady=12,
                                   activebackground=_lighten(C['red'], 0.15))
        self.btn_estop.pack(side='left', fill='y')

        # Glowny obszar: wykres + panel
        main = tk.Frame(parent, bg=C['bg'])
        main.pack(fill='both', expand=True, padx=16, pady=(0, 12))

        # PRAWO - panel sterowania (pakowany PIERWSZY!)
        # Stala szerokosc 312px rezerwuje miejsce z prawej ZANIM rozszerzajacy sie
        # wykres zajmie cavity. Inaczej canvas matplotlib przy przerysowaniu (zoom/
        # home/resize) zada pelnego rozmiaru i zgniata panel pakowany pozniej -> panel znika.
        self._build_panel(main)
        # LEWO - wykres (wypelnia pozostala przestrzen)
        self._build_chart(main)

    def _stat_card(self, parent, title, unit, color):
        card = tk.Frame(parent, bg=C['panel'])
        card.pack(side='left', fill='x', expand=True, padx=(0, 4))
        tk.Frame(card, bg=color, height=3).pack(fill='x')
        inner = tk.Frame(card, bg=C['panel'])
        inner.pack(fill='both', expand=True, padx=7, pady=5)
        tk.Label(inner, text=title, bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(7)), anchor='w').pack(anchor='w')
        vrow = tk.Frame(inner, bg=C['panel'])
        vrow.pack(anchor='w', pady=(1, 0))
        val = tk.Label(vrow, text="--", bg=C['panel'], fg=color,
                       font=(FONT, fsz(16), 'bold'))
        val.pack(side='left')
        unit_lbl = tk.Label(vrow, text=" " + unit, bg=C['panel'], fg=C['dim2'],
                            font=(FONT, fsz(7)))
        unit_lbl.pack(side='left', pady=(4, 0))
        return {'val': val, 'unit': unit, 'unit_lbl': unit_lbl, 'extra': None, 'row': vrow}

    def _build_chart(self, parent):
        wrap = tk.Frame(parent, bg=C['panel'])
        wrap.pack(side='left', fill='both', expand=True, padx=(0, 12))
        tk.Frame(wrap, bg=C['border2'], height=3).pack(fill='x')

        hd = tk.Frame(wrap, bg=C['panel'])
        hd.pack(fill='x', padx=14, pady=(10, 4))
        tk.Label(hd, text="LIVE CHART", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(side='left')

        # Statystyki dotarcia do setpointu (prawa strona naglowka)
        self.reach_lbl = tk.Label(hd, text="", bg=C['panel'], fg=C['green'],
                                  font=(FONT, fsz(9), 'bold'))
        self.reach_lbl.pack(side='right')

        self.fig = Figure(figsize=(9, 6), facecolor=C['panel'], dpi=110)
        gs = self.fig.add_gridspec(2, 1, height_ratios=[3, 1], hspace=0.2,
                                   left=0.07, right=0.97, top=0.97, bottom=0.08)
        self.ax1 = self.fig.add_subplot(gs[0])
        self.ax2 = self.fig.add_subplot(gs[1], sharex=self.ax1)
        for ax in [self.ax1, self.ax2]:
            ax.set_facecolor(C['panel2'])

        self.cv = FigureCanvasTkAgg(self.fig, master=wrap)
        self.cv.get_tk_widget().pack(fill='both', expand=True, padx=8, pady=(0, 4))

        # Pasek narzedzi wykresu: pauza, okno czasu, zoom matplotlib
        toolbar_row = tk.Frame(wrap, bg=C['panel'])
        toolbar_row.pack(fill='x', padx=8, pady=(0, 8))

        # Przycisk PAUSE - zatrzymuje przewijanie zeby przyblizyc
        self.btn_pause = tk.Button(toolbar_row, text="⏸ PAUSE", command=self.toggle_pause,
                                   bg=C['bg2'], fg=C['yellow'], font=(FONT, fsz(9), 'bold'),
                                   relief='flat', cursor='hand2', bd=0, padx=12, pady=6,
                                   highlightthickness=1, highlightbackground=C['yellow'],
                                   activebackground=C['panel3'])
        self.btn_pause.pack(side='left', padx=(0, 6))

        # Wybor okna czasu (ile ostatnich sekund pokazac)
        tk.Label(toolbar_row, text="WINDOW:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(8))).pack(side='left', padx=(8, 4))
        for label, secs in [("ALL", 0), ("5m", 300), ("2m", 120), ("1m", 60)]:
            b = tk.Button(toolbar_row, text=label,
                         command=lambda s=secs: self.set_chart_window(s),
                         bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(8)),
                         relief='flat', cursor='hand2', bd=0, padx=10, pady=5,
                         activebackground=C['panel3'])
            b.pack(side='left', padx=2)

        # Matplotlib toolbar (zoom, pan, save) - kompaktowy
        tb_frame = tk.Frame(toolbar_row, bg=C['panel'])
        tb_frame.pack(side='right')
        try:
            self.mpl_toolbar = NavigationToolbar2Tk(self.cv, tb_frame, pack_toolbar=False)
            self.mpl_toolbar.config(bg=C['panel'])
            self.mpl_toolbar.update()
            self.mpl_toolbar.pack(side='right')
        except Exception as e:
            print(f"toolbar err: {e}")

    def toggle_pause(self):
        """Pauza/wznow przewijanie wykresu (do przyblizania)"""
        self.chart_paused = not self.chart_paused
        if not hasattr(self, 'btn_pause'):
            return
        if self.chart_paused:
            self.btn_pause.config(text="▶ RESUME", fg=C['green'],
                                 highlightbackground=C['green'])
        else:
            self.btn_pause.config(text="⏸ PAUSE", fg=C['yellow'],
                                 highlightbackground=C['yellow'])

    def set_chart_window(self, secs):
        """Ustaw okno czasowe wykresu (0=wszystko)"""
        self.chart_window = secs

    def _build_panel(self, parent):
        """Prawy panel sterowania - waski pasek z przewijaniem"""
        panel = tk.Frame(parent, bg=C['bg2'], width=312)
        panel.pack(side='right', fill='y')
        panel.pack_propagate(False)
        tk.Frame(panel, bg=C['red'], width=6).pack(side='left', fill='y')

        # Przewijalny obszar - Canvas + Scrollbar (panel moze byc dluzszy niz ekran)
        scroll_wrap = tk.Frame(panel, bg=C['bg2'])
        scroll_wrap.pack(side='left', fill='both', expand=True)
        pcanvas = tk.Canvas(scroll_wrap, bg=C['bg2'], highlightthickness=0,
                            width=290)
        psb = tk.Scrollbar(scroll_wrap, orient='vertical', command=pcanvas.yview)
        pcanvas.configure(yscrollcommand=psb.set)
        psb.pack(side='right', fill='y')
        pcanvas.pack(side='left', fill='both', expand=True)

        inner = tk.Frame(pcanvas, bg=C['bg2'])
        inner_id = pcanvas.create_window((0, 0), window=inner, anchor='nw')

        def _on_inner_config(e):
            pcanvas.configure(scrollregion=pcanvas.bbox('all'))
        inner.bind('<Configure>', _on_inner_config)
        def _on_canvas_config(e):
            pcanvas.itemconfig(inner_id, width=e.width)
        pcanvas.bind('<Configure>', _on_canvas_config)
        # Przewijanie kolkiem myszy
        def _on_wheel(e):
            pcanvas.yview_scroll(int(-1 * (e.delta / 120)), 'units')
        pcanvas.bind('<Enter>', lambda e: pcanvas.bind_all('<MouseWheel>', _on_wheel))
        pcanvas.bind('<Leave>', lambda e: pcanvas.unbind_all('<MouseWheel>'))

        inner = tk.Frame(inner, bg=C['bg2'])
        inner.pack(fill='both', expand=True, padx=16, pady=14)

        tk.Label(inner, text="CONTROL", bg=C['bg2'], fg=C['text'],
                 font=(FONT, fsz(13), 'bold')).pack(anchor='w')
        tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=(8, 12))

        # Suwaki nastaw
        self.sl_sp = SliderField(inner, "TARGET", -15, 100, 25.0,
                                 C['orange'], "°C", 1,
                                 on_change=lambda v: self.send(f"SP:{v:.1f}"))
        self.sl_ru = SliderField(inner, "HEAT RATE", 0.5, 80, 2.0,
                                 C['yellow'], "°C/min", 1,
                                 on_change=lambda v: self.send(f"RU:{v:.1f}"))
        self.sl_rd = SliderField(inner, "COOL RATE", 0.5, 80, 2.0,
                                 C['cyan'], "°C/min", 1,
                                 on_change=lambda v: self.send(f"RD:{v:.1f}"))
        self.sl_tmax = SliderField(inner, "MAX TEMP", 50, 115, 80,
                                   C['red'], "°C", 0,
                                   on_change=lambda v: self.send(f"TMAX:{v:.0f}"))

        tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=(2, 12))

        # WENTYLATORY - przycisk on/off + suwak predkosci
        fan_hd = tk.Frame(inner, bg=C['bg2'])
        fan_hd.pack(fill='x', pady=(0, 4))
        tk.Label(fan_hd, text="FANS", bg=C['bg2'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(side='left')
        self.fan_on = False
        self.btn_fan = tk.Button(fan_hd, text="○ OFF", command=self.toggle_fan,
                                 bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(9), 'bold'),
                                 relief='flat', cursor='hand2', bd=0, padx=12, pady=4,
                                 highlightthickness=1, highlightbackground=C['dim'],
                                 activebackground=C['panel3'])
        self.btn_fan.pack(side='right')
        self.sl_fan = SliderField(inner, "FAN SPEED", 0, 100, 100,
                                  C['blue'], "%", 0,
                                  on_change=lambda v: self.set_fan_speed(v))

        tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=(2, 12))

        # AUTO badge - kierunek wyznaczany automatycznie
        auto = tk.Frame(inner, bg=C['bg2'], highlightthickness=1,
                        highlightbackground=C['green'])
        auto.pack(fill='x', pady=(0, 10))
        tk.Label(auto, text="● AUTO: direction by setpoint", bg=C['bg2'],
                 fg=C['green'], font=(FONT, fsz(9))).pack(padx=8, pady=6)

        # Profile wieloetapowe
        # Profile + Presety
        bf_pp = tk.Frame(inner, bg=C['bg2'])
        bf_pp.pack(fill='x', pady=(0, 8))
        mk_btn_outline(bf_pp, "PROFILES", self.open_profiles, C['purple']).pack(
            side='left', fill='x', expand=True, padx=(0, 3))
        mk_btn_outline(bf_pp, "PRESETS", self.open_presets, C['green']).pack(
            side='left', fill='x', expand=True, padx=(3, 0))

        # Status kalibracji - klikalny (gdy kalibracja trwa, pokazuje postep)
        self.cal_status = tk.Label(inner, text="", bg=C['bg2'], fg=C['purple'],
                                   font=(FONT, fsz(8)), anchor='w', cursor='hand2')
        self.cal_status.pack(fill='x', pady=(0, 4))
        self.cal_status.bind('<Button-1>', lambda e: self.open_cal_window())

        tk.Label(inner, text="▶ START uses panel values",
                 bg=C['bg2'], fg=C['green'], font=(FONT, fsz(8))).pack(anchor='w', pady=(4, 0))
        tk.Label(inner, text="PID tuning & calibration → ADVANCED tab",
                 bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(8))).pack(anchor='w', pady=(2, 0))

        self._set_panel_enabled(False)

    def build_advanced(self, parent):
        """Zakladka ADVANCED - PID, kalibracja, polaryzacja, Flash, reset"""
        wrap = tk.Frame(parent, bg=C['bg'])
        wrap.pack(fill='both', expand=True, padx=20, pady=16)

        # Przewijalny obszar (duzo opcji)
        acanvas = tk.Canvas(wrap, bg=C['bg'], highlightthickness=0)
        asb = tk.Scrollbar(wrap, orient='vertical', command=acanvas.yview)
        acanvas.configure(yscrollcommand=asb.set)
        asb.pack(side='right', fill='y')
        acanvas.pack(side='left', fill='both', expand=True)
        col = tk.Frame(acanvas, bg=C['bg'])
        cid = acanvas.create_window((0, 0), window=col, anchor='nw')
        col.bind('<Configure>', lambda e: acanvas.configure(scrollregion=acanvas.bbox('all')))
        acanvas.bind('<Configure>', lambda e: acanvas.itemconfig(cid, width=e.width))
        acanvas.bind('<Enter>', lambda e: acanvas.bind_all('<MouseWheel>',
                     lambda ev: acanvas.yview_scroll(int(-ev.delta/120), 'units')))
        acanvas.bind('<Leave>', lambda e: acanvas.unbind_all('<MouseWheel>'))

        # Ograniczenie szerokosci dla czytelnosci
        inner = tk.Frame(col, bg=C['bg'])
        inner.pack(fill='x', padx=4, pady=4)
        inner.configure(width=560)

        tk.Label(inner, text="ADVANCED — PID & CALIBRATION", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(14), 'bold')).pack(anchor='w')
        tk.Label(inner, text="Tuning, calibration and device memory",
                 bg=C['bg'], fg=C['dim'], font=(FONT, fsz(9))).pack(anchor='w', pady=(2, 16))

        # ── PID TUNING ──
        sec1 = self._adv_section(inner, "PID TUNING", C['cyan'])
        pid_hd = tk.Frame(sec1, bg=C['bg2'])
        pid_hd.pack(fill='x', pady=(0, 8))
        tk.Label(pid_hd, text="Manual PID gains", bg=C['bg2'], fg=C['dim'],
                 font=(FONT, fsz(9))).pack(side='left')
        self.btn_st = mk_btn(pid_hd, "SELF-TUNE", self.do_selftune, C['cyan'])
        self.btn_st.pack(side='right')
        self.sl_kp = SliderField(sec1, "Kp", 1, 30, 10.0, C['cyan'], "", 1,
                                 on_change=lambda v: self.send(f"KP:{v:.1f}"))
        self.sl_ki = SliderField(sec1, "Ki", 0, 1.5, 0.3, C['cyan'], "", 2,
                                 on_change=lambda v: self.send(f"KI:{v:.2f}"))
        self.sl_kd = SliderField(sec1, "Kd", 0, 80, 0.8, C['cyan'], "", 2,
                                 on_change=lambda v: self.send(f"KD:{v:.2f}"))
        # Feed-forward (grzanie): HOLD = moc na utrzymanie, RAMP = moc na dynamike rampy.
        # Stroj na zywo: za mocno na starcie -> zmniejsz RAMP; nie dochodzi -> zwieksz.
        self.sl_kffh = SliderField(sec1, "FF HOLD (KFFH)", 0, 8, 2.5, C['yellow'], "PWM/°C", 2,
                                   on_change=lambda v: self.send(f"KFFH:{v:.2f}"))
        self.sl_kffr = SliderField(sec1, "FF RAMP (KFFR)", 0, 4, 1.0, C['yellow'], "PWM/(°C/min)", 2,
                                   on_change=lambda v: self.send(f"KFFR:{v:.2f}"))

        # ── AUTO-CALIBRATION ──
        sec2 = self._adv_section(inner, "AUTO-CALIBRATION", C['purple'])
        self.btn_autocal = mk_btn(sec2, "⚙ AUTO-CAL (select range)",
                                  self.do_autocal, C['purple'], fg='#fff')
        self.btn_autocal.pack(fill='x', pady=(0, 6))
        mk_btn_outline(sec2, "📋 VIEW CAL TABLE", self.show_cal_table,
                       C['purple']).pack(fill='x', pady=(0, 6))
        tk.Label(sec2, text="Calibrates PID for temp × ramp grid, saves to Flash.\n"
                 "View table shows stored Kp/Ki/Kd per point.",
                 bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(8)),
                 justify='left').pack(anchor='w')

        # ── THERMOCOUPLE OFFSET ──
        sec3 = self._adv_section(inner, "THERMOCOUPLE", C['purple'])
        self.sl_off = SliderField(sec3, "CAL OFFSET", -20, 20, 0.0,
                                  C['purple'], "°C", 1,
                                  on_change=lambda v: self.send(f"OFFSET:{v:.1f}"))

        # ── PELTIER POLARITY ──
        sec4 = self._adv_section(inner, "PELTIER POLARITY", C['orange'])
        pol_frame = tk.Frame(sec4, bg=C['bg2'])
        pol_frame.pack(fill='x')
        self.pol_indicator = tk.Label(pol_frame, text="POL: ?", bg=C['bg2'],
                                      fg=C['dim2'], font=(FONT, fsz(10), 'bold'))
        self.pol_indicator.pack(side='left')
        mk_btn_outline(pol_frame, "RE-DETECT", self.do_repol, C['dim']).pack(side='right')
        tk.Label(sec4, text="Detected once, saved permanently",
                 bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(8))).pack(anchor='w', pady=(6, 0))

        # ── DEVICE FLASH MEMORY ──
        sec5 = self._adv_section(inner, "DEVICE FLASH", C['green'])
        bf2 = tk.Frame(sec5, bg=C['bg2'])
        bf2.pack(fill='x')
        mk_btn_outline(bf2, "SAVE", lambda: self.send("SAVE"), C['green']).pack(
            side='left', fill='x', expand=True, padx=(0, 3))
        mk_btn_outline(bf2, "LOAD", lambda: self.send("LOAD"), C['cyan']).pack(
            side='left', fill='x', expand=True, padx=(3, 0))
        tk.Label(sec5, text="Save/load settings to device internal memory",
                 bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(8))).pack(anchor='w', pady=(6, 0))

        # ── PC CALIBRATION BACKUP ──
        sec6 = self._adv_section(inner, "PC CALIBRATION BACKUP", C['purple'])
        bf3 = tk.Frame(sec6, bg=C['bg2'])
        bf3.pack(fill='x')
        mk_btn_outline(bf3, "⤓ SAVE TO PC",
                       lambda: self.dump_calibration_to_pc(silent=False),
                       C['purple']).pack(side='left', fill='x', expand=True, padx=(0, 3))
        mk_btn_outline(bf3, "⤒ LOAD FROM PC",
                       self._manual_load_cal, C['cyan']).pack(
                       side='left', fill='x', expand=True, padx=(3, 0))
        tk.Label(sec6, text="Backup profiles to a file (auto-loaded on connect)",
                 bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(8))).pack(anchor='w', pady=(6, 0))

        # ── RESET ──
        sec7 = self._adv_section(inner, "RESET", C['red'])
        mk_btn_outline(sec7, "↺ RESET ALL SETTINGS", self.do_reset, C['red']).pack(fill='x')

    def _adv_section(self, parent, title, color):
        """Pomocnicza - ramka sekcji w zakladce ADVANCED"""
        tk.Frame(parent, bg=color, height=2).pack(fill='x', pady=(12, 0))
        tk.Label(parent, text=title, bg=C['bg'], fg=color,
                 font=(FONT, fsz(10), 'bold')).pack(anchor='w', pady=(4, 6))
        box = tk.Frame(parent, bg=C['bg2'])
        box.pack(fill='x')
        inner = tk.Frame(box, bg=C['bg2'])
        inner.pack(fill='x', padx=12, pady=10)
        return inner

    def _set_panel_enabled(self, en):
        # Suwaki zawsze aktywne (mozna ustawic wartosci przed polaczeniem)
        # START/STOP tez aktywne - sprawdzaja polaczenie w momencie klikniecia
        # (dezaktywujemy tylko gdy chcemy wyraznie zablokowac)
        for sl in ['sl_sp', 'sl_ru', 'sl_rd', 'sl_tmax', 'sl_kp', 'sl_ki', 'sl_kd', 'sl_off', 'sl_fan']:
            if hasattr(self, sl):
                getattr(self, sl).set_enabled(True)
        # Przyciski zawsze klikalnie - reaguja komunikatem jesli brak polaczenia
        for b in ['btn_run', 'btn_st', 'btn_autocal', 'btn_estop', 'btn_freeze', 'btn_fan']:
            if hasattr(self, b):
                getattr(self, b).config(state='normal')


    # ────────────────────────────────────────────────────
    #  AKCJE PRZYCISKOW
    # ────────────────────────────────────────────────────
    def toggle_run(self):
        """Przelacznik START/STOP w jednym przycisku"""
        if self.is_running:
            self.do_stop()
        else:
            self.do_start()

    def _update_run_button(self, running):
        """Aktualizuj wyglad przycisku: zielony START / czerwony STOP"""
        self.is_running = running
        if not hasattr(self, 'btn_run'):
            return
        if running:
            self.btn_run.config(text="■ STOP", bg=C['red'], fg='#fff',
                               activebackground=_lighten(C['red'], 0.15))
        else:
            self.btn_run.config(text="▶ START", bg=C['green'], fg='#1a1c1f',
                               activebackground=_lighten(C['green'], 0.15))

    def do_start(self):
        """START - wyslij wszystkie nastawy z panelu, potem uruchom"""
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        # RESET statystyk dojscia - kazdy START liczy nowa srednia od zera.
        # Dzieki temu mozna robic pomiary jeden po drugim bez starych danych.
        self.reach_start_t = None
        self.reach_start_temp = None
        self.reach_target = self.sl_sp.get()
        self.reach_done = False
        self.reach_time = None
        self.reach_avg_rate = None
        self.reach_dir = None
        self.last_setpoint_target = None
        self._last_reach_summary = None
        if hasattr(self, 'reach_lbl'):
            self.reach_lbl.config(text="→ starting...", fg=C['dim'])
        # Wyslij komplet nastaw z panelu
        self.send(f"SP:{self.sl_sp.get():.1f}")
        self.send(f"RU:{self.sl_ru.get():.1f}")
        self.send(f"RD:{self.sl_rd.get():.1f}")
        self.send(f"TMAX:{self.sl_tmax.get():.0f}")
        self.send(f"KP:{self.sl_kp.get():.1f}")
        self.send(f"KI:{self.sl_ki.get():.2f}")
        self.send(f"KD:{self.sl_kd.get():.2f}")
        self.send(f"OFFSET:{self.sl_off.get():.1f}")
        time.sleep(0.05)
        self.send("START")
        self._update_run_button(True)

    def do_stop(self):
        self.send("STOP")
        self.send("AUTOCALSTOP")  # przerwij tez kalibracje jesli trwa
        if hasattr(self, 'cal_status'):
            self.cal_status.config(text="")
        self._update_run_button(False)

    def do_estop(self):
        """Awaryjne zatrzymanie - natychmiast wylacza PWM"""
        self.send("ESTOP")
        self.send("AUTOCALSTOP")
        if hasattr(self, 'cal_status'):
            self.cal_status.config(text="")

    def toggle_fan(self):
        """Wlacz/wylacz wentylatory"""
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        self.fan_on = not self.fan_on
        if self.fan_on:
            spd = int(self.sl_fan.get()) if hasattr(self, 'sl_fan') else 100
            if spd == 0: spd = 100; self.sl_fan.set(100, silent=True)
            self.send(f"FAN:{spd}")
            self.btn_fan.config(text="● ON", fg=C['green'], highlightbackground=C['green'])
        else:
            self.send("FANOFF")
            self.btn_fan.config(text="○ OFF", fg=C['dim2'], highlightbackground=C['dim'])

    def set_fan_speed(self, v):
        """Ustaw predkosc wentylatorow (suwak)"""
        spd = int(v)
        self.send(f"FAN:{spd}")
        # Suwak na 0 = wylacz, >0 = wlacz
        if hasattr(self, 'btn_fan'):
            if spd > 0:
                self.fan_on = True
                self.btn_fan.config(text="● ON", fg=C['green'], highlightbackground=C['green'])
            else:
                self.fan_on = False
                self.btn_fan.config(text="○ OFF", fg=C['dim2'], highlightbackground=C['dim'])

    def do_freeze(self):
        """Zamroz gal do stanu stalego (wymiana probki)"""
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        if messagebox.askyesno("Freeze gal",
                "Cool the gal to solid state for sample swap?\n\n"
                "Gently ramps down to 20°C and HOLDS it there\n"
                "(keeps cooling active to prevent re-melting).\n\n"
                "You'll see 'GAL SOLID' when ready.\n"
                "Press STOP when done swapping the sample."):
            self.send("FREEZE")
            if hasattr(self, 'reach_lbl'):
                self.reach_lbl.config(text="❄ Freezing gal...", fg=C['cyan'])

    def do_reset(self):
        """Reset nastaw do domyslnych"""
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        if messagebox.askyesno("Reset settings",
                "Restore default settings?\n"
                "This clears all profiles and calibration!"):
            self.send("RESET")

    def do_repol(self):
        """Wymus ponowne wykrycie polaryzacji Peltiera"""
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        if messagebox.askyesno("Re-detect polarity",
                "Re-detect Peltier polarity?\n\n"
                "The device will briefly heat to check direction (~4s).\n"
                "Do not touch the thermocouple during the test.\n"
                "Result is saved permanently."):
            self.send("REPOL")

    def _update_pol_indicator(self):
        """Aktualizuj wskaznik polaryzacji w panelu"""
        if not hasattr(self, 'pol_indicator'):
            return
        if self.dev_pol_set:
            txt = "POL: SWAPPED" if self.dev_pol_swapped else "POL: NORMAL"
            col = C['orange'] if self.dev_pol_swapped else C['green']
            self.pol_indicator.config(text=f"● {txt}", fg=col)
        else:
            self.pol_indicator.config(text="POL: not set", fg=C['dim2'])

    def do_selftune(self):
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        if messagebox.askyesno("Self-Tune",
                "Start PID auto-tuning?\nTakes ~2 minutes.\n"
                "Device must be running (START)."):
            self.send("SELFTUNE")

    def do_autocal(self):
        """Otworz okno wyboru zakresu auto-kalibracji"""
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        CalRangeDialog(self.root, self)

    def start_autocal(self, temp_min, temp_max, ramps):
        """Uruchom auto-kalibracje z wybranym zakresem i listą ramp"""
        # Wyslij zakres temp
        self.send(f"CALRANGE:{temp_min:.0f},{temp_max:.0f}")
        time.sleep(0.1)
        # Wyslij liste ramp (KLUCZOWE - to definiuje przez co przejdzie kalibracja)
        ramps_str = ",".join(f"{r:.0f}" for r in ramps)
        self.send(f"SETCALRAMPS:{ramps_str}")
        time.sleep(0.1)
        self.cal_running = True
        self.cal_t0 = time.time()
        self.cal_current = 0
        self.send("AUTOCAL")
        if hasattr(self, 'cal_status'):
            self.cal_status.config(text="Calibration starting... (click for progress)")
        self.root.after(600, self.open_cal_window)

    def open_cal_window(self):
        """Otworz okno postepu kalibracji"""
        if not self.cal_plan and not self.cal_running:
            messagebox.showinfo("Calibration",
                "Calibration is not running.\n"
                "Click AUTO-CAL to start.")
            return
        # Jesli okno juz otwarte - tylko podnies
        if hasattr(self, 'cal_win') and self.cal_win and tk._default_root:
            try:
                self.cal_win.win.lift()
                return
            except: pass
        self.cal_win = CalibrationWindow(self.root, self)

    def _refresh_cal_view(self):
        """Odswiez okno kalibracji jesli otwarte + status w panelu"""
        # Status w panelu glownym
        if hasattr(self, 'cal_status'):
            if self.cal_running and self.cal_total > 0:
                eta = self._cal_eta()
                eta_s = f" · ~{int(eta//60)}min" if eta else ""
                self.cal_status.config(
                    text=f"Kalibracja {self.cal_current}/{self.cal_total}{eta_s} (klik=szczegoly)")
            elif self.cal_current >= self.cal_total and self.cal_total > 0:
                self.cal_status.config(text="✓ Calibration done")
        # Okno szczegolow
        if hasattr(self, 'cal_win') and self.cal_win:
            try: self.cal_win.refresh()
            except: pass

    def open_profiles(self):
        """Okno edycji profili wieloetapowych"""
        ProfileWindow(self.root, self)

    # ────────────────────────────────────────────────────
    #  PRESETY - zapisywalne zestawy nastaw
    # ────────────────────────────────────────────────────
    def _gather_settings(self):
        """Zbierz wszystkie aktualne nastawy z suwakow"""
        s = {}
        for key, attr in [('sp','sl_sp'),('ru','sl_ru'),('rd','sl_rd'),
                          ('tmax','sl_tmax'),('kp','sl_kp'),('ki','sl_ki'),
                          ('kd','sl_kd'),('off','sl_off'),('fan','sl_fan')]:
            if hasattr(self, attr):
                try: s[key] = getattr(self, attr).get()
                except: pass
        return s

    def _load_presets(self):
        """Wczytaj presety z pliku JSON"""
        if not self.presets_file.exists():
            return {}
        try:
            with open(self.presets_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except:
            return {}

    def _save_presets(self, presets):
        """Zapisz presety do pliku JSON"""
        try:
            with open(self.presets_file, 'w', encoding='utf-8') as f:
                json.dump(presets, f, indent=2)
            return True
        except Exception as e:
            print(f"presets save err: {e}")
            return False

    def open_presets(self):
        """Otworz okno zarzadzania presetami"""
        PresetWindow(self.root, self)

    def apply_preset(self, settings):
        """Zastosuj preset - ustaw suwaki i wyslij do urzadzenia"""
        mapping = [('sp','sl_sp','SP',1),('ru','sl_ru','RU',1),('rd','sl_rd','RD',1),
                   ('tmax','sl_tmax','TMAX',0),('kp','sl_kp','KP',1),('ki','sl_ki','KI',2),
                   ('kd','sl_kd','KD',2),('off','sl_off','OFFSET',1),('fan','sl_fan','FAN',0)]
        for key, attr, cmd, dec in mapping:
            if key in settings and hasattr(self, attr):
                val = settings[key]
                try:
                    getattr(self, attr).set(val, silent=True)
                    if self.connected:
                        self.send(f"{cmd}:{val:.{dec}f}")
                except Exception as e:
                    print(f"apply preset {key}: {e}")
        # Aktualizuj stan wentylatora wg fan
        if 'fan' in settings and hasattr(self, 'btn_fan'):
            fv = settings['fan']
            self.fan_on = (fv > 0)
            if fv > 0:
                self.btn_fan.config(text="● ON", fg=C['green'], highlightbackground=C['green'])
            else:
                self.btn_fan.config(text="○ OFF", fg=C['dim2'], highlightbackground=C['dim'])

    # ────────────────────────────────────────────────────
    #  ZAKLADKA POLACZENIE
    # ────────────────────────────────────────────────────
    def build_conn(self, parent):
        wrap = tk.Frame(parent, bg=C['bg'])
        wrap.pack(fill='both', expand=True, padx=24, pady=24)

        card = tk.Frame(wrap, bg=C['panel'])
        card.pack(fill='x', pady=(0, 16))
        tk.Frame(card, bg=C['blue'], height=3).pack(fill='x')
        inner = tk.Frame(card, bg=C['panel'])
        inner.pack(fill='x', padx=20, pady=16)

        tk.Label(inner, text="SERIAL CONNECTION", bg=C['panel'], fg=C['text'],
                 font=(FONT, fsz(12), 'bold')).pack(anchor='w', pady=(0, 12))

        tk.Label(inner, text="Available ports:", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(10))).pack(anchor='w')

        lf = tk.Frame(inner, bg=C['panel'])
        lf.pack(fill='x', pady=8)
        sb = tk.Scrollbar(lf)
        sb.pack(side='right', fill='y')
        self.conn_list = tk.Listbox(lf, bg=C['bg2'], fg=C['text'],
                                    font=(FONT, fsz(10)), height=6,
                                    selectbackground=C['blue'], borderwidth=0,
                                    highlightthickness=1, highlightbackground=C['border'],
                                    yscrollcommand=sb.set, activestyle='none')
        self.conn_list.pack(side='left', fill='both', expand=True)
        sb.config(command=self.conn_list.yview)

        br = tk.Frame(inner, bg=C['panel'])
        br.pack(fill='x', pady=(8, 0))
        mk_btn(br, "REFRESH", self.refresh_ports, C['cyan']).pack(side='left', padx=(0, 8))
        self.conn_btn = mk_btn(br, "CONNECT", self.conn_from_tab, C['green'])
        self.conn_btn.pack(side='left', padx=(0, 8))
        mk_btn_outline(br, "DISCONNECT", self.disconnect, C['red']).pack(side='left')

        # Info
        info = tk.Frame(wrap, bg=C['panel'])
        info.pack(fill='x')
        tk.Frame(info, bg=C['dim2'], height=3).pack(fill='x')
        ii = tk.Frame(info, bg=C['panel'])
        ii.pack(fill='x', padx=20, pady=16)
        tk.Label(ii, text="INSTRUCTIONS", bg=C['panel'], fg=C['text'],
                 font=(FONT, fsz(11), 'bold')).pack(anchor='w', pady=(0, 8))
        for line in [
            "1. Connect ItsyBitsy (firmware v19 PC MODE) via USB",
            "2. Select COM port from the list and click CONNECT",
            "3. Sliders sync automatically with the device",
            "4. Set parameters and click START",
            "5. Chart shows live data, samples are logged to CSV",
        ]:
            tk.Label(ii, text=line, bg=C['panel'], fg=C['dim'],
                     font=(FONT, fsz(9)), anchor='w').pack(anchor='w', pady=1)

        self.refresh_ports()

    def refresh_ports(self):
        self.conn_list.delete(0, 'end')
        self._ports = list(serial.tools.list_ports.comports())
        for p in self._ports:
            self.conn_list.insert('end', f"  {p.device}   {p.description or '?'}")
        if self._ports:
            self.conn_list.selection_set(0)

    def conn_from_tab(self):
        s = self.conn_list.curselection()
        if s and self._ports:
            port = self._ports[s[0]].device
            self.connect(port)

    # ────────────────────────────────────────────────────
    #  ZAKLADKA ARCHIWUM
    # ────────────────────────────────────────────────────
    def build_arch(self, parent):
        wrap = tk.Frame(parent, bg=C['bg'])
        wrap.pack(fill='both', expand=True, padx=16, pady=16)

        hd = tk.Frame(wrap, bg=C['bg'])
        hd.pack(fill='x', pady=(0, 12))
        tk.Label(hd, text="CYCLE ARCHIVE", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(12), 'bold')).pack(side='left')
        tk.Label(hd, text="  tip: tick boxes to overlay & compare cycles",
                 bg=C['bg'], fg=C['dim2'], font=(FONT, fsz(8))).pack(side='left', padx=(8, 0))
        mk_btn(hd, "REFRESH", self.refresh_arch, C['cyan']).pack(side='right')

        body = tk.Frame(wrap, bg=C['bg'])
        body.pack(fill='both', expand=True)

        # Lista cykli z checkboxami (do porownywania)
        lf = tk.Frame(body, bg=C['panel'], width=340)
        lf.pack(side='left', fill='y', padx=(0, 12))
        lf.pack_propagate(False)
        tk.Frame(lf, bg=C['purple'], height=3).pack(fill='x')
        lhd = tk.Frame(lf, bg=C['panel'])
        lhd.pack(fill='x', padx=12, pady=8)
        tk.Label(lhd, text="SAVED CYCLES", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(side='left')
        mk_btn_outline(lhd, "CLEAR", self._arch_clear_sel, C['dim']).pack(side='right')

        # Przewijalna lista checkboxow
        list_wrap = tk.Frame(lf, bg=C['bg2'])
        list_wrap.pack(fill='both', expand=True, padx=8, pady=(0, 8))
        asb = tk.Scrollbar(list_wrap)
        asb.pack(side='right', fill='y')
        self.arch_canvas = tk.Canvas(list_wrap, bg=C['bg2'], highlightthickness=0,
                                    yscrollcommand=asb.set)
        self.arch_canvas.pack(side='left', fill='both', expand=True)
        asb.config(command=self.arch_canvas.yview)
        self.arch_items = tk.Frame(self.arch_canvas, bg=C['bg2'])
        self._arch_win = self.arch_canvas.create_window((0, 0), window=self.arch_items, anchor='nw')
        self.arch_items.bind('<Configure>',
            lambda e: self.arch_canvas.config(scrollregion=self.arch_canvas.bbox('all')))
        # KLUCZOWE: okno wewnetrzne musi miec szerokosc canvasu, inaczej
        # wiersze nie rozciagaja sie i przycisk ✕ (side='right') wypada poza widok
        self.arch_canvas.bind('<Configure>',
            lambda e: self.arch_canvas.itemconfig(self._arch_win, width=e.width))
        self.arch_canvas.bind('<Enter>', lambda e: self.arch_canvas.bind_all(
            '<MouseWheel>', lambda ev: self.arch_canvas.yview_scroll(int(-ev.delta/120), 'units')))
        self.arch_canvas.bind('<Leave>', lambda e: self.arch_canvas.unbind_all('<MouseWheel>'))

        self.arch_vars = {}   # {path: BooleanVar}

        # Wykres
        cf = tk.Frame(body, bg=C['panel'])
        cf.pack(side='left', fill='both', expand=True)
        tk.Frame(cf, bg=C['border2'], height=3).pack(fill='x')
        self.fig_a = Figure(figsize=(8, 6), facecolor=C['panel'], dpi=110)
        self.ax_a = self.fig_a.add_subplot(111)
        self.ax_a.set_facecolor(C['panel2'])
        self.cv_a = FigureCanvasTkAgg(self.fig_a, master=cf)
        self.cv_a.get_tk_widget().pack(fill='both', expand=True, padx=8, pady=(8, 4))
        # Pierwszy rysunek PRZED toolbarem - inicjalizuje canvas
        self.cv_a.draw()

        # Pasek narzedzi matplotlib - WLASNY wiersz, po draw()
        # Toolbar musi powstac po pierwszym draw, inaczej zoom/pan nie dzialaja
        tbf = tk.Frame(cf, bg='#3a3f44')
        tbf.pack(fill='x', padx=8, pady=(4, 0))
        try:
            self.mpl_toolbar_a = NavigationToolbar2Tk(self.cv_a, tbf, pack_toolbar=False)
            self.mpl_toolbar_a.config(bg='#3a3f44')
            # Przyciski toolbara czytelne na ciemnym tle
            for child in self.mpl_toolbar_a.winfo_children():
                try: child.config(bg='#3a3f44')
                except: pass
            self.mpl_toolbar_a.update()
            self.mpl_toolbar_a.pack(side='left', fill='x')
        except Exception as e:
            print(f"arch toolbar err: {e}")

        # Przyciski eksportu - wiersz pod toolbarem
        atb = tk.Frame(cf, bg=C['panel'])
        atb.pack(fill='x', padx=8, pady=(2, 8))

        mk_btn_outline(atb, "⤓ CSV", self.export_arch_csv, C['green']).pack(
            side='right', padx=(4, 0))
        mk_btn_outline(atb, "⤓ PNG", self.save_arch_chart, C['cyan']).pack(
            side='right', padx=(4, 0))
        mk_btn_outline(atb, "📄 PDF", self.export_arch_pdf, C['orange']).pack(
            side='right', padx=(4, 0))
        mk_btn_outline(atb, "📊 STATS", self.show_arch_stats, C['purple']).pack(
            side='right', padx=(4, 0))
        mk_btn_outline(atb, "📁", self.open_log_folder, C['dim']).pack(
            side='right', padx=(4, 0))
        # Tryb osi X
        self.arch_align = tk.BooleanVar(value=True)
        tk.Checkbutton(atb, text="align from t=0", variable=self.arch_align,
                      command=self._redraw_arch, bg=C['panel'], fg=C['dim'],
                      selectcolor=C['bg2'], activebackground=C['panel'],
                      font=(FONT, fsz(8)), bd=0, highlightthickness=0).pack(side='left')

        # Panel nastaw przebiegu (pokazuje SP/rampy/PID zaznaczonego cyklu)
        self.arch_settings = tk.Frame(cf, bg=C['bg2'])
        self.arch_settings.pack(fill='x', padx=8, pady=(0, 8))
        self.arch_settings_lbl = tk.Label(self.arch_settings, text="",
                                         bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(9)),
                                         anchor='w', justify='left')
        self.arch_settings_lbl.pack(fill='x', padx=10, pady=6)

        self.refresh_arch()
        # Narysuj pusty wykres od razu - inicjalizuje canvas i toolbar
        self._redraw_arch()

    def _cycle_display_name(self, path):
        """Czytelna nazwa cyklu: usuwa prefiks c_/cykl_ i zamienia _ na spacje"""
        from pathlib import Path as _P
        s = _P(path).stem
        if s.startswith('cykl_'): s = s[5:]
        elif s.startswith('c_'): s = s[2:]
        return s.replace('_', ' ')

    def _bind_tooltip(self, widget, text):
        """Prosty tooltip pokazujacy pelny tekst po najechaniu"""
        tip = {'win': None}
        def show(e):
            if tip['win']: return
            tw = tk.Toplevel(widget)
            tw.wm_overrideredirect(True)
            tw.wm_geometry(f"+{e.x_root+10}+{e.y_root+10}")
            tk.Label(tw, text=text, bg='#1a1c1f', fg='#e8e8e8',
                     font=(FONT, fsz(8)), padx=6, pady=3,
                     relief='solid', bd=1).pack()
            tip['win'] = tw
        def hide(e):
            if tip['win']:
                tip['win'].destroy(); tip['win'] = None
        widget.bind('<Enter>', show)
        widget.bind('<Leave>', hide)

    def refresh_arch(self):
        # Wyczysc liste checkboxow
        for w in self.arch_items.winfo_children():
            w.destroy()
        self.arch_vars = {}
        files = sorted([f for f in self.log_dir.glob("*.csv") if (f.name.startswith("cykl_") or f.name.startswith("c_")) and not f.name.startswith("_tmp")],
                       key=lambda f: f.stat().st_mtime, reverse=True)
        # Paleta kolorow dla porownania
        self._arch_colors = [C['blue'], C['orange'], C['green'], C['red'],
                            C['cyan'], C['purple'], C['yellow'], '#ff8fab']
        if not files:
            tk.Label(self.arch_items, text="No saved cycles yet.\nRun a cycle and give it a name.",
                     bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(9)), justify='left').pack(
                     anchor='w', padx=12, pady=12)
            return

        # Grupowanie po dacie (dzien modyfikacji pliku)
        from datetime import datetime as _dt
        import time as _time
        groups = {}
        for f in files:
            day = _dt.fromtimestamp(f.stat().st_mtime).strftime("%Y-%m-%d")
            groups.setdefault(day, []).append(f)

        today = _dt.now().strftime("%Y-%m-%d")
        i = 0
        for day, day_files in groups.items():
            # Naglowek grupy (data)
            day_label = "Today" if day == today else day
            hdr = tk.Frame(self.arch_items, bg=C['panel'])
            hdr.pack(fill='x', pady=(6, 1))
            tk.Label(hdr, text=f"▸ {day_label}  ({len(day_files)})", bg=C['panel'],
                     fg=C['cyan'], font=(FONT, fsz(8), 'bold'), anchor='w').pack(
                     side='left', padx=8, pady=3)
            # Pliki w grupie
            for f in day_files:
                row = tk.Frame(self.arch_items, bg=C['bg2'])
                row.pack(fill='x', pady=1)
                var = tk.BooleanVar(value=False)
                self.arch_vars[str(f)] = var
                col = self._arch_colors[i % len(self._arch_colors)]
                i += 1
                # KOLEJNOSC PACK: kosz NAJPIERW (side=right) = zawsze widoczny,
                # potem kropka (left), na koncu checkbox wypelnia srodek.
                # Dzieki temu dluga nazwa nie zaslania kosza.
                delb = tk.Button(row, text="🗑", command=lambda p=f: self._delete_cycle(p),
                                bg=C['bg2'], fg=C['red'], font=(FONT, fsz(11), 'bold'),
                                relief='flat', cursor='hand2', bd=0, padx=10, pady=2,
                                activebackground=C['red'], activeforeground='#fff')
                delb.pack(side='right', padx=(2, 6))
                dot = tk.Frame(row, bg=col, width=10, height=10)
                dot.pack(side='left', padx=(8, 4))
                dot.pack_propagate(False)
                # Nazwa skrocona jesli za dluga (zeby nie rozpychala wiersza)
                full_name = self._cycle_display_name(f)
                disp_name = full_name if len(full_name) <= 22 else full_name[:20] + "…"
                cb = tk.Checkbutton(row, text=disp_name,
                                   variable=var, command=self._redraw_arch,
                                   bg=C['bg2'], fg=C['text'], selectcolor=C['panel'],
                                   activebackground=C['bg2'], activeforeground=col,
                                   font=(FONT, fsz(9)), bd=0, highlightthickness=0,
                                   anchor='w')
                # Pelna nazwa w tooltipie (po najechaniu)
                if len(full_name) > 22:
                    self._bind_tooltip(cb, full_name)
                cb.pack(side='left', fill='x', expand=True)

    def _delete_cycle(self, path):
        """Usun plik cyklu z archiwum (z potwierdzeniem)"""
        from pathlib import Path as _P
        name = self._cycle_display_name(_P(path))
        if messagebox.askyesno("Delete cycle",
                f"Permanently delete this cycle?\n\n{name}\n\nThis cannot be undone."):
            try:
                _P(path).unlink()
                self.refresh_arch()
                self._redraw_arch()
            except Exception as e:
                messagebox.showerror("Delete error", str(e))

    def _cycle_settings(self, path):
        """Odczytaj nastawy przebiegu z CSV: target SP, rampy, PID. Zwraca dict lub None"""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                rows = list(csv.DictReader(f))
        except Exception:
            return None
        # Znajdz pierwszy poprawny wiersz danych
        valid = [r for r in rows if r.get('czas_s', '').replace('.','').replace('-','').isdigit()]
        if not valid:
            return None
        s = {}
        # Target setpoint - najczestsza wartosc setpoint_cel (cel koncowy)
        try:
            sps = [float(r['setpoint_cel']) for r in valid if r.get('setpoint_cel')]
            s['target'] = max(set(sps), key=sps.count) if sps else None
        except: s['target'] = None
        # PID - z pierwszego wiersza (stale przez przebieg lub z kalibracji)
        try:
            s['kp'] = float(valid[0].get('Kp', 0))
            s['ki'] = float(valid[0].get('Ki', 0))
            s['kd'] = float(valid[0].get('Kd', 0))
        except: s['kp'] = s['ki'] = s['kd'] = None
        # Oszacuj rampe z nachylenia setpoint_aktywny na poczatku
        try:
            t0 = float(valid[0]['czas_s'])
            sa0 = float(valid[0]['setpoint_aktywny'])
            # znajdz punkt ~10s pozniej
            ramp = None
            for r in valid:
                tt = float(r['czas_s'])
                if tt - t0 >= 5:
                    sa = float(r['setpoint_aktywny'])
                    dt_min = (tt - t0) / 60.0
                    if dt_min > 0:
                        ramp = abs(sa - sa0) / dt_min
                    break
            s['ramp'] = ramp
        except: s['ramp'] = None
        return s

    def _arch_clear_sel(self):
        """Odznacz wszystkie cykle"""
        for v in self.arch_vars.values():
            v.set(False)
        self._redraw_arch()

    def _load_cycle_data(self, path):
        """Wczytaj dane cyklu z CSV (odporne na komentarze). Zwraca (t,temp,spt,pwm) lub None"""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = list(csv.DictReader(f))
        except Exception:
            return None
        t, temp, spt, pwm = [], [], [], []
        temp2 = []
        sa_list = []
        for r in data:
            cz = r.get('czas_s', '')
            if not cz or cz.startswith('#'):
                continue
            try:
                tt = float(cz)
                tm = float(r.get('temperatura_C', 'nan'))
                sp = float(r.get('setpoint_cel', 'nan'))
            except (ValueError, TypeError):
                continue
            t.append(tt); temp.append(tm); spt.append(sp)
            # setpoint aktywny (rampa) - osobno do wykresu
            try:
                sa_list.append(float(r.get('setpoint_aktywny', 'nan')))
            except:
                sa_list.append(None)
            try:
                pwm.append(float(r.get('PWM_%', r.get('PWM', 0))))
            except:
                pwm.append(0)
            # temp2 - druga termopara (opcjonalna kolumna)
            try:
                t2v = r.get('temperatura2_C', '')
                temp2.append(float(t2v) if t2v else None)
            except:
                temp2.append(None)
        if not t:
            return None
        # Dolacz temp2 i setpoint aktywny jako atrybuty (kompatybilnie - zwracamy 4)
        self._last_temp2 = temp2
        self._last_sa = sa_list
        return t, temp, spt, pwm

    def _compute_stats(self, data):
        """Oblicz pelne statystyki przebiegu. data=(t,temp,spt,pwm). Zwraca dict."""
        import statistics
        t, temp, spt, pwm = data
        st = {}
        st['tmin'] = min(temp)
        st['tmax'] = max(temp)
        st['duration'] = t[-1] - t[0] if len(t) > 1 else 0
        st['target'] = spt[-1] if spt else 0

        # Srednie tempo narastania (start -> max)
        idx_max = temp.index(st['tmax'])
        rise_time = t[idx_max] - t[0] if idx_max > 0 else 0
        st['avg_rise'] = (st['tmax'] - temp[0]) / (rise_time/60.0) if rise_time > 5 else 0

        # Overshoot - ile temp przekroczyla target (w fazie ustalonej)
        target = st['target']
        st['overshoot'] = max(0, st['tmax'] - target) if target else 0

        # Czas ustalania - kiedy temp weszla i zostala w +/-1C od target
        st['settle_time'] = None
        if target:
            band = 1.0
            for i, tm in enumerate(temp):
                if abs(tm - target) <= band:
                    # sprawdz czy zostala w pasie do konca (lub przez 80% reszty)
                    rest = temp[i:]
                    in_band = sum(1 for x in rest if abs(x-target) <= band)
                    if in_band >= len(rest)*0.8:
                        st['settle_time'] = t[i] - t[0]
                        break

        # Blad ustalony - srednie odchylenie w ostatnich 20% probek
        n = len(temp)
        tail = temp[int(n*0.8):] if n > 5 else temp
        if target and tail:
            st['steady_error'] = statistics.mean(abs(x - target) for x in tail)
        else:
            st['steady_error'] = 0

        # Max odchylenie od setpointu (rampy) - jak dobrze nadazal
        devs = [abs(temp[i] - spt[i]) for i in range(len(temp))]
        st['max_dev'] = max(devs) if devs else 0

        # Odchylenie standardowe szumu - w fazie ustalonej (ostatnie 20%)
        # To miara jakosci pomiaru (szum termopary)
        if len(tail) > 2:
            st['noise_std'] = statistics.stdev(tail)
        else:
            st['noise_std'] = 0

        return st

    def _redraw_arch(self):
        """Narysuj wszystkie zaznaczone cykle (porownanie)"""
        selected = [(p, v) for p, v in self.arch_vars.items() if v.get()]
        self.ax_a.clear()
        self.ax_a.set_facecolor(C['panel2'])

        if not selected:
            self.ax_a.text(0.5, 0.5, "Tick one or more cycles to display",
                          ha='center', va='center', color=C['dim2'],
                          fontsize=11, transform=self.ax_a.transAxes)
            self.cv_a.draw()
            if hasattr(self, 'arch_settings_lbl'):
                self.arch_settings_lbl.config(text="")
            return

        files = sorted([f for f in self.log_dir.glob("*.csv") if (f.name.startswith("cykl_") or f.name.startswith("c_")) and not f.name.startswith("_tmp")], reverse=True)
        file_order = {str(f): i for i, f in enumerate(files)}
        align = self.arch_align.get()

        multi = len(selected) > 1
        # Sprawdz maksymalny czas (do wyboru jednostki osi: s czy min)
        max_t = 0
        for path, _ in selected:
            d = self._load_cycle_data(path)
            if d and d[0]:
                span = d[0][-1] - (d[0][0] if align else 0)
                max_t = max(max_t, span)
        use_min = max_t > 180  # powyzej 3 min -> osi w minutach
        tdiv = 60.0 if use_min else 1.0

        for path, _ in selected:
            d = self._load_cycle_data(path)
            if not d: continue
            t, temp, spt, pwm = d
            sa = getattr(self, '_last_sa', None)
            ci = file_order.get(path, 0) % len(self._arch_colors)
            col = self._arch_colors[ci]
            # Os X: od zera (align) albo absolutna, przeliczona na wybrana jednostke
            t0 = t[0] if align else 0
            tx = [(x - t0) / tdiv for x in t]
            from pathlib import Path as _P
            name = self._cycle_display_name(_P(path))
            # Przy jednym cyklu pokaz target + setpoint-ramp + temp; przy wielu tylko temp
            if not multi:
                self.ax_a.plot(tx, spt, color=C['orange'], lw=1.2, ls='--',
                              label='target', alpha=0.55)
                # Setpoint aktywny (rampa) - kropkowana linia
                if sa and any(v is not None for v in sa):
                    txs = [tx[i] for i in range(len(sa)) if i < len(tx) and sa[i] is not None]
                    sas = [v for v in sa if v is not None]
                    if sas:
                        self.ax_a.plot(txs, sas, color=C['cyan'], lw=1.1, ls=':',
                                      label='setpoint (ramp)', alpha=0.7)
                self.ax_a.plot(tx, temp, color=col, lw=2, label='temp (gal)')
                # Druga termopara jesli dostepna
                t2 = getattr(self, '_last_temp2', None)
                if t2 and any(v is not None for v in t2):
                    tx2 = [tx[i] for i in range(len(t2)) if i < len(tx) and t2[i] is not None]
                    ty2 = [v for v in t2 if v is not None]
                    if ty2:
                        self.ax_a.plot(tx2, ty2, color=C['purple'], lw=1.5,
                                      label='temp 2', alpha=0.8)
            else:
                self.ax_a.plot(tx, temp, color=col, lw=1.8, label=name)

        # Opis osi czasu - z jednostka i informacja o wyrownaniu
        unit_txt = 'minutes' if use_min else 'seconds'
        xlabel = f'time [{unit_txt}]'
        if align: xlabel += '  ·  aligned from 0'
        self.ax_a.set_xlabel(xlabel, color=C['dim'], fontsize=9)
        self.ax_a.set_ylabel('temperature [°C]', color=C['dim'], fontsize=9)
        self.ax_a.tick_params(colors=C['dim'], labelsize=8)
        self.ax_a.legend(facecolor=C['panel'], edgecolor=C['border'],
                        labelcolor=C['dim'], fontsize=8, loc='best')
        self.ax_a.grid(True, alpha=0.3, color=C['grid'])
        for sp in self.ax_a.spines.values():
            sp.set_color(C['border'])

        # Tytul: statystyki (jeden cykl) lub liczba porownywanych
        if not multi:
            d = self._load_cycle_data(selected[0][0])
            if d:
                t, temp, spt, pwm = d
                tmin, tmax = min(temp), max(temp)
                dur = t[-1] - t[0] if len(t) > 1 else 0
                idx_max = temp.index(tmax)
                rise_time = t[idx_max] - t[0] if idx_max > 0 else 0
                avg_rise = (tmax - temp[0]) / (rise_time / 60.0) if rise_time > 5 else 0
                m = int(dur // 60); s2 = int(dur % 60)
                self.ax_a.set_title(
                    f"{tmin:.1f}-{tmax:.1f}°C · {m}m{s2}s · avg rise {avg_rise:.2f}°C/min",
                    color=C['dim'], fontsize=9, loc='left')
        else:
            self.ax_a.set_title(f"Comparing {len(selected)} cycles",
                              color=C['dim'], fontsize=9, loc='left')
        self.fig_a.tight_layout()
        self.cv_a.draw()

        # Panel nastaw przebiegu (tylko przy jednym zaznaczonym cyklu)
        if hasattr(self, 'arch_settings_lbl'):
            if not multi:
                cs = self._cycle_settings(selected[0][0])
                if cs:
                    def fmt(v, suf=''):
                        return f"{v:.1f}{suf}" if v is not None else "?"
                    txt = (f"SETTINGS:   target {fmt(cs['target'],'°C')}   ·   "
                           f"ramp ~{fmt(cs['ramp'],'°C/min')}   ·   "
                           f"PID  Kp {fmt(cs['kp'])}  Ki {fmt(cs['ki'])}  Kd {fmt(cs['kd'])}")
                    self.arch_settings_lbl.config(text=txt)
                else:
                    self.arch_settings_lbl.config(text="")
            else:
                self.arch_settings_lbl.config(text=f"({len(selected)} cycles selected — settings shown for single selection)")

    def _selected_arch_path(self):
        """Pierwszy zaznaczony cykl (do eksportu)"""
        for p, v in self.arch_vars.items():
            if v.get():
                from pathlib import Path as _P
                return _P(p)
        return None

    def export_arch_csv(self):
        """Eksportuj zaznaczony cykl CSV"""
        path = self._selected_arch_path()
        if not path:
            messagebox.showinfo("No selection", "Tick a cycle in the list first.")
            return
        try:
            from tkinter import filedialog
            dest = filedialog.asksaveasfilename(
                title="Export cycle CSV", defaultextension=".csv",
                initialfile=path.name,
                filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
            if dest:
                import shutil
                shutil.copy(path, dest)
                messagebox.showinfo("Exported", f"Cycle exported to:\n{dest}")
        except Exception as e:
            messagebox.showerror("Export error", str(e))

    def save_arch_chart(self):
        """Zapisz aktualny wykres (z porownaniem) jako obraz"""
        if not any(v.get() for v in self.arch_vars.values()):
            messagebox.showinfo("No selection", "Tick at least one cycle first.")
            return
        try:
            from tkinter import filedialog
            dest = filedialog.asksaveasfilename(
                title="Save chart as image", defaultextension=".png",
                initialfile="comparison.png",
                filetypes=[("PNG image", "*.png"), ("PDF", "*.pdf"), ("SVG", "*.svg")])
            if dest:
                self.fig_a.savefig(dest, dpi=150, facecolor=C['panel'],
                                   bbox_inches='tight')
                messagebox.showinfo("Saved", f"Chart saved to:\n{dest}")
        except Exception as e:
            messagebox.showerror("Save error", str(e))

    def open_log_folder(self):
        """Otworz folder z logami"""
        try:
            import subprocess
            p = str(self.log_dir)
            if sys.platform == 'win32':
                os.startfile(p)
            elif sys.platform == 'darwin':
                subprocess.run(['open', p])
            else:
                subprocess.run(['xdg-open', p])
        except Exception:
            messagebox.showinfo("Folder", f"Logs are in:\n{self.log_dir}")

    def load_arch(self, evt=None):
        """Zachowane dla kompatybilnosci - przekierowuje do redraw"""
        self._redraw_arch()

    def show_arch_stats(self):
        """Pokaz okno ze statystykami zaznaczonego cyklu"""
        path = self._selected_arch_path()
        if not path:
            messagebox.showinfo("No selection", "Tick a cycle in the list first.")
            return
        data = self._load_cycle_data(path)
        if not data:
            messagebox.showerror("Error", "Could not load cycle data.")
            return
        st = self._compute_stats(data)

        win = tk.Toplevel(self.root)
        win.title("Cycle statistics")
        win.configure(bg=C['bg'])
        win.geometry("440x520")
        win.transient(self.root)
        tk.Frame(win, bg=C['purple'], height=4).pack(fill='x')
        inner = tk.Frame(win, bg=C['bg'])
        inner.pack(fill='both', expand=True, padx=24, pady=20)

        from pathlib import Path as _P
        tk.Label(inner, text="CYCLE STATISTICS", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(14), 'bold')).pack(anchor='w')
        tk.Label(inner, text=self._cycle_display_name(_P(path)), bg=C['bg'], fg=C['dim'],
                 font=(FONT, fsz(9))).pack(anchor='w', pady=(2, 16))

        def settle_str():
            return f"{st['settle_time']:.0f} s" if st['settle_time'] is not None else "not reached"

        rows = [
            ("Temperature range", f"{st['tmin']:.1f} – {st['tmax']:.1f} °C", C['blue']),
            ("Target", f"{st['target']:.1f} °C", C['orange']),
            ("Duration", f"{int(st['duration']//60)}m {int(st['duration']%60)}s", C['text']),
            ("Avg rise rate", f"{st['avg_rise']:.2f} °C/min", C['cyan']),
            ("─", "", None),
            ("Overshoot", f"{st['overshoot']:.2f} °C", C['red'] if st['overshoot']>1 else C['green']),
            ("Settling time (±1°C)", settle_str(), C['text']),
            ("Steady-state error", f"{st['steady_error']:.3f} °C", C['text']),
            ("Max deviation from ramp", f"{st['max_dev']:.2f} °C", C['text']),
            ("─", "", None),
            ("Noise σ (measurement quality)", f"±{st['noise_std']:.3f} °C", 
             C['green'] if st['noise_std']<0.2 else C['yellow'] if st['noise_std']<0.5 else C['red']),
        ]
        for label, val, col in rows:
            if label == "─":
                tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=8)
                continue
            r = tk.Frame(inner, bg=C['bg2'])
            r.pack(fill='x', pady=2)
            tk.Label(r, text=label, bg=C['bg2'], fg=C['dim'],
                     font=(FONT, fsz(9)), anchor='w').pack(side='left', padx=10, pady=6)
            tk.Label(r, text=val, bg=C['bg2'], fg=col or C['text'],
                     font=(FONT, fsz(10), 'bold'), anchor='e').pack(side='right', padx=10)

        # Interpretacja szumu
        noise = st['noise_std']
        interp = ("Excellent - low noise" if noise < 0.2 else
                  "Moderate noise" if noise < 0.5 else
                  "High noise - check shielding/grounding")
        tk.Label(inner, text=f"Noise: {interp}", bg=C['bg'],
                 fg=C['dim2'], font=(FONT, fsz(8)), wraplength=380,
                 justify='left').pack(anchor='w', pady=(12, 0))

    def export_arch_pdf(self):
        """Generuj raport PDF: wykres + statystyki + nastawy + data"""
        path = self._selected_arch_path()
        if not path:
            messagebox.showinfo("No selection", "Tick a cycle in the list first.")
            return
        data = self._load_cycle_data(path)
        if not data:
            messagebox.showerror("Error", "Could not load cycle data.")
            return

        try:
            from tkinter import filedialog
            from pathlib import Path as _P
            dest = filedialog.asksaveasfilename(
                title="Save PDF report", defaultextension=".pdf",
                initialfile=f"{_P(path).stem}_report.pdf",
                filetypes=[("PDF report", "*.pdf")])
            if not dest:
                return
            self._build_pdf_report(path, data, dest)
            messagebox.showinfo("Report saved", f"PDF report saved to:\n{dest}")
        except Exception as e:
            messagebox.showerror("PDF error", f"Could not create report:\n{e}")

    def _build_pdf_report(self, path, data, dest):
        """Zbuduj raport PDF za pomoca matplotlib (bez dodatkowych bibliotek)"""
        from matplotlib.backends.backend_pdf import PdfPages
        from matplotlib.figure import Figure
        from pathlib import Path as _P
        import datetime

        t, temp, spt, pwm = data
        st = self._compute_stats(data)
        # Os czasu od zera
        t0 = t[0]
        tx = [x - t0 for x in t]

        with PdfPages(dest) as pdf:
            fig = Figure(figsize=(8.27, 11.69))  # A4 pionowo
            fig.patch.set_facecolor('white')

            # Naglowek
            fig.text(0.5, 0.96, "Peltier Control - Cycle Report", ha='center',
                     fontsize=16, fontweight='bold')
            ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
            fig.text(0.5, 0.935, f"{self._cycle_display_name(_P(path))}  ·  generated {ts}",
                     ha='center', fontsize=9, color='gray')

            # Wykres temperatury (gorna polowa)
            ax1 = fig.add_axes([0.1, 0.55, 0.82, 0.32])
            ax1.plot(tx, spt, color='#e8833a', lw=1.2, ls='--', label='target', alpha=0.7)
            ax1.plot(tx, temp, color='#2b7fd4', lw=1.8, label='temperature')
            ax1.set_xlabel('time [s]', fontsize=9)
            ax1.set_ylabel('temperature [°C]', fontsize=9)
            ax1.legend(fontsize=9, loc='best')
            ax1.grid(True, alpha=0.3)
            ax1.set_title('Temperature profile', fontsize=11, loc='left')

            # Wykres PWM (pod spodem)
            ax2 = fig.add_axes([0.1, 0.40, 0.82, 0.10])
            ax2.fill_between(tx, pwm, color='#3ea662', alpha=0.5)
            ax2.set_xlabel('time [s]', fontsize=8)
            ax2.set_ylabel('PWM [%]', fontsize=8)
            ax2.grid(True, alpha=0.3)

            # Tabela statystyk (dol)
            def settle_str():
                return f"{st['settle_time']:.0f} s" if st['settle_time'] is not None else "not reached"
            stats_lines = [
                ("STATISTICS", ""),
                ("Temperature range", f"{st['tmin']:.1f} - {st['tmax']:.1f} °C"),
                ("Target", f"{st['target']:.1f} °C"),
                ("Duration", f"{int(st['duration']//60)}m {int(st['duration']%60)}s"),
                ("Average rise rate", f"{st['avg_rise']:.2f} °C/min"),
                ("Overshoot", f"{st['overshoot']:.2f} °C"),
                ("Settling time (±1°C)", settle_str()),
                ("Steady-state error", f"{st['steady_error']:.3f} °C"),
                ("Max deviation from ramp", f"{st['max_dev']:.2f} °C"),
                ("Noise σ (quality)", f"±{st['noise_std']:.3f} °C"),
            ]
            y = 0.32
            for label, val in stats_lines:
                if label == "STATISTICS":
                    fig.text(0.1, y, label, fontsize=11, fontweight='bold')
                else:
                    fig.text(0.12, y, label, fontsize=9, color='#333')
                    fig.text(0.55, y, val, fontsize=9, fontweight='bold')
                y -= 0.025

            pdf.savefig(fig)


    # ────────────────────────────────────────────────────
    #  TICK + WYKRES
    # ────────────────────────────────────────────────────
    def tick(self):
        try:
            # SELF-TUNE/kalibracja zmienily PID - zaktualizuj suwaki (tabele)
            stp = getattr(self, '_st_pid_update', None)
            if stp is not None:
                self._st_pid_update = None
                try:
                    if hasattr(self, 'sl_kp'): self.sl_kp.set(stp[0], silent=True)
                    if hasattr(self, 'sl_ki'): self.sl_ki.set(stp[1], silent=True)
                    if hasattr(self, 'sl_kd'): self.sl_kd.set(stp[2], silent=True)
                except: pass
            rows = []
            while not self.data_queue.empty():
                rows.append(self.data_queue.get_nowait())
            for row in rows:
                now2, temp, st, sa, pwm, kp, ki, kd, state, prev = row
                self.t.append(now2); self.temp.append(temp)
                self.spt.append(st); self.spa.append(sa)
                self.pwm.append(pwm); self.kp.append(kp)
                self.ki.append(ki); self.kd.append(kd)
                self.states.append(state)
                # Ogranicz dlugosc buforow
                if len(self.t) > self.maxlen:
                    for a in [self.t, self.temp, self.spt, self.spa,
                              self.pwm, self.kp, self.ki, self.kd, self.states]:
                        del a[0]
                # Start cyklu
                if state == 'AUTO' and prev != 'AUTO' and not self.cyc_on:
                    self._cyc_start(temp)
                    # Rozpocznij sledzenie dotarcia do setpointu
                    self.reach_start_t = now2
                    self.reach_start_temp = temp
                    self.reach_target = st
                    self.reach_done = False
                    self.reach_time = None
                    self.reach_avg_rate = None
                    self.last_setpoint_target = st
                elif self.cyc_on and state == 'MAN' and prev in ('AUTO', 'COOLDOWN', 'FREEZE', 'FREEZE_READY'):
                    # Koniec cyklu - przejscie z pracy do MAN (STOP).
                    # Bez cooldown teraz idzie prosto AUTO->MAN.
                    self.cyc_stop("done")

                # Wykrywanie zmiany docelowego setpointu podczas pracy (nowe dotarcie)
                if state == 'AUTO' and self.last_setpoint_target is not None:
                    if abs(st - self.last_setpoint_target) > 0.5:
                        # Setpoint zmieniony - zacznij liczyc od nowa
                        self.reach_start_t = now2
                        self.reach_start_temp = temp
                        self.reach_target = st
                        self.reach_done = False
                        self.last_setpoint_target = st

                # Sprawdz czy osiagnieto setpoint (w granicach 0.5C, stabilnie)
                if (state == 'AUTO' and not self.reach_done
                        and self.reach_target is not None
                        and self.reach_start_t is not None):
                    if abs(temp - self.reach_target) <= 0.5:
                        self.reach_done = True
                        self.reach_time = now2 - self.reach_start_t
                        delta = self.reach_target - self.reach_start_temp
                        dT = abs(delta)
                        if self.reach_time > 0:
                            self.reach_avg_rate = dT / (self.reach_time / 60.0)
                        # Kierunek przejscia: grzanie czy chlodzenie
                        self.reach_dir = "HEAT" if delta > 0 else "COOL"
                        # Zapamietaj statystyki dotarcia dla tego cyklu
                        self._last_reach_summary = {
                            'target': self.reach_target,
                            'time_s': self.reach_time,
                            'avg_rate': self.reach_avg_rate,
                            'dir': self.reach_dir,
                        }
        except Exception as e:
            print(f"tick err: {e}")

        if self.t:
            try: self.update_cards()
            except Exception as e: print(f"cards err: {e}")
            try: self.draw_chart()
            except Exception as e: print(f"chart err: {e}")

        self.root.after(250, self.tick)

    def update_cards(self):
        if not self.t: return
        temp = self.temp[-1]; spt = self.spt[-1]; pwm = self.pwm[-1]
        self.cards['temp']['val'].config(text=f"{temp:.2f}")
        # Karta drugiej termopary
        t2 = getattr(self, '_latest_temp2', None)
        if 'temp2' in self.cards:
            self.cards['temp2']['val'].config(text=f"{t2:.2f}" if t2 is not None else "--")
        self.cards['sp']['val'].config(text=f"{spt:.1f}")
        # AVG RATE - srednie tempo przejscia (od startu dochodzenia do teraz)
        # Bardziej uzyteczne niz chwilowe - pokazuje realna srednia rampe
        avg_rate = 0.0
        if (self.reach_start_t is not None and self.reach_start_temp is not None
                and self.t and self.cur_state == 'AUTO'):
            elapsed = self.t[-1] - self.reach_start_t
            if elapsed > 2:  # min 2s zeby uniknac dzielenia przez male liczby
                avg_rate = (temp - self.reach_start_temp) / (elapsed / 60.0)
        # Po dotarciu pokaz finalna srednia
        if self.reach_done and self.reach_avg_rate is not None:
            d = getattr(self, 'reach_dir', '')
            sign = 1 if d == 'HEAT' else -1
            avg_rate = sign * self.reach_avg_rate
        self.cards['rate']['val'].config(text=f"{avg_rate:+.1f}")
        # PWM + kierunek (HEAT/COOL/HOLD widoczny w jednostce)
        diff = spt - temp
        arrow = "% ▲HEAT" if diff > 0.3 else ("% ▼COOL" if diff < -0.3 else "% ●HOLD")
        self.cards['pwm']['val'].config(text=f"{pwm:.0f}")
        # Kolor kierunku
        acol = C['red'] if diff > 0.3 else (C['cyan'] if diff < -0.3 else C['dim2'])
        self.cards['pwm']['unit_lbl'].config(text=" " + arrow, fg=acol)

        # Statystyki dotarcia / status FREEZE
        if hasattr(self, 'reach_lbl'):
            # FREEZE - priorytet (najwazniejszy komunikat dla usera)
            if self.cur_state == 'FREEZE_READY':
                self.reach_lbl.config(text="❄ GAL SOLID — ready to swap sample", fg=C['cyan'])
            elif self.cur_state == 'FREEZE':
                self.reach_lbl.config(text=f"❄ Freezing gal → hold 20°C", fg=C['cyan'])
            elif self.reach_done and self.reach_time is not None:
                m = int(self.reach_time // 60); s = int(self.reach_time % 60)
                tstr = f"{m}m {s}s" if m > 0 else f"{s}s"
                rate_str = f"{self.reach_avg_rate:.2f}" if self.reach_avg_rate else "?"
                d = getattr(self, 'reach_dir', '')
                dcol = C['red'] if d == 'HEAT' else C['cyan']
                self.reach_lbl.config(
                    text=f"✓ {d} REACHED in {tstr} · avg {rate_str}°C/min", fg=dcol)
            elif (self.cur_state == 'AUTO' and self.reach_start_t is not None
                  and not self.reach_done):
                # W trakcie dochodzenia - pokaz uplyniety czas
                if self.t:
                    elapsed = self.t[-1] - self.reach_start_t
                    m = int(elapsed // 60); s = int(elapsed % 60)
                    tstr = f"{m}m {s}s" if m > 0 else f"{s}s"
                    self.reach_lbl.config(
                        text=f"→ reaching {self.reach_target:.1f}°C · {tstr}", fg=C['yellow'])
            else:
                self.reach_lbl.config(text="")
        if not self.t: return
        # Pauza - nie odswiezaj (pozwala przyblizyc/obejrzec zatrzymany wykres)
        if self.chart_paused:
            return
        t = self.t; temp = self.temp; spt = self.spt; spa = self.spa; pwm = self.pwm

        # Okno czasowe - pokaz tylko ostatnie N sekund jesli ustawione
        if self.chart_window > 0 and len(t) > 1:
            t_now = t[-1]
            cutoff = t_now - self.chart_window
            # Znajdz indeks od ktorego pokazac
            i0 = 0
            for i in range(len(t) - 1, -1, -1):
                if t[i] < cutoff:
                    i0 = i
                    break
            t = t[i0:]; temp = temp[i0:]; spt = spt[i0:]
            spa = spa[i0:]; pwm = pwm[i0:]

        self.ax1.clear()
        self.ax1.set_facecolor(C['panel2'])
        # target final (przerywana pomaranczowa)
        self.ax1.plot(t, spt, color=C['orange'], lw=1.3, ls='--', label='target', alpha=0.7)
        # actual setpoint - rampa (kropkowana cyan) - pokazuje jak setpoint pelznie
        self.ax1.plot(t, spa, color=C['cyan'], lw=1.5, ls=':', label='setpoint (ramp)')
        # temperatura rzeczywista (gruba niebieska)
        self.ax1.plot(t, temp, color=C['blue'], lw=2.2, label='temp')
        self.ax1.set_ylabel('°C', color=C['dim'], fontsize=9)
        self.ax1.tick_params(colors=C['dim'], labelsize=8, length=0)
        self.ax1.grid(True, axis='y', alpha=0.35, color=C['grid'])
        for sp in ['top', 'right']: self.ax1.spines[sp].set_visible(False)
        for sp in ['left', 'bottom']: self.ax1.spines[sp].set_color(C['border'])
        leg = self.ax1.legend(facecolor=C['panel'], edgecolor=C['border'],
                             labelcolor=C['dim'], fontsize=8, loc='upper right')

        self.ax2.clear()
        self.ax2.set_facecolor(C['panel2'])
        self.ax2.fill_between(t, 0, pwm, color=C['green'], alpha=0.3)
        self.ax2.plot(t, pwm, color=C['green'], lw=1.5)
        self.ax2.set_ylabel('PWM %', color=C['dim'], fontsize=9)
        self.ax2.set_xlabel('time [s]', color=C['dim'], fontsize=9)
        self.ax2.set_ylim(-105, 105)
        self.ax2.tick_params(colors=C['dim'], labelsize=8, length=0)
        self.ax2.grid(True, axis='y', alpha=0.35, color=C['grid'])
        for sp in ['top', 'right']: self.ax2.spines[sp].set_visible(False)
        for sp in ['left', 'bottom']: self.ax2.spines[sp].set_color(C['border'])

        self.cv.draw_idle()

    # ────────────────────────────────────────────────────
    #  CSV CYKLU
    # ────────────────────────────────────────────────────
    def _cyc_start(self, temp0):
        self.cyc_on = True
        self.cyc_t0 = time.time()
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        # Plik tymczasowy - nazwe nada uzytkownik po STOP
        self.cyc_ts = ts
        self.cyc_fn = self.log_dir / f"_tmp_cykl_{ts}.csv"
        self.cyc_file = open(self.cyc_fn, 'w', newline='', encoding='utf-8')
        self.cyc_wr = csv.writer(self.cyc_file)
        self.cyc_wr.writerow(['czas_s', 'temperatura_C', 'setpoint_aktywny',
                              'setpoint_cel', 'PWM', 'PWM_%', 'Kp', 'Ki', 'Kd', 'stan',
                              'temperatura2_C'])
        self.cyc_rows = 0
        print(f"CYC START T={temp0:.1f}")

    def cyc_log(self, t, temp, sa, st, pwm, kp, ki, kd, state, temp2=None):
        if self.cyc_wr:
            try:
                t2str = f"{temp2:.2f}" if temp2 is not None else ""
                self.cyc_wr.writerow([f"{t:.2f}", f"{temp:.2f}", f"{sa:.2f}",
                                     f"{st:.2f}", pwm, f"{pwm*100/255:.1f}",
                                     f"{kp:.3f}", f"{ki:.4f}", f"{kd:.3f}", state, t2str])
                self.cyc_file.flush()
                self.cyc_rows += 1
            except: pass

    def cyc_stop(self, reason=""):
        if self.cyc_file:
            try: self.cyc_file.close()
            except: pass
        had_data = self.cyc_on and getattr(self, 'cyc_rows', 0) > 0
        tmp_path = self.cyc_fn
        self.cyc_on = False; self.cyc_file = None; self.cyc_wr = None
        print(f"CYC STOP: {reason} ({getattr(self,'cyc_rows',0)} próbek)")
        # Zapytaj o nazwe i zapisz do archiwum (w watku GUI)
        if had_data and tmp_path and tmp_path.exists():
            self.root.after(0, lambda: self._ask_save_name(tmp_path))
        elif tmp_path and tmp_path.exists():
            # Brak danych - usun plik tymczasowy
            try: tmp_path.unlink()
            except: pass

    def _ask_save_name(self, tmp_path):
        """Okno z pytaniem o nazwe cyklu do archiwum"""
        SaveCycleDialog(self.root, self, tmp_path)

    def save_cycle_as(self, tmp_path, name):
        """Zapisz cykl pod nazwa = opis uzytkownika (timestamp tylko przy duplikacie)"""
        import re as _re
        # Zachowaj czytelny opis: pozwol na spacje, myslniki, podkreslenia
        clean = name.strip()
        safe = _re.sub(r'[^\w\-\s]', '', clean).strip()
        safe = _re.sub(r'\s+', '_', safe) or "cykl"
        # Plik: prefix c_ (do wyszukiwania w archiwum) + opis
        dest = self.log_dir / f"c_{safe}.csv"
        # Jesli istnieje - dodaj timestamp zeby nie nadpisac
        if dest.exists():
            ts = datetime.now().strftime("%m%d_%H%M")
            dest = self.log_dir / f"c_{safe}_{ts}.csv"
        try:
            tmp_path.rename(dest)
            print(f"Zapisano cykl: {dest.name}")
        except Exception as e:
            print(f"Blad zapisu: {e}")
        if hasattr(self, 'refresh_arch'):
            try: self.refresh_arch()
            except: pass

    def discard_cycle(self, tmp_path):
        """Odrzuc cykl - usun plik tymczasowy"""
        try:
            if tmp_path.exists(): tmp_path.unlink()
            print("Cykl odrzucony")
        except: pass


# ════════════════════════════════════════════════════════
#  DIALOG WYBORU ZAKRESU AUTO-KALIBRACJI
# ════════════════════════════════════════════════════════
class CalRangeDialog:
    def __init__(self, parent, app):
        self.app = app
        self.win = tk.Toplevel(parent)
        self.win.title("Auto-Calibration Range")
        self.win.configure(bg=C['bg'])
        self.win.geometry("560x680")
        self.win.minsize(540, 640)
        self.win.transient(parent)
        self.win.grab_set()
        # Wycentruj wzgledem rodzica
        self.win.update_idletasks()
        try:
            px = parent.winfo_rootx() + parent.winfo_width()//2 - 280
            py = parent.winfo_rooty() + parent.winfo_height()//2 - 340
            self.win.geometry(f"+{max(0,px)}+{max(0,py)}")
        except: pass

        tk.Frame(self.win, bg=C['purple'], height=4).pack(fill='x')
        inner = tk.Frame(self.win, bg=C['bg'])
        inner.pack(fill='both', expand=True, padx=24, pady=20)

        tk.Label(inner, text="AUTO-CALIBRATION RANGE", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(14), 'bold')).pack(anchor='w')
        tk.Label(inner, text="Select temperature range and ramps to calibrate",
                 bg=C['bg'], fg=C['dim'], font=(FONT, fsz(9))).pack(anchor='w', pady=(2, 16))

        # Zakres temperatur - suwaki
        tmin0 = getattr(app, 'dev_cal_min', 50.0)
        tmax0 = getattr(app, 'dev_cal_max', 100.0)

        self.sl_tmin = SliderField(inner, "TEMP FROM", -10, 100, tmin0,
                                   C['cyan'], "°C", 0)
        self.sl_tmax = SliderField(inner, "TEMP TO", 0, 115, tmax0,
                                   C['orange'], "°C", 0)

        tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=(4, 8))

        # Krok temperatury (info - firmware uzywa co 10C)
        tk.Label(inner, text="TEMP STEP: 10°C (fixed)", bg=C['bg'], fg=C['dim2'],
                 font=(FONT, fsz(9))).pack(anchor='w', pady=(0, 12))

        # MAX RATE - suwak (do 80)
        self.sl_maxrate = SliderField(inner, "MAX RATE", 5, 80, 40,
                                      C['yellow'], "°C/min", 0,
                                      on_change=lambda v: self._update_estimate())

        # KROK RATE - wybor 5/10/20/40
        tk.Label(inner, text="RATE STEP [°C/min]:", bg=C['bg'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(anchor='w', pady=(8, 6))

        self.rate_step = 5  # domyslny krok
        self.step_btns = {}
        step_frame = tk.Frame(inner, bg=C['bg'])
        step_frame.pack(fill='x', pady=(0, 12))
        for st in [5, 10, 20, 40]:
            b = tk.Button(step_frame, text=f"{st}",
                         command=lambda s=st: self._set_step(s),
                         bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(12), 'bold'),
                         relief='flat', cursor='hand2', bd=0, padx=18, pady=10,
                         activebackground=C['panel3'])
            b.pack(side='left', padx=4, fill='x', expand=True)
            self.step_btns[st] = b
        self._set_step(10)  # zaznacz domyslny krok 10 (przy max 80 = 8 ramp)

        # Podglad generowanej listy ramp
        self.ramps_preview = tk.Label(inner, text="", bg=C['bg'], fg=C['cyan'],
                                     font=(FONT, fsz(10)))
        self.ramps_preview.pack(anchor='w', pady=(0, 8))

        # Szacowany czas
        self.est_lbl = tk.Label(inner, text="", bg=C['bg'], fg=C['yellow'],
                               font=(FONT, fsz(10), 'bold'))
        self.est_lbl.pack(anchor='w', pady=(0, 12))
        self._update_estimate()

        # Przyciski
        bf = tk.Frame(inner, bg=C['bg'])
        bf.pack(fill='x')
        mk_btn(bf, "▶ START CALIBRATION", self.start, C['purple'], fg='#fff').pack(
            side='left', fill='x', expand=True, padx=(0, 4))
        mk_btn_outline(bf, "CANCEL", self.win.destroy, C['dim']).pack(
            side='left', fill='x', expand=True, padx=(4, 0))

    def _set_step(self, step):
        """Ustaw krok rate i podswietl przycisk"""
        self.rate_step = step
        for s, b in self.step_btns.items():
            if s == step:
                b.config(bg=C['cyan'], fg='#1a1c1f')
            else:
                b.config(bg=C['bg2'], fg=C['dim'])
        self._update_estimate()

    def _gen_ramps(self):
        """Generuj liste ramp z max + krok. Np. max=20 krok=5 -> [5,10,15,20]"""
        try:
            maxr = self.sl_maxrate.get()
        except:
            maxr = 20
        step = self.rate_step
        ramps = []
        r = step
        while r <= maxr + 0.01 and len(ramps) < 20:
            ramps.append(int(round(r)))
            r += step
        if not ramps:  # gdy max < krok, uzyj samego max
            ramps = [int(round(maxr))]
        return ramps

    def _update_estimate(self):
        try:
            tmin = self.sl_tmin.get(); tmax = self.sl_tmax.get()
            n_temps = max(1, int((tmax - tmin) / 10) + 1)
            ramps = self._gen_ramps()
            n_ramps = len(ramps)
            total = n_temps * n_ramps
            est_min = total * 4  # ~4 min/krok
            # Podglad listy ramp
            if hasattr(self, 'ramps_preview'):
                self.ramps_preview.config(
                    text=f"Ramps: {', '.join(str(r) for r in ramps)} °C/min")
            self.est_lbl.config(text=f"≈ {total} steps · ~{est_min} min total")
        except Exception as e:
            print(f"est err: {e}")

    def start(self):
        tmin = self.sl_tmin.get(); tmax = self.sl_tmax.get()
        if tmax <= tmin:
            messagebox.showerror("Invalid range", "TEMP TO must be greater than TEMP FROM.")
            return
        ramps = self._gen_ramps()
        if not ramps:
            messagebox.showerror("No ramps", "Invalid rate settings.")
            return
        n_temps = int((tmax - tmin) / 10) + 1
        total = n_temps * len(ramps)
        if not messagebox.askyesno("Start calibration",
                f"Start auto-calibration?\n\n"
                f"Temp range: {tmin:.0f}-{tmax:.0f}°C (step 10°C)\n"
                f"Ramps: {', '.join(str(r) for r in ramps)} °C/min\n"
                f"Total: {total} steps\n\n"
                "Takes several minutes. Can be stopped with STOP."):
            return
        self.app.start_autocal(tmin, tmax, ramps)
        self.win.destroy()

class CalibrationWindow:
    # Fazy jednego kroku relay (kolejnosc = przebieg w firmware)
    PHASES = [('heating', '① Grzanie'), ('stabil', '② Stabilizacja'),
              ('relay', '③ Relay pomiar')]

    def __init__(self, parent, app):
        self.app = app
        self.win = tk.Toplevel(parent)
        self.win.title("Calibration progress")
        self.win.configure(bg=C['bg'])
        self.win.geometry("640x780")
        self.win.minsize(600, 700)
        self.win.transient(parent)
        self.win.update_idletasks()
        try:
            px = parent.winfo_rootx() + parent.winfo_width()//2 - 320
            py = parent.winfo_rooty() + parent.winfo_height()//2 - 390
            self.win.geometry(f"+{max(0,px)}+{max(0,py)}")
        except: pass

        tk.Frame(self.win, bg=C['purple'], height=4).pack(fill='x')
        inner = tk.Frame(self.win, bg=C['bg'])
        inner.pack(fill='both', expand=True, padx=20, pady=16)

        tk.Label(inner, text="CALIBRATION PROGRESS", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(14), 'bold')).pack(anchor='w')
        tk.Label(inner, text="Relay autotuning — jeden test na temperaturę (wypełnia wszystkie rampy)",
                 bg=C['bg'], fg=C['dim'], font=(FONT, fsz(9))).pack(anchor='w', pady=(2, 10))

        # Pasek postepu (liczony w temperaturach)
        self.prog_frame = tk.Frame(inner, bg=C['bg2'], height=30)
        self.prog_frame.pack(fill='x', pady=(0, 10))
        self.prog_frame.pack_propagate(False)
        self.prog_bar = tk.Frame(self.prog_frame, bg=C['purple'], height=30)
        self.prog_bar.place(x=0, y=0, relheight=1, relwidth=0)
        self.prog_text = tk.Label(self.prog_frame, text="0 / 0 temperatur", bg=C['bg2'],
                                  fg=C['text'], font=(FONT, fsz(11), 'bold'))
        self.prog_text.place(relx=0.5, rely=0.5, anchor='center')

        # Biezacy krok
        info = tk.Frame(inner, bg=C['panel'])
        info.pack(fill='x', pady=(0, 10))
        ii = tk.Frame(info, bg=C['panel'])
        ii.pack(fill='x', padx=14, pady=12)

        row1 = tk.Frame(ii, bg=C['panel']); row1.pack(fill='x', pady=2)
        tk.Label(row1, text="TERAZ:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(9)), width=11, anchor='w').pack(side='left')
        self.lbl_now = tk.Label(row1, text="—", bg=C['panel'], fg=C['orange'],
                                font=(FONT, fsz(12), 'bold'), anchor='w')
        self.lbl_now.pack(side='left')

        # Wskaznik fazy: grzanie -> stabilizacja -> relay
        phase_row = tk.Frame(ii, bg=C['panel']); phase_row.pack(fill='x', pady=(8, 4))
        tk.Label(phase_row, text="FAZA:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(9)), width=11, anchor='w').pack(side='left')
        self.phase_lbls = {}
        for key, label in self.PHASES:
            l = tk.Label(phase_row, text=label, bg=C['bg2'], fg=C['dim2'],
                         font=(FONT, fsz(9)), padx=8, pady=4)
            l.pack(side='left', padx=(0, 4))
            self.phase_lbls[key] = l

        row2 = tk.Frame(ii, bg=C['panel']); row2.pack(fill='x', pady=2)
        tk.Label(row2, text="NASTĘPNA:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(9)), width=11, anchor='w').pack(side='left')
        self.lbl_next = tk.Label(row2, text="—", bg=C['panel'], fg=C['cyan'],
                                 font=(FONT, fsz(11)), anchor='w')
        self.lbl_next.pack(side='left')

        row3 = tk.Frame(ii, bg=C['panel']); row3.pack(fill='x', pady=2)
        tk.Label(row3, text="POZOSTAŁO:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(9)), width=11, anchor='w').pack(side='left')
        self.lbl_eta = tk.Label(row3, text="—", bg=C['panel'], fg=C['yellow'],
                                font=(FONT, fsz(11), 'bold'), anchor='w')
        self.lbl_eta.pack(side='left')

        # Lista temperatur do kalibracji
        tk.Label(inner, text="TEMPERATURY", bg=C['bg'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(anchor='w', pady=(4, 4))

        list_wrap = tk.Frame(inner, bg=C['bg2'])
        list_wrap.pack(fill='both', expand=True)
        sb = tk.Scrollbar(list_wrap)
        sb.pack(side='right', fill='y')
        self.canvas = tk.Canvas(list_wrap, bg=C['bg2'], highlightthickness=0,
                               yscrollcommand=sb.set)
        self.canvas.pack(side='left', fill='both', expand=True)
        sb.config(command=self.canvas.yview)
        self.steps_frame = tk.Frame(self.canvas, bg=C['bg2'])
        self.canvas.create_window((0, 0), window=self.steps_frame, anchor='nw')
        self.steps_frame.bind('<Configure>',
            lambda e: self.canvas.config(scrollregion=self.canvas.bbox('all')))

        mk_btn_outline(inner, "■ ABORT CALIBRATION", self.abort, C['red']).pack(
            fill='x', pady=(12, 0))

        self.step_widgets = []
        self.refresh()

    def _step_label(self, t, r):
        """Czytelna etykieta kroku. Bezpieczna dla relay, gdzie r jest stringiem
        (stare formatowanie {r:.0f} wywalalo wyjatek i lista nigdy sie nie budowala)."""
        try:
            if isinstance(r, str):     # tryb relay: jeden test na temperature
                return f"{t:.0f}°C"
            return f"{t:.0f}°C  @  {r:.0f}°C/min"
        except Exception:
            return f"{t}"

    def refresh(self):
        app = self.app
        total = app.cal_total or len(app.cal_plan)
        cur = app.cal_current
        phase = getattr(app, 'cal_phase', None)

        # Pasek postepu
        frac = (cur / total) if total else 0
        self.prog_bar.place_configure(relwidth=min(1.0, frac))
        self.prog_text.config(text=f"{cur} / {total} temperatur")

        # TERAZ
        if app.cal_cur_temp is not None:
            tnum = f"   ({cur}/{total})" if cur else ""
            self.lbl_now.config(text=f"{app.cal_cur_temp:.0f}°C{tnum}")
        else:
            self.lbl_now.config(text="— (czekam na urządzenie)")

        # Podswietl aktywna faze
        for key, _ in self.PHASES:
            if key == phase:
                self.phase_lbls[key].config(bg=C['orange'], fg='#1a1c1f')
            else:
                self.phase_lbls[key].config(bg=C['bg2'], fg=C['dim2'])

        # NASTEPNA temperatura
        if 0 < cur < len(app.cal_plan):
            nt, nr = app.cal_plan[cur]
            self.lbl_next.config(text=self._step_label(nt, nr))
        elif cur >= len(app.cal_plan) and len(app.cal_plan) > 0:
            self.lbl_next.config(text="(ostatnia)")
        else:
            self.lbl_next.config(text="—")

        # ETA
        eta = app._cal_eta()
        if eta is not None:
            m = int(eta // 60); s = int(eta % 60)
            self.lbl_eta.config(text=f"~{m} min {s} s")
        elif cur >= total and total > 0:
            self.lbl_eta.config(text="ZAKOŃCZONO ✓")
        else:
            self.lbl_eta.config(text="—")

        # Lista temperatur - buduj raz, potem aktualizuj statusy
        if len(self.step_widgets) != len(app.cal_plan):
            for w in self.steps_frame.winfo_children():
                w.destroy()
            self.step_widgets = []
            for i, (t, r) in enumerate(app.cal_plan):
                row = tk.Frame(self.steps_frame, bg=C['bg2'])
                row.pack(fill='x', pady=1)
                bar = tk.Frame(row, bg=C['bg2'], width=4)
                bar.pack(side='left', fill='y')
                num = tk.Label(row, text=f"{i+1:2d}", bg=C['bg2'], fg=C['dim2'],
                              font=(FONT, fsz(9)), width=4, anchor='w')
                num.pack(side='left')
                txt = tk.Label(row, text=self._step_label(t, r),
                              bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(10)), anchor='w')
                txt.pack(side='left', fill='x', expand=True, padx=(2, 0))
                stat = tk.Label(row, text="", bg=C['bg2'], fg=C['dim2'],
                               font=(FONT, fsz(9)), anchor='e', width=18)
                stat.pack(side='right')
                self.step_widgets.append((bar, num, txt, stat))

        # Statusy + kolory
        phase_txt = {'heating': '→ grzanie', 'stabil': '~ stabilizacja',
                     'relay': '◇ relay pomiar'}
        for i, (bar, num, txt, stat) in enumerate(self.step_widgets):
            step_no = i + 1
            if step_no < cur:
                bar.config(bg=C['green']); txt.config(fg=C['dim2'])
                num.config(fg=C['green']); stat.config(text="✓ gotowe", fg=C['green'])
            elif step_no == cur:
                bar.config(bg=C['orange']); txt.config(fg=C['text'])
                num.config(fg=C['orange'])
                stat.config(text=phase_txt.get(phase, "● teraz"), fg=C['orange'])
                try: self.canvas.yview_moveto(max(0, (i-3))/max(1, len(self.step_widgets)))
                except: pass
            else:
                bar.config(bg=C['bg2']); txt.config(fg=C['dim'])
                num.config(fg=C['dim2']); stat.config(text="oczekuje", fg=C['dim2'])

    def abort(self):
        if messagebox.askyesno("Abort?", "Abort calibration?"):
            self.app.send("AUTOCALSTOP")
            self.app.send("STOP")
            self.app.cal_running = False
            self.win.destroy()


# ════════════════════════════════════════════════════════
#  OKNO PRESETÓW (zapisywalne zestawy nastaw)
# ════════════════════════════════════════════════════════
class PresetWindow:
    def __init__(self, parent, app):
        self.app = app
        self.win = tk.Toplevel(parent)
        self.win.title("Presets")
        self.win.configure(bg=C['bg'])
        self.win.geometry("520x560")
        self.win.minsize(480, 480)
        self.win.transient(parent)
        self.win.update_idletasks()
        try:
            px = parent.winfo_rootx() + parent.winfo_width()//2 - 260
            py = parent.winfo_rooty() + parent.winfo_height()//2 - 280
            self.win.geometry(f"+{max(0,px)}+{max(0,py)}")
        except: pass

        tk.Frame(self.win, bg=C['green'], height=4).pack(fill='x')
        inner = tk.Frame(self.win, bg=C['bg'])
        inner.pack(fill='both', expand=True, padx=24, pady=20)

        tk.Label(inner, text="PRESETS", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(14), 'bold')).pack(anchor='w')
        tk.Label(inner, text="Save & load complete settings (setpoint, ramps, PID, fan)",
                 bg=C['bg'], fg=C['dim'], font=(FONT, fsz(9))).pack(anchor='w', pady=(2, 16))

        # Zapis nowego presetu
        save_box = tk.Frame(inner, bg=C['bg2'])
        save_box.pack(fill='x', pady=(0, 16))
        si = tk.Frame(save_box, bg=C['bg2'])
        si.pack(fill='x', padx=12, pady=10)
        tk.Label(si, text="Save current settings as:", bg=C['bg2'], fg=C['dim'],
                 font=(FONT, fsz(9))).pack(anchor='w', pady=(0, 4))
        erow = tk.Frame(si, bg=C['bg2'])
        erow.pack(fill='x')
        self.name_entry = tk.Entry(erow, bg=C['bg'], fg=C['text'], font=(FONT, fsz(11)),
                                   relief='flat', bd=0, insertbackground=C['green'],
                                   highlightthickness=2, highlightbackground=C['green'])
        self.name_entry.pack(side='left', fill='x', expand=True, ipady=5, padx=(0, 8))
        self.name_entry.insert(0, "My preset")
        self.name_entry.bind('<Return>', lambda e: self.save_preset())
        mk_btn(erow, "SAVE", self.save_preset, C['green']).pack(side='right')

        # Lista zapisanych presetow
        tk.Label(inner, text="SAVED PRESETS", bg=C['bg'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(anchor='w', pady=(0, 6))
        list_wrap = tk.Frame(inner, bg=C['bg2'])
        list_wrap.pack(fill='both', expand=True)
        psb = tk.Scrollbar(list_wrap)
        psb.pack(side='right', fill='y')
        self.canvas = tk.Canvas(list_wrap, bg=C['bg2'], highlightthickness=0,
                               yscrollcommand=psb.set)
        self.canvas.pack(side='left', fill='both', expand=True)
        psb.config(command=self.canvas.yview)
        self.items = tk.Frame(self.canvas, bg=C['bg2'])
        self.canvas.create_window((0, 0), window=self.items, anchor='nw')
        self.items.bind('<Configure>',
            lambda e: self.canvas.config(scrollregion=self.canvas.bbox('all')))

        self.refresh_list()

    def refresh_list(self):
        for w in self.items.winfo_children():
            w.destroy()
        presets = self.app._load_presets()
        if not presets:
            tk.Label(self.items, text="No presets yet.\nSave current settings above.",
                     bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(9)), justify='left').pack(
                     anchor='w', padx=12, pady=12)
            return
        for name, settings in presets.items():
            row = tk.Frame(self.items, bg=C['bg2'])
            row.pack(fill='x', pady=2, padx=4)
            info = tk.Frame(row, bg=C['bg2'])
            info.pack(side='left', fill='x', expand=True)
            tk.Label(info, text=name, bg=C['bg2'], fg=C['text'],
                     font=(FONT, fsz(10), 'bold'), anchor='w').pack(anchor='w')
            # Krotki opis nastaw
            desc = f"SP {settings.get('sp','?')}°C · ↑{settings.get('ru','?')} ↓{settings.get('rd','?')}°C/min · fan {settings.get('fan','?')}%"
            tk.Label(info, text=desc, bg=C['bg2'], fg=C['dim2'],
                     font=(FONT, fsz(8)), anchor='w').pack(anchor='w')
            # Przyciski
            mk_btn(row, "LOAD", lambda n=name: self.load_preset(n), C['green']).pack(
                side='left', padx=(4, 2))
            mk_btn_outline(row, "DEL", lambda n=name: self.del_preset(n), C['red']).pack(
                side='left', padx=(2, 0))

    def save_preset(self):
        name = self.name_entry.get().strip()
        if not name:
            messagebox.showinfo("Name required", "Enter a preset name.")
            return
        presets = self.app._load_presets()
        if name in presets:
            if not messagebox.askyesno("Overwrite?", f"Preset '{name}' exists. Overwrite?"):
                return
        presets[name] = self.app._gather_settings()
        if self.app._save_presets(presets):
            self.refresh_list()
            messagebox.showinfo("Saved", f"Preset '{name}' saved.")

    def load_preset(self, name):
        presets = self.app._load_presets()
        if name in presets:
            self.app.apply_preset(presets[name])
            messagebox.showinfo("Loaded", f"Preset '{name}' applied.")
            self.win.destroy()

    def del_preset(self, name):
        if messagebox.askyesno("Delete?", f"Delete preset '{name}'?"):
            presets = self.app._load_presets()
            presets.pop(name, None)
            self.app._save_presets(presets)
            self.refresh_list()


# ════════════════════════════════════════════════════════
#  DIALOG ZAPISU CYKLU
# ════════════════════════════════════════════════════════
class SaveCycleDialog:
    def __init__(self, parent, app, tmp_path):
        self.app = app
        self.tmp_path = tmp_path
        self.win = tk.Toplevel(parent)
        self.win.title("Save cycle")
        self.win.configure(bg=C['bg'])
        self.win.geometry("440x230")
        self.win.transient(parent)
        self.win.grab_set()  # modalne

        tk.Frame(self.win, bg=C['green'], height=4).pack(fill='x')
        inner = tk.Frame(self.win, bg=C['bg'])
        inner.pack(fill='both', expand=True, padx=24, pady=20)

        tk.Label(inner, text="SAVE CYCLE TO ARCHIVE", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(13), 'bold')).pack(anchor='w')

        # Info ile probek
        rows = getattr(app, 'cyc_rows', 0)
        tk.Label(inner, text=f"Recorded {rows} data samples",
                 bg=C['bg'], fg=C['dim'], font=(FONT, fsz(9))).pack(anchor='w', pady=(4, 16))

        tk.Label(inner, text="Cycle name:", bg=C['bg'], fg=C['dim'],
                 font=(FONT, fsz(10))).pack(anchor='w')
        self.entry = tk.Entry(inner, bg=C['bg2'], fg=C['text'],
                              font=(FONT, fsz(12)), relief='flat', bd=0,
                              insertbackground=C['green'],
                              highlightthickness=2, highlightbackground=C['green'],
                              highlightcolor=_lighten(C['green'], 0.2))
        self.entry.pack(fill='x', ipady=6, pady=(4, 16))
        # Domyslna nazwa
        default = datetime.now().strftime("test_%H%M")
        self.entry.insert(0, default)
        self.entry.select_range(0, 'end')
        self.entry.focus()
        self.entry.bind('<Return>', lambda e: self.save())

        # Przyciski
        bf = tk.Frame(inner, bg=C['bg'])
        bf.pack(fill='x')
        mk_btn(bf, "SAVE", self.save, C['green']).pack(side='left', fill='x',
                                                          expand=True, padx=(0, 4))
        mk_btn_outline(bf, "DISCARD", self.discard, C['red']).pack(side='left',
                                                          fill='x', expand=True, padx=(4, 0))

        self.win.protocol("WM_DELETE_WINDOW", self.save)  # zamkniecie = zapisz

    def save(self):
        name = self.entry.get().strip()
        if not name:
            name = datetime.now().strftime("cykl_%H%M")
        self.app.save_cycle_as(self.tmp_path, name)
        self.win.destroy()

    def discard(self):
        if messagebox.askyesno("Discard?",
                "Discard this cycle?\nData will be permanently deleted."):
            self.app.discard_cycle(self.tmp_path)
            self.win.destroy()


# ════════════════════════════════════════════════════════
#  OKNO PROFILI WIELOETAPOWYCH
# ════════════════════════════════════════════════════════
class ProfileWindow:
    def __init__(self, parent, app):
        self.app = app
        self.win = tk.Toplevel(parent)
        self.win.title("Multi-step profiles")
        self.win.configure(bg=C['bg'])
        self.win.geometry("520x480")
        self.win.transient(parent)

        tk.Frame(self.win, bg=C['purple'], height=4).pack(fill='x')
        hd = tk.Frame(self.win, bg=C['bg'])
        hd.pack(fill='x', padx=16, pady=12)
        tk.Label(hd, text="MULTI-STEP PROFILES", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(12), 'bold')).pack(side='left')

        # Tabela etapow
        self.rows_frame = tk.Frame(self.win, bg=C['bg'])
        self.rows_frame.pack(fill='both', expand=True, padx=16)

        # Naglowki
        h = tk.Frame(self.rows_frame, bg=C['bg'])
        h.pack(fill='x', pady=(0, 4))
        for txt, w in [("#", 3), ("TEMP °C", 10), ("RATE", 8), ("TIME min", 10), ("", 6)]:
            tk.Label(h, text=txt, bg=C['bg'], fg=C['dim2'],
                     font=(FONT, fsz(9)), width=w, anchor='w').pack(side='left')

        self.steps_container = tk.Frame(self.rows_frame, bg=C['bg'])
        self.steps_container.pack(fill='both', expand=True)

        # Formularz dodawania
        addf = tk.Frame(self.win, bg=C['panel'])
        addf.pack(fill='x', padx=16, pady=12)
        tk.Frame(addf, bg=C['green'], height=3).pack(fill='x')
        ai = tk.Frame(addf, bg=C['panel'])
        ai.pack(fill='x', padx=12, pady=10)
        tk.Label(ai, text="ADD STEP:", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(9))).pack(side='left', padx=(0, 8))
        self.e_temp = tk.Entry(ai, width=6, bg=C['bg2'], fg=C['orange'],
                               font=(FONT, fsz(10)), justify='center', relief='flat',
                               highlightthickness=1, highlightbackground=C['border'])
        self.e_temp.pack(side='left', padx=2); self.e_temp.insert(0, "40")
        self.e_ramp = tk.Entry(ai, width=6, bg=C['bg2'], fg=C['yellow'],
                               font=(FONT, fsz(10)), justify='center', relief='flat',
                               highlightthickness=1, highlightbackground=C['border'])
        self.e_ramp.pack(side='left', padx=2); self.e_ramp.insert(0, "2.0")
        self.e_time = tk.Entry(ai, width=6, bg=C['bg2'], fg=C['dim'],
                               font=(FONT, fsz(10)), justify='center', relief='flat',
                               highlightthickness=1, highlightbackground=C['border'])
        self.e_time.pack(side='left', padx=2); self.e_time.insert(0, "10")
        mk_btn(ai, "+ ADD", self.add_step, C['green']).pack(side='left', padx=(8, 0))

        # Uruchom
        rf = tk.Frame(self.win, bg=C['bg'])
        rf.pack(fill='x', padx=16, pady=(0, 12))
        mk_btn(rf, "▶ RUN PROFILE", self.run_profile, C['purple'], fg='#fff').pack(
            fill='x')

        self.refresh_steps()

    def add_step(self):
        try:
            temp = float(self.e_temp.get().replace(',', '.'))
            ramp = float(self.e_ramp.get().replace(',', '.'))
            tmin = float(self.e_time.get().replace(',', '.'))
            self.app.profile_steps.append({'temp': temp, 'ramp': ramp, 'time': tmin})
            self.refresh_steps()
        except ValueError:
            messagebox.showerror("Error", "Wpisz poprawne liczby.")

    def del_step(self, idx):
        if 0 <= idx < len(self.app.profile_steps):
            self.app.profile_steps.pop(idx)
            self.refresh_steps()

    def refresh_steps(self):
        for w in self.steps_container.winfo_children():
            w.destroy()
        for i, s in enumerate(self.app.profile_steps):
            r = tk.Frame(self.steps_container, bg=C['bg2'])
            r.pack(fill='x', pady=2)
            tk.Frame(r, bg=C['orange'], width=4).pack(side='left', fill='y')
            tk.Label(r, text=str(i+1), bg=C['bg2'], fg=C['text'],
                     font=(FONT, fsz(10), 'bold'), width=3, anchor='w').pack(side='left', padx=(6,0))
            tk.Label(r, text=f"{s['temp']:.0f}", bg=C['bg2'], fg=C['orange'],
                     font=(FONT, fsz(10)), width=10, anchor='w').pack(side='left')
            tk.Label(r, text=f"{s['ramp']:.1f}", bg=C['bg2'], fg=C['yellow'],
                     font=(FONT, fsz(10)), width=8, anchor='w').pack(side='left')
            tk.Label(r, text=f"{s['time']:.0f}", bg=C['bg2'], fg=C['dim'],
                     font=(FONT, fsz(10)), width=10, anchor='w').pack(side='left')
            tk.Button(r, text="DEL", command=lambda idx=i: self.del_step(idx),
                      bg=C['bg2'], fg=C['red'], font=(FONT, fsz(8), 'bold'),
                      relief='flat', cursor='hand2', bd=0,
                      activebackground=C['panel3']).pack(side='left', padx=4)

    def run_profile(self):
        """Wykonaj profil - sekwencyjnie wysylaj etapy z opoznieniem"""
        if not self.app.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        if not self.app.profile_steps:
            messagebox.showinfo("Empty profile", "Add at least one step.")
            return
        if not messagebox.askyesno("Run profile",
                f"Run profile with {len(self.app.profile_steps)} steps?\n"
                "Steps will run sequentially."):
            return
        threading.Thread(target=self._run_profile_thread, daemon=True).start()
        self.win.destroy()

    def _run_profile_thread(self):
        """Watek wykonujacy profil"""
        for i, s in enumerate(self.app.profile_steps):
            self.app.send(f"SP:{s['temp']:.1f}")
            self.app.send(f"RU:{s['ramp']:.1f}")
            self.app.send(f"RD:{s['ramp']:.1f}")
            if i == 0:
                time.sleep(0.1)
                self.app.send("START")
            # Czekaj czas etapu (time w minutach)
            time.sleep(max(1, s['time'] * 60))
        # Po profilu - stop
        self.app.send("STOP")
        print("Profil zakonczony")


# ════════════════════════════════════════════════════════
#  MAIN
# ════════════════════════════════════════════════════════
def _enable_dpi_awareness():
    """Wlacz DPI awareness na Windows - eliminuje rozmyty tekst przy skalowaniu 125%/150%."""
    if sys.platform != 'win32':
        return 1.0
    try:
        import ctypes
        # Per-Monitor DPI Aware v2 (Windows 10 1703+) - najlepsza ostrosc
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(2)
        except Exception:
            # Fallback dla starszych Windows
            ctypes.windll.user32.SetProcessDPIAware()
        # Odczytaj rzeczywiste skalowanie
        try:
            hdc = ctypes.windll.user32.GetDC(0)
            dpi = ctypes.windll.gdi32.GetDeviceCaps(hdc, 88)  # LOGPIXELSX
            ctypes.windll.user32.ReleaseDC(0, hdc)
            return dpi / 96.0
        except Exception:
            return 1.0
    except Exception:
        return 1.0


def main():
    # WAZNE: DPI awareness PRZED utworzeniem okna - daje ostry tekst
    scale = _enable_dpi_awareness()

    # Ustaw globalny mnoznik fontow wg DPI (ostre I czytelne)
    global FS
    if scale and scale > 1.05:
        FS = scale  # np. 1.25 dla 125%, 1.5 dla 150%
    else:
        FS = 1.0

    root = tk.Tk()

    # Tk scaling dla widgetow ttk (Notebook itp.)
    try:
        if scale and scale > 1.05:
            root.tk.call('tk', 'scaling', scale)
    except Exception:
        pass

    app = PeltierControl(root)

    def on_close():
        app.disconnect()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == "__main__":
    main()
