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
        self.cal_running = False
        self.cal_t0 = None         # czas startu kalibracji
        self.cal_step_times = []   # czasy ukonczenia krokow (do ETA)
        self.cal_win = None        # okno postepu kalibracji

        # Zapis kalibracji na dysku PC
        self.cal_file = self.log_dir / "kalibracja.json"
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
            self.set_status(True, f"{port} - 115200")
            self.running = True
            threading.Thread(target=self.reader, daemon=True).start()
            # Pobierz konfiguracje startowa
            self.root.after(1500, lambda: self.send("GET"))
            # Auto-wczytaj zapisana kalibracje z PC (jesli istnieje)
            self.root.after(2200, self._auto_load_calibration)
        except Exception as e:
            messagebox.showerror("Blad", f"{port}:\n{e}")
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

                # Linia danych CSV (9 pol)
                p = raw.split(',')
                if len(p) < 9: continue
                try: float(p[0])
                except ValueError: continue
                try:
                    d = dict(temp=float(p[1]), sa=float(p[2]), st=float(p[3]),
                             pwm=int(p[4]), kp=float(p[5]), ki=float(p[6]),
                             kd=float(p[7]), state=p[8].strip())
                except: continue

                if self.t0 is None: self.t0 = time.time()
                now = time.time() - self.t0
                state = d['state']

                if self.cyc_on and state in ('AUTO', 'COOLDOWN'):
                    self.cyc_log(time.time() - self.cyc_t0 if self.cyc_t0 else 0,
                                d['temp'], d['sa'], d['st'],
                                d['pwm'], d['kp'], d['ki'], d['kd'], state)

                prev = self.last_state
                self.last_state = state
                self.cur_state = state
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
            if 'SP' in d and hasattr(self, 'sl_sp'):    self.sl_sp.set(float(d['SP']))
            if 'RU' in d and hasattr(self, 'sl_ru'):    self.sl_ru.set(float(d['RU']))
            if 'RD' in d and hasattr(self, 'sl_rd'):    self.sl_rd.set(float(d['RD']))
            if 'TMAX' in d and hasattr(self, 'sl_tmax'): self.sl_tmax.set(float(d['TMAX']))
            if 'KP' in d and hasattr(self, 'sl_kp'):    self.sl_kp.set(float(d['KP']))
            if 'KI' in d and hasattr(self, 'sl_ki'):    self.sl_ki.set(float(d['KI']))
            if 'KD' in d and hasattr(self, 'sl_kd'):    self.sl_kd.set(float(d['KD']))
            if 'OFFSET' in d and hasattr(self, 'sl_off'): self.sl_off.set(float(d['OFFSET']))
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
            # Zaktualizuj wskaznik polaryzacji w UI jesli istnieje
            if hasattr(self, '_update_pol_indicator'):
                self._update_pol_indicator()
        except Exception as e:
            print(f"apply_cfg err: {e}")

    def _parse_calplan(self, txt):
        """CALPLAN:24,temps=50/60/70,ramps=2/5/10/20 - buduj liste krokow"""
        try:
            d = {}
            # pierwsza czesc to total
            parts = txt.split(',')
            total = int(parts[0])
            temps, ramps = [], []
            for part in parts[1:]:
                if part.startswith('temps='):
                    temps = [float(x) for x in part[6:].split('/') if x]
                elif part.startswith('ramps='):
                    ramps = [float(x) for x in part[6:].split('/') if x]
            # Buduj plan: dla kazdej temp, wszystkie rampy (kolejnosc jak firmware)
            plan = []
            for t in temps:
                for r in ramps:
                    plan.append((t, r))
            self.cal_plan = plan
            self.cal_total = total or len(plan)
            self.cal_current = 0
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
                    self.cal_cur_ramp = float(part[2:])
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
            self.cal_status.config(text="✓ Kalibracja zakonczona - zapisywanie na PC...")
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
            messagebox.showwarning("Brak polaczenia", "Polacz sie z urzadzeniem.")
            return
        if not self.cal_file.exists():
            messagebox.showinfo("Brak kalibracji",
                "Nie znaleziono zapisanej kalibracji na PC.\n"
                "Najpierw wykonaj kalibracje lub zapisz ja przyciskiem\n"
                "'ZAPISZ KALIBR. NA PC'.")
            return
        # Pokaz date zapisu
        try:
            with open(self.cal_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            saved = data.get('saved', '?')
            nvalid = sum(1 for p in data.get('profiles', []) if p.get('valid'))
        except:
            saved = '?'; nvalid = 0
        if messagebox.askyesno("Wgraj kalibracje z PC",
                f"Wgrac zapisana kalibracje do urzadzenia?\n\n"
                f"Zapisana: {saved}\n"
                f"Profili: {nvalid}\n\n"
                "Nadpisze obecna kalibracje w urzadzeniu."):
            self.load_calibration_from_pc()

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
                self.cal_status.config(text=f"✓ Kalibracja zapisana na PC ({n_valid} profili)")
            if self._caldump_purpose == 'save':
                try:
                    messagebox.showinfo("Kalibracja zapisana",
                        f"Profile PID + offset zapisane na dysku:\n{self.cal_file}\n\n"
                        f"Zapisano {n_valid} skalibrowanych profili.\n"
                        "Przy nastepnym polaczeniu zostana automatycznie wgrane.")
                except: pass
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
                            text=f"✓ Wgrano kalibracje z PC ({len(profiles)} profili)")
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
        t1 = tk.Frame(nb, bg=C['bg']); nb.add(t1, text='STEROWANIE')
        t2 = tk.Frame(nb, bg=C['bg']); nb.add(t2, text='ARCHIWUM')
        t3 = tk.Frame(nb, bg=C['bg']); nb.add(t3, text='POLACZENIE')
        self.build_live(t1)
        self.build_arch(t2)
        self.build_conn(t3)

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
        if hasattr(self, 'btn_start'):
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
        self.cards['sp']   = self._stat_card(cards, "SETPOINT", "°C", C['orange'])
        self.cards['rate'] = self._stat_card(cards, "TEMPO", "°C/min", C['yellow'])
        self.cards['pwm']  = self._stat_card(cards, "PWM", "%", C['green'])

        # Przyciski START/STOP/E-STOP (prawa czesc paska) - zawsze widoczne
        ctrl = tk.Frame(topbar, bg=C['bg'])
        ctrl.pack(side='right', padx=(8, 0))
        self.btn_start = tk.Button(ctrl, text="▶ START", command=self.do_start,
                                   bg=C['green'], fg='#1a1c1f', font=(FONT, fsz(13), 'bold'),
                                   relief='flat', cursor='hand2', bd=0, padx=18, pady=14,
                                   activebackground=_lighten(C['green'], 0.15))
        self.btn_start.pack(side='left', padx=(0, 4), fill='y')
        self.btn_stop = tk.Button(ctrl, text="■ STOP", command=self.do_stop,
                                  bg=C['bg2'], fg=C['red'], font=(FONT, fsz(13), 'bold'),
                                  relief='flat', cursor='hand2', bd=0, padx=18, pady=14,
                                  highlightthickness=2, highlightbackground=C['red'],
                                  activebackground=C['panel3'])
        self.btn_stop.pack(side='left', padx=(0, 4), fill='y')
        self.btn_estop = tk.Button(ctrl, text="⛔", command=self.do_estop,
                                   bg=C['red'], fg='#fff', font=(FONT, fsz(15), 'bold'),
                                   relief='flat', cursor='hand2', bd=0, padx=14, pady=14,
                                   activebackground=_lighten(C['red'], 0.15))
        self.btn_estop.pack(side='left', fill='y')

        # Glowny obszar: wykres + panel
        main = tk.Frame(parent, bg=C['bg'])
        main.pack(fill='both', expand=True, padx=16, pady=(0, 12))

        # LEWO - wykres
        self._build_chart(main)
        # PRAWO - panel sterowania
        self._build_panel(main)

    def _stat_card(self, parent, title, unit, color):
        card = tk.Frame(parent, bg=C['panel'])
        card.pack(side='left', fill='x', expand=True, padx=(0, 6))
        tk.Frame(card, bg=color, height=3).pack(fill='x')
        inner = tk.Frame(card, bg=C['panel'])
        inner.pack(fill='both', expand=True, padx=10, pady=6)
        tk.Label(inner, text=title, bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(8)), anchor='w').pack(anchor='w')
        vrow = tk.Frame(inner, bg=C['panel'])
        vrow.pack(anchor='w', pady=(2, 0))
        val = tk.Label(vrow, text="--", bg=C['panel'], fg=color,
                       font=(FONT, fsz(19), 'bold'))
        val.pack(side='left')
        unit_lbl = tk.Label(vrow, text=" " + unit, bg=C['panel'], fg=C['dim2'],
                            font=(FONT, fsz(8)))
        unit_lbl.pack(side='left', pady=(5, 0))
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
        self.cv.get_tk_widget().pack(fill='both', expand=True, padx=8, pady=(0, 8))

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
        self.sl_ru = SliderField(inner, "HEAT RATE", 0.5, 40, 2.0,
                                 C['yellow'], "°C/min", 1,
                                 on_change=lambda v: self.send(f"RU:{v:.1f}"))
        self.sl_rd = SliderField(inner, "COOL RATE", 0.5, 40, 2.0,
                                 C['cyan'], "°C/min", 1,
                                 on_change=lambda v: self.send(f"RD:{v:.1f}"))
        self.sl_tmax = SliderField(inner, "MAX TEMP", 50, 115, 80,
                                   C['red'], "°C", 0,
                                   on_change=lambda v: self.send(f"TMAX:{v:.0f}"))

        tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=(2, 12))

        # PID
        pid_hd = tk.Frame(inner, bg=C['bg2'])
        pid_hd.pack(fill='x', pady=(0, 8))
        tk.Label(pid_hd, text="PID", bg=C['bg2'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(side='left')
        self.btn_st = mk_btn(pid_hd, "SELF-TUNE", self.do_selftune, C['cyan'])
        self.btn_st.pack(side='right')

        # Auto-kalibracja (pelna - wszystkie profile temp x rampa)
        cal_row = tk.Frame(inner, bg=C['bg2'])
        cal_row.pack(fill='x', pady=(0, 8))
        self.btn_autocal = mk_btn(cal_row, "⚙ AUTO-CAL",
                                  self.do_autocal, C['purple'], fg='#fff')
        self.btn_autocal.pack(fill='x')
        # Status kalibracji - klikalny, otwiera okno postepu
        self.cal_status = tk.Label(inner, text="", bg=C['bg2'], fg=C['purple'],
                                   font=(FONT, fsz(8)), anchor='w', cursor='hand2')
        self.cal_status.pack(fill='x', pady=(0, 4))
        self.cal_status.bind('<Button-1>', lambda e: self.open_cal_window())

        self.sl_kp = SliderField(inner, "Kp", 1, 30, 10.0, C['cyan'], "", 1,
                                 on_change=lambda v: self.send(f"KP:{v:.1f}"))
        self.sl_ki = SliderField(inner, "Ki", 0, 3, 0.3, C['cyan'], "", 2,
                                 on_change=lambda v: self.send(f"KI:{v:.2f}"))
        self.sl_kd = SliderField(inner, "Kd", 0, 3, 0.8, C['cyan'], "", 2,
                                 on_change=lambda v: self.send(f"KD:{v:.2f}"))

        tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=(2, 12))

        # Kalibracja
        self.sl_off = SliderField(inner, "CAL OFFSET", -20, 20, 0.0,
                                  C['purple'], "°C", 1,
                                  on_change=lambda v: self.send(f"OFFSET:{v:.1f}"))

        tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=(2, 12))

        # AUTO badge + wskaznik polaryzacji
        auto = tk.Frame(inner, bg=C['bg2'], highlightthickness=1,
                        highlightbackground=C['green'])
        auto.pack(fill='x', pady=(0, 8))
        tk.Label(auto, text="● AUTO: direction by setpoint", bg=C['bg2'],
                 fg=C['green'], font=(FONT, fsz(9))).pack(padx=8, pady=6)

        # Polaryzacja - wskaznik + re-detekcja
        pol_frame = tk.Frame(inner, bg=C['bg2'])
        pol_frame.pack(fill='x', pady=(0, 10))
        self.pol_indicator = tk.Label(pol_frame, text="POL: ?", bg=C['bg2'],
                                      fg=C['dim2'], font=(FONT, fsz(9)))
        self.pol_indicator.pack(side='left')
        mk_btn_outline(pol_frame, "RE-DETECT", self.do_repol, C['dim']).pack(side='right')

        # (START / STOP / E-STOP przeniesione na gorny pasek - obok kafelkow)

        # Profile + Flash kompakt
        bf2 = tk.Frame(inner, bg=C['bg2'])
        bf2.pack(fill='x', pady=(8, 6))
        mk_btn_outline(bf2, "PROFILES", self.open_profiles, C['purple']).pack(
            side='left', fill='x', expand=True, padx=(0, 3))
        mk_btn_outline(bf2, "SAVE", lambda: self.send("SAVE"), C['green']).pack(
            side='left', fill='x', expand=True, padx=3)
        mk_btn_outline(bf2, "LOAD", lambda: self.send("LOAD"), C['cyan']).pack(
            side='left', fill='x', expand=True, padx=(3, 0))

        # Kalibracja na dysk PC (trwala kopia)
        bf3 = tk.Frame(inner, bg=C['bg2'])
        bf3.pack(fill='x', pady=(0, 6))
        mk_btn_outline(bf3, "⤓ SAVE CAL TO PC",
                       lambda: self.dump_calibration_to_pc(silent=False),
                       C['purple']).pack(side='left', fill='x', expand=True, padx=(0, 3))
        mk_btn_outline(bf3, "⤒ LOAD FROM PC",
                       self._manual_load_cal, C['cyan']).pack(
                       side='left', fill='x', expand=True, padx=(3, 0))

        # Reset nastaw
        mk_btn_outline(inner, "↺ RESET", self.do_reset, C['dim']).pack(
            fill='x', pady=(0, 8))

        tk.Label(inner, text="▶ START uses panel values",
                 bg=C['bg2'], fg=C['green'], font=(FONT, fsz(8))).pack(anchor='w', pady=(4, 0))

        self._set_panel_enabled(False)

    def _set_panel_enabled(self, en):
        # Suwaki zawsze aktywne (mozna ustawic wartosci przed polaczeniem)
        # START/STOP tez aktywne - sprawdzaja polaczenie w momencie klikniecia
        # (dezaktywujemy tylko gdy chcemy wyraznie zablokowac)
        for sl in ['sl_sp', 'sl_ru', 'sl_rd', 'sl_tmax', 'sl_kp', 'sl_ki', 'sl_kd', 'sl_off']:
            if hasattr(self, sl):
                getattr(self, sl).set_enabled(True)
        # Przyciski zawsze klikalnie - reaguja komunikatem jesli brak polaczenia
        for b in ['btn_start', 'btn_stop', 'btn_st', 'btn_autocal', 'btn_estop']:
            if hasattr(self, b):
                getattr(self, b).config(state='normal')


    # ────────────────────────────────────────────────────
    #  AKCJE PRZYCISKOW
    # ────────────────────────────────────────────────────
    def do_start(self):
        """START - wyslij wszystkie nastawy z panelu, potem uruchom"""
        if not self.connected:
            messagebox.showwarning("Brak polaczenia", "Najpierw polacz sie z urzadzeniem.")
            return
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

    def do_stop(self):
        self.send("STOP")
        self.send("AUTOCALSTOP")  # przerwij tez kalibracje jesli trwa
        if hasattr(self, 'cal_status'):
            self.cal_status.config(text="")

    def do_estop(self):
        """Awaryjne zatrzymanie - natychmiast wylacza PWM"""
        self.send("ESTOP")
        self.send("AUTOCALSTOP")
        if hasattr(self, 'cal_status'):
            self.cal_status.config(text="")

    def do_reset(self):
        """Reset nastaw do domyslnych"""
        if not self.connected:
            messagebox.showwarning("Brak polaczenia", "Polacz sie z urzadzeniem.")
            return
        if messagebox.askyesno("Reset nastaw",
                "Przywrocic domyslne nastawy?\n"
                "Wyczysci wszystkie profile i kalibracje!"):
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
        """Uruchom auto-kalibracje z wybranym zakresem"""
        # Wyslij zakres do urzadzenia
        self.send(f"CALRANGE:{temp_min:.0f},{temp_max:.0f}")
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
            messagebox.showinfo("Kalibracja",
                "Kalibracja nie jest uruchomiona.\n"
                "Kliknij AUTO-KALIBRACJA aby rozpoczac.")
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
                self.cal_status.config(text="✓ Kalibracja zakonczona")
        # Okno szczegolow
        if hasattr(self, 'cal_win') and self.cal_win:
            try: self.cal_win.refresh()
            except: pass

    def open_profiles(self):
        """Okno edycji profili wieloetapowych"""
        ProfileWindow(self.root, self)

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
            "1. Podlacz ItsyBitsy (firmware v19 PC MODE) przez USB",
            "2. Wybierz port COM z listy i kliknij POLACZ",
            "3. Suwaki zsynchronizuja sie automatycznie z urzadzeniem",
            "4. Ustaw parametry i kliknij START",
            "5. Wykres pokazuje przebieg na zywo, dane zapisuja sie do CSV",
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
        mk_btn(hd, "REFRESH", self.refresh_arch, C['cyan']).pack(side='right')

        body = tk.Frame(wrap, bg=C['bg'])
        body.pack(fill='both', expand=True)

        # Lista plikow
        lf = tk.Frame(body, bg=C['panel'], width=280)
        lf.pack(side='left', fill='y', padx=(0, 12))
        lf.pack_propagate(False)
        tk.Frame(lf, bg=C['purple'], height=3).pack(fill='x')
        tk.Label(lf, text="SAVED CYCLES", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(anchor='w', padx=12, pady=8)
        sb = tk.Scrollbar(lf)
        sb.pack(side='right', fill='y')
        self.a_list = tk.Listbox(lf, bg=C['bg2'], fg=C['text'],
                                font=(FONT, fsz(9)), selectbackground=C['purple'],
                                borderwidth=0, highlightthickness=0,
                                yscrollcommand=sb.set, activestyle='none')
        self.a_list.pack(side='left', fill='both', expand=True, padx=8, pady=(0, 8))
        sb.config(command=self.a_list.yview)
        self.a_list.bind('<<ListboxSelect>>', self.load_arch)

        # Wykres archiwum
        cf = tk.Frame(body, bg=C['panel'])
        cf.pack(side='left', fill='both', expand=True)
        tk.Frame(cf, bg=C['border2'], height=3).pack(fill='x')
        self.fig_a = Figure(figsize=(8, 6), facecolor=C['panel'], dpi=110)
        self.ax_a = self.fig_a.add_subplot(111)
        self.ax_a.set_facecolor(C['panel2'])
        self.cv_a = FigureCanvasTkAgg(self.fig_a, master=cf)
        self.cv_a.get_tk_widget().pack(fill='both', expand=True, padx=8, pady=8)

        self.refresh_arch()

    def refresh_arch(self):
        self.a_list.delete(0, 'end')
        for f in sorted(self.log_dir.glob("cykl_*.csv"), reverse=True):
            self.a_list.insert('end', f"  {f.stem}")

    def load_arch(self, evt=None):
        s = self.a_list.curselection()
        if not s: return
        fs = sorted(self.log_dir.glob("cykl_*.csv"), reverse=True)
        if s[0] >= len(fs): return
        path = fs[s[0]]
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = list(csv.DictReader(f))
        except Exception as e:
            print(f"load err: {e}"); return
        if not data: return
        try:
            t = [float(r['czas_s']) for r in data]
            temp = [float(r['temperatura_C']) for r in data]
            spt = [float(r['setpoint_cel']) for r in data]
        except: return

        self.ax_a.clear()
        self.ax_a.set_facecolor(C['panel2'])
        self.ax_a.plot(t, temp, color=C['blue'], lw=2, label='temp')
        self.ax_a.plot(t, spt, color=C['orange'], lw=1.5, ls='--', label='target')
        self.ax_a.set_xlabel('time [s]', color=C['dim'], fontsize=9)
        self.ax_a.set_ylabel('°C', color=C['dim'], fontsize=9)
        self.ax_a.tick_params(colors=C['dim'], labelsize=8)
        self.ax_a.legend(facecolor=C['panel'], edgecolor=C['border'],
                        labelcolor=C['dim'], fontsize=9)
        self.ax_a.grid(True, alpha=0.3, color=C['grid'])
        for sp in self.ax_a.spines.values():
            sp.set_color(C['border'])
        self.fig_a.tight_layout()
        self.cv_a.draw()


    # ────────────────────────────────────────────────────
    #  TICK + WYKRES
    # ────────────────────────────────────────────────────
    def tick(self):
        try:
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
                elif self.cyc_on and prev == 'COOLDOWN' and state == 'MAN':
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
                        dT = abs(self.reach_target - self.reach_start_temp)
                        if self.reach_time > 0:
                            self.reach_avg_rate = dT / (self.reach_time / 60.0)
                        # Zapisz do CSV cyklu jako komentarz
                        if self.cyc_on and self.cyc_wr:
                            try:
                                self.cyc_wr.writerow([
                                    f"# REACHED target={self.reach_target:.1f}C",
                                    f"time={self.reach_time:.1f}s",
                                    f"avg_rate={self.reach_avg_rate:.2f}C/min", '', '', '', '', '', '', ''])
                                self.cyc_file.flush()
                            except: pass
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
        self.cards['sp']['val'].config(text=f"{spt:.1f}")
        # Tempo - srednia z ostatnich probek
        rate = 0.0
        if len(self.temp) > 10:
            dt = self.t[-1] - self.t[-10]
            if dt > 0: rate = (self.temp[-1] - self.temp[-10]) / dt * 60
        self.cards['rate']['val'].config(text=f"{rate:+.1f}")
        # PWM + kierunek (HEAT/COOL/HOLD widoczny w jednostce)
        diff = spt - temp
        arrow = "% ▲HEAT" if diff > 0.3 else ("% ▼COOL" if diff < -0.3 else "% ●HOLD")
        self.cards['pwm']['val'].config(text=f"{pwm:.0f}")
        # Kolor kierunku
        acol = C['red'] if diff > 0.3 else (C['cyan'] if diff < -0.3 else C['dim2'])
        self.cards['pwm']['unit_lbl'].config(text=" " + arrow, fg=acol)

        # Statystyki dotarcia do setpointu
        if hasattr(self, 'reach_lbl'):
            if self.reach_done and self.reach_time is not None:
                m = int(self.reach_time // 60); s = int(self.reach_time % 60)
                tstr = f"{m}m {s}s" if m > 0 else f"{s}s"
                rate_str = f"{self.reach_avg_rate:.2f}" if self.reach_avg_rate else "?"
                self.reach_lbl.config(
                    text=f"✓ REACHED in {tstr} · avg {rate_str}°C/min", fg=C['green'])
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
        t = self.t; temp = self.temp; spt = self.spt; spa = self.spa; pwm = self.pwm

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
                              'setpoint_cel', 'PWM', 'PWM_%', 'Kp', 'Ki', 'Kd', 'stan'])
        self.cyc_rows = 0
        print(f"CYC START T={temp0:.1f}")

    def cyc_log(self, t, temp, sa, st, pwm, kp, ki, kd, state):
        if self.cyc_wr:
            try:
                self.cyc_wr.writerow([f"{t:.2f}", f"{temp:.2f}", f"{sa:.2f}",
                                     f"{st:.2f}", pwm, f"{pwm*100/255:.1f}",
                                     f"{kp:.3f}", f"{ki:.4f}", f"{kd:.3f}", state])
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
        """Zapisz cykl pod podana nazwa do archiwum"""
        import re as _re
        safe = _re.sub(r'[^\w\-]', '_', name.strip()) or "cykl"
        ts = getattr(self, 'cyc_ts', datetime.now().strftime("%Y%m%d_%H%M%S"))
        dest = self.log_dir / f"cykl_{safe}_{ts}.csv"
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
        self.win.geometry("480x520")
        self.win.transient(parent)
        self.win.grab_set()

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

        tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=(4, 12))

        # Krok temperatury (info - firmware uzywa co 10C)
        tk.Label(inner, text="STEP: 10°C (fixed)", bg=C['bg'], fg=C['dim2'],
                 font=(FONT, fsz(9))).pack(anchor='w', pady=(0, 12))

        # Lista ramp do zaznaczenia
        tk.Label(inner, text="RAMPS TO CALIBRATE [°C/min]:", bg=C['bg'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(anchor='w', pady=(0, 8))

        ramps_frame = tk.Frame(inner, bg=C['bg'])
        ramps_frame.pack(fill='x', pady=(0, 12))
        # Dostepne rampy (zgodne z firmware PR[]={2,5,10,20})
        self.ramp_vars = {}
        for r in [2, 5, 10, 20]:
            var = tk.BooleanVar(value=True)
            self.ramp_vars[r] = var
            cb = tk.Checkbutton(ramps_frame, text=f"{r}", variable=var,
                               bg=C['bg2'], fg=C['text'], font=(FONT, fsz(11), 'bold'),
                               selectcolor=C['panel'], activebackground=C['bg2'],
                               activeforeground=C['cyan'], bd=0,
                               highlightthickness=0, padx=16, pady=8)
            cb.pack(side='left', padx=4)

        tk.Label(inner, text="Note: firmware calibrates selected ramps.\n"
                 "More ramps/temps = longer calibration.",
                 bg=C['bg'], fg=C['dim2'], font=(FONT, fsz(8)), justify='left').pack(
                 anchor='w', pady=(0, 16))

        # Szacowany czas
        self.est_lbl = tk.Label(inner, text="", bg=C['bg'], fg=C['yellow'],
                               font=(FONT, fsz(10), 'bold'))
        self.est_lbl.pack(anchor='w', pady=(0, 12))
        self._update_estimate()
        # Aktualizuj szacunek przy zmianach
        for r, var in self.ramp_vars.items():
            var.trace_add('write', lambda *a: self._update_estimate())

        # Przyciski
        bf = tk.Frame(inner, bg=C['bg'])
        bf.pack(fill='x')
        mk_btn(bf, "▶ START CALIBRATION", self.start, C['purple'], fg='#fff').pack(
            side='left', fill='x', expand=True, padx=(0, 4))
        mk_btn_outline(bf, "CANCEL", self.win.destroy, C['dim']).pack(
            side='left', fill='x', expand=True, padx=(4, 0))

    def _update_estimate(self):
        try:
            tmin = self.sl_tmin.get(); tmax = self.sl_tmax.get()
            n_temps = max(1, int((tmax - tmin) / 10) + 1)
            n_ramps = sum(1 for v in self.ramp_vars.values() if v.get())
            total = n_temps * n_ramps
            # Szacunek ~3-5 min na krok
            est_min = total * 4
            self.est_lbl.config(
                text=f"≈ {total} steps · ~{est_min} min total")
        except: pass

    def start(self):
        tmin = self.sl_tmin.get(); tmax = self.sl_tmax.get()
        if tmax <= tmin:
            messagebox.showerror("Invalid range", "TEMP TO must be greater than TEMP FROM.")
            return
        ramps = [r for r, v in self.ramp_vars.items() if v.get()]
        if not ramps:
            messagebox.showerror("No ramps", "Select at least one ramp.")
            return
        n_temps = int((tmax - tmin) / 10) + 1
        total = n_temps * len(ramps)
        if not messagebox.askyesno("Start calibration",
                f"Start auto-calibration?\n\n"
                f"Range: {tmin:.0f}-{tmax:.0f}°C (step 10°C)\n"
                f"Ramps: {', '.join(str(r) for r in ramps)} °C/min\n"
                f"Total: {total} steps\n\n"
                "Takes several minutes. Can be stopped with STOP."):
            return
        self.app.start_autocal(tmin, tmax, ramps)
        self.win.destroy()


# ════════════════════════════════════════════════════════
#  OKNO POSTĘPU KALIBRACJI
# ════════════════════════════════════════════════════════
class CalibrationWindow:
    def __init__(self, parent, app):
        self.app = app
        self.win = tk.Toplevel(parent)
        self.win.title("Postęp kalibracji")
        self.win.configure(bg=C['bg'])
        self.win.geometry("560x640")
        self.win.transient(parent)

        tk.Frame(self.win, bg=C['purple'], height=4).pack(fill='x')
        inner = tk.Frame(self.win, bg=C['bg'])
        inner.pack(fill='both', expand=True, padx=20, pady=16)

        tk.Label(inner, text="POSTĘP KALIBRACJI", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(14), 'bold')).pack(anchor='w')

        # Pasek postepu
        self.prog_frame = tk.Frame(inner, bg=C['bg2'], height=32)
        self.prog_frame.pack(fill='x', pady=(12, 4))
        self.prog_frame.pack_propagate(False)
        self.prog_bar = tk.Frame(self.prog_frame, bg=C['purple'], height=32)
        self.prog_bar.place(x=0, y=0, relheight=1, relwidth=0)
        self.prog_text = tk.Label(self.prog_frame, text="0 / 0", bg=C['bg2'],
                                  fg=C['text'], font=(FONT, fsz(11), 'bold'))
        self.prog_text.place(relx=0.5, rely=0.5, anchor='center')

        # Info: aktualny / nastepny / ETA
        info = tk.Frame(inner, bg=C['panel'])
        info.pack(fill='x', pady=(8, 12))
        ii = tk.Frame(info, bg=C['panel'])
        ii.pack(fill='x', padx=14, pady=10)

        row1 = tk.Frame(ii, bg=C['panel']); row1.pack(fill='x', pady=2)
        tk.Label(row1, text="TERAZ:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(9)), width=10, anchor='w').pack(side='left')
        self.lbl_now = tk.Label(row1, text="—", bg=C['panel'], fg=C['orange'],
                                font=(FONT, fsz(11), 'bold'), anchor='w')
        self.lbl_now.pack(side='left')

        row2 = tk.Frame(ii, bg=C['panel']); row2.pack(fill='x', pady=2)
        tk.Label(row2, text="NASTĘPNY:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(9)), width=10, anchor='w').pack(side='left')
        self.lbl_next = tk.Label(row2, text="—", bg=C['panel'], fg=C['cyan'],
                                 font=(FONT, fsz(11)), anchor='w')
        self.lbl_next.pack(side='left')

        row3 = tk.Frame(ii, bg=C['panel']); row3.pack(fill='x', pady=2)
        tk.Label(row3, text="POZOSTAŁO:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(9)), width=10, anchor='w').pack(side='left')
        self.lbl_eta = tk.Label(row3, text="—", bg=C['panel'], fg=C['yellow'],
                                font=(FONT, fsz(11), 'bold'), anchor='w')
        self.lbl_eta.pack(side='left')

        # Lista krokow
        tk.Label(inner, text="WSZYSTKIE KROKI", bg=C['bg'], fg=C['dim'],
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

        # Przycisk przerwij
        mk_btn_outline(inner, "■ PRZERWIJ KALIBRACJĘ", self.abort, C['red']).pack(
            fill='x', pady=(12, 0))

        self.step_widgets = []
        self.refresh()

    def refresh(self):
        app = self.app
        total = app.cal_total or len(app.cal_plan)
        cur = app.cal_current

        # Pasek
        frac = (cur / total) if total else 0
        self.prog_bar.place_configure(relwidth=frac)
        self.prog_text.config(text=f"{cur} / {total}")

        # Teraz
        if app.cal_cur_temp is not None and app.cal_cur_ramp is not None:
            self.lbl_now.config(text=f"{app.cal_cur_temp:.0f}°C @ {app.cal_cur_ramp:.0f}°C/min")
        # Nastepny
        if cur < len(app.cal_plan):
            nt, nr = app.cal_plan[cur] if cur < len(app.cal_plan) else (None, None)
            if nt is not None:
                self.lbl_next.config(text=f"{nt:.0f}°C @ {nr:.0f}°C/min")
        else:
            self.lbl_next.config(text="(ostatni krok)")
        # ETA
        eta = app._cal_eta()
        if eta is not None:
            m = int(eta // 60); s = int(eta % 60)
            self.lbl_eta.config(text=f"~{m} min {s} s")
        elif cur >= total and total > 0:
            self.lbl_eta.config(text="ZAKOŃCZONO ✓")

        # Lista krokow - buduj raz, potem aktualizuj kolory
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
                txt = tk.Label(row, text=f"{t:.0f}°C  @  {r:.0f}°C/min",
                              bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(10)), anchor='w')
                txt.pack(side='left', fill='x', expand=True)
                stat = tk.Label(row, text="", bg=C['bg2'], fg=C['dim2'],
                               font=(FONT, fsz(9)), anchor='e', width=12)
                stat.pack(side='right')
                self.step_widgets.append((bar, num, txt, stat))

        # Aktualizuj kolory/statusy
        for i, (bar, num, txt, stat) in enumerate(self.step_widgets):
            step_no = i + 1
            if step_no < cur:
                bar.config(bg=C['green']); txt.config(fg=C['dim2'])
                num.config(fg=C['green']); stat.config(text="✓ gotowe", fg=C['green'])
            elif step_no == cur:
                bar.config(bg=C['orange']); txt.config(fg=C['text'])
                num.config(fg=C['orange']); stat.config(text="● TERAZ", fg=C['orange'])
                # Przewin do aktualnego
                try: self.canvas.yview_moveto(max(0, (i-3))/max(1,len(self.step_widgets)))
                except: pass
            else:
                bar.config(bg=C['bg2']); txt.config(fg=C['dim'])
                num.config(fg=C['dim2']); stat.config(text="oczekuje", fg=C['dim2'])

    def abort(self):
        if messagebox.askyesno("Przerwać?", "Przerwać kalibrację?"):
            self.app.send("AUTOCALSTOP")
            self.app.send("STOP")
            self.app.cal_running = False
            self.win.destroy()


# ════════════════════════════════════════════════════════
#  DIALOG ZAPISU CYKLU
# ════════════════════════════════════════════════════════
class SaveCycleDialog:
    def __init__(self, parent, app, tmp_path):
        self.app = app
        self.tmp_path = tmp_path
        self.win = tk.Toplevel(parent)
        self.win.title("Zapisz cykl")
        self.win.configure(bg=C['bg'])
        self.win.geometry("440x230")
        self.win.transient(parent)
        self.win.grab_set()  # modalne

        tk.Frame(self.win, bg=C['green'], height=4).pack(fill='x')
        inner = tk.Frame(self.win, bg=C['bg'])
        inner.pack(fill='both', expand=True, padx=24, pady=20)

        tk.Label(inner, text="ZAPISZ CYKL DO ARCHIWUM", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(13), 'bold')).pack(anchor='w')

        # Info ile probek
        rows = getattr(app, 'cyc_rows', 0)
        tk.Label(inner, text=f"Zarejestrowano {rows} próbek pomiarowych",
                 bg=C['bg'], fg=C['dim'], font=(FONT, fsz(9))).pack(anchor='w', pady=(4, 16))

        tk.Label(inner, text="Nazwa cyklu:", bg=C['bg'], fg=C['dim'],
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
        mk_btn(bf, "ZAPISZ", self.save, C['green']).pack(side='left', fill='x',
                                                          expand=True, padx=(0, 4))
        mk_btn_outline(bf, "ODRZUĆ", self.discard, C['red']).pack(side='left',
                                                          fill='x', expand=True, padx=(4, 0))

        self.win.protocol("WM_DELETE_WINDOW", self.save)  # zamkniecie = zapisz

    def save(self):
        name = self.entry.get().strip()
        if not name:
            name = datetime.now().strftime("cykl_%H%M")
        self.app.save_cycle_as(self.tmp_path, name)
        self.win.destroy()

    def discard(self):
        if messagebox.askyesno("Odrzucić?",
                "Na pewno odrzucić ten cykl?\nDane zostaną bezpowrotnie usunięte."):
            self.app.discard_cycle(self.tmp_path)
            self.win.destroy()


# ════════════════════════════════════════════════════════
#  OKNO PROFILI WIELOETAPOWYCH
# ════════════════════════════════════════════════════════
class ProfileWindow:
    def __init__(self, parent, app):
        self.app = app
        self.win = tk.Toplevel(parent)
        self.win.title("Profile wieloetapowe")
        self.win.configure(bg=C['bg'])
        self.win.geometry("520x480")
        self.win.transient(parent)

        tk.Frame(self.win, bg=C['purple'], height=4).pack(fill='x')
        hd = tk.Frame(self.win, bg=C['bg'])
        hd.pack(fill='x', padx=16, pady=12)
        tk.Label(hd, text="PROFILE WIELOETAPOWE", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(12), 'bold')).pack(side='left')

        # Tabela etapow
        self.rows_frame = tk.Frame(self.win, bg=C['bg'])
        self.rows_frame.pack(fill='both', expand=True, padx=16)

        # Naglowki
        h = tk.Frame(self.rows_frame, bg=C['bg'])
        h.pack(fill='x', pady=(0, 4))
        for txt, w in [("#", 3), ("TEMP °C", 10), ("RAMPA", 8), ("CZAS min", 10), ("", 6)]:
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
        tk.Label(ai, text="DODAJ ETAP:", bg=C['panel'], fg=C['dim'],
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
        mk_btn(ai, "+ DODAJ", self.add_step, C['green']).pack(side='left', padx=(8, 0))

        # Uruchom
        rf = tk.Frame(self.win, bg=C['bg'])
        rf.pack(fill='x', padx=16, pady=(0, 12))
        mk_btn(rf, "▶ URUCHOM PROFIL", self.run_profile, C['purple'], fg='#fff').pack(
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
            messagebox.showerror("Blad", "Wpisz poprawne liczby.")

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
            tk.Button(r, text="USUN", command=lambda idx=i: self.del_step(idx),
                      bg=C['bg2'], fg=C['red'], font=(FONT, fsz(8), 'bold'),
                      relief='flat', cursor='hand2', bd=0,
                      activebackground=C['panel3']).pack(side='left', padx=4)

    def run_profile(self):
        """Wykonaj profil - sekwencyjnie wysylaj etapy z opoznieniem"""
        if not self.app.connected:
            messagebox.showwarning("Brak polaczenia", "Polacz sie z urzadzeniem.")
            return
        if not self.app.profile_steps:
            messagebox.showinfo("Pusty profil", "Dodaj przynajmniej jeden etap.")
            return
        if not messagebox.askyesno("Uruchom profil",
                f"Wykonac profil z {len(self.app.profile_steps)} etapami?\n"
                "Etapy beda wykonywane sekwencyjnie."):
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
