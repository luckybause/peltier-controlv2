#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PeltierControl v6.0 - BRUTALIST
Pelny panel sterowania PID Peltiera z dwukierunkowa komunikacja.
Sterowanie z aplikacji: setpoint, rampa, PID, kalibracja, profile.
Wymaga firmware v19 (PC MODE) na ItsyBitsy M0.
"""

import sys, os, time, csv, threading, queue
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
                  font=(FONT, 10, 'bold'), padx=16, pady=8,
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
                  font=(FONT, 10, 'bold'), padx=14, pady=7,
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
                 font=(FONT, 9), anchor='w').pack(side='left')
        if unit:
            tk.Label(top, text=unit, bg=C['bg2'], fg=C['dim2'],
                     font=(FONT, 8), anchor='e').pack(side='right')

        # Wiersz: suwak + pole
        row = tk.Frame(self.frame, bg=C['bg2'])
        row.pack(fill='x', pady=(4, 0))

        # Pole liczbowe (Entry) - po prawej
        self.entry = tk.Entry(row, width=7, bg=C['panel'], fg=color,
                              font=(FONT, 12, 'bold'), justify='center',
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
                     padding=[20, 10], font=(FONT, 10, 'bold'), borderwidth=0)
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
            # Po         konfiguracji startowej
            self.root.after(1500, lambda: self.send("GET"))
        except Exception as e:
            messagebox.showerror("Blad", f"{port}:\n{e}")
            self.set_status(False, "")

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

                # Status kalibracji CALSTAT:5/36,T=40,R=2
                if raw.startswith("CALSTAT:"):
                    txt = raw[8:]
                    self.root.after(0, lambda t=txt: self.cal_status.config(
                        text=f"Kalibracja: {t}"))
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
        except Exception as e:
            print(f"apply_cfg err: {e}")


    # ────────────────────────────────────────────────────
    #  BUDOWA UI
    # ────────────────────────────────────────────────────
    def _build_ui(self):
        # Pasek tytulowy z lampka statusu
        top = tk.Frame(self.root, bg=C['bg2'], height=44)
        top.pack(fill='x'); top.pack_propagate(False)
        tk.Frame(top, bg=C['red'], width=6).pack(side='left', fill='y')
        tk.Label(top, text="  PELTIER CONTROL", bg=C['bg2'], fg=C['text'],
                 font=(FONT, 13, 'bold')).pack(side='left', padx=(8, 0))
        tk.Label(top, text="v6.0", bg=C['bg2'], fg=C['dim2'],
                 font=(FONT, 9)).pack(side='left', padx=8)

        # Status po prawej
        sf = tk.Frame(top, bg=C['bg2'])
        sf.pack(side='right', padx=16)
        self.s_dot = tk.Canvas(sf, width=14, height=14, bg=C['bg2'], highlightthickness=0)
        self.s_dot.pack(side='left', padx=(0, 8))
        self._draw_dot(C['dim2'], glow=False)
        self.s_lbl = tk.Label(sf, text="ROZLACZONY", bg=C['bg2'], fg=C['dim'],
                              font=(FONT, 10))
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
            self.s_lbl.config(text=msg or "POLACZONY", fg=C['green'])
        else:
            self._draw_dot(C['dim2'], glow=False)
            self.s_lbl.config(text=msg or "ROZLACZONY", fg=C['dim'])
        # Aktywuj/dezaktywuj panel
        if hasattr(self, 'btn_start'):
            self._set_panel_enabled(connected)

    # ────────────────────────────────────────────────────
    #  EKRAN LIVE: wykres (lewo) + panel sterowania (prawo)
    # ────────────────────────────────────────────────────
    def build_live(self, parent):
        # Gorne karty statystyk
        cards = tk.Frame(parent, bg=C['bg'])
        cards.pack(fill='x', padx=16, pady=(12, 8))
        self.cards = {}
        self.cards['temp'] = self._stat_card(cards, "TEMPERATURA", "°C", C['blue'])
        self.cards['sp']   = self._stat_card(cards, "SETPOINT CEL", "°C", C['orange'])
        self.cards['rate'] = self._stat_card(cards, "TEMPO", "°C/min", C['yellow'])
        self.cards['pwm']  = self._stat_card(cards, "MOC PWM", "%", C['green'])

        # Glowny obszar: wykres + panel
        main = tk.Frame(parent, bg=C['bg'])
        main.pack(fill='both', expand=True, padx=16, pady=(0, 12))

        # LEWO - wykres
        self._build_chart(main)
        # PRAWO - panel sterowania
        self._build_panel(main)

    def _stat_card(self, parent, title, unit, color):
        card = tk.Frame(parent, bg=C['panel'])
        card.pack(side='left', fill='x', expand=True, padx=(0, 8))
        tk.Frame(card, bg=color, height=3).pack(fill='x')
        inner = tk.Frame(card, bg=C['panel'])
        inner.pack(fill='both', expand=True, padx=14, pady=10)
        tk.Label(inner, text=title, bg=C['panel'], fg=C['dim2'],
                 font=(FONT, 9), anchor='w').pack(anchor='w')
        vrow = tk.Frame(inner, bg=C['panel'])
        vrow.pack(anchor='w', pady=(4, 0))
        val = tk.Label(vrow, text="--", bg=C['panel'], fg=color,
                       font=(FONT, 26, 'bold'))
        val.pack(side='left')
        unit_lbl = tk.Label(vrow, text=" " + unit, bg=C['panel'], fg=C['dim2'],
                            font=(FONT, 10))
        unit_lbl.pack(side='left', pady=(8, 0))
        return {'val': val, 'unit': unit, 'unit_lbl': unit_lbl, 'extra': None, 'row': vrow}

    def _build_chart(self, parent):
        wrap = tk.Frame(parent, bg=C['panel'])
        wrap.pack(side='left', fill='both', expand=True, padx=(0, 12))
        tk.Frame(wrap, bg=C['border2'], height=3).pack(fill='x')

        hd = tk.Frame(wrap, bg=C['panel'])
        hd.pack(fill='x', padx=14, pady=(10, 4))
        tk.Label(hd, text="PRZEBIEG W CZASIE", bg=C['panel'], fg=C['dim'],
                 font=(FONT, 10, 'bold')).pack(side='left')

        self.fig = Figure(figsize=(9, 6), facecolor=C['panel'])
        gs = self.fig.add_gridspec(2, 1, height_ratios=[3, 1], hspace=0.2,
                                   left=0.07, right=0.97, top=0.97, bottom=0.08)
        self.ax1 = self.fig.add_subplot(gs[0])
        self.ax2 = self.fig.add_subplot(gs[1], sharex=self.ax1)
        for ax in [self.ax1, self.ax2]:
            ax.set_facecolor(C['panel2'])

        self.cv = FigureCanvasTkAgg(self.fig, master=wrap)
        self.cv.get_tk_widget().pack(fill='both', expand=True, padx=8, pady=(0, 8))

    def _build_panel(self, parent):
        """Prawy panel sterowania - waski pasek"""
        panel = tk.Frame(parent, bg=C['bg2'], width=300)
        panel.pack(side='right', fill='y')
        panel.pack_propagate(False)
        tk.Frame(panel, bg=C['red'], width=6).pack(side='left', fill='y')

        inner = tk.Frame(panel, bg=C['bg2'])
        inner.pack(fill='both', expand=True, padx=16, pady=14)

        tk.Label(inner, text="STEROWANIE", bg=C['bg2'], fg=C['text'],
                 font=(FONT, 13, 'bold')).pack(anchor='w')
        tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=(8, 12))

        # Suwaki nastaw
        self.sl_sp = SliderField(inner, "TEMP. DOCELOWA", -15, 100, 25.0,
                                 C['orange'], "°C", 1,
                                 on_change=lambda v: self.send(f"SP:{v:.1f}"))
        self.sl_ru = SliderField(inner, "RAMPA GRZANIA", 0.5, 40, 2.0,
                                 C['yellow'], "°C/min", 1,
                                 on_change=lambda v: self.send(f"RU:{v:.1f}"))
        self.sl_rd = SliderField(inner, "RAMPA CHLODZENIA", 0.5, 40, 2.0,
                                 C['cyan'], "°C/min", 1,
                                 on_change=lambda v: self.send(f"RD:{v:.1f}"))
        self.sl_tmax = SliderField(inner, "MAX TEMP (ZABEZP.)", 50, 115, 80,
                                   C['red'], "°C", 0,
                                   on_change=lambda v: self.send(f"TMAX:{v:.0f}"))

        tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=(2, 12))

        # PID
        pid_hd = tk.Frame(inner, bg=C['bg2'])
        pid_hd.pack(fill='x', pady=(0, 8))
        tk.Label(pid_hd, text="PID", bg=C['bg2'], fg=C['dim'],
                 font=(FONT, 10, 'bold')).pack(side='left')
        self.btn_st = mk_btn(pid_hd, "SELF-TUNE", self.do_selftune, C['cyan'])
        self.btn_st.pack(side='right')

        # Auto-kalibracja (pelna - 36 profili)
        cal_row = tk.Frame(inner, bg=C['bg2'])
        cal_row.pack(fill='x', pady=(0, 8))
        self.btn_autocal = mk_btn(cal_row, "⚙ AUTO-KALIBRACJA (36 profili)",
                                  self.do_autocal, C['purple'], fg='#fff')
        self.btn_autocal.pack(fill='x')
        self.cal_status = tk.Label(inner, text="", bg=C['bg2'], fg=C['purple'],
                                   font=(FONT, 8), anchor='w')
        self.cal_status.pack(fill='x', pady=(0, 4))

        self.sl_kp = SliderField(inner, "Kp", 1, 30, 10.0, C['cyan'], "", 1,
                                 on_change=lambda v: self.send(f"KP:{v:.1f}"))
        self.sl_ki = SliderField(inner, "Ki", 0, 3, 0.3, C['cyan'], "", 2,
                                 on_change=lambda v: self.send(f"KI:{v:.2f}"))
        self.sl_kd = SliderField(inner, "Kd", 0, 3, 0.8, C['cyan'], "", 2,
                                 on_change=lambda v: self.send(f"KD:{v:.2f}"))

        tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=(2, 12))

        # Kalibracja
        self.sl_off = SliderField(inner, "OFFSET KALIBRACJI", -20, 20, 0.0,
                                  C['purple'], "°C", 1,
                                  on_change=lambda v: self.send(f"OFFSET:{v:.1f}"))

        tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=(2, 12))

        # AUTO badge
        auto = tk.Frame(inner, bg=C['bg2'], highlightthickness=1,
                        highlightbackground=C['green'])
        auto.pack(fill='x', pady=(0, 12))
        tk.Label(auto, text="● AUTO: kierunek wg setpointu", bg=C['bg2'],
                 fg=C['green'], font=(FONT, 9)).pack(padx=8, pady=6)

        # START / STOP
        bf = tk.Frame(inner, bg=C['bg2'])
        bf.pack(fill='x', pady=(0, 6))
        self.btn_start = mk_btn(bf, "▶ START", self.do_start, C['green'])
        self.btn_start.pack(side='left', fill='x', expand=True, padx=(0, 4))
        self.btn_stop = mk_btn_outline(bf, "■ STOP", self.do_stop, C['red'])
        self.btn_stop.pack(side='left', fill='x', expand=True, padx=(4, 0))

        # E-STOP (awaryjne natychmiastowe zatrzymanie)
        self.btn_estop = mk_btn(inner, "⛔ E-STOP (natychmiast)", self.do_estop, C['red'], fg='#fff')
        self.btn_estop.pack(fill='x', pady=(0, 10))

        # Profile + Flash kompakt
        bf2 = tk.Frame(inner, bg=C['bg2'])
        bf2.pack(fill='x', pady=(0, 6))
        mk_btn_outline(bf2, "PROFILE", self.open_profiles, C['purple']).pack(
            side='left', fill='x', expand=True, padx=(0, 3))
        mk_btn_outline(bf2, "ZAPIS", lambda: self.send("SAVE"), C['green']).pack(
            side='left', fill='x', expand=True, padx=3)
        mk_btn_outline(bf2, "WCZYT", lambda: self.send("LOAD"), C['cyan']).pack(
            side='left', fill='x', expand=True, padx=(3, 0))

        # Reset nastaw
        mk_btn_outline(inner, "↺ RESET NASTAW", self.do_reset, C['dim']).pack(
            fill='x', pady=(0, 8))

        tk.Label(inner, text="▶ START uzywa wartosci z panelu",
                 bg=C['bg2'], fg=C['green'], font=(FONT, 8)).pack(anchor='w', pady=(4, 0))

        self._set_panel_enabled(False)

    def _set_panel_enabled(self, en):
        for sl in ['sl_sp', 'sl_ru', 'sl_rd', 'sl_tmax', 'sl_kp', 'sl_ki', 'sl_kd', 'sl_off']:
            if hasattr(self, sl):
                getattr(self, sl).set_enabled(en)
        st = 'normal' if en else 'disabled'
        for b in ['btn_start', 'btn_stop', 'btn_st', 'btn_autocal', 'btn_estop']:
            if hasattr(self, b):
                getattr(self, b).config(state=st)


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

    def do_selftune(self):
        if not self.connected: return
        if messagebox.askyesno("Self-Tune",
                "Uruchomic auto-strojenie PID?\nTrwa ok. 2 minuty.\n"
                "Urzadzenie musi byc w trybie pracy (START)."):
            self.send("SELFTUNE")

    def do_autocal(self):
        """Pelna automatyczna kalibracja - 36 profili (temp x rampa)"""
        if not self.connected:
            messagebox.showwarning("Brak polaczenia", "Polacz sie z urzadzeniem.")
            return
        if messagebox.askyesno("Auto-Kalibracja",
                "Uruchomic PELNA automatyczna kalibracje?\n\n"
                "Przejdzie przez 36 kombinacji temperatura x rampa\n"
                "i dostroi PID dla kazdej. Zapisze do pamieci Flash.\n\n"
                "UWAGA: trwa kilkadziesiat minut!\n"
                "Mozna przerwac przyciskiem STOP."):
            self.send("AUTOCAL")
            self.cal_status.config(text="Kalibracja uruchomiona...")

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

        tk.Label(inner, text="POLACZENIE SERIAL", bg=C['panel'], fg=C['text'],
                 font=(FONT, 12, 'bold')).pack(anchor='w', pady=(0, 12))

        tk.Label(inner, text="Dostepne porty:", bg=C['panel'], fg=C['dim'],
                 font=(FONT, 10)).pack(anchor='w')

        lf = tk.Frame(inner, bg=C['panel'])
        lf.pack(fill='x', pady=8)
        sb = tk.Scrollbar(lf)
        sb.pack(side='right', fill='y')
        self.conn_list = tk.Listbox(lf, bg=C['bg2'], fg=C['text'],
                                    font=(FONT, 10), height=6,
                                    selectbackground=C['blue'], borderwidth=0,
                                    highlightthickness=1, highlightbackground=C['border'],
                                    yscrollcommand=sb.set, activestyle='none')
        self.conn_list.pack(side='left', fill='both', expand=True)
        sb.config(command=self.conn_list.yview)

        br = tk.Frame(inner, bg=C['panel'])
        br.pack(fill='x', pady=(8, 0))
        mk_btn(br, "ODSWIEZ", self.refresh_ports, C['cyan']).pack(side='left', padx=(0, 8))
        self.conn_btn = mk_btn(br, "POLACZ", self.conn_from_tab, C['green'])
        self.conn_btn.pack(side='left', padx=(0, 8))
        mk_btn_outline(br, "ROZLACZ", self.disconnect, C['red']).pack(side='left')

        # Info
        info = tk.Frame(wrap, bg=C['panel'])
        info.pack(fill='x')
        tk.Frame(info, bg=C['dim2'], height=3).pack(fill='x')
        ii = tk.Frame(info, bg=C['panel'])
        ii.pack(fill='x', padx=20, pady=16)
        tk.Label(ii, text="INSTRUKCJA", bg=C['panel'], fg=C['text'],
                 font=(FONT, 11, 'bold')).pack(anchor='w', pady=(0, 8))
        for line in [
            "1. Podlacz ItsyBitsy (firmware v19 PC MODE) przez USB",
            "2. Wybierz port COM z listy i kliknij POLACZ",
            "3. Suwaki zsynchronizuja sie automatycznie z urzadzeniem",
            "4. Ustaw parametry i kliknij START",
            "5. Wykres pokazuje przebieg na zywo, dane zapisuja sie do CSV",
        ]:
            tk.Label(ii, text=line, bg=C['panel'], fg=C['dim'],
                     font=(FONT, 9), anchor='w').pack(anchor='w', pady=1)

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
        tk.Label(hd, text="ARCHIWUM CYKLI", bg=C['bg'], fg=C['text'],
                 font=(FONT, 12, 'bold')).pack(side='left')
        mk_btn(hd, "ODSWIEZ", self.refresh_arch, C['cyan']).pack(side='right')

        body = tk.Frame(wrap, bg=C['bg'])
        body.pack(fill='both', expand=True)

        # Lista plikow
        lf = tk.Frame(body, bg=C['panel'], width=280)
        lf.pack(side='left', fill='y', padx=(0, 12))
        lf.pack_propagate(False)
        tk.Frame(lf, bg=C['purple'], height=3).pack(fill='x')
        tk.Label(lf, text="ZAPISANE CYKLE", bg=C['panel'], fg=C['dim'],
                 font=(FONT, 10, 'bold')).pack(anchor='w', padx=12, pady=8)
        sb = tk.Scrollbar(lf)
        sb.pack(side='right', fill='y')
        self.a_list = tk.Listbox(lf, bg=C['bg2'], fg=C['text'],
                                font=(FONT, 9), selectbackground=C['purple'],
                                borderwidth=0, highlightthickness=0,
                                yscrollcommand=sb.set, activestyle='none')
        self.a_list.pack(side='left', fill='both', expand=True, padx=8, pady=(0, 8))
        sb.config(command=self.a_list.yview)
        self.a_list.bind('<<ListboxSelect>>', self.load_arch)

        # Wykres archiwum
        cf = tk.Frame(body, bg=C['panel'])
        cf.pack(side='left', fill='both', expand=True)
        tk.Frame(cf, bg=C['border2'], height=3).pack(fill='x')
        self.fig_a = Figure(figsize=(8, 6), facecolor=C['panel'])
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
        self.ax_a.plot(t, spt, color=C['orange'], lw=1.5, ls='--', label='setpoint')
        self.ax_a.set_xlabel('czas [s]', color=C['dim'], fontsize=9)
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
                elif self.cyc_on and prev == 'COOLDOWN' and state == 'MAN':
                    self.cyc_stop("done")
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

    def draw_chart(self):
        if not self.t: return
        t = self.t; temp = self.temp; spt = self.spt; pwm = self.pwm

        self.ax1.clear()
        self.ax1.set_facecolor(C['panel2'])
        # setpoint przerywana
        self.ax1.plot(t, spt, color=C['orange'], lw=1.5, ls='--', label='setpoint')
        # temperatura
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
        self.ax2.set_xlabel('czas [s]', color=C['dim'], fontsize=9)
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
        self.cyc_fn = self.log_dir / f"cykl_{ts}.csv"
        self.cyc_file = open(self.cyc_fn, 'w', newline='', encoding='utf-8')
        self.cyc_wr = csv.writer(self.cyc_file)
        self.cyc_wr.writerow(['czas_s', 'temperatura_C', 'setpoint_aktywny',
                              'setpoint_cel', 'PWM', 'PWM_%', 'Kp', 'Ki', 'Kd', 'stan'])
        print(f"CYC START T={temp0:.1f}")

    def cyc_log(self, t, temp, sa, st, pwm, kp, ki, kd, state):
        if self.cyc_wr:
            try:
                self.cyc_wr.writerow([f"{t:.2f}", f"{temp:.2f}", f"{sa:.2f}",
                                     f"{st:.2f}", pwm, f"{pwm*100/255:.1f}",
                                     f"{kp:.3f}", f"{ki:.4f}", f"{kd:.3f}", state])
                self.cyc_file.flush()
            except: pass

    def cyc_stop(self, reason=""):
        if self.cyc_file:
            try: self.cyc_file.close()
            except: pass
        self.cyc_on = False; self.cyc_file = None; self.cyc_wr = None
        print(f"CYC STOP: {reason}")
        if hasattr(self, 'refresh_arch'):
            try: self.refresh_arch()
            except: pass


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
                 font=(FONT, 12, 'bold')).pack(side='left')

        # Tabela etapow
        self.rows_frame = tk.Frame(self.win, bg=C['bg'])
        self.rows_frame.pack(fill='both', expand=True, padx=16)

        # Naglowki
        h = tk.Frame(self.rows_frame, bg=C['bg'])
        h.pack(fill='x', pady=(0, 4))
        for txt, w in [("#", 3), ("TEMP °C", 10), ("RAMPA", 8), ("CZAS min", 10), ("", 6)]:
            tk.Label(h, text=txt, bg=C['bg'], fg=C['dim2'],
                     font=(FONT, 9), width=w, anchor='w').pack(side='left')

        self.steps_container = tk.Frame(self.rows_frame, bg=C['bg'])
        self.steps_container.pack(fill='both', expand=True)

        # Formularz dodawania
        addf = tk.Frame(self.win, bg=C['panel'])
        addf.pack(fill='x', padx=16, pady=12)
        tk.Frame(addf, bg=C['green'], height=3).pack(fill='x')
        ai = tk.Frame(addf, bg=C['panel'])
        ai.pack(fill='x', padx=12, pady=10)
        tk.Label(ai, text="DODAJ ETAP:", bg=C['panel'], fg=C['dim'],
                 font=(FONT, 9)).pack(side='left', padx=(0, 8))
        self.e_temp = tk.Entry(ai, width=6, bg=C['bg2'], fg=C['orange'],
                               font=(FONT, 10), justify='center', relief='flat',
                               highlightthickness=1, highlightbackground=C['border'])
        self.e_temp.pack(side='left', padx=2); self.e_temp.insert(0, "40")
        self.e_ramp = tk.Entry(ai, width=6, bg=C['bg2'], fg=C['yellow'],
                               font=(FONT, 10), justify='center', relief='flat',
                               highlightthickness=1, highlightbackground=C['border'])
        self.e_ramp.pack(side='left', padx=2); self.e_ramp.insert(0, "2.0")
        self.e_time = tk.Entry(ai, width=6, bg=C['bg2'], fg=C['dim'],
                               font=(FONT, 10), justify='center', relief='flat',
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
                     font=(FONT, 10, 'bold'), width=3, anchor='w').pack(side='left', padx=(6,0))
            tk.Label(r, text=f"{s['temp']:.0f}", bg=C['bg2'], fg=C['orange'],
                     font=(FONT, 10), width=10, anchor='w').pack(side='left')
            tk.Label(r, text=f"{s['ramp']:.1f}", bg=C['bg2'], fg=C['yellow'],
                     font=(FONT, 10), width=8, anchor='w').pack(side='left')
            tk.Label(r, text=f"{s['time']:.0f}", bg=C['bg2'], fg=C['dim'],
                     font=(FONT, 10), width=10, anchor='w').pack(side='left')
            tk.Button(r, text="USUN", command=lambda idx=i: self.del_step(idx),
                      bg=C['bg2'], fg=C['red'], font=(FONT, 8, 'bold'),
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
def main():
    root = tk.Tk()
    app = PeltierControl(root)

    def on_close():
        app.disconnect()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == "__main__":
    main()
