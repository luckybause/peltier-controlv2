#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IGNI - Illumination & Gradient Nanoampere Instrument

Control bench for photocurrent and pyrocurrent measurements: it drives the
Peltier stage, so the sample can be held at a temperature (photocurrent, the
illumination term) or ramped at a commanded rate (pyrocurrent, the gradient
term dT/dt), and it archives every run.

  Illumination  - the optical / photocurrent side of the experiment
  Gradient      - the controlled dT/dt that produces the pyrocurrent
  Nanoampere    - the magnitude of what actually gets measured
  Instrument    - it is one, and it is treated as one

Two-way link with the board: setpoint, ramps, PID, calibration, profiles.
Requires firmware v19 (PC MODE) or newer on the ItsyBitsy M0.
"""

import sys, os, time, csv, json, threading, queue, contextlib
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
    print("tkinter not available"); input(); sys.exit(1)
try:
    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
except ImportError as e:
    print(f"pip install matplotlib numpy\n{e}"); input(); sys.exit(1)

# ════════════════════════════════════════════════════════
#  BRUTALIST THEME - concrete, steel, raw edges
# ════════════════════════════════════════════════════════
# ── PALETTE ─────────────────────────────────────────────────────────────
# Reworked in APP .15. The old palette was cold "concrete grey"; this one is
# warmer and darker - charcoal and oiled leather with burnished bronze - and
# every accent is picked from the Witcher-sign family, which happens to map
# cleanly onto what this rig already needed to signal:
#     Aard  (pale steel-blue)  -> temperature, the thing we measure
#     Igni  (ember amber)      -> setpoint / heating, the thing we command
#     Quen  (shield gold)      -> rate, headings, the medallion
#     Axii  (moss green)       -> PWM / OK / START
#     Yrden (violet)           -> calibration and profiles
#     blood red                -> STOP, alarms, recording
# The KEYS are unchanged, so every widget keeps its meaning; only the hues
# moved. Contrast was checked against the panel background - the readouts a
# safety-relevant instrument depends on stay high-contrast on purpose.
C = {
    'bg':       '#22201d',   # charcoal, faintly warm
    'bg2':      '#191715',   # near-black (bars, fields)
    'panel':    '#2a2724',   # cards
    'panel2':   '#191715',   # inner elements
    'panel3':   '#38342f',   # hover
    'border':   '#3d3833',   # frames
    'border2':  '#575047',   # lighter frames
    'text':     '#efe7d8',   # parchment
    'dim':      '#b3a892',   # dimmed parchment
    'dim2':     '#736a5c',   # very dimmed
    'blue':     '#7fb6d9',   # Aard  - temperature
    'orange':   '#d98436',   # Igni  - setpoint
    'yellow':   '#d9b036',   # Quen  - rate / headings
    'green':    '#8fae5c',   # Axii  - pwm / ok / start
    'red':      '#a3251f',   # blood - stop / alarm
    'cyan':     '#6fb2b8',   # frost - cooling
    'purple':   '#8f6bb5',   # Yrden - calibration / profiles
    'rec':      '#a3251f',   # recording
    'grid':     '#312d29',   # chart grid
    'gold':     '#c8a24a',   # medallion / section rules
}

# Fonts - monospace for brutalist
FONT      = 'Consolas'
FONT_UI   = 'Roboto Mono'   # falls back to Consolas if not available

# Version number of the PC APPLICATION (not to be confused with FW - the
# firmware version the app reads from the board with the VER command and shows
# separately in the title bar). Bump it with every version sent out, so the
# title bar immediately tells you whether this really is the new file.
APP_BUILD = "2026-09-01.16"

# Safety limits for the automatic MEASUREMENT SERIES (see the PeltierControl
# class, self.series_*) - a safeguard in case the approach/return never
# reaches its target (e.g. a disconnected sensor) - the series should move
# on instead of hanging forever.
SERIES_HEAT_TIMEOUT_S = 20 * 60
SERIES_COOL_TIMEOUT_S = 12 * 60

# ── "REACHED" CRITERION ──────────────────────────────────────────────────
# Was: a single sample within 0.5 C and done. For the DESCENT leg in a
# SERIES (which ends immediately after reaching), this meant that EVERY
# descent was cut off the moment it entered tolerance - in the logs from
# 20260831 all seven descents end at 30.35-30.49 C and contain NOT A
# SINGLE hold sample. That made it impossible to check at all whether the
# descent gets to the target and whether it crosses it - and that is
# exactly what we were trying to diagnose.
# Now: |error| <= REACH_TOL_C held CONTINUOUSLY for REACH_STABLE_S.
# A single brush against the tolerance caused by noise is no longer enough.
REACH_TOL_C    = 0.2
REACH_STABLE_S = 3.0

# Global font size multiplier (set at startup according to DPI)
FS = 1.0
def fsz(n):
    """Scales the font size according to the global DPI."""
    return max(6, int(round(n * FS)))

def SC(px):
    """Scales a size IN PIXELS by the same multiplier as the fonts.

    THE PROBLEM THIS FIXES: fonts were scaled by FS (e.g. x1.5 at DPI
    150%), but window sizes were HARD-CODED IN PIXELS ("640x780" etc.).
    Because the application is Per-Monitor DPI Aware v2, Windows DOES NOT
    SCALE IT - those 640x780 are physical pixels, so on a high-DPI screen
    the window is PHYSICALLY SMALLER while the text in it is AT THE SAME
    TIME larger. The effect: the text did not fit and was clipped - worst
    of all in the calibration windows, because they have the most content
    (560x680 and 640x780). Now every fixed size goes through SC().
    """
    return int(round(px * FS))

def make_scrollable(parent, bg, padx=0, pady=0):
    """Returns a frame that scrolls vertically when the content does not fit the window.

    Why: at high DPI, windows are clipped to the screen height (see
    size_win), so the content can be taller than the window. Without
    scrolling the bottom fields and buttons are then physically unreachable.
    """
    wrap = tk.Frame(parent, bg=bg)
    wrap.pack(fill='both', expand=True)
    cv = tk.Canvas(wrap, bg=bg, highlightthickness=0, bd=0)
    sb = tk.Scrollbar(wrap, orient='vertical', command=cv.yview)
    cv.configure(yscrollcommand=sb.set)
    sb.pack(side='right', fill='y')
    cv.pack(side='left', fill='both', expand=True, padx=padx, pady=pady)
    inner = tk.Frame(cv, bg=bg)
    wid = cv.create_window((0, 0), window=inner, anchor='nw')

    def _cfg(_e=None):
        try:
            cv.configure(scrollregion=cv.bbox('all'))
            cv.itemconfigure(wid, width=cv.winfo_width())
        except Exception:
            pass
    inner.bind('<Configure>', _cfg)
    cv.bind('<Configure>', _cfg)

    def _wheel(e):
        try:
            cv.yview_scroll(int(-e.delta / 120), 'units')
        except Exception:
            pass
    # Mouse wheel only while the cursor is over this area - so that it does not
    # take over scrolling for the whole application.
    cv.bind('<Enter>', lambda e: cv.bind_all('<MouseWheel>', _wheel))
    cv.bind('<Leave>', lambda e: cv.unbind_all('<MouseWheel>'))
    return inner


def size_win(win, w, h, minw=None, minh=None, parent=None):
    """Set the window size: scale by DPI, clip to the screen, center it.

    Clipping to the screen matters: after x1.5 scaling a 640x780 window would
    grow to 960x1170, that is MORE than the height of a typical 1080p screen -
    and part of the content (buttons included) would land outside the visible
    area. Windows are also always resizable, so they can be enlarged by hand.
    """
    sw, sh = win.winfo_screenwidth(), win.winfo_screenheight()
    W = min(SC(w), max(320, sw - SC(40)))
    H = min(SC(h), max(240, sh - SC(80)))
    win.geometry(f"{W}x{H}")
    if minw is not None and minh is not None:
        win.minsize(min(SC(minw), W), min(SC(minh), H))
    try:
        win.resizable(True, True)
    except Exception:
        pass
    try:
        if parent is not None:
            parent.update_idletasks()
            px = parent.winfo_rootx() + parent.winfo_width() // 2 - W // 2
            py = parent.winfo_rooty() + parent.winfo_height() // 2 - H // 2
        else:
            px = (sw - W) // 2
            py = (sh - H) // 2
        px = max(0, min(px, sw - W))
        py = max(0, min(py, sh - H))
        win.geometry(f"+{px}+{py}")
    except Exception:
        pass
    return W, H

def _font(size, weight='normal'):
    """Returns a font tuple with a fallback"""
    return (FONT, size, weight) if weight != 'normal' else (FONT, size)

def _darken(hex_color, amount=0.30):
    """Darkens a hex color by the given amount - the counterpart of _lighten(),
    used for the shadow facets of the emblem."""
    hex_color = hex_color.lstrip('#')
    r, g, b = (int(hex_color[i:i+2], 16) for i in (0, 2, 4))
    f = max(0.0, 1.0 - amount)
    return f'#{int(r*f):02x}{int(g*f):02x}{int(b*f):02x}'


def _lighten(hex_color, amount=0.15):
    """Lightens a hex color by the given amount"""
    hex_color = hex_color.lstrip('#')
    r = int(hex_color[0:2], 16)
    g = int(hex_color[2:4], 16)
    b = int(hex_color[4:6], 16)
    r = min(255, int(r + (255 - r) * amount))
    g = min(255, int(g + (255 - g) * amount))
    b = min(255, int(b + (255 - b) * amount))
    return f'#{r:02x}{g:02x}{b:02x}'

def mk_btn(parent, text, cmd, bg=None, fg='#1a1c1f', **kw):
    """Brutalist button - sharp edges, monospace"""
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
    """Button with an outline instead of a fill"""
    b = tk.Button(parent, text=text, command=cmd, bg=C['bg2'], fg=color,
                  font=(FONT, fsz(10), 'bold'), padx=14, pady=7,
                  relief='flat', cursor='hand2', bd=0,
                  highlightthickness=2, highlightbackground=color,
                  highlightcolor=color,
                  activebackground=C['panel3'], activeforeground=color, **kw)
    return b


# ════════════════════════════════════════════════════════
#  WIDGET: Slider + numeric field (key panel element)
# ════════════════════════════════════════════════════════
class SliderField:
    """Slider plus a numeric field next to it. Type a value or drag the slider.
       on_change(value) is called on change (debounced)."""
    def __init__(self, parent, label, vmin, vmax, vinit, color,
                 unit='', decimals=1, on_change=None, width=170):
        self.vmin = vmin; self.vmax = vmax
        self.color = color; self.decimals = decimals
        self.on_change = on_change
        self._last_sent = None
        self._after_id = None

        # Container
        self.frame = tk.Frame(parent, bg=C['bg2'])
        self.frame.pack(fill='x', pady=(0, 14))

        # Label + unit
        top = tk.Frame(self.frame, bg=C['bg2'])
        top.pack(fill='x')
        tk.Label(top, text=label, bg=C['bg2'], fg=C['dim'],
                 font=(FONT, fsz(9)), anchor='w').pack(side='left')
        if unit:
            tk.Label(top, text=unit, bg=C['bg2'], fg=C['dim2'],
                     font=(FONT, fsz(8)), anchor='e').pack(side='right')

        # Row: slider + field
        row = tk.Frame(self.frame, bg=C['bg2'])
        row.pack(fill='x', pady=(4, 0))

        # Numeric field (Entry) - on the right
        self.entry = tk.Entry(row, width=7, bg=C['panel'], fg=color,
                              font=(FONT, fsz(12), 'bold'), justify='center',
                              relief='flat', bd=0,
                              highlightthickness=1.5, highlightbackground=color,
                              highlightcolor=_lighten(color, 0.2),
                              insertbackground=color)
        self.entry.pack(side='right', ipady=4, padx=(8, 0))
        self.entry.bind('<Return>', self._on_entry)
        self.entry.bind('<FocusOut>', self._on_entry)

        # Slider (Scale) - fills the rest
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
        """Send the change with a 150ms delay so as not to flood the serial link"""
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
        """Set the value. silent=True does not call on_change (sync from the device)."""
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
#  DECODING FIRMWARE ERROR CODES (ERR:code=N,...,active=0/1)
# ════════════════════════════════════════════════════════
ERR_CODES = {
    1: "Main thermocouple fault",
    2: "Thermocouple reading out of range / noise",
    3: "TEMP MAX - safe shutdown",
    4: "MAX31856 not responding (SPI/connection)",
}
# MAX31856 fault register bitmask (Adafruit_MAX31856.h)
TC_FAULT_BITS = [
    (0x80, "cold junction range (CJ range)"),
    (0x40, "thermocouple range (TC range)"),
    (0x20, "cold junction too high (CJ high)"),
    (0x10, "cold junction too low (CJ low)"),
    (0x08, "thermocouple too hot (TC high)"),
    (0x04, "thermocouple too cold (TC low)"),
    (0x02, "overvoltage/undervoltage (OV/UV)"),
    (0x01, "open circuit - wire broken/disconnected"),
]


# ── CSV COLUMN NAMES: ENGLISH NOW, POLISH STILL READABLE ────────────────
# The measurement CSV used to have Polish headers. From APP .15 new files are
# written with English ones, but every file already sitting in the data folder
# has the old names - and the ARCHIVE tab must keep opening them. So the
# writer emits CSV_COLS (English) and the reader passes every row through
# _csv_row(), which makes BOTH spellings resolve. Nothing downstream had to
# change, and no existing measurement becomes unreadable.
CSV_COLS = ['time_s', 'temperature_C', 'setpoint_active', 'setpoint_target',
            'PWM', 'PWM_%', 'Kp', 'Ki', 'Kd', 'state', 'temperature2_C',
            'ff', 'p_term', 'i_term', 'd_term', 'pid_raw', 'react_scale',
            'amb_est', 'pc_time']
# old (on disk) -> new (written from now on)
CSV_ALIAS = {
    'czas_s':            'time_s',
    'temperatura_C':     'temperature_C',
    'setpoint_aktywny':  'setpoint_active',
    'setpoint_cel':      'setpoint_target',
    'stan':              'state',
    'temperatura2_C':    'temperature2_C',
    'czas_pc':           'pc_time',
}
_CSV_ALIAS_REV = {v: k for k, v in CSV_ALIAS.items()}

def _csv_row(r):
    """One CSV row readable under BOTH the old Polish and the new English
    column names. Cheap (a handful of dict writes per row) and it keeps the
    whole archive - hundreds of files - working without a migration step."""
    for old, new in CSV_ALIAS.items():
        if old in r and new not in r:
            r[new] = r[old]
        elif new in r and old not in r:
            r[old] = r[new]
    return r

# ── EMBLEM ──────────────────────────────────────────────────────────────
# An ORIGINAL heraldic beast-head mark, defined once as polygons in a
# normalised -1..1 space and rendered by two tiny back-ends (Tk canvas for
# the title bar, matplotlib patches for the chart watermark). No image files
# ship with the app and nothing is traced from anyone else's artwork.
#
# ON THE OBVIOUS QUESTION: this is deliberately NOT a redrawn Witcher School
# of the Wolf medallion. Redrawing a protected logo produces a derivative
# work, which is still protected - and that particular head is a trademark on
# top of that. What is NOT protectable is the genre vocabulary: an angular
# animal head, faceted planes, gold on near-black. So the vocabulary is
# borrowed and the drawing is our own: a closed, calm, symmetrical head
# instead of a snarling open-jawed one, swept horns instead of a spiked
# halo, plain ring instead of a fanned crest, and no row of sign glyphs
# underneath - that row is the part that makes their composition theirs.
#
# y points UP in these coordinates; the Tk renderer flips it.
# ONE silhouette rather than a pile of overlapping pieces - the first two
# attempts stacked separate ears/brow/cheeks/muzzle polygons and the seams
# between them made the middle of the face mushy. Traced clockwise from the
# left ear tip: ears with a forehead dip between them, skull, two ranks of
# cheek ruff, and a blunt snout.
# Control points of the head, traced clockwise from the left ear tip, in a
# -1..1 space with y pointing UP. They are NOT drawn as a polygon: everything
# goes through _spline() first.
#
# WHY A SPLINE. Two earlier attempts drew straight-edged polygons. The first
# came out as a rat (needle snout, swept-back ears); the second, once the
# facets were tidied up, came out as a Transformers faceplate - because hard
# straight edges meeting at machined angles is exactly what a robot mask is.
# Fur and bone are curved, so the outline is now interpolated into a smooth
# closed curve and the internal "panel line" cuts are gone. The tufts are
# deliberately NOT matched in length pair-for-pair; slight irregularity is
# most of what separates fur from bodywork.
EMBLEM_SOLID = [[
    (-0.54, 1.10), (-0.34, 0.66), (-0.13, 0.70), (0.13, 0.70), (0.34, 0.66),
    ( 0.58, 1.12), ( 0.74, 0.54), ( 0.88, 0.18), (1.02, -0.12), (0.68, -0.22),
    ( 0.88, -0.54), ( 0.46, -0.44), (0.42, -0.66), (0.27, -0.56),
    ( 0.24, -0.84), ( 0.00, -0.96), (-0.24, -0.84), (-0.27, -0.56),
    (-0.40, -0.68), (-0.46, -0.44), (-0.84, -0.58), (-0.70, -0.22),
    (-1.02, -0.12), (-0.86, 0.18), (-0.72, 0.52),
]]
# ── FACETS: the chiselled-metal pass ────────────────────────────────────
# The smooth silhouette alone read flat. These are the planes the light falls
# on, laid over the base shape with a single convention: the light comes from
# the upper LEFT, so left-facing planes are lit and right-facing ones are in
# shadow, with a bright ridge running down the centre of the face.
# They are drawn WITHOUT spline smoothing on purpose - hard facet edges are
# what makes it look chiselled rather than moulded - and they are deliberately
# coarse: five planes per side, not twenty, because this also has to survive
# being 10 px wide in the title bar.
EMBLEM_LIT = [
    [(-0.54, 1.06), (-0.34, 0.66), (-0.15, 0.73), (-0.31, 0.92)],          # ear
    [(-0.34, 0.66), (-0.13, 0.70), (0.00, 0.22), (-0.22, -0.04),
     (-0.64, 0.14), (-0.72, 0.46)],                                        # skull
    [(-1.00, -0.12), (-0.66, -0.20), (-0.50, -0.42), (-0.84, -0.56)],      # ruff
    [(-0.22, -0.04), (0.00, 0.04), (0.00, -0.90), (-0.17, -0.78),
     (-0.27, -0.52)],                                                      # snout
]
EMBLEM_SHADE = [
    [( 0.58, 1.08), ( 0.34, 0.66), ( 0.15, 0.73), ( 0.33, 0.92)],
    [( 0.34, 0.66), ( 0.13, 0.70), (0.00, 0.22), (0.22, -0.04),
     ( 0.64, 0.14), ( 0.74, 0.46)],
    [( 1.00, -0.12), ( 0.66, -0.20), (0.50, -0.42), (0.86, -0.54)],
    [( 0.22, -0.04), ( 0.00, 0.04), (0.00, -0.90), (0.18, -0.78),
     ( 0.27, -0.52)],
]
# The catch-light: a narrow strip down the muzzle ridge, brightest of all.
EMBLEM_RIDGE = [
    [(-0.052, 0.52), (0.052, 0.52), (0.028, -0.50), (0.0, -0.60),
     (-0.028, -0.50)],
]

# Only the eyes are punched back out. Every extra cut is one more thing that
# reads as a seam on a mask, and at 10 px in the title bar it is also one more
# thing that turns to mud.
EMBLEM_VOID = [
    [(-0.60, 0.30), (-0.34, 0.16), (-0.20, 0.00), (-0.34, -0.04), (-0.56, 0.10)],
    [( 0.60, 0.30), ( 0.34, 0.16), ( 0.20, 0.00), ( 0.34, -0.04), ( 0.56, 0.10)],
]


def _spline(pts, steps=10):
    """Closed Catmull-Rom through pts - the curve passes through every control
    point, so the ear and tuft tips stay sharp while everything between them
    curves. Returns a dense point list both renderers can use."""
    n = len(pts)
    out = []
    for i in range(n):
        p0 = pts[(i - 1) % n]; p1 = pts[i]
        p2 = pts[(i + 1) % n]; p3 = pts[(i + 2) % n]
        for j in range(steps):
            t = j / steps
            t2 = t * t; t3 = t2 * t
            out.append((
                0.5 * ((2 * p1[0]) + (-p0[0] + p2[0]) * t +
                       (2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * t2 +
                       (-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * t3),
                0.5 * ((2 * p1[1]) + (-p0[1] + p2[1]) * t +
                       (2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * t2 +
                       (-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * t3)))
    return out


def draw_medallion(cv, cx, cy, r, color, bg):
    """Render the emblem onto a Tk canvas, centred at (cx, cy) with radius r.
    Purely decorative - nothing in the control path depends on it."""
    cv.create_oval(cx - r, cy - r, cx + r, cy + r,
                   outline=color, width=max(1, int(r * 0.12)))
    k = r * 0.62
    def pts(poly):
        out = []
        for x, y in poly:
            out += [cx + x * k, cy - y * k]       # minus: Tk y grows downward
        return out
    lit   = _lighten(color, 0.42)
    shade = _darken(color, 0.34)
    ridge = _lighten(color, 0.62)
    for poly in EMBLEM_SOLID:
        cv.create_polygon(*pts(_spline(poly)), fill=color, outline=color)
    for poly in EMBLEM_SHADE:
        cv.create_polygon(*pts(poly), fill=shade, outline=shade)
    for poly in EMBLEM_LIT:
        cv.create_polygon(*pts(poly), fill=lit, outline=lit)
    for poly in EMBLEM_RIDGE:
        cv.create_polygon(*pts(poly), fill=ridge, outline=ridge)
    for poly in EMBLEM_VOID:
        cv.create_polygon(*pts(_spline(poly, 6)), fill=bg, outline=bg)


def section(parent, title, color=None, bg=None, pady=(14, 6)):
    """One consistent section header for every tab: a hairline rule, a short
    all-caps title and a thin coloured tick. Before .15 each tab invented its
    own heading style (some bold labels, some coloured bars, some nothing at
    all), which is most of why the panels read as a wall of controls."""
    color = color or C['gold']
    bg = bg or C['bg2']
    head = tk.Frame(parent, bg=bg)
    head.pack(fill='x', pady=pady)
    tk.Frame(head, bg=color, width=SC(3), height=SC(11)).pack(side='left')
    tk.Label(head, text=title, bg=bg, fg=color,
             font=(FONT, fsz(8), 'bold')).pack(side='left', padx=(SC(6), SC(8)))
    tk.Frame(head, bg=C['border'], height=1).pack(side='left', fill='x',
                                                  expand=True, pady=(SC(5), 0))
    return head

def medallion_watermark(fig, color=None, alpha=0.075, size=0.34,
                        cx=0.5, cy=0.5):
    """The same emblem (EMBLEM_SOLID/EMBLEM_VOID) as a faint watermark behind
    the traces, so the mark in the title bar and the one on the chart are the
    same animal.

    Deliberately very low alpha: this is an instrument, and decoration that
    competes with the curves would be a bug, not a feature. It lives in figure
    coordinates below every axis (zorder 0), so it never moves when the data
    rescales and never intercepts a click. print_theme() hides it entirely for
    anything that leaves the app."""
    from matplotlib.patches import Ellipse, Polygon
    color = color or C['gold']
    r = size / 2.0
    # Figure coordinates run 0..1 on BOTH axes, so anything "round" there comes
    # out stretched by the figure aspect. Dividing every x by ar undoes that.
    ar = fig.get_figwidth() / max(fig.get_figheight(), 1e-6)
    def P(x, y):
        return (cx + x * r * 0.62 / ar, cy + y * r * 0.62)
    ring = Ellipse((cx, cy), 2 * r / ar, 2 * r, transform=fig.transFigure,
                   figure=fig, fill=False, ec=color, lw=r * 90, alpha=alpha,
                   zorder=0)
    ring.set_clip_on(False); ring.set_gid('wolf')
    fig.patches.append(ring)
    for poly, col, al in ([(_spline(p), color, alpha) for p in EMBLEM_SOLID] +
                          [(_spline(p, 6), fig.get_facecolor(), 1.0)
                           for p in EMBLEM_VOID]):
        pa = Polygon([P(x, y) for x, y in poly], closed=True,
                     transform=fig.transFigure, figure=fig,
                     fc=col, ec='none', alpha=al, zorder=0)
        pa.set_clip_on(False); pa.set_gid('wolf')
        fig.patches.append(pa)


# ── PRINT / EXPORT PALETTE ──────────────────────────────────────────────
# Anything that LEAVES the app - a saved PNG, an SVG, a PDF report, a chart
# pasted into a thesis or a mail - must be readable on white paper. The
# on-screen theme is deliberately dark, so exporting the screen colours gave
# a black rectangle with parchment-coloured text: fine on a monitor, useless
# printed, and a waste of toner.
# The keys are identical to C, so every drawing routine keeps working - only
# the values swap for the duration of the export (see print_theme()).
C_PRINT = {
    'bg':       '#ffffff',
    'bg2':      '#ffffff',
    'panel':    '#ffffff',
    'panel2':   '#ffffff',
    'panel3':   '#f0f0f0',
    'border':   '#9a9a9a',
    'border2':  '#c0c0c0',
    'text':     '#101010',
    'dim':      '#2a2a2a',   # axis labels/ticks - near-black, not grey
    'dim2':     '#666666',
    'blue':     '#1f6fb4',   # temperature
    'orange':   '#d2691e',   # setpoint
    'yellow':   '#b8860b',   # rate
    'green':    '#2e7d32',   # pwm
    'red':      '#b3261e',
    'cyan':     '#00796b',   # cooling / second trace
    'purple':   '#6a3fa0',
    'rec':      '#b3261e',
    'grid':     '#cccccc',
    'gold':     '#8a7326',
}


@contextlib.contextmanager
def print_theme(*figures):
    """Swap the whole app palette to the print one for the duration of a save.

    Every drawing routine reads C[...] at DRAW time, so mutating C in place
    and re-running the redraw is enough to restyle both charts completely -
    lines, ticks, labels, legend, grid and spines - without maintaining a
    second copy of the plotting code. The medallion watermark is hidden as
    well: it is chrome for the screen, and it has no business on a figure
    that ends up in a report."""
    saved = dict(C)
    saved_faces = [(f, f.get_facecolor()) for f in figures]
    hidden = []
    C.update(C_PRINT)
    for f in figures:
        f.set_facecolor('white')
        for pt in f.patches:
            if pt.get_gid() == 'wolf' and pt.get_visible():
                pt.set_visible(False); hidden.append(pt)
    try:
        yield
    finally:
        C.clear(); C.update(saved)
        for f, fc in saved_faces:
            f.set_facecolor(fc)
        for pt in hidden:
            pt.set_visible(True)

def decode_tc_fault(bits):
    names = [name for mask, name in TC_FAULT_BITS if bits & mask]
    return ", ".join(names) if names else f"bitmask 0x{bits:02X}"


# ════════════════════════════════════════════════════════
#  MAIN APPLICATION
# ════════════════════════════════════════════════════════
class PeltierControl:
    def __init__(self, root):
        self.root = root
        self.root.title(f"IGNI - photocurrent & pyrocurrent bench  [APP {APP_BUILD}]")
        self.root.configure(bg=C['bg'])
        # The main window size is ALSO scaled by DPI and clipped to the screen -
        # see the comment at SC()/size_win().
        size_win(self.root, 1280, 800, 1100, 720)

        # Serial
        self.ser = None
        self.port_name = None
        self.baud = 115200
        self.running = False
        self.connected = False

        # Measurement data (buffers)
        self.maxlen = 3000
        self.t = []; self.temp = []; self.spt = []; self.spa = []
        self.pwm = []; self.kp = []; self.ki = []; self.kd = []; self.states = []
        self.t0 = None
        self.data_queue = queue.Queue()
        self.last_state = 'MAN'
        self.cur_state = 'MAN'

        # Tracking the approach to the setpoint (statistics)
        self.reach_start_t = None    # approach start time (s)
        self.reach_start_temp = None # temp at the start
        self.reach_target = None     # target temp
        self.reach_done = False      # whether it was reached
        self.reach_in_tol_t = None   # since when we have been within tolerance
                                     # (see REACH_TOL_C/REACH_STABLE_S)
        self.reach_time = None       # how long the approach took [s]
        self.reach_avg_rate = None   # average ramp [C/min]
        self.last_setpoint_target = None

        # ── SEPARATE tracking of the RAMP PHASE (not reach_* above) ──────
        # THE PROBLEM THIS FIXES: "AVG RATE" and "avg ...C/min" in the bar
        # were counted from the start UP TO entering +/-0.5C of the target -
        # that is, TOGETHER with the approach tail, which can last longer
        # than the ramp itself. With 30 C/min commanded it showed "avg
        # 12.16 C/min", which looks as if the ramp ran 2.5x too slow, while
        # in reality the ramp ran at ~26 C/min and only the APPROACH over
        # the last 0.5C took the rest. These two things must be measured
        # SEPARATELY, because they are fixed by completely different changes
        # (ramp rate = FF, approach tail = loss compensation + integrator).
        # Ramp phase = as long as the ramp GENERATOR (active setpoint spA) is
        # still travelling to the target. Once spA arrives, the ramp is over -
        # regardless of where the real temperature is.
        self.ramp_t0 = None          # ramp start time
        self.ramp_temp0 = None       # temp at the start of the ramp
        self.ramp_done = False       # whether the ramp generator arrived
        self.ramp_secs = None        # how long the ramp itself took [s]
        self.ramp_rate = None        # REAL rate achieved during the ramp [C/min]
        self.ramp_cmd_rate = None    # COMMANDED rate (from the panel) [C/min]
        self.ramp_lag = None         # how far from the target when the ramp ended [C]

        # Polarity and calibration range (from the device)
        self.dev_pol_swapped = False
        self.dev_pol_set = False
        self.dev_cal_min = 50.0
        self.dev_cal_max = 100.0

        # Firmware version number read from the board (VER command) - to
        # verify that the board really does have the new software
        self.dev_fw_build = None

        # Live chart control
        self.chart_paused = False      # scrolling paused (for zooming)
        self.chart_window = 0          # 0 = whole run, >0 = last N seconds

        # ── WHERE THE DATA ENDS UP ───────────────────────────────────────
        # cfg_dir  - PERMANENT app folder (calibration, presets, settings).
        #            It does not travel with the data, so changing where
        #            measurements are saved never "loses" the calibration.
        # log_dir  - folder FOR MEASUREMENT DATA, chosen by the user (ARCHIVE
        #            tab -> CHANGE / NEW). Remembered between runs in
        #            ustawienia.json.
        self.cfg_dir = Path.home() / "PeltierLogi"
        self.cfg_dir.mkdir(exist_ok=True)
        self.settings_file = self.cfg_dir / "ustawienia.json"
        self.log_dir = self._load_data_dir()
        self.cyc_on = False; self.cyc_file = None; self.cyc_wr = None
        # Name hint for the NEXT archive save (see cyc_stop) - when set, it
        # skips the interactive "SAVE CYCLE TO ARCHIVE" dialog (which is
        # modal - it would block the automatic measurement SERIES).
        self.series_name_hint = None

        # ── MEASUREMENT SERIES (automatic chain of SP/RATE tests) ────────
        # Goal: instead of running the tests one after another by hand and
        # pasting me screenshots, the app walks the list itself (SP, RATE,
        # hold time), archives each test under a readable name (without
        # asking for a name), and I read the resulting files from the
        # PeltierLogi folder (I have access to it) and prepare fixes at once.
        self.series_steps = []       # list of dict(sp=, rate=, hold_s=)
        self.series_idx = 0
        self.series_running = False
        self.series_leg = None       # 'heat' | 'cool' | None
        self.series_phase = None     # 'ramping' | 'holding' | None
        self.series_phase_t0 = None
        self.series_base_sp = 25.0   # which temp to return to between tests
        self.series_skip_archive = False  # True during the return leg - see cyc_stop
        self._series_saved_rd = None      # COOL RATE from CONTROL, restored after the series
        self.cyc_t0 = None; self.cyc_fn = None

        # Profiles (list of stages: dict temp/ramp/time)
        self.profile_steps = []

        # Device synchronization status
        self.dev_cal = False       # whether the device has a calibration
        self.last_cfg_time = 0

        # Calibration state
        self.cal_plan = []         # list of (temp, ramp) for all steps
        self.cal_total = 0         # number of steps
        self.cal_current = 0       # current step (1-based)
        self.cal_cur_temp = None
        self.cal_cur_ramp = None
        self.cal_phase = None      # phase of the current step: 'heating'/'stabil'/'relay'
        self.cal_running = False
        self.cal_t0 = None         # calibration start time
        self.cal_step_times = []   # start times of consecutive steps (for the ETA)
        self.cal_win = None        # calibration progress window
        self.cal_warnings = []     # list of (temp, cycles, amp) for points with relay_fail in this session
        self.cal_ramp_warnings = []  # list of (temp, ramp, err) for ramp_track_fail in this session

        # Diagnostics / error log - every Serial line that is neither a known
        # protocol message nor CSV telemetry ends up here (instead of being
        # silently discarded), and formal ERR: lines are decoded into text.
        self.diag_log = []        # list of (ts, level, text); level: ERR/WARN/INFO
        self.err_active = {}      # code -> description, active (uncleared) hardware errors
        self.diag_unseen = 0      # counter of new ERR/WARN since the window was last opened
        self.diag_win = None      # reference to the open diagnostics window (or None)

        # Calibration saved on the PC disk - in the PERMANENT cfg_dir, not the
        # data folder (see the comment at self.cfg_dir): changing where the
        # measurements go must not cut the app off from the device calibration.
        self.cal_file = self.cfg_dir / "kalibracja.json"
        self.presets_file = self.cfg_dir / "presety.json"
        self._caldump_buf = []     # buffer of received profiles
        self._caldump_active = False
        self._caldump_purpose = None  # 'save' or None
        self._pending_offset = None   # offset to be saved with the dump

        # Status pulsing
        self._pulse_state = 0

        self._build_styles()
        self._build_ui()
        self._pulse()
        self.tick()
        # Auto-connect: try to connect to the device after startup
        self.root.after(800, self._auto_connect)
    def _auto_connect(self):
        """Automatic connection - detect and connect to the ItsyBitsy"""
        if self.connected:
            return
        try:
            ports = list(serial.tools.list_ports.comports())
        except Exception:
            return
        if not ports:
            return
        # Priority: ports whose description matches ItsyBitsy/Adafruit/USB
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
        # Connect only if something sensible (any port if there is only one)
        if score(best) > 0 or len(ports) == 1:
            self.connect(best.device)

    def _build_styles(self):
        st = ttk.Style()
        try: st.theme_use('clam')
        except: pass
        st.configure('TNotebook', background=C['bg2'], borderwidth=0, tabmargins=[0,0,0,0])
        st.configure('TNotebook.Tab', background=C['bg2'], foreground=C['dim2'],
                     padding=[SC(18), SC(10)], font=(FONT, fsz(10), 'bold'),
                     borderwidth=0)
        # The selected tab is marked by BOTH a lighter ground and gold text -
        # on the darker .15 background a background change alone was too
        # subtle to spot at a glance.
        st.map('TNotebook.Tab',
               background=[('selected', C['bg']), ('active', C['panel3'])],
               foreground=[('selected', C['gold']), ('active', C['text'])])
        st.configure('Vertical.TScrollbar', background=C['panel3'],
                     troughcolor=C['bg2'], bordercolor=C['bg2'],
                     arrowcolor=C['dim2'], borderwidth=0)

    # ────────────────────────────────────────────────────
    #  SERIAL COMMUNICATION
    # ────────────────────────────────────────────────────
    def send(self, cmd):
        """Send a command to the device"""
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
            self._cfg_synced = False  # allow a one-time slider synchronisation
            self.set_status(True, f"{port} - 115200")
            self.running = True
            threading.Thread(target=self.reader, daemon=True).start()
            # Fetch the startup configuration + firmware version number (VER responds
            # to the command immediately, so it also works when the board has been
            # powered on for a long time - unlike BUILD:, which is sent only once in
            # setup() and which the app could miss if it connected AFTER startup).
            self.root.after(1500, lambda: self.send("GET"))
            self.root.after(1600, lambda: self.send("VER"))
            # Auto-load the calibration saved on the PC (if one exists)
            self.root.after(2200, self._auto_load_calibration)
        except Exception as e:
            messagebox.showerror("Error", f"{port}:\n{e}")
            self.set_status(False, "")

    def _auto_load_calibration(self):
        """On connection - automatically upload the saved calibration"""
        if not self.connected:
            return
        if self.cal_file.exists():
            ok = self.load_calibration_from_pc()
            if ok:
                print("Auto-loaded calibration from PC on connection")

    def disconnect(self):
        self.running = False
        if self.cyc_on: self.cyc_stop("Disconnected")
        if self.ser:
            try: self.ser.close()
            except: pass
            self.ser = None
        self.set_status(False, "")
        self.dev_fw_build = None
        if hasattr(self, 'fw_build_lbl'):
            self.fw_build_lbl.config(text="FW: —", fg=C['dim2'])

    def clear_buf(self):
        for a in [self.t, self.temp, self.spt, self.spa,
                  self.pwm, self.kp, self.ki, self.kd, self.states]:
            a.clear()
        self.t0 = None

    def reader(self):
        """Serial reader thread - parses CSV and CFG"""
        if self.ser and self.ser.is_open:
            self.ser.reset_input_buffer()
        while self.running:
            try:
                if not self.ser or not self.ser.is_open: break
                raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not raw: continue

                # Configuration line CFG:SP=...,RU=...
                if raw.startswith("CFG:"):
                    self._parse_cfg(raw[4:])
                    continue

                # Calibration plan CALPLAN:24,temps=50/60/70,ramps=2/5/10/20
                if raw.startswith("CALPLAN:"):
                    self._parse_calplan(raw[8:])
                    continue

                # Calibration dump - start
                if raw.startswith("CALDUMP:"):
                    self._caldump_buf = []
                    self._caldump_active = True
                    continue
                # A single profile PROF:idx,KpH,...
                if raw.startswith("PROF:") and self._caldump_active:
                    self._caldump_buf.append(raw[5:])
                    continue
                # End of the dump
                if raw == "CALDUMPEND":
                    self._caldump_active = False
                    self.root.after(0, self._finish_caldump_save)
                    continue

                # Calibration status CALSTAT:5/24,T=40,R=2
                if raw.startswith("CALSTAT:"):
                    self._parse_calstat(raw[8:])
                    continue

                # Warning: the relay test did not catch oscillation, base values
                # were used CALWARN:T=90,cycles=1,relay_fail
                if raw.startswith("CALWARN:"):
                    self._parse_calwarn(raw[8:])
                    continue

                # Hardware/safety error code ERR:code=1,bits=0x01,active=1
                if raw.startswith("ERR:"):
                    self._parse_err(raw[4:])
                    continue

                # Firmware version number - sent once in setup() AND on every
                # "VER" request (see connect()). Shown in the title bar
                # (FW: ...) so it is immediately visible whether the board really
                # has the new software flashed, not just whether the app is new.
                if raw.startswith("BUILD:"):
                    self.root.after(0, lambda b=raw[6:].strip(): self._set_fw_build(b))
                    continue

                # CSV data line (9 fields + optional temp2 as the 10th)
                p = raw.split(',')
                is_csv = len(p) >= 9
                if is_csv:
                    try: float(p[0])
                    except ValueError: is_csv = False
                if not is_csv:
                    # Not CSV and not any of the known prefixes above - instead
                    # of silently discarding it (as before), show it in the
                    # diagnostics panel. That way the app shows EVERYTHING the
                    # firmware sends over Serial (e.g. "Flash: zapisano.",
                    # "AUTOCAL START", "RELAY FAIL - bazowe"), not only in
                    # Arduino's own Serial Monitor (which cannot run in
                    # parallel with the app on the same port anyway).
                    if raw:
                        low = raw.upper()
                        lvl = 'WARN' if any(k in low for k in
                              ('FAIL', 'ERROR', '!!!', 'BLAD', 'BŁĄD')) else 'INFO'
                        self._log_diag(lvl, raw)
                    continue
                try:
                    d = dict(temp=float(p[1]), sa=float(p[2]), st=float(p[3]),
                             pwm=int(p[4]), kp=float(p[5]), ki=float(p[6]),
                             kd=float(p[7]), state=p[8].strip())
                except: continue
                # temp2 - the second thermocouple (10th field, if present)
                d['temp2'] = None
                if len(p) >= 10:
                    try:
                        v2 = float(p[9])
                        d['temp2'] = v2 if v2 != 0 else None  # 0 = none/error
                    except: pass
                self._latest_temp2 = d['temp2']  # for display on the card

                # Breakdown of the PID components (10th-15th extra fields, only in
                # AUTO - see the comment next to "dbgFF" in the firmware) - FF, P, I,
                # D, the raw PID result before clamping/slew, and the applied
                # reactScale. It goes into the run archive (cyc_log) so that further
                # diagnosis of oscillation/lag can rely on numbers from the file
                # instead of guessing from the temp/PWM chart alone.
                d['dbg'] = None
                if len(p) >= 16:
                    try:
                        d['dbg'] = dict(ff=float(p[10]), p=float(p[11]),
                                         i=float(p[12]), dd=float(p[13]),
                                         raw=float(p[14]), react=float(p[15]),
                                         # 17th column (since FW .29): estimated
                                         # temperature of the other side of the Peltier.
                                         # Older firmware does not send it -
                                         # then it stays None and the CSV has a blank.
                                         amb=(float(p[16]) if len(p) >= 17 else None))
                    except: pass

                # Time from the FIRMWARE (p[0] = czas_s) - accurate, independent of
                # application delays/queue buffering. The computer clock (time.time)
                # drifted during buffering and understated AVG RATE.
                try:
                    fw_time = float(p[0])
                except:
                    fw_time = 0
                if self.t0 is None:
                    self.t0 = fw_time  # first firmware timestamp = zero point
                now = fw_time - self.t0
                state = d['state']

                if self.cyc_on and state in ('AUTO', 'COOLDOWN', 'FREEZE', 'FREEZE_READY'):
                    self.cyc_log(time.time() - self.cyc_t0 if self.cyc_t0 else 0,
                                d['temp'], d['sa'], d['st'],
                                d['pwm'], d['kp'], d['ki'], d['kd'], state,
                                d.get('temp2'), d.get('dbg'))

                prev = self.last_state
                self.last_state = state
                self.cur_state = state
                # SELF-TUNE: when the state is ST-..., self-tune changes the PID live.
                # Copy the new Kp/Ki/Kd onto the sliders so the table stays updated.
                if state.startswith('ST') or state.startswith('CAL'):
                    self._st_pid_update = (d['kp'], d['ki'], d['kd'])
                # Detect the end of calibration (CAL/CAL-N -> MAN)
                if self.cal_running and 'CAL' in prev and state == 'MAN':
                    self.cal_running = False
                    self.cal_current = self.cal_total  # complete the bar
                    self.root.after(0, self._cal_finished)
                self.data_queue.put((now, d['temp'], d['st'], d['sa'],
                                    d['pwm']*100/255, d['kp'], d['ki'],
                                    d['kd'], state, prev))

            except serial.SerialException:
                self.running = False
                self.root.after(0, lambda: self.set_status(False, "Connection lost"))
                break
            except Exception as e:
                if self.running: print(f"reader err: {e}")
                time.sleep(0.3)

    def _parse_cfg(self, cfg):
        """Parses CFG:SP=25.5,RU=2.0,... and synchronises the sliders"""
        d = {}
        for part in cfg.split(','):
            if '=' in part:
                k, v = part.split('=', 1)
                d[k.strip()] = v.strip()
        # Synchronise the sliders (silent - without sending back)
        self.root.after(0, lambda: self._apply_cfg(d))

    def _apply_cfg(self, d):
        try:
            # Synchronise the sliders ONLY on the first CFG after connecting.
            # Afterwards the user's settings must stay (do not overwrite after STOP etc.)
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
            # Polarity
            if 'POL' in d:
                self.dev_pol_swapped = (d['POL'] == '1')
            if 'POLSET' in d:
                self.dev_pol_set = (d['POLSET'] == '1')
            # Calibration range
            if 'CALMIN' in d:
                self.dev_cal_min = float(d['CALMIN'])
            if 'CALMAX' in d:
                self.dev_cal_max = float(d['CALMAX'])
            # Fan state
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
            # Update the polarity indicator in the UI if it exists
            if hasattr(self, '_update_pol_indicator'):
                self._update_pol_indicator()
        except Exception as e:
            print(f"apply_cfg err: {e}")

    def _parse_calplan(self, txt):
        """CALPLAN:9,temps=20/30/.../90,ramps=relay - build the step list.
        Relay: one test per temperature (ramps=relay), not a temp x ramp grid."""
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
            # Build the plan
            plan = []
            if relay_mode:
                # Relay: one step per temperature
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
            self.cal_warnings = []
            self.cal_ramp_warnings = []
            self.root.after(0, self._refresh_cal_view)
        except Exception as e:
            print(f"calplan err: {e}")

    def _parse_calstat(self, txt):
        """CALSTAT:5/24,T=40,R=2 - update the progress"""
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
                    # Relay: R= is the step PHASE (heating/stabil/relay), not the ramp.
                    if rv in ('heating', 'stabil', 'relay'):
                        self.cal_phase = rv
                        self.cal_cur_ramp = 'relay'
                    elif rv.startswith('rampprep:') or rv.startswith('ramptest:'):
                        # Per-ramp ramping test AFTER relay (tunes the heating
                        # Kp/Ki/Kd separately for every ramp from calRamps) -
                        # R=rampprep:20 (backing off) / R=ramptest:20 (running at
                        # that ramp, tracking ASP).
                        key, _, rate = rv.partition(':')
                        self.cal_phase = key
                        try: self.cal_cur_ramp = float(rate)
                        except Exception: self.cal_cur_ramp = rate
                    else:
                        self.cal_phase = None
                        try: self.cal_cur_ramp = float(rv)
                        except: self.cal_cur_ramp = rv
            # If the step has changed - record the time (for the ETA)
            if new_current != self.cal_current:
                if self.cal_t0:
                    self.cal_step_times.append(time.time())
                self.cal_current = new_current
            self.cal_running = True
            self.root.after(0, self._refresh_cal_view)
        except Exception as e:
            print(f"calstat err: {e}")

    def _parse_calwarn(self, txt):
        """Two different warnings share the same CALWARN message:

        1) CALWARN:T=90,cycles=1,amp=140,relay_fail - the relay test for this
        temperature did not catch oscillation (too few/too fast crossings of the
        setpoint) and the firmware wrote base values instead of really
        measured ones. 'amp' is the PWM amplitude at which the test gave up -
        if that is already the max (140), even the strongest gentle excitation did
        not push the system across the setpoint in both directions (a physical
        limit of the range, not just a matter of time/noise).

        2) CALWARN:T=50,R=20,err=2.34,ramp_track_fail - the RAMPING test for a
        specific ramp (AFTER a successful relay) did not get below the ASP
        tracking error threshold during the test - this ONE cell (temp,ramp) keeps
        the base profile from relay (which is still a real measurement, NOT the
        values 10.0/0.30/0.80 - unlike (1)!). This is NOT the
        same as relay_fail and should not mark the whole temperature as
        "base/fail" in the table - hence a separate list (cal_ramp_warnings)."""
        try:
            d = {}
            for part in txt.split(','):
                if '=' in part:
                    k, v = part.split('=', 1)
                    d[k.strip()] = v.strip()
            temp = float(d.get('T', 'nan'))
            if temp != temp:  # reject NaN
                return
            if 'R' in d and 'err' in d:
                # (2) ramp_track_fail
                try: ramp = float(d['R'])
                except Exception: ramp = None
                try: err = float(d['err'])
                except Exception: err = None
                self.cal_ramp_warnings.append((temp, ramp, err))
                self._log_diag('WARN', f"Calibration: ramp test {ramp}°C/min "
                               f"@ {temp}°C did not keep up with ASP (err={err}°C)")
            else:
                # (1) relay_fail
                cycles = int(d.get('cycles', '0'))
                amp = int(d['amp']) if 'amp' in d else None
                self.cal_warnings.append((temp, cycles, amp))
                self._log_diag('WARN', f"Calibration: relay test @ {temp}°C did not catch "
                               f"oscillation (cycles={cycles}, amp={amp}) - base values used")
            self.root.after(0, self._refresh_cal_view)
        except Exception as e:
            print(f"calwarn err: {e}")

    def _parse_err(self, txt):
        """ERR:code=N,...,active=0/1 - hardware/safety error code from the
        firmware. The firmware sends this ONLY on an edge (once when it appears, once
        when it clears), so here we only decode it and write it to the log - zero
        risk of flooding Serial. code=1/2 update err_active (shown
        as an active alarm until active=0 arrives), code=3/4 are
        one-off events but are also kept in err_active for later reference."""
        try:
            d = {}
            for part in txt.split(','):
                if '=' in part:
                    k, v = part.split('=', 1)
                    d[k.strip()] = v.strip()
            code = int(d.get('code', '-1'))
            active = d.get('active', '1') == '1'
            base = ERR_CODES.get(code, f"Unknown error code ({code})")
            detail = ""
            if code == 1 and 'bits' in d:
                try: detail = " - " + decode_tc_fault(int(d['bits'], 16))
                except Exception: pass
            elif code == 2 and 'val' in d:
                detail = f" - reading={d['val']}°C"
            elif code == 3:
                detail = f" - temp={d.get('temp', '?')}°C, limit={d.get('limit', '?')}°C"
            text = base + detail
            if active:
                self.err_active[code] = text
                self._log_diag('ERR', text)
            else:
                self.err_active.pop(code, None)
                self._log_diag('INFO', f"CLEARED: {base}")
        except Exception as e:
            print(f"err parse err: {e}")

    def _log_diag(self, level, text):
        """Append an entry to the diagnostics panel (level: ERR/WARN/INFO) and
        refresh the indicator in the title bar. Called both from the Serial thread
        (directly) and from the GUI - list.append() is safe in
        CPython (GIL), and the UI refresh always goes through root.after."""
        entry = (time.time(), level, text)
        self.diag_log.append(entry)
        if len(self.diag_log) > 500:
            del self.diag_log[:-500]
        if level in ('ERR', 'WARN'):
            self.diag_unseen += 1
        self.root.after(0, self._refresh_diag_indicator)
        if self.diag_win is not None:
            self.root.after(0, lambda e=entry: self.diag_win.append_entry(e))

    def _refresh_diag_indicator(self):
        """Updates the DIAG button in the title bar: colour/text according to
        whether there are active hardware alarms (err_active) and the count of unread
        entries (diag_unseen)."""
        if not hasattr(self, 'btn_diag'):
            return
        if self.err_active:
            n = len(self.err_active)
            self.btn_diag.config(text=f"⚠ ERROR x{n}", bg=C['red'], fg='#ffffff')
        elif self.diag_unseen > 0:
            self.btn_diag.config(text=f"DIAG ({self.diag_unseen})", bg=C['orange'], fg='#1a1c1f')
        else:
            self.btn_diag.config(text="DIAG", bg=C['bg2'], fg=C['dim'])

    def _set_fw_build(self, build):
        """Called after receiving BUILD:<id> from the board (at firmware startup
        OR in response to the VER command sent right after connecting).
        Shows the number in the title bar and logs it to diagnostics, so it is
        plain to see which firmware version is actually flashed."""
        self.dev_fw_build = build
        if hasattr(self, 'fw_build_lbl'):
            self.fw_build_lbl.config(text=f"FW: {build}", fg=C['green'])
        self._log_diag('INFO', f"Connected - firmware build {build}")

    def open_diag_window(self):
        self.diag_unseen = 0
        self._refresh_diag_indicator()
        if self.diag_win is not None:
            try:
                self.diag_win.win.lift()
                return
            except Exception:
                self.diag_win = None
        self.diag_win = DiagnosticsWindow(self.root, self)

    def _cal_step_stats(self):
        """(avg_step_s, elapsed_in_current_step_s) based on the timestamps
        of the starts of successive steps. avg_step=None when there is not yet
        any completed step to average."""
        times = self.cal_step_times
        now = time.time()
        if len(times) >= 2:
            durations = [times[i] - times[i - 1] for i in range(1, len(times))]
            avg_step = sum(durations) / len(durations)
        else:
            avg_step = None
        if times:
            elapsed_in_step = now - times[-1]
        elif self.cal_t0:
            elapsed_in_step = now - self.cal_t0
        else:
            elapsed_in_step = 0
        return avg_step, elapsed_in_step

    def _cal_eta(self):
        """Estimated remaining calibration time [s].
        None = not enough data yet to make an estimate.
        0 only when the calibration has REALLY finished (cal_running=False) -
        previously cur==total (the last point had only JUST STARTED) also gave 0,
        which looked like "done" even though the relay test could still be running
        for another several or a dozen-odd minutes."""
        if not self.cal_t0 or self.cal_total < 1:
            return None
        if not self.cal_running:
            return 0
        avg_step, elapsed_in_step = self._cal_step_stats()
        if avg_step is None:
            if self.cal_current < 1:
                return None
            avg_step = (time.time() - self.cal_t0) / self.cal_current
        remaining_full_steps = max(0, self.cal_total - self.cal_current)
        remaining = remaining_full_steps * avg_step + max(0, avg_step - elapsed_in_step)
        return max(0, remaining)

    def _cal_progress_fraction(self):
        """Fill fraction of the progress bar (0..1). It grows smoothly during the
        current step instead of jumping to 100% at the moment the LAST point
        has only just started (cal_current already equals cal_total, but in reality
        it may have only just begun)."""
        if not self.cal_total:
            return 0.0
        if not self.cal_running and self.cal_current >= self.cal_total:
            return 1.0
        avg_step, elapsed_in_step = self._cal_step_stats()
        completed = max(0, self.cal_current - 1)
        step_frac = 0.0
        if avg_step and avg_step > 0:
            step_frac = min(0.95, elapsed_in_step / avg_step)
        return min(1.0, (completed + step_frac) / self.cal_total)

    def _cal_finished(self):
        """Calibration finished"""
        self._refresh_cal_view()
        if hasattr(self, 'cal_status'):
            self.cal_status.config(text="✓ Calibration done - saving to PC...")
        self.dev_cal = True
        # Fetch the updated settings
        self.send("GET")
        # Automatically fetch the profiles and save them to the PC disk
        self.root.after(800, lambda: self.dump_calibration_to_pc(silent=False))

    # ────────────────────────────────────────────────────
    #  CALIBRATION - SAVE/LOAD ON THE PC DISK
    # ────────────────────────────────────────────────────
    def _manual_load_cal(self):
        """Manual upload of the calibration from the PC (with confirmation)"""
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        if not self.cal_file.exists():
            messagebox.showinfo("No calibration",
                "No saved calibration found on PC.\n"
                "Run calibration first, or save it with\n"
                "the 'SAVE CAL TO PC' button.")
            return
        # Show the save date
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
        """Fetch the profiles from the device and show the Kp/Ki/Kd table"""
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        self._caldump_buf = []
        self._caldump_active = False
        self._caldump_purpose = 'view'
        self.send("DUMPCAL")
        print("Fetching the calibration table...")

    # Base values from the firmware (KP_BASE/KI_BASE/KD_BASE_H) - this is what
    # the firmware writes as the profile when the relay test did NOT catch oscillation
    # ("RELAY FAIL - bazowe"). Every calibrated cell that lands
    # exactly on these numbers is almost certainly not a real measurement.
    _CAL_BASE_KP, _CAL_BASE_KI, _CAL_BASE_KD = 10.0, 0.3, 0.8

    def _show_cal_table_window(self, profiles):
        """Window with the table of calibrated PID values (temp x ramp)"""
        # Grid as in the firmware (PR_N=9 - must be IDENTICAL to PT[]/PR[] in the .ino,
        # otherwise idx=ri*len(PR)+ci points at the wrong cells)
        PT = [20, 30, 40, 50, 60, 70, 80, 90, 100]
        PR = [5, 10, 20, 30, 40, 50, 60, 70, 80]
        win = tk.Toplevel(self.root)
        win.title("Calibration Table")
        win.configure(bg=C['bg'])
        size_win(win, 720, 520, 560, 400, parent=self.root)
        tk.Label(win, text="CALIBRATION TABLE — heating PID (Kp / Ki / Kd)",
                 bg=C['bg'], fg=C['purple'], font=(FONT, fsz(12), 'bold')).pack(
                 anchor='w', padx=16, pady=(14, 4))
        n_valid = sum(1 for p in profiles if p['valid'])
        n_fallback = sum(1 for p in profiles if p['valid'] and self._is_base_profile(p))
        info_txt = (f"{n_valid} of {len(profiles)} grid points calibrated.  "
                    "Empty = not calibrated (uses defaults 10/0.3/0.8).")
        if n_fallback:
            info_txt += (f"\n⚠ {n_fallback} of those are exactly the base defaults — "
                         "almost certainly a failed relay test that fell back, not a real measurement "
                         "(shown in red below).")
        tk.Label(win, text=info_txt,
                 bg=C['bg'], fg=(C['red'] if n_fallback else C['dim']),
                 font=(FONT, fsz(9)), justify='left').pack(anchor='w', padx=16)
        # Map idx -> profile
        pmap = {p['idx']: p for p in profiles}
        # Scrollable table
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
        # Header: ramps
        tk.Label(inner, text="Temp\\Ramp", bg=C['panel'], fg=C['cyan'],
                 font=(FONT, fsz(9), 'bold'), width=10, anchor='w').grid(
                 row=0, column=0, sticky='nsew', padx=1, pady=1)
        for ci, r in enumerate(PR):
            tk.Label(inner, text=f"{r}°C/min", bg=C['panel'], fg=C['cyan'],
                     font=(FONT, fsz(9), 'bold'), width=16).grid(
                     row=0, column=ci+1, sticky='nsew', padx=1, pady=1)
        # Rows: temperatures
        for ri, t in enumerate(PT):
            tk.Label(inner, text=f"{t}°C", bg=C['panel'], fg=C['orange'],
                     font=(FONT, fsz(9), 'bold'), width=10, anchor='w').grid(
                     row=ri+1, column=0, sticky='nsew', padx=1, pady=1)
            for ci, r in enumerate(PR):
                idx = ri * len(PR) + ci  # pi_(ti,ri) = ti*PR_N+ri
                p = pmap.get(idx)
                if p and p['valid']:
                    txt = f"{p['KpH']:.1f} / {p['KiH']:.2f} / {p['KdH']:.2f}"
                    if self._is_base_profile(p):
                        txt += "  ⚠"
                        fg = C['red']; bg = C['bg2']
                    else:
                        fg = C['text']; bg = C['bg2']
                else:
                    txt = "—"
                    fg = C['dim2']; bg = C['panel2']
                tk.Label(inner, text=txt, bg=bg, fg=fg,
                         font=(FONT, fsz(8)), width=16).grid(
                         row=ri+1, column=ci+1, sticky='nsew', padx=1, pady=1)
        # Footer
        tk.Label(win, text="Each cell: Kp / Ki / Kd for that temperature and ramp rate.\n"
                 "On START, the app interpolates between the 4 nearest points automatically.\n"
                 "⚠ = identical to the base defaults - almost certainly a failed relay test "
                 "(no real oscillation measured), not a genuine calibration.",
                 bg=C['bg'], fg=C['dim'], font=(FONT, fsz(8)), justify='left').pack(
                 anchor='w', padx=16, pady=(0, 12))

    def _is_base_profile(self, p):
        """True if this profile's Kp/Ki/Kd are (within rounding error)
        exactly the base values from the firmware - that is, almost certainly a
        fallback after a failed relay test rather than a real measurement."""
        try:
            return (abs(p['KpH'] - self._CAL_BASE_KP) < 0.05 and
                    abs(p['KiH'] - self._CAL_BASE_KI) < 0.01 and
                    abs(p['KdH'] - self._CAL_BASE_KD) < 0.01)
        except Exception:
            return False

    def dump_calibration_to_pc(self, silent=True):
        """Asks the device for the profiles and offset, saves them to JSON"""
        if not self.connected:
            return
        self._caldump_purpose = 'save'
        # Remember the offset from the current slider
        try:
            self._pending_offset = self.sl_off.get()
        except:
            self._pending_offset = 0.0
        self.send("DUMPCAL")
        if not silent:
            print("Fetching the profiles from the device...")

    def _finish_caldump_save(self):
        """After receiving all the profiles - save them to a JSON file"""
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
            print(f"Calibration saved: {self.cal_file.name} ({n_valid}/{len(profiles)} profiles)")
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
                # Show the table in a window
                self._show_cal_table_window(profiles)
        except Exception as e:
            print(f"Calibration save error: {e}")
        self._caldump_purpose = None

    def load_calibration_from_pc(self):
        """Load the calibration from the JSON file and send it to the device"""
        if not self.cal_file.exists():
            return False
        try:
            with open(self.cal_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            profiles = data.get('profiles', [])
            offset = data.get('offset', 0.0)
            if not profiles:
                return False
            # Send the offset
            self.send(f"OFFSET:{offset:.1f}")
            # Send every profile (with a small gap so the buffer is not flooded)
            def send_profiles(i=0):
                if i >= len(profiles):
                    # After all of them - mark the calibration as ready
                    self.send("SETCALDONE:1")
                    self.dev_cal = True
                    if hasattr(self, 'cal_status'):
                        self.cal_status.config(
                            text=f"✓ Loaded calibration from PC ({len(profiles)} profiles)")
                    print(f"Uploaded {len(profiles)} profiles from PC to the device")
                    return
                p = profiles[i]
                self.send(f"SETPROF:{p['idx']},{p['KpH']:.3f},{p['KiH']:.4f},"
                         f"{p['KdH']:.3f},{p['KpC']:.3f},{p['KiC']:.4f},"
                         f"{p['KdC']:.3f},{1 if p['valid'] else 0}")
                # Next profile in 40ms
                self.root.after(40, lambda: send_profiles(i + 1))
            send_profiles(0)
            saved = data.get('saved', '?')
            print(f"Loading calibration from PC (saved: {saved})")
            return True
        except Exception as e:
            print(f"Calibration load error: {e}")
            return False


    # ────────────────────────────────────────────────────
    #  UI CONSTRUCTION
    # ────────────────────────────────────────────────────
    def _build_ui(self):
        # ── TITLE BAR ───────────────────────────────────────────────────
        # Three zones, left to right: identity (medallion + name + versions),
        # a stretch of nothing, and live state (diagnostics + link light).
        # Before .15 the versions and the state were crammed together on the
        # left and the eye had to hunt for the connection light.
        top = tk.Frame(self.root, bg=C['bg2'], height=SC(48))
        top.pack(fill='x'); top.pack_propagate(False)
        tk.Frame(top, bg=C['gold'], width=SC(4)).pack(side='left', fill='y')

        badge = tk.Canvas(top, width=SC(30), height=SC(30), bg=C['bg2'],
                          highlightthickness=0)
        badge.pack(side='left', padx=(SC(10), SC(8)))
        draw_medallion(badge, SC(15), SC(15), SC(12), C['gold'], C['bg2'])

        idbox = tk.Frame(top, bg=C['bg2'])
        idbox.pack(side='left')
        nrow = tk.Frame(idbox, bg=C['bg2'])
        nrow.pack(anchor='w')
        tk.Label(nrow, text="IGNI", bg=C['bg2'], fg=C['text'],
                 font=(FONT, fsz(15), 'bold')).pack(side='left')
        tk.Label(nrow, text="  photocurrent & pyrocurrent bench",
                 bg=C['bg2'], fg=C['dim'],
                 font=(FONT, fsz(9))).pack(side='left', pady=(SC(4), 0))
        vrow = tk.Frame(idbox, bg=C['bg2'])
        vrow.pack(anchor='w')
        tk.Label(vrow, text=f"APP {APP_BUILD}", bg=C['bg2'], fg=C['dim2'],
                 font=(FONT, fsz(8))).pack(side='left')
        tk.Label(vrow, text="·", bg=C['bg2'], fg=C['dim2'],
                 font=(FONT, fsz(8))).pack(side='left', padx=SC(5))
        self.fw_build_lbl = tk.Label(vrow, text="FW —", bg=C['bg2'], fg=C['dim2'],
                                      font=(FONT, fsz(8)))
        self.fw_build_lbl.pack(side='left')

        # Live state on the right
        sf = tk.Frame(top, bg=C['bg2'])
        sf.pack(side='right', padx=SC(16))
        self.btn_diag = tk.Button(sf, text="DIAG", command=self.open_diag_window,
                                   bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(9), 'bold'),
                                   relief='flat', cursor='hand2', bd=0, padx=SC(10), pady=SC(4),
                                   activebackground=C['panel3'])
        self.btn_diag.pack(side='left', padx=(0, SC(16)))
        self.s_dot = tk.Canvas(sf, width=SC(14), height=SC(14), bg=C['bg2'],
                               highlightthickness=0)
        self.s_dot.pack(side='left', padx=(0, SC(8)))
        self._draw_dot(C['dim2'], glow=False)
        self.s_lbl = tk.Label(sf, text="DISCONNECTED", bg=C['bg2'], fg=C['dim'],
                              font=(FONT, fsz(10)))
        self.s_lbl.pack(side='left')

        # ── TABS, ORDERED BY HOW OFTEN THEY ARE USED ────────────────────
        # Was: CONTROL / ADVANCED / ARCHIVE / SERIES / CONNECTION - which put
        # the every-day SERIES tab fourth, behind two you touch rarely, and
        # hid PID work behind the vague word "ADVANCED". New order follows the
        # actual workflow: run it -> automate it -> look at what came out ->
        # tune it -> deal with the hardware.
        nb = ttk.Notebook(self.root)
        nb.pack(fill='both', expand=True, padx=0, pady=0)
        t1 = tk.Frame(nb, bg=C['bg']); nb.add(t1, text='  CONTROL  ')
        t5 = tk.Frame(nb, bg=C['bg']); nb.add(t5, text='  SERIES  ')
        t3 = tk.Frame(nb, bg=C['bg']); nb.add(t3, text='  ARCHIVE  ')
        t2 = tk.Frame(nb, bg=C['bg']); nb.add(t2, text='  TUNING  ')
        t4 = tk.Frame(nb, bg=C['bg']); nb.add(t4, text='  DEVICE  ')
        self.build_live(t1)
        self.build_series(t5)
        self.build_arch(t3)
        self.build_advanced(t2)
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
        # Enable/disable the panel
        if hasattr(self, 'btn_run'):
            self._set_panel_enabled(connected)

    # ────────────────────────────────────────────────────
    #  LIVE SCREEN: chart (left) + control panel (right)
    # ────────────────────────────────────────────────────
    def build_live(self, parent):
        # Top bar: compact stat cards + START/STOP buttons
        topbar = tk.Frame(parent, bg=C['bg'])
        topbar.pack(fill='x', padx=16, pady=(10, 6))

        # Cards (left part, stretched)
        cards = tk.Frame(topbar, bg=C['bg'])
        cards.pack(side='left', fill='x', expand=True)
        self.cards = {}
        self.cards['temp'] = self._stat_card(cards, "TEMP", "°C", C['blue'])
        self.cards['temp2'] = self._stat_card(cards, "TEMP 2", "°C", C['cyan'])
        self.cards['sp']   = self._stat_card(cards, "SETPOINT", "°C", C['orange'])
        self.cards['rate'] = self._stat_card(cards, "AVG RATE", "°C/min", C['yellow'])
        self.cards['pwm']  = self._stat_card(cards, "PWM", "%", C['green'])

        # START/STOP/E-STOP buttons (right part of the bar) - always visible
        ctrl = tk.Frame(topbar, bg=C['bg'])
        ctrl.pack(side='right', padx=(8, 0))
        self.is_running = False  # state: is a run in progress
        self.btn_run = tk.Button(ctrl, text="▶ START", command=self.toggle_run,
                                 bg=C['green'], fg='#1a1c1f', font=(FONT, fsz(12), 'bold'),
                                 relief='flat', cursor='hand2', bd=0, padx=16, pady=12,
                                 activebackground=_lighten(C['green'], 0.15))
        self.btn_run.pack(side='left', padx=(0, 4), fill='y')
        # FREEZE - freeze the gal for a sample swap
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

        # Main area: chart + panel
        main = tk.Frame(parent, bg=C['bg'])
        main.pack(fill='both', expand=True, padx=16, pady=(0, 12))

        # RIGHT - control panel (packed FIRST!)
        # The fixed 312px width reserves space on the right BEFORE the expanding
        # chart claims the cavity. Otherwise the matplotlib canvas, on a redraw (zoom/
        # home/resize), demands its full size and crushes the panel packed later -> panel vanishes.
        self._build_panel(main)
        # LEFT - chart (fills the remaining space)
        self._build_chart(main)

    def _stat_card(self, parent, title, unit, color):
        """One live readout. Reworked in .15: the value is the thing you read
        from across the room, so it got bigger (16 -> 22) and the card got
        real breathing room; the caption and unit went quieter. The coloured
        rule on top is the only thing tying it to its curve on the chart, so
        it stayed - just thicker."""
        card = tk.Frame(parent, bg=C['panel'])
        card.pack(side='left', fill='both', expand=True, padx=(0, SC(5)))
        tk.Frame(card, bg=color, height=SC(4)).pack(fill='x')
        inner = tk.Frame(card, bg=C['panel'])
        inner.pack(fill='both', expand=True, padx=SC(11), pady=(SC(7), SC(8)))
        tk.Label(inner, text=title, bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(8)), anchor='w').pack(anchor='w')
        vrow = tk.Frame(inner, bg=C['panel'])
        vrow.pack(anchor='w', pady=(SC(3), 0))
        val = tk.Label(vrow, text="--", bg=C['panel'], fg=color,
                       font=(FONT, fsz(22), 'bold'))
        val.pack(side='left')
        unit_lbl = tk.Label(vrow, text=" " + unit, bg=C['panel'], fg=C['dim2'],
                            font=(FONT, fsz(8)))
        unit_lbl.pack(side='left', pady=(SC(8), 0))
        return {'val': val, 'unit': unit, 'unit_lbl': unit_lbl, 'extra': None, 'row': vrow}

    def _build_chart(self, parent):
        wrap = tk.Frame(parent, bg=C['panel'])
        wrap.pack(side='left', fill='both', expand=True, padx=(0, 12))
        tk.Frame(wrap, bg=C['border2'], height=3).pack(fill='x')

        hd = tk.Frame(wrap, bg=C['panel'])
        hd.pack(fill='x', padx=14, pady=(10, 4))
        tk.Label(hd, text="LIVE CHART", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(side='left')

        # Setpoint approach statistics (right side of the header)
        self.reach_lbl = tk.Label(hd, text="", bg=C['panel'], fg=C['green'],
                                  font=(FONT, fsz(9), 'bold'))
        self.reach_lbl.pack(side='right')

        self.fig = Figure(figsize=(9, 6), facecolor=C['panel'], dpi=110)
        medallion_watermark(self.fig, size=0.30, cx=0.52, cy=0.55)
        gs = self.fig.add_gridspec(2, 1, height_ratios=[3, 1], hspace=0.2,
                                   left=0.07, right=0.97, top=0.97, bottom=0.08)
        self.ax1 = self.fig.add_subplot(gs[0])
        self.ax2 = self.fig.add_subplot(gs[1], sharex=self.ax1)
        for ax in [self.ax1, self.ax2]:
            ax.set_facecolor(C['panel2'])

        self.cv = FigureCanvasTkAgg(self.fig, master=wrap)
        self.cv.get_tk_widget().pack(fill='both', expand=True, padx=8, pady=(0, 4))

        # Chart toolbar: pause, time window, matplotlib zoom
        toolbar_row = tk.Frame(wrap, bg=C['panel'])
        toolbar_row.pack(fill='x', padx=8, pady=(0, 8))

        # PAUSE button - stops scrolling so you can zoom in
        self.btn_pause = tk.Button(toolbar_row, text="⏸ PAUSE", command=self.toggle_pause,
                                   bg=C['bg2'], fg=C['yellow'], font=(FONT, fsz(9), 'bold'),
                                   relief='flat', cursor='hand2', bd=0, padx=12, pady=6,
                                   highlightthickness=1, highlightbackground=C['yellow'],
                                   activebackground=C['panel3'])
        self.btn_pause.pack(side='left', padx=(0, 6))

        # Time window selection (how many last seconds to show)
        tk.Label(toolbar_row, text="WINDOW:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(8))).pack(side='left', padx=(8, 4))
        for label, secs in [("ALL", 0), ("5m", 300), ("2m", 120), ("1m", 60)]:
            b = tk.Button(toolbar_row, text=label,
                         command=lambda s=secs: self.set_chart_window(s),
                         bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(8)),
                         relief='flat', cursor='hand2', bd=0, padx=10, pady=5,
                         activebackground=C['panel3'])
            b.pack(side='left', padx=2)

        # Matplotlib toolbar (zoom, pan, save) - compact
        tb_frame = tk.Frame(toolbar_row, bg=C['panel'])
        tb_frame.pack(side='right')
        try:
            self.mpl_toolbar = NavigationToolbar2Tk(self.cv, tb_frame, pack_toolbar=False)
            # The toolbar's own floppy-disk button bypasses everything we do
            # here and would write the dark screen theme straight into the
            # file. Wrap it so it goes through the print palette too.
            _orig_save = self.mpl_toolbar.save_figure
            def _save_light(*a, **kw):
                args = getattr(self, '_live_args', None)
                with print_theme(self.fig):
                    if args: self._redraw_live(*args)
                    r = _orig_save(*a, **kw)
                if args: self._redraw_live(*args)   # wroc do motywu ekranowego
                return r
            self.mpl_toolbar.save_figure = _save_light
            self.mpl_toolbar.config(bg=C['panel'])
            self.mpl_toolbar.update()
            self.mpl_toolbar.pack(side='right')
        except Exception as e:
            print(f"toolbar err: {e}")

    def toggle_pause(self):
        """Pause/resume chart scrolling (for zooming in)"""
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
        """Set the chart time window (0=all)"""
        self.chart_window = secs

    def _build_panel(self, parent):
        """Right control panel - narrow scrollable strip"""
        panel = tk.Frame(parent, bg=C['bg2'], width=SC(312))
        panel.pack(side='right', fill='y')
        panel.pack_propagate(False)
        tk.Frame(panel, bg=C['red'], width=6).pack(side='left', fill='y')

        # Scrollable area - Canvas + Scrollbar (the panel can be taller than the screen)
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
        # Mouse wheel scrolling
        def _on_wheel(e):
            pcanvas.yview_scroll(int(-1 * (e.delta / 120)), 'units')
        pcanvas.bind('<Enter>', lambda e: pcanvas.bind_all('<MouseWheel>', _on_wheel))
        pcanvas.bind('<Leave>', lambda e: pcanvas.unbind_all('<MouseWheel>'))

        inner = tk.Frame(inner, bg=C['bg2'])
        inner.pack(fill='both', expand=True, padx=16, pady=14)

        # The panel used to open with a bare "CONTROL" heading that just
        # repeated the tab name, and its three groups were separated by
        # anonymous hairlines. Named sections say what each group is FOR.
        tk.Label(inner, text="RUN SETUP", bg=C['bg2'], fg=C['text'],
                 font=(FONT, fsz(13), 'bold')).pack(anchor='w')
        tk.Label(inner, text="applied on START", bg=C['bg2'], fg=C['dim2'],
                 font=(FONT, fsz(8))).pack(anchor='w', pady=(1, 0))
        section(inner, "SETPOINT & RAMPS", C['orange'], C['bg2'], pady=(12, 8))

        # Setting sliders
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

        section(inner, "HEATSINK FANS", C['blue'], C['bg2'])

        # FANS - on/off button + speed slider
        fan_hd = tk.Frame(inner, bg=C['bg2'])
        fan_hd.pack(fill='x', pady=(0, 4))
        tk.Label(fan_hd, text="STATE", bg=C['bg2'], fg=C['dim'],
                 font=(FONT, fsz(9), 'bold')).pack(side='left')
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

        section(inner, "STORED SETUPS", C['purple'], C['bg2'])

        # AUTO badge - direction determined automatically
        auto = tk.Frame(inner, bg=C['bg2'], highlightthickness=1,
                        highlightbackground=C['green'])
        auto.pack(fill='x', pady=(0, 10))
        tk.Label(auto, text="● AUTO: direction by setpoint", bg=C['bg2'],
                 fg=C['green'], font=(FONT, fsz(9))).pack(padx=8, pady=6)

        # Multi-step profiles
        # Profiles + Presets
        bf_pp = tk.Frame(inner, bg=C['bg2'])
        bf_pp.pack(fill='x', pady=(0, 8))
        mk_btn_outline(bf_pp, "PROFILES", self.open_profiles, C['purple']).pack(
            side='left', fill='x', expand=True, padx=(0, 3))
        mk_btn_outline(bf_pp, "PRESETS", self.open_presets, C['green']).pack(
            side='left', fill='x', expand=True, padx=(3, 0))

        # Calibration status - clickable (shows progress while calibration runs)
        self.cal_status = tk.Label(inner, text="", bg=C['bg2'], fg=C['purple'],
                                   font=(FONT, fsz(8)), anchor='w', cursor='hand2')
        self.cal_status.pack(fill='x', pady=(0, 4))
        self.cal_status.bind('<Button-1>', lambda e: self.open_cal_window())

        tk.Label(inner, text="▶ START uses panel values",
                 bg=C['bg2'], fg=C['green'], font=(FONT, fsz(8))).pack(anchor='w', pady=(4, 0))
        tk.Label(inner, text="PID tuning & calibration → TUNING tab",
                 bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(8)),
                 justify='left', wraplength=SC(312) - SC(44)
                 ).pack(anchor='w', fill='x', pady=(2, 0))

        self._set_panel_enabled(False)

    def build_advanced(self, parent):
        """ADVANCED tab - PID, calibration, polarity, Flash, reset"""
        wrap = tk.Frame(parent, bg=C['bg'])
        wrap.pack(fill='both', expand=True, padx=20, pady=16)

        # Scrollable area (many options)
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

        # Width limit for readability
        inner = tk.Frame(col, bg=C['bg'])
        inner.pack(fill='x', padx=4, pady=4)
        inner.configure(width=560)

        tk.Label(inner, text="TUNING — PID & CALIBRATION", bg=C['bg'], fg=C['text'],
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
        # Feed-forward (heating): HOLD = power to hold, RAMP = power for ramp dynamics.
        # Tune live: too strong at the start -> lower RAMP; not reaching -> raise it.
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
        """Section frame in the TUNING tab. Since .15 it draws the SAME header
        as section() everywhere else - the tab used to have its own heading
        style (a full-width coloured bar), which made the app look like two
        different programs depending on which tab you were on."""
        section(parent, title, color, C['bg'], pady=(16, 8))
        box = tk.Frame(parent, bg=C['bg2'])
        box.pack(fill='x')
        inner = tk.Frame(box, bg=C['bg2'])
        inner.pack(fill='x', padx=SC(12), pady=SC(10))
        return inner

    def _set_panel_enabled(self, en):
        # Sliders always enabled (values can be set before connecting)
        # START/STOP enabled too - they check the connection at click time
        # (we disable only when we explicitly want to block them)
        for sl in ['sl_sp', 'sl_ru', 'sl_rd', 'sl_tmax', 'sl_kp', 'sl_ki', 'sl_kd', 'sl_off', 'sl_fan']:
            if hasattr(self, sl):
                getattr(self, sl).set_enabled(True)
        # Buttons always clickable - they respond with a message if not connected
        for b in ['btn_run', 'btn_st', 'btn_autocal', 'btn_estop', 'btn_freeze', 'btn_fan']:
            if hasattr(self, b):
                getattr(self, b).config(state='normal')


    # ────────────────────────────────────────────────────
    #  BUTTON ACTIONS
    # ────────────────────────────────────────────────────
    def toggle_run(self):
        """START/STOP toggle in a single button"""
        if self.is_running:
            self.do_stop()
        else:
            self.do_start()

    def _update_run_button(self, running):
        """Update button appearance: green START / red STOP"""
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
        """START - send all panel settings, then run"""
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        # RESET the approach stats - every START counts a fresh average from zero.
        # This makes it possible to run measurements back to back without stale data.
        self.reach_start_t = None
        self.reach_start_temp = None
        self.reach_target = self.sl_sp.get()
        self.reach_done = False
        self.reach_in_tol_t = None
        self.reach_time = None
        self.reach_avg_rate = None
        self.reach_dir = None
        self.last_setpoint_target = None
        self._last_reach_summary = None
        if hasattr(self, 'reach_lbl'):
            self.reach_lbl.config(text="→ starting...", fg=C['dim'])
        # Send the full set of panel settings
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
        # A manual STOP also aborts the automatic measurement SERIES, if one
        # happens to be running - otherwise the app would soon "resurrect" it
        # with the next step, which would be confusing during manual intervention.
        if self.series_running:
            self._series_abort("manual STOP")
        self.send("STOP")
        self.send("AUTOCALSTOP")  # also abort calibration if it is running
        if hasattr(self, 'cal_status'):
            self.cal_status.config(text="")
        self._update_run_button(False)

    def do_estop(self):
        """Emergency stop - disables PWM immediately"""
        self.send("ESTOP")
        self.send("AUTOCALSTOP")
        if hasattr(self, 'cal_status'):
            self.cal_status.config(text="")

    def toggle_fan(self):
        """Turn the fans on/off"""
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
        """Set fan speed (slider)"""
        spd = int(v)
        self.send(f"FAN:{spd}")
        # Slider at 0 = off, >0 = on
        if hasattr(self, 'btn_fan'):
            if spd > 0:
                self.fan_on = True
                self.btn_fan.config(text="● ON", fg=C['green'], highlightbackground=C['green'])
            else:
                self.fan_on = False
                self.btn_fan.config(text="○ OFF", fg=C['dim2'], highlightbackground=C['dim'])

    def do_freeze(self):
        """Freeze the gal to solid state (sample swap)"""
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
        """Reset settings to defaults"""
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        if messagebox.askyesno("Reset settings",
                "Restore default settings?\n"
                "This clears all profiles and calibration!"):
            self.send("RESET")

    def do_repol(self):
        """Force re-detection of the Peltier polarity"""
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
        """Update the polarity indicator in the panel"""
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
        """Open the auto-calibration range selection window"""
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        CalRangeDialog(self.root, self)

    def start_autocal(self, temp_min, temp_max, ramps):
        """Run auto-calibration with the selected range and ramp list"""
        # Send the temp range
        self.send(f"CALRANGE:{temp_min:.0f},{temp_max:.0f}")
        time.sleep(0.1)
        # Send the ramp list (CRUCIAL - this defines what the calibration will cover)
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
        """Open the calibration progress window"""
        if not self.cal_plan and not self.cal_running:
            messagebox.showinfo("Calibration",
                "Calibration is not running.\n"
                "Click AUTO-CAL to start.")
            return
        # If the window is already open - just raise it
        if hasattr(self, 'cal_win') and self.cal_win and tk._default_root:
            try:
                self.cal_win.win.lift()
                return
            except: pass
        self.cal_win = CalibrationWindow(self.root, self)

    def _refresh_cal_view(self):
        """Refresh the calibration window if open + the panel status"""
        # Status in the main panel
        if hasattr(self, 'cal_status'):
            if self.cal_running and self.cal_total > 0:
                eta = self._cal_eta()
                eta_s = f" · ~{int(eta//60)}min" if eta else ""
                self.cal_status.config(
                    text=f"Calibration {self.cal_current}/{self.cal_total}{eta_s} (click=details)")
            elif self.cal_current >= self.cal_total and self.cal_total > 0:
                self.cal_status.config(text="✓ Calibration done")
        # Details window
        if hasattr(self, 'cal_win') and self.cal_win:
            try: self.cal_win.refresh()
            except: pass
        # TARGET/HEAT RATE/COOL RATE: the firmware IGNORES SP/RU/RD commands while
        # calibration is running (sys==CAL - see procCmd) - so an accidental
        # slider move cannot disturb the relay measurement in progress. Previously
        # the sliders were "always enabled" (see _set_panel_enabled), so the user
        # could move them with no warning and nothing happened - they thought
        # the target had changed while the firmware quietly ignored it. Disable them
        # visually for the duration of calibration so it is clear they are dead.
        for sl in ('sl_sp', 'sl_ru', 'sl_rd'):
            if hasattr(self, sl):
                getattr(self, sl).set_enabled(not self.cal_running)

    def open_profiles(self):
        """Multi-step profile editor window"""
        ProfileWindow(self.root, self)

    # ────────────────────────────────────────────────────
    #  PRESETS - saveable sets of settings
    # ────────────────────────────────────────────────────
    def _gather_settings(self):
        """Collect all current settings from the sliders"""
        s = {}
        for key, attr in [('sp','sl_sp'),('ru','sl_ru'),('rd','sl_rd'),
                          ('tmax','sl_tmax'),('kp','sl_kp'),('ki','sl_ki'),
                          ('kd','sl_kd'),('off','sl_off'),('fan','sl_fan')]:
            if hasattr(self, attr):
                try: s[key] = getattr(self, attr).get()
                except: pass
        return s

    def _load_presets(self):
        """Load presets from the JSON file"""
        if not self.presets_file.exists():
            return {}
        try:
            with open(self.presets_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except:
            return {}

    def _save_presets(self, presets):
        """Save presets to the JSON file"""
        try:
            with open(self.presets_file, 'w', encoding='utf-8') as f:
                json.dump(presets, f, indent=2)
            return True
        except Exception as e:
            print(f"presets save err: {e}")
            return False

    def open_presets(self):
        """Open the preset management window"""
        PresetWindow(self.root, self)

    def apply_preset(self, settings):
        """Apply a preset - set the sliders and send to the device"""
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
        # Update the fan state according to fan
        if 'fan' in settings and hasattr(self, 'btn_fan'):
            fv = settings['fan']
            self.fan_on = (fv > 0)
            if fv > 0:
                self.btn_fan.config(text="● ON", fg=C['green'], highlightbackground=C['green'])
            else:
                self.btn_fan.config(text="○ OFF", fg=C['dim2'], highlightbackground=C['dim'])

    # ────────────────────────────────────────────────────
    #  CONNECTION TAB
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
    #  ARCHIVE TAB
    # ────────────────────────────────────────────────────
    def build_arch(self, parent):
        wrap = tk.Frame(parent, bg=C['bg'])
        wrap.pack(fill='both', expand=True, padx=16, pady=16)

        hd = tk.Frame(wrap, bg=C['bg'])
        hd.pack(fill='x', pady=(0, 6))
        tk.Label(hd, text="CYCLE ARCHIVE", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(12), 'bold')).pack(side='left')
        tk.Label(hd, text="  tick cycles on the left, pick curves and X axis below",
                 bg=C['bg'], fg=C['dim2'], font=(FONT, fsz(8))).pack(side='left', padx=(8, 0))
        mk_btn(hd, "REFRESH", self.refresh_arch, C['cyan']).pack(side='right')

        # ── MEASUREMENT DATA FOLDER ─────────────────────────────────────
        # Shown here, because this is the place where the user browses the
        # saved measurements - the natural place to see and change where
        # they actually land.
        dd = tk.Frame(wrap, bg=C['bg2'])
        dd.pack(fill='x', pady=(0, 10))
        tk.Frame(dd, bg=C['green'], width=SC(4)).pack(side='left', fill='y')
        tk.Label(dd, text="DATA:", bg=C['bg2'], fg=C['dim'],
                 font=(FONT, fsz(9), 'bold')).pack(side='left', padx=(10, 6), pady=6)
        self.data_dir_lbl = tk.Label(dd, text="", bg=C['bg2'], fg=C['text'],
                                     font=(FONT, fsz(9)), anchor='w')
        self.data_dir_lbl.pack(side='left', fill='x', expand=True, pady=6)
        mk_btn_outline(dd, "📂 OPEN", self.open_log_folder, C['dim']).pack(
            side='right', padx=(4, 8), pady=4)
        mk_btn_outline(dd, "＋ NEW", self.create_data_dir, C['green']).pack(
            side='right', padx=4, pady=4)
        mk_btn_outline(dd, "CHANGE…", self.choose_data_dir, C['cyan']).pack(
            side='right', padx=4, pady=4)
        self._update_data_dir_label()
        self._bind_tooltip(self.data_dir_lbl, str(self.log_dir))

        body = tk.Frame(wrap, bg=C['bg'])
        body.pack(fill='both', expand=True)

        # Cycle list with checkboxes (for comparison)
        lf = tk.Frame(body, bg=C['panel'], width=SC(340))
        lf.pack(side='left', fill='y', padx=(0, 12))
        lf.pack_propagate(False)
        tk.Frame(lf, bg=C['purple'], height=3).pack(fill='x')
        lhd = tk.Frame(lf, bg=C['panel'])
        lhd.pack(fill='x', padx=12, pady=8)
        tk.Label(lhd, text="SAVED CYCLES", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(side='left')
        mk_btn_outline(lhd, "CLEAR", self._arch_clear_sel, C['dim']).pack(side='right')

        # Scrollable list of checkboxes
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
        # KEY POINT: the inner window must have the canvas width, otherwise
        # the rows do not stretch and the ✕ button (side='right') falls out of view
        self.arch_canvas.bind('<Configure>',
            lambda e: self.arch_canvas.itemconfig(self._arch_win, width=e.width))
        self.arch_canvas.bind('<Enter>', lambda e: self.arch_canvas.bind_all(
            '<MouseWheel>', lambda ev: self.arch_canvas.yview_scroll(int(-ev.delta/120), 'units')))
        self.arch_canvas.bind('<Leave>', lambda e: self.arch_canvas.unbind_all('<MouseWheel>'))

        self.arch_vars = {}   # {path: BooleanVar}

        # Chart
        cf = tk.Frame(body, bg=C['panel'])
        cf.pack(side='left', fill='both', expand=True)
        tk.Frame(cf, bg=C['border2'], height=3).pack(fill='x')
        # THE PACKING ORDER MATTERS. The figure has its OWN requested size
        # (figsize x dpi = approx. 880x500 px). When the canvas is packed first
        # with expand=True, pack gives it that requested size, and the rows
        # packed AFTER it get whatever is left - which on a shorter window is
        # exactly 1 pixel. Symptom: the "X AXIS" and "CURVES" bars existed but
        # were 1x1 in size and invisible (found by a geometry test at
        # 1600x900 and 1366x768). That is why all control rows are packed
        # FIRST, from the bottom (side='bottom'), and the canvas gets the rest.
        self.fig_a = Figure(figsize=(8, 4.5), facecolor=C['panel'], dpi=110)
        medallion_watermark(self.fig_a, size=0.34, cx=0.52, cy=0.55)
        self.ax_a = self.fig_a.add_subplot(111)
        self.ax_a.set_facecolor(C['panel2'])

        # Run settings panel - the bottom row
        self.arch_settings = tk.Frame(cf, bg=C['bg2'])
        self.arch_settings.pack(side='bottom', fill='x', padx=8, pady=(0, 8))
        self.arch_settings_lbl = tk.Label(self.arch_settings, text="",
                                         bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(9)),
                                         anchor='w', justify='left')
        self.arch_settings_lbl.pack(fill='x', padx=10, pady=6)

        crow = tk.Frame(cf, bg=C['panel'])
        crow.pack(side='bottom', fill='x', padx=8, pady=(0, 6))
        # X AXIS has its OWN row - sharing it with the export buttons made
        # the last options ("PC clock", "ramp start", "rel. temperature")
        # not fit into the width at larger fonts, so they got size 1x1,
        # i.e. they vanished.
        xrow = tk.Frame(cf, bg=C['panel'])
        xrow.pack(side='bottom', fill='x', padx=8, pady=(0, 4))
        atb = tk.Frame(cf, bg=C['panel'])
        atb.pack(side='bottom', fill='x', padx=8, pady=(2, 6))
        tbf = tk.Frame(cf, bg='#3a3f44')
        tbf.pack(side='bottom', fill='x', padx=8, pady=(4, 0))

        # The canvas gets ALL the remaining space
        self.cv_a = FigureCanvasTkAgg(self.fig_a, master=cf)
        self.cv_a.get_tk_widget().pack(fill='both', expand=True, padx=8, pady=(8, 4))
        # First draw BEFORE the toolbar - it initializes the canvas
        self.cv_a.draw()
        try:
            self.mpl_toolbar_a = NavigationToolbar2Tk(self.cv_a, tbf, pack_toolbar=False)
            self.mpl_toolbar_a.config(bg='#3a3f44')
            # Toolbar buttons readable on the dark background
            for child in self.mpl_toolbar_a.winfo_children():
                try: child.config(bg='#3a3f44')
                except: pass
            self.mpl_toolbar_a.update()
            self.mpl_toolbar_a.pack(side='left', fill='x')
        except Exception as e:
            print(f"arch toolbar err: {e}")

        # Export buttons - in the 'atb' row created above
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
        # ── X AXIS MODE ─────────────────────────────────────────────────
        # arch_align stays for compatibility with the old code (the export
        # uses it, among others), but it is now driven by the mode below.
        self.arch_align = tk.BooleanVar(value=True)
        self.arch_xmode = tk.StringVar(value='t0')
        tk.Label(xrow, text="X AXIS:", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(8), 'bold')).pack(side='left', padx=(0, 6))
        for val, txt in (('t0', 'from start'), ('abs', 'file time'),
                         ('pc', 'PC clock'), ('ramp', 'ramp start'),
                         ('temp', 'rel. temperature')):
            tk.Radiobutton(xrow, text=txt, value=val, variable=self.arch_xmode,
                           command=self._on_xmode_change, bg=C['panel'], fg=C['dim'],
                           selectcolor=C['bg2'], activebackground=C['panel'],
                           activeforeground=C['text'], font=(FONT, fsz(8)),
                           bd=0, highlightthickness=0).pack(side='left')
        # Reference temperature for the "rel. temperature" mode
        self.arch_treflbl = tk.Label(xrow, text="T=", bg=C['panel'], fg=C['dim'],
                                     font=(FONT, fsz(8)))
        self.arch_treflbl.pack(side='left', padx=(8, 2))
        self.arch_tref = tk.Entry(xrow, width=6, bg=C['bg2'], fg=C['text'],
                                  font=(FONT, fsz(9)), relief='flat',
                                  insertbackground=C['text'])
        self.arch_tref.insert(0, "40.0")
        self.arch_tref.pack(side='left')
        self.arch_tref.bind('<Return>', lambda e: self._redraw_arch())
        self.arch_tref.bind('<FocusOut>', lambda e: self._redraw_arch())

        # ── WHICH CURVES TO DRAW ────────────────────────────────────────
        tk.Label(crow, text="CURVES:", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(8), 'bold')).pack(side='left', padx=(0, 6))
        self.arch_show = {}
        for key, txt, dflt in (('temp', 'temperature', True),
                               ('sa', 'setpoint', True),
                               ('st', 'target', True),
                               ('t2', 'temp 2', False),
                               ('pwm', 'PWM', False)):
            v = tk.BooleanVar(value=dflt)
            self.arch_show[key] = v
            tk.Checkbutton(crow, text=txt, variable=v, command=self._redraw_arch,
                           bg=C['panel'], fg=C['dim'], selectcolor=C['bg2'],
                           activebackground=C['panel'], activeforeground=C['text'],
                           font=(FONT, fsz(8)), bd=0, highlightthickness=0
                           ).pack(side='left', padx=(0, 4))
        mk_btn_outline(crow, "SELECT ALL", self._arch_select_all, C['dim']
                       ).pack(side='right')
        # Comparing runs AGAINST EACH OTHER: instead of temperatures we draw
        # the DIFFERENCE of every run relative to the first one ticked
        # (interpolated onto a common time axis). Differences are far easier
        # to see than two nearly identical curves overlaid on each other.
        self.arch_delta = tk.BooleanVar(value=False)
        tk.Checkbutton(xrow, text="delta vs 1st", variable=self.arch_delta,
                       command=self._redraw_arch, bg=C['panel'], fg=C['yellow'],
                       selectcolor=C['bg2'], activebackground=C['panel'],
                       activeforeground=C['text'], font=(FONT, fsz(8)),
                       bd=0, highlightthickness=0).pack(side='right', padx=(0, 10))

        # (run settings panel created above, as the bottom row)

        self.refresh_arch()
        # Draw an empty chart right away - it initializes the canvas and toolbar
        self._redraw_arch()

    def build_series(self, parent):
        """SERIES tab - a list of tests (SP/RATE/hold) executed automatically
        one after another, each auto-archived (without asking for a name) -
        the files land in PeltierLogi ready for analysis."""
        wrap = tk.Frame(parent, bg=C['bg'])
        wrap.pack(fill='both', expand=True, padx=16, pady=16)

        tk.Label(wrap, text="MEASUREMENT SERIES", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(12), 'bold')).pack(anchor='w')
        tk.Label(wrap,
                 text="Add tests (SP/RATE/hold time) - the app runs them one by one, "
                      "returns to base between tests and archives every result itself.",
                 bg=C['bg'], fg=C['dim2'], font=(FONT, fsz(8)), justify='left',
                 wraplength=SC(760)
                 ).pack(anchor='w', pady=(2, SC(12)))

        body = tk.Frame(wrap, bg=C['bg'])
        body.pack(fill='both', expand=True)

        # ── Left column: adding a step + base settings ───────────────
        # WIDTH: it was hard-coded as 280 PIXELS together with
        # pack_propagate(False), so the column NEVER grew to fit the content.
        # A measurement with real font metrics showed that at FS=1.0 the widest
        # caption is 342 px (and with margins 378 is needed) - so the text was
        # clipped ALREADY WITHOUT DPI scaling, and at FS=1.5 not even the field
        # labels themselves fit ("HOLD AFTER REACHED (s)" = 288 px).
        # Now: the width is scaled by SC(), with headroom, and long descriptions
        # have wraplength (they wrap instead of stretching/overflowing).
        SER_W = SC(320)
        SER_PAD = SC(14)
        self._ser_wrap = SER_W - 2 * SER_PAD - SC(6)
        left = tk.Frame(body, bg=C['panel'], width=SER_W)
        left.pack(side='left', fill='y', padx=(0, SC(12)))
        left.pack_propagate(False)
        tk.Frame(left, bg=C['cyan'], height=SC(3)).pack(fill='x')
        # The column content is SCROLLABLE: at larger fonts (FS=1.5) it is
        # taller than the window and the lowest items (QUICK FILL, the
        # quickfill button) were physically out of reach - cut off by the screen.
        lin = make_scrollable(left, C['panel'], padx=SER_PAD, pady=SC(12))

        def _field(label, default):
            tk.Label(lin, text=label, bg=C['panel'], fg=C['dim'],
                     font=(FONT, fsz(9))).pack(anchor='w', pady=(8, 2))
            e = tk.Entry(lin, bg=C['bg2'], fg=C['text'], font=(FONT, fsz(11), 'bold'),
                         relief='flat', insertbackground=C['text'])
            e.insert(0, default)
            e.pack(fill='x', ipady=4)
            return e

        self.series_e_sp = _field("SP (°C)", "50.0")
        self.series_e_rate = _field("HEAT RATE (°C/min)", "30.0")
        self.series_e_hold = _field("HOLD AFTER REACHED (s)", "60")

        mk_btn(lin, "+ ADD TEST", self._on_series_add, C['cyan']
               ).pack(fill='x', pady=(12, 4))
        mk_btn_outline(lin, "DELETE SELECTED", self._on_series_remove, C['dim']
                       ).pack(fill='x', pady=(0, 4))
        mk_btn_outline(lin, "CLEAR LIST", self._on_series_clear, C['dim']
                       ).pack(fill='x')

        tk.Frame(lin, bg=C['border2'], height=1).pack(fill='x', pady=12)
        tk.Label(lin, text="BASE BETWEEN TESTS (°C)", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(9))).pack(anchor='w', pady=(0, 2))
        self.series_e_base = tk.Entry(lin, bg=C['bg2'], fg=C['text'],
                                       font=(FONT, fsz(11), 'bold'), relief='flat',
                                       insertbackground=C['text'])
        self.series_e_base.insert(0, f"{self.series_base_sp:.1f}")
        self.series_e_base.bind('<FocusOut>', self._on_series_base_change)
        self.series_e_base.bind('<Return>', self._on_series_base_change)
        self.series_e_base.pack(fill='x', ipady=4)

        tk.Label(lin, text="RETURN RATE (°C/min)", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(9))).pack(anchor='w', pady=(10, 2))
        self.series_e_return_rate = tk.Entry(lin, bg=C['bg2'], fg=C['text'],
                                              font=(FONT, fsz(11), 'bold'), relief='flat',
                                              insertbackground=C['text'])
        # CHANGED after analysing log 20260827 145616 (step/"hump" at the start
        # of every return): the default 25.0 was SLOWER than the natural
        # (passive, with the fans at 100%) cooling of the object right after
        # a hot hold - the data shows a real temp drop right after the
        # start on the order of tens of C/min, much faster than the
        # commanded 25. Result: the ramp (spA) immediately fell BEHIND
        # THE REAL drop (temp<spA), the PID read that as
        # "too cold too early" and added heat in order to BRAKE the cooling to
        # the commanded rate - hence the visible rebound "hump" (temp briefly
        # RISES) right after the start of every return. The firmware already has
        # exactly the same pattern ("full retreat speed") for other returns
        # to base (see rU=rD=RAMP_MAX in the .ino) - here we do the same: a very
        # large value, which the firmware safely clamps anyway to its own
        # RAMP_MAX (constrain(fv,RAMP_MIN,RAMP_MAX) on the RD command) - so the
        # ramp is NEVER slower than the natural cooling and there is nothing
        # to "brake" with extra heating. Still editable by hand in the field
        # (e.g. if someone DELIBERATELY wants a slower, controlled return).
        self.series_e_return_rate.insert(0, "80.0")
        self.series_e_return_rate.pack(fill='x', ipady=4)
        tk.Label(lin,
                 text="Independent of COOL RATE on CONTROL. Max by default - "
                      "a slower return gives a 'hump' at the start.",
                 bg=C['panel'], fg=C['dim2'], font=(FONT, fsz(8)), justify='left',
                 wraplength=self._ser_wrap
                 ).pack(anchor='w', fill='x', pady=(3, 0))

        # The descent as a FULL-FLEDGED TEST, not just a trip back to the start.
        # WHY: the whole power model (FF_GAIN/FF_TEMP_GAIN in the firmware) is
        # calibrated from HEATING data - for cooling we have no
        # reliable calibration, because so far the descents were run at a fixed,
        # fast return rate (and earlier were not archived at all).
        # Ticking this box makes the descent after every test run at
        # THE SAME rate as the test - so an R10..R70 series gives a full set of
        # 7 heating runs AND 7 cooling runs at different rates,
        # exactly what is needed to calibrate the cooling branch
        # with the same method as the heating one.
        # MODE: "test series" (return to base after every test - for
        # comparing single ramps) or "program" (the steps run one after
        # another from the point where the previous one ended - for defining
        # profiles such as: go to 50, hold, drop to 30, hold).
        # Every step is saved as a SEPARATE, normal measurement in the
        # archive anyway, so it is compared exactly like a manual one.
        self.series_mode = tk.StringVar(value='seria')
        tk.Label(lin, text="MODE", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(9))).pack(anchor='w', pady=(SC(10), 2))
        for val, txt in (('seria', 'test series (return to base)'),
                         ('program', 'program (step by step)')):
            tk.Radiobutton(lin, text=txt, value=val, variable=self.series_mode,
                           bg=C['panel'], fg=C['dim'], selectcolor=C['bg2'],
                           activebackground=C['panel'], activeforeground=C['text'],
                           font=(FONT, fsz(8)), bd=0, highlightthickness=0,
                           anchor='w', wraplength=self._ser_wrap,
                           justify='left').pack(anchor='w', fill='x')
        pf = tk.Frame(lin, bg=C['panel'])
        pf.pack(fill='x', pady=(SC(6), 0))
        mk_btn_outline(pf, "SAVE PROGRAM", self._series_save_prog, C['dim']).pack(
            side='left', fill='x', expand=True, padx=(0, 2))
        mk_btn_outline(pf, "LOAD", self._series_load_prog, C['dim']).pack(
            side='left', fill='x', expand=True, padx=(2, 0))

        self.series_cool_as_test = tk.BooleanVar(value=False)
        tk.Checkbutton(lin, text="descent also as TEST",
                       variable=self.series_cool_as_test,
                       bg=C['panel'], fg=C['dim'], selectcolor=C['bg2'],
                       activebackground=C['panel'], activeforeground=C['text'],
                       font=(FONT, fsz(9)), bd=0, highlightthickness=0,
                       anchor='w').pack(anchor='w', fill='x', pady=(SC(10), 0))
        tk.Label(lin,
                 text="Collects data for cooling calibration. Otherwise the descent "
                      "runs at max rate (return only).",
                 bg=C['panel'], fg=C['dim2'], font=(FONT, fsz(8)), justify='left',
                 wraplength=self._ser_wrap
                 ).pack(anchor='w', fill='x', pady=(3, 0))

        tk.Label(lin, text="QUICK FILL", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(9))).pack(anchor='w', pady=(16, 2))
        # Shorter caption - the full one ("SP from field x ramps 10/20/30/40/50/60/70")
        # did not fit in the column at larger fonts, and a button does not
        # wrap text, so the ends were cut off.
        mk_btn_outline(lin, "SP × ramps 10…70",
                       self._on_series_quickfill, C['purple']).pack(fill='x')
        tk.Label(lin, text="adds 7 tests: 10/20/30/40/50/60/70 °C/min",
                 bg=C['panel'], fg=C['dim2'], font=(FONT, fsz(8)), justify='left',
                 wraplength=self._ser_wrap).pack(anchor='w', fill='x', pady=(3, 0))

        # ── Right column: list + status + start/stop ────────────
        right = tk.Frame(body, bg=C['panel'])
        right.pack(side='left', fill='both', expand=True)
        tk.Frame(right, bg=C['border2'], height=3).pack(fill='x')

        rhd = tk.Frame(right, bg=C['panel'])
        rhd.pack(fill='x', padx=14, pady=(10, 4))
        tk.Label(rhd, text="TEST LIST", bg=C['panel'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(side='left')

        self.series_listbox = tk.Listbox(right, bg=C['bg2'], fg=C['text'],
                                          font=(FONT, fsz(10)), relief='flat',
                                          selectbackground=C['cyan'], height=14,
                                          highlightthickness=0, bd=0)
        self.series_listbox.pack(fill='both', expand=True, padx=14, pady=(0, 8))

        stf = tk.Frame(right, bg=C['panel'])
        stf.pack(fill='x', padx=14, pady=(0, 10))
        self.series_status_lbl = tk.Label(stf, text="Series inactive",
                                           bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(9)),
                                           anchor='w', justify='left')
        self.series_status_lbl.pack(fill='x', ipady=8, padx=2)

        btnf = tk.Frame(right, bg=C['panel'])
        btnf.pack(fill='x', padx=14, pady=(0, 14))
        self.btn_series_run = mk_btn(btnf, "▶ START SERIES", self._on_series_toggle, C['green'])
        self.btn_series_run.pack(fill='x')

        self._series_refresh_list()

    def _on_series_add(self):
        try:
            sp = float(self.series_e_sp.get().replace(',', '.'))
            rate = float(self.series_e_rate.get().replace(',', '.'))
            hold = float(self.series_e_hold.get().replace(',', '.'))
        except ValueError:
            messagebox.showwarning("Invalid value", "SP/RATE/hold must be numbers.")
            return
        self.series_add_step(sp, rate, hold)

    def _on_series_remove(self):
        sel = self.series_listbox.curselection()
        if sel:
            self.series_remove_step(sel[0])

    def _on_series_clear(self):
        self.series_steps = []
        self._series_refresh_list()

    def _on_series_base_change(self, evt=None):
        try:
            self.series_base_sp = float(self.series_e_base.get().replace(',', '.'))
        except ValueError:
            self.series_e_base.delete(0, 'end')
            self.series_e_base.insert(0, f"{self.series_base_sp:.1f}")

    def _on_series_quickfill(self):
        try:
            sp = float(self.series_e_sp.get().replace(',', '.'))
        except ValueError:
            messagebox.showwarning("Invalid value", "Enter SP first.")
            return
        for rate in (10, 20, 30, 40, 50, 60, 70):
            self.series_add_step(sp, rate, 60)

    def _on_series_toggle(self):
        if self.series_running:
            self._series_abort("manual STOP SERIES")
            self.send("STOP")
            self._update_run_button(False)
        else:
            self.series_start()

    def _cycle_display_name(self, path):
        """Readable cycle name: strips the c_/cykl_ prefix and turns _ into spaces"""
        from pathlib import Path as _P
        s = _P(path).stem
        if s.startswith('cykl_'): s = s[5:]
        elif s.startswith('c_'): s = s[2:]
        return s.replace('_', ' ')

    def _bind_tooltip(self, widget, text):
        """Simple tooltip showing the full text on hover"""
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
        # Clear the checkbox list
        for w in self.arch_items.winfo_children():
            w.destroy()
        self.arch_vars = {}
        files = sorted([f for f in self.log_dir.glob("*.csv") if (f.name.startswith("cykl_") or f.name.startswith("c_")) and not f.name.startswith("_tmp")],
                       key=lambda f: f.stat().st_mtime, reverse=True)
        # Color palette for the comparison
        self._arch_colors = [C['blue'], C['orange'], C['green'], C['red'],
                            C['cyan'], C['purple'], C['yellow'], '#ff8fab']
        if not files:
            tk.Label(self.arch_items, text="No saved cycles yet.\nRun a cycle and give it a name.",
                     bg=C['bg2'], fg=C['dim2'], font=(FONT, fsz(9)), justify='left').pack(
                     anchor='w', padx=12, pady=12)
            return

        # Grouping by date (file modification day)
        from datetime import datetime as _dt
        import time as _time
        groups = {}
        for f in files:
            day = _dt.fromtimestamp(f.stat().st_mtime).strftime("%Y-%m-%d")
            groups.setdefault(day, []).append(f)

        today = _dt.now().strftime("%Y-%m-%d")
        i = 0
        for day, day_files in groups.items():
            # Group header (date)
            day_label = "Today" if day == today else day
            hdr = tk.Frame(self.arch_items, bg=C['panel'])
            hdr.pack(fill='x', pady=(6, 1))
            tk.Label(hdr, text=f"▸ {day_label}  ({len(day_files)})", bg=C['panel'],
                     fg=C['cyan'], font=(FONT, fsz(8), 'bold'), anchor='w').pack(
                     side='left', padx=8, pady=3)
            # Files in the group
            for f in day_files:
                row = tk.Frame(self.arch_items, bg=C['bg2'])
                row.pack(fill='x', pady=1)
                var = tk.BooleanVar(value=False)
                self.arch_vars[str(f)] = var
                col = self._arch_colors[i % len(self._arch_colors)]
                i += 1
                # PACK ORDER: the bin FIRST (side=right) = always visible,
                # then the dot (left), and finally the checkbox fills the middle.
                # This way a long name does not cover the bin.
                delb = tk.Button(row, text="🗑", command=lambda p=f: self._delete_cycle(p),
                                bg=C['bg2'], fg=C['red'], font=(FONT, fsz(11), 'bold'),
                                relief='flat', cursor='hand2', bd=0, padx=10, pady=2,
                                activebackground=C['red'], activeforeground='#fff')
                delb.pack(side='right', padx=(2, 6))
                dot = tk.Frame(row, bg=col, width=10, height=10)
                dot.pack(side='left', padx=(8, 4))
                dot.pack_propagate(False)
                # Name shortened if too long (so it does not stretch the row)
                full_name = self._cycle_display_name(f)
                disp_name = full_name if len(full_name) <= 22 else full_name[:20] + "…"
                cb = tk.Checkbutton(row, text=disp_name,
                                   variable=var, command=self._redraw_arch,
                                   bg=C['bg2'], fg=C['text'], selectcolor=C['panel'],
                                   activebackground=C['bg2'], activeforeground=col,
                                   font=(FONT, fsz(9)), bd=0, highlightthickness=0,
                                   anchor='w')
                # Full name in the tooltip (on hover)
                if len(full_name) > 22:
                    self._bind_tooltip(cb, full_name)
                cb.pack(side='left', fill='x', expand=True)

    def _delete_cycle(self, path):
        """Delete a cycle file from the archive (with confirmation)"""
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
        """Read the run settings from the CSV: target SP, ramps, PID. Returns a dict or None"""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                rows = [_csv_row(r) for r in csv.DictReader(f)]
        except Exception:
            return None
        # Find the first valid data row
        valid = [r for r in rows if r.get('czas_s', '').replace('.','').replace('-','').isdigit()]
        if not valid:
            return None
        s = {}
        # Target setpoint - the most frequent setpoint_cel value (final target)
        try:
            sps = [float(r['setpoint_cel']) for r in valid if r.get('setpoint_cel')]
            s['target'] = max(set(sps), key=sps.count) if sps else None
        except: s['target'] = None
        # PID - from the first row (constant over the run or from calibration)
        try:
            s['kp'] = float(valid[0].get('Kp', 0))
            s['ki'] = float(valid[0].get('Ki', 0))
            s['kd'] = float(valid[0].get('Kd', 0))
        except: s['kp'] = s['ki'] = s['kd'] = None
        # Estimate the ramp from the setpoint_aktywny slope at the beginning
        try:
            t0 = float(valid[0]['czas_s'])
            sa0 = float(valid[0]['setpoint_aktywny'])
            # find a point ~10s later
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
        """Deselect all cycles"""
        for v in self.arch_vars.values():
            v.set(False)
        self._redraw_arch()

    def _load_cycle_data(self, path):
        """Load cycle data from the CSV (comment-tolerant). Returns (t,temp,spt,pwm) or None"""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = [_csv_row(r) for r in csv.DictReader(f)]
        except Exception:
            return None
        t, temp, spt, pwm = [], [], [], []
        temp2 = []
        sa_list = []
        pc_raw = []
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
            # active setpoint (ramp) - separately for the chart
            try:
                sa_list.append(float(r.get('setpoint_aktywny', 'nan')))
            except:
                sa_list.append(None)
            try:
                pwm.append(float(r.get('PWM_%', r.get('PWM', 0))))
            except:
                pwm.append(0)
            # temp2 - the second thermocouple (optional column)
            try:
                t2v = r.get('temperatura2_C', '')
                temp2.append(float(t2v) if t2v else None)
            except:
                temp2.append(None)
            # PC time (column added later - old files do not have it)
            pc_raw.append(r.get('czas_pc', '') or '')
        if not t:
            return None
        # Attach temp2 and the active setpoint as attributes (compatibly - we return 4)
        self._last_temp2 = temp2
        self._last_sa = sa_list
        self._last_pc = self._pc_seconds(pc_raw, t, path)
        return t, temp, spt, pwm

    def _pc_seconds(self, pc_raw, t, path):
        """Convert the czas_pc column into epoch seconds. For old files (without
        that column) it rebuilds the time axis from the file modification date:
        mtime is the moment of CLOSING, so start = mtime - run length."""
        out = []
        ok = False
        for sraw in pc_raw:
            v = None
            if sraw:
                try:
                    dt = datetime.strptime(sraw[:23], "%Y-%m-%d %H:%M:%S.%f")
                    v = dt.timestamp(); ok = True
                except Exception:
                    try:
                        dt = datetime.strptime(sraw[:19], "%Y-%m-%d %H:%M:%S")
                        v = dt.timestamp(); ok = True
                    except Exception:
                        v = None
            out.append(v)
        if ok:
            # fill any gaps linearly relative to czas_s
            base = next((i for i, v in enumerate(out) if v is not None), None)
            if base is not None:
                t0 = out[base] - t[base]
                out = [v if v is not None else t0 + t[i] for i, v in enumerate(out)]
            return out
        # fallback: from the file mtime
        try:
            end = Path(path).stat().st_mtime
            t0 = end - (t[-1] - t[0])
            return [t0 + (x - t[0]) for x in t]
        except Exception:
            return [None] * len(t)

    def _compute_stats(self, data):
        """Compute the full run statistics. data=(t,temp,spt,pwm). Returns a dict."""
        import statistics
        t, temp, spt, pwm = data
        st = {}
        st['tmin'] = min(temp)
        st['tmax'] = max(temp)
        st['duration'] = t[-1] - t[0] if len(t) > 1 else 0
        st['target'] = spt[-1] if spt else 0

        # Average rise rate (start -> max)
        idx_max = temp.index(st['tmax'])
        rise_time = t[idx_max] - t[0] if idx_max > 0 else 0
        st['avg_rise'] = (st['tmax'] - temp[0]) / (rise_time/60.0) if rise_time > 5 else 0

        # Overshoot - how far temp exceeded the target (in the settled phase)
        target = st['target']
        st['overshoot'] = max(0, st['tmax'] - target) if target else 0

        # Settling time - when temp entered and stayed within +/-1C of the target
        st['settle_time'] = None
        if target:
            band = 1.0
            for i, tm in enumerate(temp):
                if abs(tm - target) <= band:
                    # check whether it stayed in the band to the end (or for 80% of the rest)
                    rest = temp[i:]
                    in_band = sum(1 for x in rest if abs(x-target) <= band)
                    if in_band >= len(rest)*0.8:
                        st['settle_time'] = t[i] - t[0]
                        break

        # Steady-state error - mean deviation over the last 20% of samples
        n = len(temp)
        tail = temp[int(n*0.8):] if n > 5 else temp
        if target and tail:
            st['steady_error'] = statistics.mean(abs(x - target) for x in tail)
        else:
            st['steady_error'] = 0

        # Max deviation from the setpoint (ramp) - how well it tracked
        devs = [abs(temp[i] - spt[i]) for i in range(len(temp))]
        st['max_dev'] = max(devs) if devs else 0

        # Standard deviation of the noise - in the settled phase (last 20%)
        # This is a measure of measurement quality (thermocouple noise)
        if len(tail) > 2:
            st['noise_std'] = statistics.stdev(tail)
        else:
            st['noise_std'] = 0

        return st

    def _on_xmode_change(self):
        """X axis radiobutton -> sync the old arch_align and redraw."""
        self.arch_align.set(self.arch_xmode.get() != 'abs')
        self._redraw_arch()

    def _arch_select_all(self):
        """Select all cycles (and when all are already selected - deselect)."""
        if not self.arch_vars:
            return
        target = not all(v.get() for v in self.arch_vars.values())
        for v in self.arch_vars.values():
            v.set(target)
        self._redraw_arch()

    def _arch_t_offset(self, t, temp, mode, tref):
        """X axis offset for a single run, according to the selected mode."""
        if mode == 'abs':
            return 0.0
        if mode == 'temp':
            # Find the FIRST crossing of tref (with linear interpolation
            # between samples) and take that moment as zero. This way
            # runs with different starting temperatures overlay
            # at exactly the same thermal point, not the same time point.
            for i in range(1, len(temp)):
                a, b = temp[i-1], temp[i]
                if (a - tref) * (b - tref) <= 0 and a != b:
                    f = (tref - a) / (b - a)
                    return t[i-1] + f * (t[i] - t[i-1])
                if a == tref:
                    return t[i-1]
            return t[0]      # tref not reached - align from the start
        if mode == 'ramp':
            # Zero = the moment the ramp REALLY starts. We detect it from
            # the active setpoint (spA): while it stands still, we are pre-start.
            # This aligns runs with a different "run-up" length before
            # the actual ramp (e.g. when one started from a cold device).
            sa = self._last_sa or []
            for i in range(1, min(len(sa), len(t))):
                if sa[i] is not None and sa[0] is not None and abs(sa[i]-sa[0]) > 0.05:
                    return t[i]
            # no spA (old file) - fallback: the first clear temp change
            for i in range(1, len(temp)):
                if abs(temp[i]-temp[0]) > 0.3:
                    return t[i]
            return t[0]
        return t[0]          # 't0'
    def _redraw_arch(self):
        """Draw all selected runs (comparison)"""
        selected = [(p, v) for p, v in self.arch_vars.items() if v.get()]
        self.ax_a.clear()
        # The second axis (PWM) is created on demand - we delete the old one on
        # every redraw, otherwise more and more axes would pile up.
        if getattr(self, '_ax_pwm', None) is not None:
            try: self._ax_pwm.remove()
            except Exception: pass
            self._ax_pwm = None
        self.ax_a.set_facecolor(C['panel2'])

        mode = self.arch_xmode.get() if hasattr(self, 'arch_xmode') else 't0'
        show = {k: v.get() for k, v in getattr(self, 'arch_show', {}).items()} or \
               {'temp': True, 'sa': True, 'st': True, 't2': False, 'pwm': False}
        try:
            tref = float(self.arch_tref.get().replace(',', '.'))
        except Exception:
            tref = 40.0
        # The T= field only makes sense in "rel. to temperature" mode
        if hasattr(self, 'arch_tref'):
            st_ = 'normal' if mode == 'temp' else 'disabled'
            try:
                self.arch_tref.config(state=st_)
                self.arch_treflbl.config(fg=C['cyan'] if mode == 'temp' else C['dim2'])
            except Exception:
                pass

        if not selected:
            self.ax_a.text(0.5, 0.5, "Select one or more runs on the left",
                          ha='center', va='center', color=C['dim2'],
                          fontsize=11, transform=self.ax_a.transAxes)
            self.cv_a.draw()
            if hasattr(self, 'arch_settings_lbl'):
                self.arch_settings_lbl.config(text="")
            return

        files = sorted([f for f in self.log_dir.glob("*.csv") if (f.name.startswith("cykl_") or f.name.startswith("c_")) and not f.name.startswith("_tmp")], reverse=True)
        file_order = {str(f): i for i, f in enumerate(files)}

        multi = len(selected) > 1

        # ── Collect the data of everything selected, compute the offsets ──
        series = []
        for path, _ in selected:
            d = self._load_cycle_data(path)
            if not d: continue
            t, temp, spt, pwm = d
            series.append(dict(path=path, t=t, temp=temp, spt=spt, pwm=pwm,
                               sa=list(self._last_sa or []),
                               t2=list(self._last_temp2 or []),
                               pc=list(self._last_pc or [])))
        if not series:
            self.cv_a.draw(); return
        # Order = the same as in the list on the left. Important for the
        # "difference rel. to 1st" mode - the reference must be the trace the
        # user sees first, not an arbitrary ordering of the selection dict.
        series.sort(key=lambda z: file_order.get(z['path'], 10**6))

        if mode == 'pc':
            # Common axis = PC clock. We take zero from the EARLIEST
            # run, so that the numbers on the axis stay small, while the
            # labels still show the real time of day (formatter below).
            starts = [s['pc'][0] for s in series if s['pc'] and s['pc'][0] is not None]
            self._pc_zero = min(starts) if starts else 0.0
            for s in series:
                s['off'] = 0.0
        else:
            for s in series:
                s['off'] = self._arch_t_offset(s['t'], s['temp'], mode, tref)

        # Axis unit: seconds or minutes
        spans = []
        for s in series:
            if mode == 'pc' and s['pc'] and s['pc'][0] is not None:
                spans.append(s['pc'][-1] - s['pc'][0])
            else:
                spans.append(s['t'][-1] - s['t'][0])
        max_t = max(spans) if spans else 0
        use_min = max_t > 180
        tdiv = 60.0 if use_min else 1.0

        # ── COMPARISON AGAINST EACH OTHER ───────────────────────────────
        delta_mode = bool(getattr(self, 'arch_delta', None) and self.arch_delta.get()
                          and len(series) > 1)
        ref = series[0] if delta_mode else None
        ref_x, ref_name = None, ""
        if delta_mode:
            from pathlib import Path as _P
            ref_name = self._cycle_display_name(_P(ref['path']))
            if mode == 'pc' and ref['pc'] and ref['pc'][0] is not None:
                ref_x = [((v if v is not None else 0) - self._pc_zero) / tdiv for v in ref['pc']]
            else:
                ref_x = [(x - ref['off']) / tdiv for x in ref['t']]
            # In difference mode the setpoints only clutter the picture
            show = dict(show); show['sa'] = False; show['st'] = False

        def _interp(xs, ys, x):
            """Linear interpolation of ys(xs) at point x (outside the range - edge value)."""
            if not xs: return 0.0
            if x <= xs[0]: return ys[0]
            if x >= xs[-1]: return ys[-1]
            lo, hi = 0, len(xs) - 1
            while lo < hi - 1:
                mid = (lo + hi) // 2
                if xs[mid] <= x: lo = mid
                else: hi = mid
            dx = xs[hi] - xs[lo]
            if dx == 0: return ys[lo]
            f = (x - xs[lo]) / dx
            return ys[lo] + f * (ys[hi] - ys[lo])

        ax2 = None
        for s in series:
            path = s['path']
            ci = file_order.get(path, 0) % len(self._arch_colors)
            col = self._arch_colors[ci]
            if mode == 'pc' and s['pc'] and s['pc'][0] is not None:
                tx = [((v if v is not None else 0) - self._pc_zero) / tdiv for v in s['pc']]
            else:
                tx = [(x - s['off']) / tdiv for x in s['t']]
            from pathlib import Path as _P
            name = self._cycle_display_name(_P(path))
            base = f"{name} · " if multi else ""
            if show.get('st'):
                self.ax_a.plot(tx, s['spt'], color=C['orange'], lw=1.2, ls='--',
                              label=(base + 'target') if not multi else None, alpha=0.5)
            if show.get('sa') and s['sa'] and any(v is not None for v in s['sa']):
                xs = [tx[i] for i in range(min(len(s['sa']), len(tx))) if s['sa'][i] is not None]
                ys = [v for v in s['sa'][:len(tx)] if v is not None]
                if ys:
                    self.ax_a.plot(xs, ys, color=(C['cyan'] if not multi else col),
                                  lw=1.1, ls=':', alpha=0.75,
                                  label=(base + 'setpoint (ramp)') if not multi else None)
            if show.get('temp'):
                if delta_mode and ref is not None:
                    if s is ref:
                        self.ax_a.axhline(0, color=C['dim2'], lw=1.0, ls='--', alpha=0.7)
                        continue
                    dy = [s['temp'][i] - _interp(ref_x, ref['temp'], tx[i])
                          for i in range(len(tx))]
                    self.ax_a.plot(tx, dy, color=col, lw=1.8,
                                  label=f"{name} − {ref_name}")
                else:
                    self.ax_a.plot(tx, s['temp'], color=col, lw=(2 if not multi else 1.8),
                                  label=(name if multi else 'temperature'))
            if show.get('t2') and s['t2'] and any(v is not None for v in s['t2']):
                xs = [tx[i] for i in range(min(len(s['t2']), len(tx))) if s['t2'][i] is not None]
                ys = [v for v in s['t2'][:len(tx)] if v is not None]
                if ys:
                    self.ax_a.plot(xs, ys, color=C['purple'], lw=1.5, alpha=0.8,
                                  label=(base + 'thermocouple 2'))
            if show.get('pwm'):
                if ax2 is None:
                    ax2 = self.ax_a.twinx(); self._ax_pwm = ax2
                    ax2.set_ylabel('PWM [%]', color=C['dim'], fontsize=9)
                    ax2.tick_params(colors=C['dim'], labelsize=8)
                ax2.plot(tx, s['pwm'], color=col, lw=0.9, ls='-.', alpha=0.5)

        # Reference line in "rel. to temperature" mode
        if mode == 'temp':
            self.ax_a.axhline(tref, color=C['dim2'], lw=0.8, ls='--', alpha=0.6)
            self.ax_a.axvline(0, color=C['dim2'], lw=0.8, ls='--', alpha=0.6)

        # X axis labels as wall-clock time in PC mode
        if mode == 'pc':
            import matplotlib.ticker as _mt
            z = getattr(self, '_pc_zero', 0.0)
            def _fmt(v, pos):
                try: return datetime.fromtimestamp(z + v * tdiv).strftime("%H:%M:%S")
                except Exception: return ""
            self.ax_a.xaxis.set_major_formatter(_mt.FuncFormatter(_fmt))

        # Time axis caption - unit + information about the reference point
        unit_txt = 'min' if use_min else 's'
        if mode == 'pc':
            xlabel = 'PC clock [h:min:s]'
        else:
            xlabel = f'time [{unit_txt}]'
            xlabel += {'t0':   '  ·  0 = start of the run',
                       'abs':  '  ·  file own time',
                       'ramp': '  ·  0 = start of the ramp',
                       'temp': f'  ·  0 = crossing {tref:.1f}°C',
                       }.get(mode, '')
        self.ax_a.set_xlabel(xlabel, color=C['dim'], fontsize=9)
        self.ax_a.set_ylabel(
            (f'temperature difference vs {ref_name} [°C]' if delta_mode else 'temperature [°C]'),
            color=C['dim'], fontsize=9)
        self.ax_a.tick_params(colors=C['dim'], labelsize=8)
        self.ax_a.legend(facecolor=C['panel'], edgecolor=C['border'],
                        labelcolor=C['dim'], fontsize=8, loc='best')
        self.ax_a.grid(True, alpha=0.3, color=C['grid'])
        for sp in self.ax_a.spines.values():
            sp.set_color(C['border'])

        # Title: statistics (single run) or the number being compared
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

        # Run settings panel (only when a single run is selected)
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
        """First selected run (for export)"""
        for p, v in self.arch_vars.items():
            if v.get():
                from pathlib import Path as _P
                return _P(p)
        return None

    def export_arch_csv(self):
        """Download the CSV of the selected measurements.

        One selected -> an ordinary "save as". More than one -> we ask
        for a folder and copy them all there under their original names (without
        this you had to export them one at a time).
        """
        from pathlib import Path as _P
        sel = [_P(p) for p, v in self.arch_vars.items() if v.get()]
        if not sel:
            messagebox.showinfo("No selection", "Select a measurement in the list on the left.")
            return
        from tkinter import filedialog
        import shutil
        try:
            if len(sel) == 1:
                dest = filedialog.asksaveasfilename(
                    title="Download measurement CSV", defaultextension=".csv",
                    initialfile=sel[0].name,
                    filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
                if not dest:
                    return
                shutil.copy(sel[0], dest)
                messagebox.showinfo("Downloaded", f"Saved:\n{dest}")
            else:
                folder = filedialog.askdirectory(title=f"Where to save {len(sel)} CSV files?")
                if not folder:
                    return
                done, failed = 0, []
                for f in sel:
                    try:
                        shutil.copy(f, _P(folder) / f.name); done += 1
                    except Exception as e:
                        failed.append(f"{f.name}: {e}")
                msg = f"Saved {done} of {len(sel)} files in:\n{folder}"
                if failed:
                    msg += "\n\nFailed:\n" + "\n".join(failed[:5])
                messagebox.showinfo("Downloaded", msg)
        except Exception as e:
            messagebox.showerror("Export error", str(e))

    def save_arch_chart(self):
        """Save the current chart (with the comparison) as an image"""
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
                # ALWAYS export on white - see print_theme(). The chart is
                # rebuilt once in the print palette, saved, then rebuilt again
                # in the screen palette, so what you see on screen is
                # untouched and what lands in the file is printable.
                with print_theme(self.fig_a):
                    self._redraw_arch()
                    self.fig_a.savefig(dest, dpi=200, facecolor='white',
                                       edgecolor='none', bbox_inches='tight')
                self._redraw_arch()
                messagebox.showinfo("Saved", f"Chart saved to:\n{dest}")
        except Exception as e:
            messagebox.showerror("Save error", str(e))

    # ════════════════════════════════════════════════════════
    #  FOLDER FOR MEASUREMENT DATA (chosen by the user)
    # ════════════════════════════════════════════════════════
    def _load_data_dir(self):
        """Read the remembered data folder; if missing/unavailable - the default."""
        default = self.cfg_dir
        try:
            if self.settings_file.exists():
                with open(self.settings_file, 'r', encoding='utf-8') as f:
                    d = json.load(f)
                p = d.get('data_dir')
                if p:
                    q = Path(p)
                    # We do not force-create it here - if the user unplugged the
                    # drive or deleted the folder, we quietly fall back to the
                    # default instead of crashing the application at startup.
                    if q.is_dir():
                        return q
                    try:
                        q.mkdir(parents=True, exist_ok=True)
                        return q
                    except Exception:
                        print(f"Data folder '{q}' unavailable - using {default}")
        except Exception as e:
            print(f"ustawienia.json: {e}")
        default.mkdir(exist_ok=True)
        return default

    def _save_data_dir(self):
        """Remember the chosen data folder (merging it with the other settings)."""
        d = {}
        try:
            if self.settings_file.exists():
                with open(self.settings_file, 'r', encoding='utf-8') as f:
                    d = json.load(f)
        except Exception:
            d = {}
        d['data_dir'] = str(self.log_dir)
        try:
            with open(self.settings_file, 'w', encoding='utf-8') as f:
                json.dump(d, f, indent=2)
        except Exception as e:
            messagebox.showwarning("Settings", f"Folder choice not saved:\n{e}")

    def _set_data_dir(self, newdir):
        """Switch the data folder to `newdir` (Path) and refresh the archive."""
        newdir = Path(newdir)
        if newdir == self.log_dir:
            return
        # We do not switch while a run is being saved - the temporary file is
        # already open in the old folder and archiving would go nowhere.
        if self.cyc_on:
            messagebox.showwarning(
                "Measurement in progress",
                "I will not change the folder while a run is being saved.\n"
                "Stop the measurement (STOP) and try again.")
            return
        try:
            newdir.mkdir(parents=True, exist_ok=True)
            probe = newdir / ".peltier_zapis_test"
            probe.write_text("ok", encoding='utf-8')
            probe.unlink()
        except Exception as e:
            messagebox.showerror("Data folder", f"I cannot write to:\n{newdir}\n\n{e}")
            return
        self.log_dir = newdir
        self._save_data_dir()
        self._update_data_dir_label()
        try:
            self.refresh_arch()
            self._redraw_arch()
        except Exception:
            pass
        self._series_status(f"Data is now saved in: {self.log_dir}")

    def choose_data_dir(self):
        """Point to an EXISTING folder for measurement data."""
        from tkinter import filedialog
        p = filedialog.askdirectory(title="Select a folder for measurement data",
                                    initialdir=str(self.log_dir))
        if p:
            self._set_data_dir(p)

    def create_data_dir(self):
        """Create a NEW data folder (we ask for the parent and the name)."""
        from tkinter import filedialog, simpledialog
        parent = filedialog.askdirectory(title="Where to create the new data folder?",
                                         initialdir=str(self.log_dir))
        if not parent:
            return
        name = simpledialog.askstring("New folder", "Folder name:",
                                      initialvalue=datetime.now().strftime("Pomiary_%Y-%m-%d"),
                                      parent=self.root)
        if not name:
            return
        import re as _re
        safe = _re.sub(r'[<>:"/\\|?*]', '_', name).strip().strip('.')
        if not safe:
            messagebox.showwarning("New folder", "Empty name.")
            return
        self._set_data_dir(Path(parent) / safe)

    def _update_data_dir_label(self):
        if hasattr(self, 'data_dir_lbl'):
            p = str(self.log_dir)
            # We shorten the middle of a long path - the end (the folder name)
            # matters most, and the full path is in the tooltip anyway.
            show = p if len(p) <= 52 else p[:20] + " … " + p[-29:]
            self.data_dir_lbl.config(text=show)

    def open_log_folder(self):
        """Open the folder with the logs"""
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
        """Kept for compatibility - redirects to redraw"""
        self._redraw_arch()

    def show_arch_stats(self):
        """Show a window with the statistics of the selected run"""
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
        size_win(win, 440, 520, 380, 380, parent=self.root)
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

        # Noise interpretation
        noise = st['noise_std']
        interp = ("Excellent - low noise" if noise < 0.2 else
                  "Moderate noise" if noise < 0.5 else
                  "High noise - check shielding/grounding")
        tk.Label(inner, text=f"Noise: {interp}", bg=C['bg'],
                 fg=C['dim2'], font=(FONT, fsz(8)), wraplength=380,
                 justify='left').pack(anchor='w', pady=(12, 0))

    def export_arch_pdf(self):
        """Generate a PDF report: chart + statistics + settings + date"""
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
        """Build the PDF report using matplotlib (no extra libraries)"""
        from matplotlib.backends.backend_pdf import PdfPages
        from matplotlib.figure import Figure
        from pathlib import Path as _P
        import datetime

        t, temp, spt, pwm = data
        st = self._compute_stats(data)
        # Time axis starting from zero
        t0 = t[0]
        tx = [x - t0 for x in t]

        with PdfPages(dest) as pdf:
            fig = Figure(figsize=(8.27, 11.69))  # A4 portrait
            fig.patch.set_facecolor('white')

            # Header
            fig.text(0.5, 0.96, "IGNI - Run Report", ha='center',
                     fontsize=16, fontweight='bold')
            ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
            fig.text(0.5, 0.935, f"{self._cycle_display_name(_P(path))}  ·  generated {ts}",
                     ha='center', fontsize=9, color='gray')

            # Temperature chart (upper half)
            ax1 = fig.add_axes([0.1, 0.55, 0.82, 0.32])
            ax1.plot(tx, spt, color='#e8833a', lw=1.2, ls='--', label='target', alpha=0.7)
            ax1.plot(tx, temp, color='#2b7fd4', lw=1.8, label='temperature')
            ax1.set_xlabel('time [s]', fontsize=9)
            ax1.set_ylabel('temperature [°C]', fontsize=9)
            ax1.legend(fontsize=9, loc='best')
            ax1.grid(True, alpha=0.3)
            ax1.set_title('Temperature profile', fontsize=11, loc='left')

            # PWM chart (underneath)
            ax2 = fig.add_axes([0.1, 0.40, 0.82, 0.10])
            ax2.fill_between(tx, pwm, color='#3ea662', alpha=0.5)
            ax2.set_xlabel('time [s]', fontsize=8)
            ax2.set_ylabel('PWM [%]', fontsize=8)
            ax2.grid(True, alpha=0.3)

            # Statistics table (bottom)
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
    #  TICK + CHART
    # ────────────────────────────────────────────────────
    def tick(self):
        try:
            # SELF-TUNE/calibration changed the PID - update the sliders (tables)
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
                # Limit the buffer length
                if len(self.t) > self.maxlen:
                    for a in [self.t, self.temp, self.spt, self.spa,
                              self.pwm, self.kp, self.ki, self.kd, self.states]:
                        del a[0]
                # Run start
                if state == 'AUTO' and prev != 'AUTO' and not self.cyc_on:
                    self._cyc_start(temp)
                    # Start tracking the approach to the setpoint
                    self.reach_start_t = now2
                    self.reach_start_temp = temp
                    self.reach_target = st
                    self.reach_done = False
                    self.reach_in_tol_t = None
                    self.reach_time = None
                    self.reach_avg_rate = None
                    self.last_setpoint_target = st
                    self._ramp_reset(now2, temp, st)
                elif self.cyc_on and state == 'MAN' and prev in ('AUTO', 'COOLDOWN', 'FREEZE', 'FREEZE_READY'):
                    # End of the run - transition from operation to MAN (STOP).
                    # Without cooldown it now goes straight AUTO->MAN.
                    self.cyc_stop("done")

                # Detect a change of the target setpoint while running (new approach)
                if state == 'AUTO' and self.last_setpoint_target is not None:
                    if abs(st - self.last_setpoint_target) > 0.5:
                        # Setpoint changed - start counting from scratch
                        self.reach_start_t = now2
                        self.reach_start_temp = temp
                        self.reach_target = st
                        self.reach_done = False
                        self.reach_in_tol_t = None
                        self.last_setpoint_target = st
                        self._ramp_reset(now2, temp, st)

                # End of the RAMP PHASE = the ramp generator (spA) reached the target.
                # We compute this BEFORE checking the temperature approach, so that
                # both measures stay independent (see the comment at self.ramp_t0).
                if (state == 'AUTO' and not self.ramp_done
                        and self.ramp_t0 is not None and abs(sa - st) <= 0.05):
                    self.ramp_done = True
                    self.ramp_secs = now2 - self.ramp_t0
                    if self.ramp_secs > 1.0 and self.ramp_temp0 is not None:
                        self.ramp_rate = (temp - self.ramp_temp0) / (self.ramp_secs / 60.0)
                    self.ramp_lag = st - temp

                # Check whether the setpoint was reached: |error| <= REACH_TOL_C
                # held continuously for REACH_STABLE_S (see the comment
                # at those constants at the top of the file).
                if (state == 'AUTO' and not self.reach_done
                        and self.reach_target is not None
                        and self.reach_start_t is not None):
                    if abs(temp - self.reach_target) > REACH_TOL_C:
                        self.reach_in_tol_t = None   # we fell out - count from scratch
                    elif self.reach_in_tol_t is None:
                        self.reach_in_tol_t = now2
                    if (self.reach_in_tol_t is not None
                            and now2 - self.reach_in_tol_t >= REACH_STABLE_S):
                        self.reach_done = True
                        self.reach_time = now2 - self.reach_start_t
                        delta = self.reach_target - self.reach_start_temp
                        dT = abs(delta)
                        if self.reach_time > 0:
                            self.reach_avg_rate = dT / (self.reach_time / 60.0)
                        # Direction of the transition: heating or cooling
                        self.reach_dir = "HEAT" if delta > 0 else "COOL"
                        # Remember the approach statistics for this run
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

        if self.series_running:
            try: self._series_tick()
            except Exception as e: print(f"series err: {e}")

        self.root.after(250, self.tick)

    def _ramp_reset(self, t0, temp0, target):
        """Start counting a NEW ramp phase (see the comment at self.ramp_t0)."""
        self.ramp_t0 = t0
        self.ramp_temp0 = temp0
        self.ramp_done = False
        self.ramp_secs = None
        self.ramp_rate = None
        self.ramp_lag = None
        # We take the COMMANDED rate from whichever slider matches the direction
        # of the transition - upwards HEAT RATE, downwards COOL RATE.
        try:
            if target is not None and temp0 is not None and target < temp0:
                self.ramp_cmd_rate = self.sl_rd.get()
            else:
                self.ramp_cmd_rate = self.sl_ru.get()
        except Exception:
            self.ramp_cmd_rate = None
    def update_cards(self):
        if not self.t: return
        temp = self.temp[-1]; spt = self.spt[-1]; pwm = self.pwm[-1]
        self.cards['temp']['val'].config(text=f"{temp:.2f}")
        # Second thermocouple card
        t2 = getattr(self, '_latest_temp2', None)
        if 'temp2' in self.cards:
            self.cards['temp2']['val'].config(text=f"{t2:.2f}" if t2 is not None else "--")
        self.cards['sp']['val'].config(text=f"{spt:.1f}")
        # AVG RATE - the rate of the RAMP ITSELF (not counting the approach tail).
        # Previously it was counted until entering +/-0.5C of the target, so the
        # approach tail dragged the result down by as much as 2.5x (see the
        # comment at self.ramp_t0) and the card showed 12.2 for a commanded 30.
        # Now: during the ramp the rate is counted from its start, and once it
        # finishes it is FROZEN at the value reached in the ramp - that way you
        # can see directly whether the ramp keeps up with the commanded RATE.
        avg_rate = 0.0
        if self.ramp_done and self.ramp_rate is not None:
            avg_rate = self.ramp_rate
        elif (self.ramp_t0 is not None and self.ramp_temp0 is not None
                and self.t and self.cur_state == 'AUTO'):
            elapsed = self.t[-1] - self.ramp_t0
            if elapsed > 2:  # min 2s to avoid dividing by small numbers
                avg_rate = (temp - self.ramp_temp0) / (elapsed / 60.0)
        self.cards['rate']['val'].config(text=f"{avg_rate:+.1f}")
        # Card color = how close to the COMMANDED rate (green >=95%, yellow >=85%)
        try:
            cmd = self.ramp_cmd_rate
            if cmd and abs(cmd) > 0.1 and abs(avg_rate) > 0.1:
                frac = abs(avg_rate) / abs(cmd)
                rcol = (C['green'] if frac >= 0.95 else
                        (C['yellow'] if frac >= 0.85 else C['red']))
                self.cards['rate']['unit_lbl'].config(
                    text=f"°C/min  {frac*100:.0f}% of cmd", fg=rcol)
            else:
                self.cards['rate']['unit_lbl'].config(text="°C/min", fg=C['dim2'])
        except Exception:
            pass
        # PWM + direction (HEAT/COOL/HOLD shown in the unit)
        diff = spt - temp
        arrow = "% ▲HEAT" if diff > 0.3 else ("% ▼COOL" if diff < -0.3 else "% ●HOLD")
        self.cards['pwm']['val'].config(text=f"{pwm:.0f}")
        # Direction color
        acol = C['red'] if diff > 0.3 else (C['cyan'] if diff < -0.3 else C['dim2'])
        self.cards['pwm']['unit_lbl'].config(text=" " + arrow, fg=acol)

        # Approach statistics / FREEZE status
        if hasattr(self, 'reach_lbl'):
            # FREEZE - priority (the most important message for the user)
            if self.cur_state == 'FREEZE_READY':
                self.reach_lbl.config(text="❄ GAL SOLID — ready to swap sample", fg=C['cyan'])
            elif self.cur_state == 'FREEZE':
                self.reach_lbl.config(text=f"❄ Freezing gal → hold 20°C", fg=C['cyan'])
            elif self.reach_done and self.reach_time is not None:
                m = int(self.reach_time // 60); s = int(self.reach_time % 60)
                tstr = f"{m}m {s}s" if m > 0 else f"{s}s"
                d = getattr(self, 'reach_dir', '')
                dcol = C['red'] if d == 'HEAT' else C['cyan']
                # SPLIT into two separate numbers instead of one misleading
                # average (see the comment at self.ramp_t0): how long the RAMP
                # itself ran and at what rate vs commanded, and separately how
                # long the APPROACH (tail) took after the ramp finished. These
                # are two different problems and they are fixed differently.
                if self.ramp_rate is not None and self.ramp_secs is not None:
                    cmd = self.ramp_cmd_rate
                    pct = (f" ({abs(self.ramp_rate)/abs(cmd)*100:.0f}% of {abs(cmd):.0f})"
                           if cmd and abs(cmd) > 0.1 else "")
                    tail = max(0.0, self.reach_time - self.ramp_secs)
                    lag = f" · remaining {self.ramp_lag:+.2f}°C" if self.ramp_lag is not None else ""
                    self.reach_lbl.config(
                        text=(f"✓ {d} · ramp {abs(self.ramp_rate):.1f}°C/min{pct}"
                              f" in {self.ramp_secs:.0f}s{lag} · approach +{tail:.0f}s"),
                        fg=dcol)
                else:
                    rate_str = f"{self.reach_avg_rate:.2f}" if self.reach_avg_rate else "?"
                    self.reach_lbl.config(
                        text=f"✓ {d} REACHED in {tstr} · avg {rate_str}°C/min", fg=dcol)
            elif (self.cur_state == 'AUTO' and self.reach_start_t is not None
                  and not self.reach_done):
                # While approaching - show the elapsed time
                if self.t:
                    elapsed = self.t[-1] - self.reach_start_t
                    m = int(elapsed // 60); s = int(elapsed % 60)
                    tstr = f"{m}m {s}s" if m > 0 else f"{s}s"
                    self.reach_lbl.config(
                        text=f"→ reaching {self.reach_target:.1f}°C · {tstr}", fg=C['yellow'])
            else:
                self.reach_lbl.config(text="")
        if not self.t: return
        # Paused - do not refresh (lets you zoom in on / inspect the frozen chart)
        if self.chart_paused:
            return
        t = self.t; temp = self.temp; spt = self.spt; spa = self.spa; pwm = self.pwm

        # Time window - show only the last N seconds if set
        if self.chart_window > 0 and len(t) > 1:
            t_now = t[-1]
            cutoff = t_now - self.chart_window
            # Find the index to start showing from
            i0 = 0
            for i in range(len(t) - 1, -1, -1):
                if t[i] < cutoff:
                    i0 = i
                    break
            t = t[i0:]; temp = temp[i0:]; spt = spt[i0:]
            spa = spa[i0:]; pwm = pwm[i0:]

        self._live_args = (t, temp, spt, spa, pwm)
        self._redraw_live(t, temp, spt, spa, pwm)

    def _redraw_live(self, t, temp, spt, spa, pwm):
        """The live chart drawing, split out of update_cards() in .15.

        It used to be inline, which meant the only way to re-render the chart
        was to run the whole card-update routine - including its Tk widget
        writes. Exporting on a white background needs exactly this part and
        nothing else (see print_theme), so it now stands on its own."""
        self.ax1.clear()
        self.ax1.set_facecolor(C['panel2'])
        # target final (dashed orange)
        self.ax1.plot(t, spt, color=C['orange'], lw=1.3, ls='--', label='target', alpha=0.7)
        # actual setpoint - ramp (dotted cyan) - shows how the setpoint creeps
        self.ax1.plot(t, spa, color=C['cyan'], lw=1.5, ls=':', label='setpoint (ramp)')
        # actual temperature (thick blue)
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
    #  RUN CSV
    # ────────────────────────────────────────────────────
    # ────────────────────────────────────────────────────
    #  SLEEP INHIBIT FOR THE DURATION OF A MEASUREMENT
    # ────────────────────────────────────────────────────
    # IMPORTANT DISTINCTION: SCREEN BLANKING breaks nothing - the process keeps
    # running, the serial port keeps reading, the CSV keeps being appended (every
    # row is flushed immediately, see cyc_log). What does break things is
    # SUSPENDING THE WHOLE SYSTEM (S3/hibernation): USB is then re-enumerated
    # from scratch, the COM port can disappear, and the SERIES state machine
    # (which runs HERE, in the app, not in the firmware) stops switching legs -
    # the board stays at the last commanded setpoint in AUTO and holds it,
    # but the series is stalled and there is a hole in the log.
    #
    # That is why, for the duration of a measurement, we ask the system NOT TO
    # SLEEP. This is an ordinary per-process API - it does NOT change any system
    # settings, does not require administrator rights and stops working the
    # moment the lock is released or the program is closed. The screen may blank
    # normally - we deliberately do NOT keep it lit.
    def _wake_lock(self, on):
        try:
            if sys.platform.startswith('win'):
                import ctypes
                ES_CONTINUOUS      = 0x80000000
                ES_SYSTEM_REQUIRED = 0x00000001
                # ES_CONTINUOUS alone = release the lock (back to normal).
                flags = (ES_CONTINUOUS | ES_SYSTEM_REQUIRED) if on else ES_CONTINUOUS
                ok = ctypes.windll.kernel32.SetThreadExecutionState(flags)
                if on and not ok:
                    print("WARNING: failed to block system sleep")
                    return
            elif sys.platform == 'darwin':
                import subprocess
                if on:
                    if getattr(self, '_wl_proc', None) is None:
                        self._wl_proc = subprocess.Popen(
                            ['caffeinate', '-s', '-w', str(os.getpid())])
                else:
                    p = getattr(self, '_wl_proc', None)
                    if p is not None:
                        try: p.terminate()
                        except Exception: pass
                        self._wl_proc = None
            else:
                import subprocess
                if on:
                    if getattr(self, '_wl_proc', None) is None:
                        self._wl_proc = subprocess.Popen(
                            ['systemd-inhibit', '--what=sleep:idle',
                             '--who=PeltierControl', '--why=measurement in progress',
                             'sleep', 'infinity'])
                else:
                    p = getattr(self, '_wl_proc', None)
                    if p is not None:
                        try: p.terminate()
                        except Exception: pass
                        self._wl_proc = None
            print("WAKE LOCK: %s" % ("enabled (the system will not sleep during the measurement)"
                                     if on else "released"))
        except Exception as e:
            # No caffeinate/systemd-inhibit, or an exotic system - the
            # measurement should happen anyway, so we only inform.
            print("WAKE LOCK unavailable (%s) - make sure the laptop does not fall asleep" % e)

    def _cyc_start(self, temp0):
        self.cyc_on = True
        self._wake_lock(True)
        self.cyc_t0 = time.time()
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        # Temporary file - the user names it after STOP
        self.cyc_ts = ts
        self.cyc_fn = self.log_dir / f"_tmp_cykl_{ts}.csv"
        self.cyc_file = open(self.cyc_fn, 'w', newline='', encoding='utf-8')
        self.cyc_wr = csv.writer(self.cyc_file)
        # czas_pc = the COMPUTER clock (YYYY-MM-DD HH:MM:SS.mmm). czas_s is
        # counted from the start of the run, so on its own it does not let you
        # line the trace up with anything but itself - the PC time makes it
        # possible to put several runs on one real time axis and correlate them
        # with events outside the application. The column is appended AT THE END
        # so that old files and existing parsers (reading by index) keep working.
        self.cyc_wr.writerow(CSV_COLS)
        self.cyc_rows = 0
        print(f"CYC START T={temp0:.1f}")

    def cyc_log(self, t, temp, sa, st, pwm, kp, ki, kd, state, temp2=None, dbg=None):
        if self.cyc_wr:
            try:
                t2str = f"{temp2:.2f}" if temp2 is not None else ""
                if dbg:
                    dbgvals = [f"{dbg['ff']:.2f}", f"{dbg['p']:.2f}", f"{dbg['i']:.2f}",
                               f"{dbg['dd']:.2f}", f"{dbg['raw']:.2f}", f"{dbg['react']:.2f}",
                               ("" if dbg.get('amb') is None else f"{dbg['amb']:.2f}")]
                else:
                    dbgvals = ["", "", "", "", "", "", ""]
                _n = datetime.now()   # ONE call - two would give inconsistent ms
                pcnow = _n.strftime("%Y-%m-%d %H:%M:%S.") + f"{_n.microsecond//1000:03d}"
                self.cyc_wr.writerow([f"{t:.2f}", f"{temp:.2f}", f"{sa:.2f}",
                                     f"{st:.2f}", pwm, f"{pwm*100/255:.1f}",
                                     f"{kp:.3f}", f"{ki:.4f}", f"{kd:.3f}", state, t2str,
                                     *dbgvals, pcnow])
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
        # We release the sleep lock ONLY when we are not in the middle of a
        # series - between series legs cyc_stop/_cyc_start fire one right after
        # the other (see _series_roll_cycle) and it would be a shame to let the
        # system sleep in that gap.
        if not getattr(self, 'series_running', False):
            self._wake_lock(False)
        print(f"CYC STOP: {reason} ({getattr(self,'cyc_rows',0)} samples)")
        if getattr(self, 'series_skip_archive', False):
            # The "return to base" leg in a SERIES - not a test, just an approach
            # to the starting position, so there is nothing to archive (see the
            # comment in _series_launch_cool). We delete the temp file right away.
            self.series_skip_archive = False
            if tmp_path and tmp_path.exists():
                try: tmp_path.unlink()
                except: pass
            return
        if had_data and tmp_path and tmp_path.exists():
            hint = self.series_name_hint
            self.series_name_hint = None
            if hint:
                # SERIES: save it right away under a readable name, WITHOUT a
                # modal window (which would block the following automatic steps -
                # nobody is standing at the computer to close it).
                self.root.after(0, lambda: self.save_cycle_as(tmp_path, hint))
            else:
                # Ask for a name and save to the archive (in the GUI thread)
                self.root.after(0, lambda: self._ask_save_name(tmp_path))
        elif tmp_path and tmp_path.exists():
            # No data - delete the temporary file
            try: tmp_path.unlink()
            except: pass

    def _ask_save_name(self, tmp_path):
        """Window asking for the run name for the archive"""
        SaveCycleDialog(self.root, self, tmp_path)

    # ════════════════════════════════════════════════════════
    #  MEASUREMENT SERIES - an automatic sequence of SP/RATE tests with no
    #  hand on the keyboard between consecutive tests. Each test is:
    #    1) heating to SP with the given ramp (HEAT RATE) - until "reached"
    #       (uses the same reach_done logic as the cards on CONTROL)
    #    2) holding at SP for hold_s seconds (oscillation/lag is visible there)
    #    3) STOP -> archiving under a readable name (without asking)
    #    4) return to the base temperature (COOL RATE from the panel) before
    #       the next test, so that every start is from the same point
    # ════════════════════════════════════════════════════════
    def series_add_step(self, sp, rate, hold_s):
        self.series_steps.append({'sp': float(sp), 'rate': float(rate), 'hold_s': float(hold_s)})
        self._series_refresh_list()

    def series_remove_step(self, idx):
        if 0 <= idx < len(self.series_steps):
            del self.series_steps[idx]
            self._series_refresh_list()

    def _series_refresh_list(self):
        if hasattr(self, 'series_listbox'):
            self.series_listbox.delete(0, 'end')
            for i, s in enumerate(self.series_steps):
                self.series_listbox.insert('end',
                    f"{i+1}. SP={s['sp']:.1f}°C  RATE={s['rate']:.1f}°C/min  hold={s['hold_s']:.0f}s")

    def series_start(self):
        if not self.connected:
            messagebox.showwarning("Not connected", "Connect to the device first.")
            return
        if not self.series_steps:
            messagebox.showwarning("Empty series", "Add at least one test to the list.")
            return
        if self.series_running:
            return
        self.series_running = True
        self.series_idx = 0
        # Remember the COOL RATE from the CONTROL panel - the return between
        # tests uses ITS OWN fast rate (see _series_launch_cool), regardless of
        # what the user has set for the real tests.
        # We restore the original after the series finishes/is aborted.
        self._series_saved_rd = self.sl_rd.get()
        self._series_status(f"Series start: {len(self.series_steps)} tests")
        self._series_launch_heat(self.series_idx)
        if hasattr(self, 'btn_series_run'):
            self.btn_series_run.config(text="■ STOP SERIES", bg=C['red'])

    def _series_restore_rd(self):
        saved = getattr(self, '_series_saved_rd', None)
        if saved is not None:
            self.sl_rd.set(saved)
            if self.connected:
                self.send(f"RD:{saved:.1f}")
            self._series_saved_rd = None

    def _series_abort(self, reason=""):
        self.series_running = False
        self.series_leg = None
        self.series_phase = None
        self.series_name_hint = None
        self.series_skip_archive = False
        if not self.cyc_on:
            self._wake_lock(False)
        self._series_restore_rd()
        self._series_status(f"Series aborted ({reason})" if reason else "Series aborted")
        if hasattr(self, 'btn_series_run'):
            self.btn_series_run.config(text="▶ START SERIES", bg=C['green'])

    def _series_finish(self):
        self.series_running = False
        self.series_leg = None
        self.series_phase = None
        # Series finished - if no single run is in progress any more, we can
        # give the system back the right to sleep (see _wake_lock).
        if not self.cyc_on:
            self._wake_lock(False)
        self._series_restore_rd()
        self._series_status(f"Series finished - {len(self.series_steps)} tests, files in PeltierLogi")
        if hasattr(self, 'btn_series_run'):
            self.btn_series_run.config(text="▶ START SERIES", bg=C['green'])

    def _series_status(self, text):
        print(f"SERIES: {text}")
        if hasattr(self, 'series_status_lbl'):
            self.series_status_lbl.config(text=text)

    def _series_save_prog(self):
        """Save the step list to a JSON file (run program)."""
        if not self.series_steps:
            messagebox.showinfo("Empty program", "Add steps first.")
            return
        from tkinter import filedialog
        dest = filedialog.asksaveasfilename(
            title="Save program", defaultextension=".json",
            initialdir=str(self.log_dir),
            initialfile=datetime.now().strftime("program_%Y-%m-%d.json"),
            filetypes=[("Program (JSON)", "*.json"), ("All files", "*.*")])
        if not dest:
            return
        try:
            with open(dest, 'w', encoding='utf-8') as f:
                json.dump({'tryb': self.series_mode.get(),
                           'baza': self.series_base_sp,
                           'kroki': self.series_steps}, f, indent=2)
            self._series_status(f"Program saved: {dest}")
        except Exception as e:
            messagebox.showerror("Save program", str(e))

    def _series_load_prog(self):
        """Load the step list from a JSON file."""
        from tkinter import filedialog
        src = filedialog.askopenfilename(
            title="Load program", initialdir=str(self.log_dir),
            filetypes=[("Program (JSON)", "*.json"), ("All files", "*.*")])
        if not src:
            return
        try:
            with open(src, 'r', encoding='utf-8') as f:
                d = json.load(f)
            steps = d.get('kroki') or []
            clean = []
            for st in steps:
                clean.append(dict(sp=float(st['sp']), rate=float(st['rate']),
                                  hold_s=float(st.get('hold_s', 60))))
            if not clean:
                messagebox.showwarning("Program", "The file contains no steps.")
                return
            self.series_steps = clean
            if d.get('tryb') in ('seria', 'program'):
                self.series_mode.set(d['tryb'])
            if d.get('baza') is not None:
                self.series_base_sp = float(d['baza'])
                if hasattr(self, 'series_e_base'):
                    self.series_e_base.delete(0, 'end')
                    self.series_e_base.insert(0, f"{self.series_base_sp:.1f}")
            self._series_refresh_list()
            self._series_status(f"Program loaded: {len(clean)} steps")
        except Exception as e:
            messagebox.showerror("Load program", str(e))

    def _series_roll_cycle(self, hint):
        """Close the CURRENT run file under the name `hint` and immediately open
        a new one - WITHOUT stopping the controller.

        Previously every series leg was closed with STOP, because only an
        AUTO->MAN transition closed the run file. That forced a break in control -
        see the comment at _series_switch_leg.
        """
        self.series_name_hint = hint
        self.cyc_stop("end of series leg")
        temp0 = self.temp[-1] if self.temp else 0.0
        self._cyc_start(temp0)

    def _series_switch_leg(self, sp, ru=None, rd=None):
        """Switch the series to a new target WITHOUT STOP/START - the controller stays in AUTO.

        WHY (report: "the cooling ramp starts later than the end of the heating
        ramp, and the SP line is much lower"):
        Previously, between legs it went STOP -> 600 ms -> START. On STOP the
        firmware goes to MAN and ZEROES the power, and on START it does spA=lT,
        i.e. it sets the active setpoint to the CURRENT reading. In that time the
        reading had already dropped, because the heater/Peltier surface has low
        inertia and after cutting ~60 PWM units it cools down instantly.
        Measured on real logs (5 transitions, end of heating -> start of the
        descent): a gap of 1.4-1.7 s, during which the temperature fell by
        5.6-7.0 C (50.1-50.6 -> 43.1-44.6). That is why the descent ramp started
        from ~44-47 C instead of from 50 C - exactly the "SP line much lower" and
        the visible hole between the end of heating and the start of cooling.

        The firmware handles SP/RU/RD during AUTO (they only change the target and
        the rates), and START does anything ONLY when sys==MAN. So it is enough not
        to leave AUTO: spA transitions smoothly from 50 downwards, without zeroing
        the power, without a break and without a setpoint jump.
        """
        if ru is not None:
            self.send(f"RU:{ru:.1f}")
        if rd is not None:
            self.send(f"RD:{rd:.1f}")
        self.send(f"SP:{sp:.1f}")
        # The approach statistics are counted from scratch for every leg. NOTE:
        # do_start ZEROES reach_start_t, because a MAN->AUTO transition follows
        # right there, which sets it. Here there will be NO such transition (we
        # stay in AUTO), so it has to be set MANUALLY - otherwise the approach
        # detection condition in tick() (which requires reach_start_t is not None)
        # would never fire and the leg would hang until the timeout.
        now = self.t[-1] if self.t else time.time()
        cur = self.temp[-1] if self.temp else None
        self.reach_start_t = now
        self.reach_start_temp = cur
        self.reach_target = sp
        self.reach_done = False
        self.reach_in_tol_t = None
        self.reach_time = None
        self.reach_avg_rate = None
        self.reach_dir = None
        # None DELIBERATELY: the automatic setpoint-change detection in tick()
        # compares the telemetry 'st' against this value, and for a moment after
        # sending SP the telemetry still carries the OLD setpoint - writing the
        # new target here would trigger a false "setpoint changed" and reset the
        # counters we have just set. The series steers the setpoint itself anyway.
        self.last_setpoint_target = None
        self._last_reach_summary = None
        self._ramp_reset(now, cur, sp)
        self._update_run_button(True)

    def _series_launch_heat(self, idx):
        if not self.connected:
            # The automatic series runs UNATTENDED - we do not leave a modal
            # "not connected" window hanging in mid-air, we simply abort with a
            # readable status.
            self._series_abort("lost connection to the device")
            return
        step = self.series_steps[idx]
        self.sl_sp.set(step['sp'])
        self.sl_ru.set(step['rate'])
        self.do_start()
        self.series_leg = 'heat'
        self.series_phase = 'ramping'
        self.series_phase_t0 = time.time()
        self._series_status(
            f"Test {idx+1}/{len(self.series_steps)}: SP={step['sp']:.1f}°C "
            f"RATE={step['rate']:.1f}°C/min - heating...")

    def _series_launch_cool(self):
        if not self.connected:
            self._series_abort("lost connection to the device")
            return
        try:
            return_rate = float(self.series_e_return_rate.get().replace(',', '.'))
        except (ValueError, AttributeError):
            return_rate = 80.0  # safe fallback = the default "max" (see build_series)
        # The "descent also as a TEST" mode - we descend at the rate of the same
        # series step as the heating, in order to collect data for cooling
        # calibration (see the comment at self.series_cool_as_test in build_series).
        cool_is_test = False
        try:
            cool_is_test = bool(self.series_cool_as_test.get())
        except AttributeError:
            pass
        if cool_is_test and self.series_idx < len(self.series_steps):
            return_rate = self.series_steps[self.series_idx]['rate']
        # Its own FAST return rate, INDEPENDENT of the COOL RATE on CONTROL -
        # the return between tests is only an "approach to the starting position",
        # not the test itself, so there is no reason to do it at the experiment rate.
        self.sl_rd.set(return_rate)
        self.sl_sp.set(self.series_base_sp)
        # SMOOTH heating -> cooling transition: WITHOUT STOP/START (see
        # _series_switch_leg). The descent starts exactly where the heating
        # finished.
        self._series_switch_leg(self.series_base_sp, rd=return_rate)
        self.series_leg = 'cool'
        self.series_phase = 'ramping'
        self.series_phase_t0 = time.time()
        # PREVIOUSLY: the return was treated as "not data for analysis" and
        # skipped (series_skip_archive=True), so as not to clutter PeltierLogi.
        # After the user's remark about the poor look of COOLING ("I would prefer
        # it to descend evenly with the active setpoint") - and this leg is
        # precisely our only downward ramp - we NOW archive it just like the
        # heating, so that we have real data to analyse instead of guessing.
        self.series_skip_archive = False
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        # A TEST vs RETURN prefix in the file name, so that at a single glance
        # (and with a single filter during analysis) one can tell a descent done
        # as a full-blown test from an ordinary approach to the base.
        kind = "cooltest" if cool_is_test else "cool"
        self.series_name_hint = (
            f"seria_{kind}_toSP{self.series_base_sp:.0f}_R{return_rate:.0f}_{ts}")
        lbl = "DESCENT-TEST" if cool_is_test else "Return"
        self._series_status(f"{lbl} to {self.series_base_sp:.1f}°C (rate {return_rate:.0f}°C/min)...")

    def _series_tick(self):
        """Called from the main tick() (about every 250ms) when a series is active."""
        if not self.series_running or self.series_idx >= len(self.series_steps):
            return
        now = time.time()
        step = self.series_steps[self.series_idx]

        if self.series_leg == 'heat' and self.series_phase == 'ramping':
            if self.reach_done:
                self.series_phase = 'holding'
                self.series_phase_t0 = now
                self._series_status(
                    f"Test {self.series_idx+1}/{len(self.series_steps)}: "
                    f"reached SP={step['sp']:.1f} - hold {step['hold_s']:.0f}s")
            elif now - self.series_phase_t0 > SERIES_HEAT_TIMEOUT_S:
                self._series_status(f"Test {self.series_idx+1}: approach TIMEOUT - ending this test")
                self._series_end_heat_leg(tag="TIMEOUT")

        elif self.series_leg == 'heat' and self.series_phase == 'holding':
            elapsed = now - self.series_phase_t0
            remaining = max(0, step['hold_s'] - elapsed)
            self._series_status(
                f"Test {self.series_idx+1}/{len(self.series_steps)}: "
                f"holding SP={step['sp']:.1f} - {remaining:.0f}s left")
            if elapsed >= step['hold_s']:
                self._series_end_heat_leg(tag="OK")

        elif self.series_leg == 'cool' and self.series_phase == 'ramping':
            # Up to .13 the descent ended EXACTLY here - at the moment of
            # reaching, without a single hold sample (see the comment at
            # REACH_TOL_C). Now it gets a hold phase just like the heating, so
            # that the tail is visible in the log: whether the temperature
            # actually settles on the target and whether it crosses it.
            if self.reach_done:
                self.series_phase = 'holding'
                self.series_phase_t0 = now
                self._series_status(
                    f"Descent {self.series_idx+1}/{len(self.series_steps)}: "
                    f"reached {self.series_base_sp:.1f} - hold "
                    f"{step['hold_s']:.0f}s")
            elif now - self.series_phase_t0 > SERIES_COOL_TIMEOUT_S:
                self._series_status(f"Descent {self.series_idx+1}: approach TIMEOUT - ending")
                self._series_end_cool_leg()

        elif self.series_leg == 'cool' and self.series_phase == 'holding':
            elapsed = now - self.series_phase_t0
            remaining = max(0, step['hold_s'] - elapsed)
            self._series_status(
                f"Descent {self.series_idx+1}/{len(self.series_steps)}: "
                f"holding {self.series_base_sp:.1f} - {remaining:.0f}s left")
            if elapsed >= step['hold_s']:
                self._series_end_cool_leg()

    def _series_end_heat_leg(self, tag="OK"):
        # THE BUG that caused this (found after the report "10/40/70 instead of
        # the whole list"): tick() runs every 250ms, but the next step was
        # scheduled with a 600ms delay (root.after) - and the condition that
        # leads here (reach_done / hold_s elapsed) stayed TRUE for that whole
        # 600ms. Effect: tick() called this function 2-3 times before the phase
        # actually changed, and each call scheduled its OWN _series_advance -
        # series_idx jumped by 2-3 instead of by 1 (which is why out of the list
        # 10/20/30/40/50/60/70 only 10, 40, 70 actually ran - a jump of 3 every
        # time). Fix: we set the 'ending' sentinel IMMEDIATELY (synchronously),
        # so that the condition in tick() stops matching at once rather than
        # only after 600ms.
        self.series_phase = 'ending'
        step = self.series_steps[self.series_idx]
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        # In PROGRAM mode the name carries the step number - otherwise consecutive
        # steps with the same SP would be indistinguishable in the archive.
        try: _prog = (self.series_mode.get() == 'program')
        except AttributeError: _prog = False
        if _prog:
            hint = (f"prog{self.series_idx+1:02d}_SP{step['sp']:.0f}"
                    f"_R{step['rate']:.0f}_{tag}_{ts}")
        else:
            hint = f"seria_SP{step['sp']:.0f}_R{step['rate']:.0f}_{tag}_{ts}"
        prog = False
        try: prog = (self.series_mode.get() == 'program')
        except AttributeError: pass
        nxt = self.series_idx + 1
        if prog:
            # PROGRAM: no return to base - the next step starts exactly where
            # the previous one finished (smoothly, without STOP/START - see
            # _series_switch_leg).
            if nxt < len(self.series_steps) and self.connected:
                self._series_roll_cycle(hint)
                self.series_idx = nxt
                nstep = self.series_steps[nxt]
                self.sl_sp.set(nstep['sp'])
                cur = self.temp[-1] if self.temp else nstep['sp']
                down = nstep['sp'] < cur - 0.5
                if down: self.sl_rd.set(nstep['rate'])
                else:    self.sl_ru.set(nstep['rate'])
                self._series_switch_leg(nstep['sp'],
                                        rd=(nstep['rate'] if down else None),
                                        ru=(None if down else nstep['rate']))
                self.series_leg = 'heat'      # 'heat' = the leg commanded by the step
                self.series_phase = 'ramping'
                self.series_phase_t0 = time.time()
                self._series_status(
                    f"Step {nxt+1}/{len(self.series_steps)}: "
                    f"{'descent' if down else 'approach'} to {nstep['sp']:.1f}°C "
                    f"@ {nstep['rate']:.0f}°C/min")
            else:
                self.series_name_hint = hint
                self.send("STOP")
                self._update_run_button(False)
                self.root.after(600, self._series_advance)
        elif abs(self.series_base_sp - step['sp']) > 0.5:
            # SMOOTHLY: we close the heating file and immediately open the
            # descent file, WITHOUT leaving AUTO - thanks to that the start of
            # the descent = the end of the heating (see _series_switch_leg).
            self._series_roll_cycle(hint)
            self.series_leg = 'cool'
            self._series_launch_cool()
        else:
            # Target = base, so there is no descent leg - here we really finish.
            self.series_name_hint = hint
            self.send("STOP")
            self._update_run_button(False)
            self.root.after(600, self._series_advance)

    def _series_end_cool_leg(self):
        # The same sentinel guard as in _series_end_heat_leg - see the comment
        # there. This was the ACTUAL source of the bug (reach_done stayed True
        # for the whole 600ms without it), because every test in this series had
        # base_sp!=SP, so it ALWAYS went through the 'cool' leg.
        self.series_phase = 'ending'
        # series_name_hint is already set in _series_launch_cool - the descent
        # file is archived under the name "seria_cool_/cooltest_..." (cooling is
        # data too - see the comment at _series_launch_cool).
        nxt = self.series_idx + 1
        if nxt < len(self.series_steps) and self.connected:
            # There is another test - we transition SMOOTHLY, without STOP/START,
            # so the next heating ramp starts exactly where the descent finished
            # (see _series_switch_leg).
            self._series_roll_cycle(self.series_name_hint)
            self.series_idx = nxt
            step = self.series_steps[nxt]
            self.sl_sp.set(step['sp'])
            self.sl_ru.set(step['rate'])
            self._series_switch_leg(step['sp'], ru=step['rate'])
            self.series_leg = 'heat'
            self.series_phase = 'ramping'
            self.series_phase_t0 = time.time()
            self._series_status(
                f"Test {nxt+1}/{len(self.series_steps)}: SP={step['sp']:.1f}°C "
                f"RATE={step['rate']:.1f}°C/min - heating...")
        else:
            # Last step (or a lost connection) - here we really do stop.
            self.send("STOP")
            self._update_run_button(False)
            self.root.after(600, self._series_advance)

    def _series_advance(self):
        if not self.series_running:
            return
        self.series_idx += 1
        self.series_leg = None
        self.series_phase = None
        if self.series_idx >= len(self.series_steps):
            self._series_finish()
        else:
            self._series_launch_heat(self.series_idx)

    def save_cycle_as(self, tmp_path, name):
        """Save the run under a name = the user's description (timestamp only on a duplicate)"""
        import re as _re
        # Keep a readable description: allow spaces, hyphens, underscores
        clean = name.strip()
        safe = _re.sub(r'[^\w\-\s]', '', clean).strip()
        safe = _re.sub(r'\s+', '_', safe) or "cykl"
        # File: prefix c_ (for searching the archive) + description
        dest = self.log_dir / f"c_{safe}.csv"
        # If it exists - add a timestamp so as not to overwrite
        if dest.exists():
            ts = datetime.now().strftime("%m%d_%H%M")
            dest = self.log_dir / f"c_{safe}_{ts}.csv"
        try:
            tmp_path.rename(dest)
            print(f"Run saved: {dest.name}")
        except Exception as e:
            print(f"Save error: {e}")
        if hasattr(self, 'refresh_arch'):
            try: self.refresh_arch()
            except: pass

    def discard_cycle(self, tmp_path):
        """Discard the run - delete the temporary file"""
        try:
            if tmp_path.exists(): tmp_path.unlink()
            print("Run discarded")
        except: pass


# ════════════════════════════════════════════════════════
#  AUTO-CALIBRATION RANGE SELECTION DIALOG
# ════════════════════════════════════════════════════════
class CalRangeDialog:
    def __init__(self, parent, app):
        self.app = app
        self.win = tk.Toplevel(parent)
        self.win.title("Auto-Calibration Range")
        self.win.configure(bg=C['bg'])
        size_win(self.win, 560, 680, 540, 520, parent=parent)
        self.win.transient(parent)
        self.win.grab_set()

        tk.Frame(self.win, bg=C['purple'], height=4).pack(fill='x')
        # The button bar is PINNED AT THE BOTTOM OF THE WINDOW, outside the
        # scrolling area - previously START/CANCEL were at the end of a long list
        # in 'inner' and with a smaller window (or larger fonts) they went off
        # screen, so "START CALIBRATION" could not be clicked.
        self._btnbar = tk.Frame(self.win, bg=C['bg'])
        self._btnbar.pack(side='bottom', fill='x', padx=24, pady=(0, 16))
        # The rest of the content scrolls - this guarantees access to every field
        # regardless of DPI and window size.
        inner = make_scrollable(self.win, C['bg'], padx=24, pady=20)

        tk.Label(inner, text="AUTO-CALIBRATION RANGE", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(14), 'bold')).pack(anchor='w')
        tk.Label(inner, text="Select temperature range and ramps to calibrate",
                 bg=C['bg'], fg=C['dim'], font=(FONT, fsz(9))).pack(anchor='w', pady=(2, 16))

        # Temperature range - sliders
        tmin0 = getattr(app, 'dev_cal_min', 50.0)
        tmax0 = getattr(app, 'dev_cal_max', 100.0)

        self.sl_tmin = SliderField(inner, "TEMP FROM", -10, 100, tmin0,
                                   C['cyan'], "°C", 0)
        self.sl_tmax = SliderField(inner, "TEMP TO", 0, 115, tmax0,
                                   C['orange'], "°C", 0)

        tk.Frame(inner, bg=C['border'], height=1).pack(fill='x', pady=(4, 8))

        # Temperature step (info - the firmware uses every 10C)
        tk.Label(inner, text="TEMP STEP: 10°C (fixed)", bg=C['bg'], fg=C['dim2'],
                 font=(FONT, fsz(9))).pack(anchor='w', pady=(0, 12))

        # MAX RATE - slider (up to 80)
        self.sl_maxrate = SliderField(inner, "MAX RATE", 5, 80, 40,
                                      C['yellow'], "°C/min", 0,
                                      on_change=lambda v: self._update_estimate())

        # RATE STEP - choice of 5/10/20/40
        tk.Label(inner, text="RATE STEP [°C/min]:", bg=C['bg'], fg=C['dim'],
                 font=(FONT, fsz(10), 'bold')).pack(anchor='w', pady=(8, 6))

        self.rate_step = 5  # default step
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

        # RECOMMENDED - the ramp list cannot be expressed with a uniform step
        # (5, then every 10 up to 80) - this is exactly the default list from the
        # firmware (calRamps[]). A separate button instead of trying to squeeze
        # it into the RATE STEP above.
        self.custom_ramps = None
        self.btn_recommended = tk.Button(
            inner, text="RECOMMENDED (5/10/20/30/40/50/60/70/80 °C/min)",
            command=self._use_recommended_ramps,
            bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(10), 'bold'),
            relief='flat', cursor='hand2', bd=0, padx=10, pady=8,
            activebackground=C['panel3'])
        self.btn_recommended.pack(fill='x', pady=(6, 0))

        # Preview of the generated ramp list
        self.ramps_preview = tk.Label(inner, text="", bg=C['bg'], fg=C['cyan'],
                                     font=(FONT, fsz(10)))
        self.ramps_preview.pack(anchor='w', pady=(0, 8))

        # Estimated time
        self.est_lbl = tk.Label(inner, text="", bg=C['bg'], fg=C['yellow'],
                               font=(FONT, fsz(10), 'bold'))
        self.est_lbl.pack(anchor='w', pady=(0, 12))
        self._use_recommended_ramps()  # selected by default (instead of _set_step(10)) - it also called _update_estimate()

        # Buttons - in the bar pinned at the bottom of the window (see self._btnbar)
        bf = self._btnbar
        mk_btn(bf, "▶ START CALIBRATION", self.start, C['purple'], fg='#fff').pack(
            side='left', fill='x', expand=True, padx=(0, 4))
        mk_btn_outline(bf, "CANCEL", self.win.destroy, C['dim']).pack(
            side='left', fill='x', expand=True, padx=(4, 0))

    def _set_step(self, step):
        """Set the rate step (uniform step) and highlight the button - disables RECOMMENDED."""
        self.custom_ramps = None
        self.rate_step = step
        for s, b in self.step_btns.items():
            if s == step:
                b.config(bg=C['cyan'], fg='#1a1c1f')
            else:
                b.config(bg=C['bg2'], fg=C['dim'])
        if hasattr(self, 'btn_recommended'):
            self.btn_recommended.config(bg=C['bg2'], fg=C['dim'])
        self._update_estimate()

    def _use_recommended_ramps(self):
        """The default ramp list from the firmware (5, then every 10 up to 80) -
        it cannot be expressed with a uniform step, hence a separate path from _set_step."""
        self.custom_ramps = [5, 10, 20, 30, 40, 50, 60, 70, 80]
        for b in self.step_btns.values():
            b.config(bg=C['bg2'], fg=C['dim'])
        self.btn_recommended.config(bg=C['cyan'], fg='#1a1c1f')
        self._update_estimate()

    def _gen_ramps(self):
        """Generate the ramp list - either RECOMMENDED (custom_ramps) or a uniform
        step from max+step. E.g. max=20 step=5 -> [5,10,15,20]"""
        if self.custom_ramps is not None:
            return list(self.custom_ramps)
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
        if not ramps:  # when max < step, use max alone
            ramps = [int(round(maxr))]
        return ramps

    def _update_estimate(self):
        try:
            tmin = self.sl_tmin.get(); tmax = self.sl_tmax.get()
            n_temps = max(1, int((tmax - tmin) / 10) + 1)
            ramps = self._gen_ramps()
            n_ramps = len(ramps)
            # Relay: about 2-4 min/temperature typically (depends on how quickly
            # it catches the cycles - up to 10 min in the worst case). AFTER the
            # relay, a ramping test per ramp: back-off (usually <1 min) + 60s test
            # (run+tuning) = ~1.5 min/ramp.
            total_tests = n_temps * (1 + n_ramps)  # relay + each ramp, per temperature
            est_min = n_temps * (3 + n_ramps * 1.5)
            # Preview of the ramp list
            if hasattr(self, 'ramps_preview'):
                self.ramps_preview.config(
                    text=f"Ramps: {', '.join(str(r) for r in ramps)} °C/min")
            self.est_lbl.config(text=f"≈ {n_temps} temp × (relay + {n_ramps} ramp) = "
                                      f"{total_tests} tests · ~{est_min:.0f} min total")
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
        total_tests = n_temps * (1 + len(ramps))
        est_min = n_temps * (3 + len(ramps) * 1.5)
        if not messagebox.askyesno("Start calibration",
                f"Start auto-calibration?\n\n"
                f"Temp range: {tmin:.0f}-{tmax:.0f}°C (step 10°C)\n"
                f"Ramps: {', '.join(str(r) for r in ramps)} °C/min\n"
                f"Total: {total_tests} tests (relay + {len(ramps)} ramp per temperature)\n"
                f"Estimated: ~{est_min:.0f} min\n\n"
                "Takes a while. Can be stopped with STOP."):
            return
        self.app.start_autocal(tmin, tmax, ramps)
        self.win.destroy()

class CalibrationWindow:
    # Phases of one step (order = the sequence in the firmware): relay first
    # measures the base Kp/Ki/Kd for the temperature, then rampprep/ramptest run
    # in a loop for every ramp from calRamps (they tune the heating branch per ramp).
    PHASES = [('heating', '① Heating'), ('stabil', '② Stabilization'),
              ('relay', '③ Relay measurement'), ('rampprep', '④ Back-off'),
              ('ramptest', '⑤ Ramp test')]
    def __init__(self, parent, app):
        self.app = app
        self.win = tk.Toplevel(parent)
        self.win.title("Calibration progress")
        self.win.configure(bg=C['bg'])
        size_win(self.win, 640, 780, 600, 560, parent=parent)
        self.win.transient(parent)

        tk.Frame(self.win, bg=C['purple'], height=4).pack(fill='x')
        inner = tk.Frame(self.win, bg=C['bg'])
        inner.pack(fill='both', expand=True, padx=20, pady=16)

        tk.Label(inner, text="CALIBRATION PROGRESS", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(14), 'bold')).pack(anchor='w')
        tk.Label(inner, text="Relay autotuning — one test per temperature (fills all ramps)",
                 bg=C['bg'], fg=C['dim'], font=(FONT, fsz(9))).pack(anchor='w', pady=(2, 10))

        # Progress bar (counted in temperatures)
        self.prog_frame = tk.Frame(inner, bg=C['bg2'], height=SC(30))
        self.prog_frame.pack(fill='x', pady=(0, 10))
        self.prog_frame.pack_propagate(False)
        self.prog_bar = tk.Frame(self.prog_frame, bg=C['purple'], height=SC(30))
        self.prog_bar.place(x=0, y=0, relheight=1, relwidth=0)
        self.prog_text = tk.Label(self.prog_frame, text="0 / 0 temperatures", bg=C['bg2'],
                                  fg=C['text'], font=(FONT, fsz(11), 'bold'))
        self.prog_text.place(relx=0.5, rely=0.5, anchor='center')

        # Current step
        info = tk.Frame(inner, bg=C['panel'])
        info.pack(fill='x', pady=(0, 10))
        ii = tk.Frame(info, bg=C['panel'])
        ii.pack(fill='x', padx=14, pady=12)

        row1 = tk.Frame(ii, bg=C['panel']); row1.pack(fill='x', pady=2)
        tk.Label(row1, text="NOW:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(9)), width=11, anchor='w').pack(side='left')
        self.lbl_now = tk.Label(row1, text="—", bg=C['panel'], fg=C['orange'],
                                font=(FONT, fsz(12), 'bold'), anchor='w')
        self.lbl_now.pack(side='left')

        # Phase indicator: heating -> stabilization -> relay
        phase_row = tk.Frame(ii, bg=C['panel']); phase_row.pack(fill='x', pady=(8, 4))
        tk.Label(phase_row, text="PHASE:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(9)), width=11, anchor='w').pack(side='left')
        self.phase_lbls = {}
        for key, label in self.PHASES:
            l = tk.Label(phase_row, text=label, bg=C['bg2'], fg=C['dim2'],
                         font=(FONT, fsz(9)), padx=8, pady=4)
            l.pack(side='left', padx=(0, 4))
            self.phase_lbls[key] = l

        row2 = tk.Frame(ii, bg=C['panel']); row2.pack(fill='x', pady=2)
        tk.Label(row2, text="NEXT:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(9)), width=11, anchor='w').pack(side='left')
        self.lbl_next = tk.Label(row2, text="—", bg=C['panel'], fg=C['cyan'],
                                 font=(FONT, fsz(11)), anchor='w')
        self.lbl_next.pack(side='left')

        row3 = tk.Frame(ii, bg=C['panel']); row3.pack(fill='x', pady=2)
        tk.Label(row3, text="REMAINING:", bg=C['panel'], fg=C['dim2'],
                 font=(FONT, fsz(9)), width=11, anchor='w').pack(side='left')
        self.lbl_eta = tk.Label(row3, text="—", bg=C['panel'], fg=C['yellow'],
                                font=(FONT, fsz(11), 'bold'), anchor='w')
        self.lbl_eta.pack(side='left')

        # Warnings: points where the relay test did not catch oscillation and
        # got base values instead of actually measured ones (visible
        # only here - previously this fact went ONLY to the raw
        # serial console as "RELAY FAIL - bazowe")
        self.lbl_warn = tk.Label(inner, text="", bg=C['bg'], fg=C['red'],
                                 font=(FONT, fsz(9)), anchor='w', justify='left',
                                 wraplength=580)
        self.lbl_warn.pack(anchor='w', pady=(0, 6))

        # List of temperatures to calibrate
        tk.Label(inner, text="TEMPERATURES", bg=C['bg'], fg=C['dim'],
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
        """Readable step label. Safe for relay, where r is a string
        (the old {r:.0f} formatting raised an exception and the list never built)."""
        try:
            if isinstance(r, str):     # relay mode: one test per temperature
                return f"{t:.0f}°C"
            return f"{t:.0f}°C  @  {r:.0f}°C/min"
        except Exception:
            return f"{t}"

    def refresh(self):
        app = self.app
        total = app.cal_total or len(app.cal_plan)
        cur = app.cal_current
        phase = getattr(app, 'cal_phase', None)

        # Progress bar - computed so it does NOT jump to 100% the moment
        # the last point is only just starting (see _cal_progress_fraction:
        # cur/total did not distinguish that from "really finished").
        frac = app._cal_progress_fraction()
        self.prog_bar.place_configure(relwidth=min(1.0, frac))
        self.prog_text.config(text=f"{min(cur, total)} / {total} temperatures")

        # NOW
        if app.cal_cur_temp is not None:
            tnum = f"   ({cur}/{total})" if cur else ""
            rtxt = ""
            if phase in ('rampprep', 'ramptest') and isinstance(app.cal_cur_ramp, (int, float)):
                rtxt = f"   @ {app.cal_cur_ramp:.0f}°C/min"
            self.lbl_now.config(text=f"{app.cal_cur_temp:.0f}°C{tnum}{rtxt}")
        else:
            self.lbl_now.config(text="— (waiting for device)")

        # Highlight the active phase
        for key, _ in self.PHASES:
            if key == phase:
                self.phase_lbls[key].config(bg=C['orange'], fg='#1a1c1f')
            else:
                self.phase_lbls[key].config(bg=C['bg2'], fg=C['dim2'])

        # NEXT temperature
        if 0 < cur < len(app.cal_plan):
            nt, nr = app.cal_plan[cur]
            self.lbl_next.config(text=self._step_label(nt, nr))
        elif cur >= len(app.cal_plan) and len(app.cal_plan) > 0:
            self.lbl_next.config(text="(last)")
        else:
            self.lbl_next.config(text="—")

        # ETA - "COMPLETED" only when the calibration has REALLY finished
        # (cal_running=False), not when the last point has only just started.
        if not app.cal_running and cur >= total and total > 0:
            self.lbl_eta.config(text="COMPLETED ✓")
        else:
            eta = app._cal_eta()
            if eta is None:
                self.lbl_eta.config(text="—")
            elif eta < 1:
                # avoid the misleading "0 min 0 s" when it is really still working
                self.lbl_eta.config(text="finalizing…")
            else:
                m = int(eta // 60); s = int(eta % 60)
                self.lbl_eta.config(text=f"~{m} min {s} s")

        # Warnings about points with a failed relay test (base values)
        warns = getattr(app, 'cal_warnings', [])
        lines = []
        if warns:
            def _wtxt(w):
                t, cycles, amp = w
                if amp is not None and amp >= 140:
                    return f"{t:.0f}°C (even max. excitation power did not help)"
                return f"{t:.0f}°C"
            temps_txt = ", ".join(_wtxt(w) for w in warns)
            lines.append(f"⚠ Relay test did not catch oscillation for: {temps_txt} — "
                         f"base values (10.0/0.30/0.80) were used instead of actually measured ones.")
        # Warnings about individual ramps that did not get below the tracking
        # error threshold (ramp test AFTER relay - the relay for that temperature
        # itself succeeded, its result stays for that ramp - this is NOT
        # the same as above, hence a separate, less alarming line).
        rwarns = getattr(app, 'cal_ramp_warnings', [])
        if rwarns:
            def _rwtxt(w):
                t, r, err = w
                rtxt = f"{r:.0f}°C/min" if r is not None else "?"
                etxt = f", error {err:.1f}°C" if err is not None else ""
                return f"{t:.0f}°C @ {rtxt}{etxt}"
            lines.append("⚠ Ramp test did not bring the ASP tracking error below threshold for: "
                         + ", ".join(_rwtxt(w) for w in rwarns)
                         + " — the base profile from relay (still actually measured) stays for those ramps.")
        self.lbl_warn.config(text="\n".join(lines))

        # Temperature list - build once, then update statuses
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

        # Statuses + colors
        phase_txt = {'heating': '→ heating', 'stabil': '~ stabilization',
                     'relay': '◇ relay measurement'}
        warn_temps = {round(w[0]) for w in getattr(app, 'cal_warnings', [])}
        for i, (bar, num, txt, stat) in enumerate(self.step_widgets):
            step_no = i + 1
            step_temp = app.cal_plan[i][0] if i < len(app.cal_plan) else None
            failed = step_temp is not None and round(step_temp) in warn_temps
            if step_no < cur:
                if failed:
                    bar.config(bg=C['red']); txt.config(fg=C['dim2'])
                    num.config(fg=C['red']); stat.config(text="⚠ base (fail)", fg=C['red'])
                else:
                    bar.config(bg=C['green']); txt.config(fg=C['dim2'])
                    num.config(fg=C['green']); stat.config(text="✓ done", fg=C['green'])
            elif step_no == cur:
                bar.config(bg=C['orange']); txt.config(fg=C['text'])
                num.config(fg=C['orange'])
                stat.config(text=phase_txt.get(phase, "● now"), fg=C['orange'])
                try: self.canvas.yview_moveto(max(0, (i-3))/max(1, len(self.step_widgets)))
                except: pass
            else:
                bar.config(bg=C['bg2']); txt.config(fg=C['dim'])
                num.config(fg=C['dim2']); stat.config(text="pending", fg=C['dim2'])

    def abort(self):
        if messagebox.askyesno("Abort?", "Abort calibration?"):
            self.app.send("AUTOCALSTOP")
            self.app.send("STOP")
            self.app.cal_running = False
            self.win.destroy()


# ════════════════════════════════════════════════════════
#  DIAGNOSTICS WINDOW - log of all firmware events + active alarms
# ════════════════════════════════════════════════════════
class DiagnosticsWindow:
    """Shows everything the firmware sends over Serial, not just the
    CSV telemetry: error codes (ERR:) decoded into a readable description,
    calibration warnings (CALWARN) and every other text line (e.g.
    'Flash: zapisano.', 'AUTOCAL START') that the app used to drop
    silently. This makes catching and reporting errors easier, because
    you no longer need to attach a separate Serial Monitor (which cannot
    be done in parallel with the app on the same COM port anyway)."""
    LEVEL_COLOR = {'ERR': 'red', 'WARN': 'orange', 'INFO': 'dim'}

    def __init__(self, parent, app):
        self.app = app
        self.win = tk.Toplevel(parent)
        self.win.title("Diagnostics")
        self.win.configure(bg=C['bg'])
        size_win(self.win, 640, 560, 480, 360, parent=parent)
        self.win.transient(parent)
        self.win.protocol("WM_DELETE_WINDOW", self._on_close)

        tk.Frame(self.win, bg=C['purple'], height=4).pack(fill='x')
        inner = tk.Frame(self.win, bg=C['bg'])
        inner.pack(fill='both', expand=True, padx=20, pady=16)

        tk.Label(inner, text="DIAGNOSTICS", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(14), 'bold')).pack(anchor='w')
        tk.Label(inner, text="All events and errors reported by the firmware",
                 bg=C['bg'], fg=C['dim'], font=(FONT, fsz(9))).pack(anchor='w', pady=(2, 12))

        # Active alarms banner (visible only when err_active is not empty)
        self.active_frame = tk.Frame(inner, bg=C['bg2'])
        self.active_frame.pack(fill='x', pady=(0, 12))

        # Log - Text with color tags per level
        log_wrap = tk.Frame(inner, bg=C['bg2'])
        log_wrap.pack(fill='both', expand=True)
        sb = tk.Scrollbar(log_wrap)
        sb.pack(side='right', fill='y')
        self.text = tk.Text(log_wrap, bg=C['bg2'], fg=C['dim'], font=(FONT, fsz(9)),
                             relief='flat', bd=0, wrap='word', state='disabled',
                             yscrollcommand=sb.set, padx=10, pady=8)
        self.text.pack(side='left', fill='both', expand=True)
        sb.config(command=self.text.yview)
        self.text.tag_config('ERR', foreground=C['red'])
        self.text.tag_config('WARN', foreground=C['orange'])
        self.text.tag_config('INFO', foreground=C['dim'])
        self.text.tag_config('ts', foreground=C['dim2'])

        btn_row = tk.Frame(inner, bg=C['bg'])
        btn_row.pack(fill='x', pady=(12, 0))
        mk_btn(btn_row, "SAVE TO FILE", self.export_log, C['cyan']).pack(side='left')
        mk_btn_outline(btn_row, "CLEAR", self.clear_log, C['dim']).pack(side='left', padx=(8, 0))
        mk_btn_outline(btn_row, "CLOSE", self._on_close, C['red']).pack(side='right')

        self.refresh_active()
        self.reload_log()

    def refresh_active(self):
        for w in self.active_frame.winfo_children():
            w.destroy()
        if not self.app.err_active:
            tk.Label(self.active_frame, text="No active alarms.", bg=C['bg2'],
                     fg=C['green'], font=(FONT, fsz(9)), anchor='w').pack(
                     fill='x', padx=10, pady=8)
            return
        for code, text in self.app.err_active.items():
            row = tk.Frame(self.active_frame, bg=C['bg2'])
            row.pack(fill='x')
            tk.Frame(row, bg=C['red'], width=4).pack(side='left', fill='y')
            tk.Label(row, text=f"⚠ [{code}] {text}", bg=C['bg2'], fg=C['red'],
                     font=(FONT, fsz(9), 'bold'), anchor='w', justify='left',
                     wraplength=560).pack(side='left', fill='x', expand=True, padx=8, pady=6)

    def reload_log(self):
        self.text.config(state='normal')
        self.text.delete('1.0', 'end')
        for entry in self.app.diag_log:
            self._insert_entry(entry)
        self.text.config(state='disabled')
        self.text.see('end')

    def append_entry(self, entry):
        """Called from app._log_diag() when the window is open - appends live
        instead of waiting for reload_log()."""
        try:
            self.text.config(state='normal')
            self._insert_entry(entry)
            self.text.config(state='disabled')
            self.text.see('end')
        except Exception:
            pass
        self.refresh_active()

    def _insert_entry(self, entry):
        ts, level, text = entry
        tstr = datetime.fromtimestamp(ts).strftime('%H:%M:%S')
        self.text.insert('end', f"[{tstr}] ", 'ts')
        self.text.insert('end', f"{level:<4} ", level)
        self.text.insert('end', f"{text}\n", level)

    def clear_log(self):
        self.app.diag_log = []
        self.reload_log()

    def export_log(self):
        try:
            fn = self.app.log_dir / f"diagnostyka_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
            with open(fn, 'w', encoding='utf-8') as f:
                for ts, level, text in self.app.diag_log:
                    tstr = datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
                    f.write(f"[{tstr}] {level:<4} {text}\n")
            messagebox.showinfo("Saved", f"Log saved:\n{fn}")
        except Exception as e:
            messagebox.showerror("Save error", str(e))

    def _on_close(self):
        self.app.diag_win = None
        self.win.destroy()


# ════════════════════════════════════════════════════════
#  PRESETS WINDOW (savable sets of settings)
# ════════════════════════════════════════════════════════
class PresetWindow:
    def __init__(self, parent, app):
        self.app = app
        self.win = tk.Toplevel(parent)
        self.win.title("Presets")
        self.win.configure(bg=C['bg'])
        size_win(self.win, 520, 560, 460, 420, parent=parent)
        self.win.transient(parent)

        tk.Frame(self.win, bg=C['green'], height=4).pack(fill='x')
        inner = tk.Frame(self.win, bg=C['bg'])
        inner.pack(fill='both', expand=True, padx=24, pady=20)

        tk.Label(inner, text="PRESETS", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(14), 'bold')).pack(anchor='w')
        tk.Label(inner, text="Save & load complete settings (setpoint, ramps, PID, fan)",
                 bg=C['bg'], fg=C['dim'], font=(FONT, fsz(9))).pack(anchor='w', pady=(2, 16))

        # Save a new preset
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

        # List of saved presets
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
            # Short description of the settings
            desc = f"SP {settings.get('sp','?')}°C · ↑{settings.get('ru','?')} ↓{settings.get('rd','?')}°C/min · fan {settings.get('fan','?')}%"
            tk.Label(info, text=desc, bg=C['bg2'], fg=C['dim2'],
                     font=(FONT, fsz(8)), anchor='w').pack(anchor='w')
            # Buttons
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
#  CYCLE SAVE DIALOG
# ════════════════════════════════════════════════════════
class SaveCycleDialog:
    def __init__(self, parent, app, tmp_path):
        self.app = app
        self.tmp_path = tmp_path
        self.win = tk.Toplevel(parent)
        self.win.title("Save cycle")
        self.win.configure(bg=C['bg'])
        size_win(self.win, 440, 230, 400, 200, parent=parent)
        self.win.transient(parent)
        self.win.grab_set()  # modal

        tk.Frame(self.win, bg=C['green'], height=4).pack(fill='x')
        inner = tk.Frame(self.win, bg=C['bg'])
        inner.pack(fill='both', expand=True, padx=24, pady=20)

        tk.Label(inner, text="SAVE CYCLE TO ARCHIVE", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(13), 'bold')).pack(anchor='w')

        # Info on the number of samples
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
        # Default name
        default = datetime.now().strftime("test_%H%M")
        self.entry.insert(0, default)
        self.entry.select_range(0, 'end')
        self.entry.focus()
        self.entry.bind('<Return>', lambda e: self.save())

        # Buttons
        bf = tk.Frame(inner, bg=C['bg'])
        bf.pack(fill='x')
        mk_btn(bf, "SAVE", self.save, C['green']).pack(side='left', fill='x',
                                                          expand=True, padx=(0, 4))
        mk_btn_outline(bf, "DISCARD", self.discard, C['red']).pack(side='left',
                                                          fill='x', expand=True, padx=(4, 0))

        self.win.protocol("WM_DELETE_WINDOW", self.save)  # closing = save

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
#  MULTI-STEP PROFILES WINDOW
# ════════════════════════════════════════════════════════
class ProfileWindow:
    def __init__(self, parent, app):
        self.app = app
        self.win = tk.Toplevel(parent)
        self.win.title("Multi-step profiles")
        self.win.configure(bg=C['bg'])
        size_win(self.win, 520, 480, 440, 360, parent=parent)
        self.win.transient(parent)

        tk.Frame(self.win, bg=C['purple'], height=4).pack(fill='x')
        hd = tk.Frame(self.win, bg=C['bg'])
        hd.pack(fill='x', padx=16, pady=12)
        tk.Label(hd, text="MULTI-STEP PROFILES", bg=C['bg'], fg=C['text'],
                 font=(FONT, fsz(12), 'bold')).pack(side='left')

        # Step table
        self.rows_frame = tk.Frame(self.win, bg=C['bg'])
        self.rows_frame.pack(fill='both', expand=True, padx=16)

        # Headers
        h = tk.Frame(self.rows_frame, bg=C['bg'])
        h.pack(fill='x', pady=(0, 4))
        for txt, w in [("#", 3), ("TEMP °C", 10), ("RATE", 8), ("TIME min", 10), ("", 6)]:
            tk.Label(h, text=txt, bg=C['bg'], fg=C['dim2'],
                     font=(FONT, fsz(9)), width=w, anchor='w').pack(side='left')

        self.steps_container = tk.Frame(self.rows_frame, bg=C['bg'])
        self.steps_container.pack(fill='both', expand=True)

        # Add form
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

        # Run
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
            messagebox.showerror("Error", "Enter valid numbers.")

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
        """Run the profile - send the steps sequentially with a delay"""
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
        """Thread that executes the profile"""
        for i, s in enumerate(self.app.profile_steps):
            self.app.send(f"SP:{s['temp']:.1f}")
            self.app.send(f"RU:{s['ramp']:.1f}")
            self.app.send(f"RD:{s['ramp']:.1f}")
            if i == 0:
                time.sleep(0.1)
                self.app.send("START")
            # Wait out the step time (time in minutes)
            time.sleep(max(1, s['time'] * 60))
        # After the profile - stop
        self.app.send("STOP")
        print("Profile finished")


# ════════════════════════════════════════════════════════
#  MAIN
# ════════════════════════════════════════════════════════
def _enable_dpi_awareness():
    """Enable DPI awareness on Windows - eliminates blurry text at 125%/150% scaling."""
    if sys.platform != 'win32':
        return 1.0
    try:
        import ctypes
        # Per-Monitor DPI Aware v2 (Windows 10 1703+) - best sharpness
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(2)
        except Exception:
            # Fallback for older Windows
            ctypes.windll.user32.SetProcessDPIAware()
        # Read the actual scaling
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
    # IMPORTANT: DPI awareness BEFORE creating the window - gives sharp text
    scale = _enable_dpi_awareness()

    # Set the global font multiplier from DPI (sharp AND readable)
    global FS
    if scale and scale > 1.05:
        FS = scale  # e.g. 1.25 for 125%, 1.5 for 150%
    else:
        FS = 1.0

    root = tk.Tk()

    # REMOVED: root.tk.call('tk','scaling', scale)
    # That was a SECOND, independent scaling stacked on top of FS - and the two fought.
    # Tk converts a font size (given positive = in POINTS) into pixels
    # through its own 'tk scaling' factor. The code set it to dpi/96
    # (e.g. 1.5), while the Windows default is 96/72 = 1.333 - and it
    # SIMULTANEOUSLY multiplied every font size by FS=1.5. The result was
    # ~1.69x instead of 1.5x, and INCONSISTENTLY across widgets (ttk versus
    # plain tk), because not every widget goes through the same path.
    # Now ONE multiplier remains: FS - fonts via fsz(), pixels via
    # SC()/size_win(). The ttk.Notebook tab font is set explicitly in
    # _build_styles() anyway, so nothing is lost here.

    app = PeltierControl(root)

    def on_close():
        # Always hand the system back its right to sleep - otherwise the lock
        # would outlive the closed program until logout (see _wake_lock).
        try: app._wake_lock(False)
        except Exception: pass
        app.disconnect()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == "__main__":
    main()
