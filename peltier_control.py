#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LACHI - Light-Activated Current & Heat Instrument.

Control bench for photocurrent and pyrocurrent measurements: it drives the
Peltier stage, so the sample can be held at a temperature (photocurrent - the
light-activated term) or ramped at a commanded rate (pyrocurrent - the heat
term, dT/dt), and it archives every run.

  Light-Activated Current  - the photocurrent side of the experiment
  Heat                     - the controlled dT/dt that drives the pyrocurrent
  Instrument               - it is one, and it is treated as one

Two-way link with the board: setpoint, ramps, PID, calibration, profiles.
Requires firmware v19 (PC MODE) or newer on the ItsyBitsy M0.

═══════════════════════════════════════════════════════════════════════════
WHY THIS FILE IS PyQt6 AND NOT TKINTER
═══════════════════════════════════════════════════════════════════════════
The instrument logic below is the same logic that ran under Tkinter, down to
the comments explaining the bugs it fixes - the sleep guard, the smooth leg
switch that closed the 1.4-1.7 s hole between heating and cooling, the ramp
phase measured separately from the approach tail. None of that changed. What
changed is the interface layer: Qt has a stylesheet engine close enough to CSS
that Apple's design tokens can be expressed directly, and a canvas widget that
embeds matplotlib natively - so the live trace still redraws four times a
second and still zooms under the mouse, which a framework that ships the figure
to the browser as an image cannot do.

The look lives in lachi/theme.py (the palette and the stylesheet),
lachi/widgets.py (the Cupertino components Qt does not ship) and
lachi/icons.py (an SF-shaped icon set, drawn rather than licensed).

The older Tkinter build stays in the repository as peltier_control.py and
still runs; nothing here touches it.
"""
from __future__ import annotations

import csv
import json
import os
import queue
import re
import statistics
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pip install pyserial")
    sys.exit(1)

from PyQt6.QtCore import QSize, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QColor, QFont, QIcon, QPainter, QPixmap
from PyQt6.QtWidgets import (QApplication, QHBoxLayout, QLabel, QListWidget,
                             QListWidgetItem, QMainWindow, QScrollArea,
                             QStackedWidget, QVBoxLayout, QWidget)

from lachi import brand, core, icons, widgets as W
from lachi.chart import ChartPanel, style_axes, style_legend
from lachi.core import (APP_BUILD, APP_NAME, CSV_COLS, ERR_CODES,
                        PING_EVERY_S, REACH_STABLE_S, REACH_TOL_C,
                        SERIES_COOL_TIMEOUT_S, SERIES_HEAT_TIMEOUT_S,
                        STALL_LIMIT_S, Var, csv_row, decode_tc_fault)
from lachi.dialogs import (CalibrationDialog, CalRangeDialog, CalTableDialog,
                           DiagnosticsDialog, PresetDialog, ProfileDialog,
                           SaveCycleDialog, StatsDialog)
from lachi.theme import PAD, Theme

# The colour a run is drawn in on the archive chart. Eight distinguishable
# system hues; a ninth run wraps round, which is what the eye can hold anyway.
ARCH_COLOR_KEYS = ['blue', 'orange', 'green', 'red', 'teal', 'purple',
                   'yellow', 'pink']


class Lachi(QMainWindow):
    # ────────────────────────────────────────────────────────────────────
    #  CONSTRUCTION
    # ────────────────────────────────────────────────────────────────────
    # Marshals a call from the serial reader thread onto the GUI thread. The
    # Tkinter build used root.after(0, fn) for this; a queued signal is the
    # same idea with the same guarantee - the callable runs on the thread that
    # owns the widgets, never on the reader.
    sig_call = pyqtSignal(object)

    def __init__(self):
        super().__init__()

        # ── appearance ──────────────────────────────────────────────────
        self.cfg_dir = Path.home() / "PeltierLogi"
        self.cfg_dir.mkdir(exist_ok=True)
        self.settings_file = self.cfg_dir / "ustawienia.json"
        self.th = Theme(self._load_setting('appearance', 'light'),
                        float(self._load_setting('ui_scale', 1.0) or 1.0))

        # ── serial ──────────────────────────────────────────────────────
        self.ser = None
        self.port_name = None
        self.baud = 115200
        self.running = False
        self.connected = False

        # ── measurement buffers ─────────────────────────────────────────
        self.maxlen = 3000
        self.t = []; self.temp = []; self.spt = []; self.spa = []
        self.pwm = []; self.kp = []; self.ki = []; self.kd = []; self.states = []
        self.t0 = None
        self.data_queue = queue.Queue()
        self.last_state = 'MAN'
        self.cur_state = 'MAN'
        self._latest_temp2 = None

        # ── tracking the approach to the setpoint ───────────────────────
        self.reach_start_t = None
        self.reach_start_temp = None
        self.reach_target = None
        self.reach_done = False
        self.reach_in_tol_t = None
        self.reach_time = None
        self.reach_avg_rate = None
        self.reach_dir = None
        self.last_setpoint_target = None
        self._last_reach_summary = None

        # ── SEPARATE tracking of the RAMP PHASE ─────────────────────────
        # "AVG RATE" used to be counted from the start until entering ±0.5 °C
        # of the target - that is, TOGETHER with the approach tail, which can
        # last longer than the ramp itself. With 30 °C/min commanded it showed
        # "avg 12.16 °C/min", which looks as if the ramp ran 2.5x too slow,
        # while in reality the ramp ran at ~26 and only the approach over the
        # last 0.5 °C took the rest. These two must be measured SEPARATELY,
        # because they are fixed by completely different changes (ramp rate =
        # feed-forward, approach tail = loss compensation + integrator).
        # Ramp phase = as long as the ramp GENERATOR (active setpoint spA) is
        # still travelling. Once spA arrives the ramp is over, wherever the
        # real temperature happens to be.
        self.ramp_t0 = None
        self.ramp_temp0 = None
        self.ramp_done = False
        self.ramp_secs = None
        self.ramp_rate = None
        self.ramp_cmd_rate = None
        self.ramp_lag = None

        # ── device state ────────────────────────────────────────────────
        self.dev_pol_swapped = False
        self.dev_pol_set = False
        self.dev_cal_min = 50.0
        self.dev_cal_max = 100.0
        self.dev_fw_build = None
        self.dev_cal = False
        self.fan_on = False
        self.is_running = False

        # ── live chart control ──────────────────────────────────────────
        self.chart_paused = False
        self.chart_window = 0          # 0 = whole run, >0 = last N seconds
        self._live_args = None

        # ── where the data ends up ──────────────────────────────────────
        # cfg_dir - PERMANENT app folder (calibration, presets, settings). It
        #           does not travel with the data, so changing where
        #           measurements are saved never "loses" the calibration.
        # log_dir - folder FOR MEASUREMENT DATA, chosen by the user on the
        #           ARCHIVE screen. Remembered between runs.
        self.log_dir = self._load_data_dir()
        self.cal_file = self.cfg_dir / "kalibracja.json"
        self.presets_file = self.cfg_dir / "presety.json"

        self.cyc_on = False
        self.cyc_file = None
        self.cyc_wr = None
        self.cyc_t0 = None
        self.cyc_fn = None
        self.cyc_rows = 0
        self.series_name_hint = None

        # ── measurement series ──────────────────────────────────────────
        self.series_steps = []
        self.series_idx = 0
        self.series_running = False
        self.series_leg = None          # 'heat' | 'cool' | None
        self.series_phase = None        # 'ramping' | 'holding' | 'ending'
        self.series_phase_t0 = None
        self.series_base_sp = 25.0
        self.series_skip_archive = False
        self._series_saved_rd = None
        self.series_mode = Var('seria')
        self.series_cool_as_test = Var(False)

        self.profile_steps = []

        # ── calibration state ───────────────────────────────────────────
        self.cal_plan = []
        self.cal_total = 0
        self.cal_current = 0
        self.cal_cur_temp = None
        self.cal_cur_ramp = None
        self.cal_phase = None
        self.cal_running = False
        self.cal_t0 = None
        self.cal_step_times = []
        self.cal_win = None
        self.cal_warnings = []
        self.cal_ramp_warnings = []
        self._caldump_buf = []
        self._caldump_active = False
        self._caldump_purpose = None
        self._pending_offset = None

        # ── diagnostics ─────────────────────────────────────────────────
        self.diag_log = []
        self.err_active = {}
        self.diag_unseen = 0
        self.diag_win = None

        # ── archive state ───────────────────────────────────────────────
        self.arch_vars = {}
        self.arch_xmode = Var('t0')
        self.arch_delta = Var(False)
        self.arch_show = {k: Var(v) for k, v in
                          (('temp', True), ('sa', True), ('st', True),
                           ('t2', False), ('pwm', False))}
        self.arch_tref = Var(40.0)
        self._ax_pwm = None
        self._last_sa = []
        self._last_temp2 = []
        self._last_pc = []

        self._icon_btns = []            # (button, icon_name, kind, size)
        self.sig_call.connect(lambda fn: fn())

        self._build_ui()
        self.set_status(False, "")

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(250)
        QTimer.singleShot(800, self._auto_connect)

    # -- small helpers ---------------------------------------------------
    def after(self, ms, fn):
        """root.after, in Qt. Kept for the ported logic's call sites."""
        QTimer.singleShot(int(ms), fn)

    def call(self, fn):
        """Run fn on the GUI thread - safe from the serial reader."""
        self.sig_call.emit(fn)

    @staticmethod
    def _restyle(w):
        """Re-evaluate a widget's property selectors.

        Qt resolves QSS property selectors once, when the widget is polished.
        Changing the property afterwards - Start becoming Stop, say - does not
        repaint it on its own; the style has to be told to look again.
        """
        w.style().unpolish(w)
        w.style().polish(w)
        w.update()

    def _icon_button(self, name, tip, cb, color='blue', size=22):
        b = W.icon_button(self.th, name, tip, cb, color, size)
        self._icon_btns.append([b, name, color, size])
        return b

    def _button(self, text, kind='plain', icon='', cb=None):
        b = W.button(self.th, text, kind, icon, cb)
        if icon:
            self._icon_btns.append([b, icon, kind, 19])
        return b

    def _tint(self, kind):
        """The glyph colour a button of this `kind` wants."""
        fixed = {'filled': '#FFFFFF', 'go': '#FFFFFF', 'stop': '#FFFFFF',
                 'warn': '#FFFFFF'}.get(kind)
        if fixed:
            return fixed
        key = kind if kind in self.th.p else (
            'red' if kind == 'destructive' else 'blue')
        val = self.th[key]
        # The label tints are rgba() strings - QColor cannot parse those, and
        # an unparsed colour paints the glyph flat black. Flatten against the
        # bar it sits on instead.
        return self.th.solid(key, 'bg') if val.startswith('rgba') else val

    def _reicon(self, b, name, kind=None):
        """Swap a registered button's glyph AND its registry entry.

        Without the second half, flipping the appearance repaints every
        button with the glyph it was BORN with: Stop would go back to
        showing a play triangle mid-run, and Resume a pause bar.
        """
        for e in self._icon_btns:
            if e[0] is b:
                e[1] = name
                if kind is not None:
                    e[2] = kind
                b.setIcon(icons.icon(name, self.th.px(e[3]),
                                     self._tint(e[2])))
                return
        b.setIcon(icons.icon(name, self.th.px(19),
                             self._tint(kind or 'plain')))

    # ════════════════════════════════════════════════════════════════════
    #  UI
    # ════════════════════════════════════════════════════════════════════
    def _build_ui(self):
        self.setWindowTitle(f"{APP_NAME} — photocurrent & pyrocurrent bench")
        self.resize(self.th.px(1180), self.th.px(880))
        self.setMinimumSize(self.th.px(900), self.th.px(640))
        self.setWindowIcon(QIcon(brand.medallion_pixmap(
            64, self.th['blue'], 'transparent')))

        page = QWidget()
        page.setObjectName('page')
        root = QVBoxLayout(page)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ── header: wordmark, status chip, diagnostics, appearance ──────
        self.header = W.HeaderBar(self.th)
        self._apply_wordmark()
        self.header.appearance.clicked.connect(self.toggle_appearance)
        self.btn_diag = self._icon_button('info', "Diagnostics",
                                          self.open_diag_window, 'label2', 20)
        self.header.layout().insertWidget(
            self.header.layout().count() - 2, self.btn_diag)
        root.addWidget(self.header)

        # ── the five screens ────────────────────────────────────────────
        self.stack = QStackedWidget()
        root.addWidget(self.stack, 1)
        for build in (self._page_control, self._page_series, self._page_archive,
                      self._page_tuning, self._page_device):
            self.stack.addWidget(build())

        # ── tab bar ─────────────────────────────────────────────────────
        self.tabs = W.TabBar(self.th, [
            ('gauge', 'Control'), ('list', 'Series'), ('archive', 'Archive'),
            ('sliders', 'Tuning'), ('gear', 'Device')])
        self.tabs.changed.connect(self.stack.setCurrentIndex)
        root.addWidget(self.tabs)

        self.setCentralWidget(page)
        self._refresh_diag_indicator()

    def _apply_wordmark(self):
        """The name is DRAWN, not typed - the same letterforms as the emblem,
        so the mark is one object rather than a picture next to some text in
        whatever font the machine happens to have."""
        h = self.th.px(30)
        px = brand.wordmark_pixmap(h, self.th['label'])
        self.header.title.setPixmap(px)
        self.header.title.setFixedHeight(
            int(px.height() / px.devicePixelRatio()))

    def _scroll_page(self):
        """A screen: a scroll area over a padded column. Returns (page, column)."""
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(
            Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        body = QWidget()
        body.setObjectName('bg')
        col = QVBoxLayout(body)
        col.setContentsMargins(self.th.px(PAD), self.th.px(4),
                               self.th.px(PAD), self.th.px(18))
        col.setSpacing(self.th.px(18))
        scroll.setWidget(body)
        return scroll, col

    def _card_header(self, text, right_text=""):
        row = QHBoxLayout()
        lb = QLabel(text)
        lb.setStyleSheet("QLabel { font-size: %dpt; font-weight: 600; }"
                         % self.th.pt('headline'))
        row.addWidget(lb)
        row.addStretch(1)
        hint = QLabel(right_text)
        hint.setStyleSheet(f"color: {self.th['label3']}; "
                           f"font-size: {self.th.pt('caption')}pt;")
        row.addWidget(hint)
        return row, lb, hint

    # ────────────────────────────────────────────────────────────────────
    #  CONTROL
    # ────────────────────────────────────────────────────────────────────
    def _page_control(self):
        page, col = self._scroll_page()

        # ── readouts ────────────────────────────────────────────────────
        stats = QHBoxLayout()
        stats.setSpacing(self.th.px(10))
        self.cards = {}
        for key, title, unit, colour in (
                ('temp', 'temperature', '°C', 'blue'),
                ('temp2', 'thermocouple 2', '°C', 'cyan'),
                ('sp', 'setpoint', '°C', 'orange'),
                ('rate', 'avg rate', '°C/min', 'teal'),
                ('pwm', 'power', '%', 'green')):
            s = W.Stat(self.th, title, unit, colour)
            self.cards[key] = s
            stats.addWidget(s)
        col.addLayout(stats)

        # ── live chart ──────────────────────────────────────────────────
        card = W.Card(self.th, pad=12)
        head, _, self.reach_lbl = self._card_header("Live", "")
        self.reach_lbl.setStyleSheet(
            f"color: {self.th['label2']}; font-size: {self.th.pt('footnote')}pt;")
        card.box.addLayout(head)

        self.chart = ChartPanel(self.th, nrows=2, height_ratios=[3, 1],
                                hspace=0.05,
                                margins=(0.045, 0.035, 0.995, 0.985))
        self.chart.redraw = self._nav_redraw
        self.chart.setMinimumHeight(self.th.px(330))
        self.ax1, self.ax2 = self.chart.axes
        card.box.addWidget(self.chart, 1)

        tools = QHBoxLayout()
        tools.setSpacing(self.th.px(8))
        self.btn_pause = self._button("Pause", 'tinted', 'pause',
                                      self.toggle_pause)
        # Fixed, not hugging the text: the label flips between "Pause" and the
        # longer "Resume", and a button that changes width under the cursor
        # makes the whole tool row jump.
        self.btn_pause.setFixedWidth(self.th.px(150))
        tools.addWidget(self.btn_pause)
        self.seg_window = W.Segmented(self.th, ["All", "5 min", "2 min", "1 min"], 0)
        self.seg_window.changed.connect(
            lambda i: self.set_chart_window([0, 300, 120, 60][i]))
        self.seg_window.setMaximumWidth(self.th.px(260))
        tools.addWidget(self.seg_window, 1)
        hint = QLabel("drag · scroll · right-click")
        hint.setStyleSheet(f"color: {self.th['label3']}; "
                           f"font-size: {self.th.pt('caption')}pt;")
        tools.addWidget(hint)
        tools.addWidget(self._icon_button('share', "Save chart as image",
                                          self.save_live_chart, 'blue', 20))
        card.box.addLayout(tools)
        col.addWidget(card)

        # ── run setup ───────────────────────────────────────────────────
        g = W.Group(self.th, "Setpoint & ramps", footer="Applied on Start.")
        self.sl_sp = W.SliderField(g, self.th, "Target", -15, 100, 25.0, "°C", 1,
                                   on_change=lambda v: self.send(f"SP:{v:.1f}"))
        self.sl_ru = W.SliderField(g, self.th, "Heat rate", 0.5, 80, 2.0,
                                   "°C/min", 1,
                                   on_change=lambda v: self.send(f"RU:{v:.1f}"))
        self.sl_rd = W.SliderField(g, self.th, "Cool rate", 0.5, 80, 2.0,
                                   "°C/min", 1,
                                   on_change=lambda v: self.send(f"RD:{v:.1f}"))
        self.sl_tmax = W.SliderField(g, self.th, "Max temperature", 50, 115, 80,
                                     "°C", 0,
                                     on_change=lambda v: self.send(f"TMAX:{v:.0f}"))
        col.addWidget(g)

        # ── fans ────────────────────────────────────────────────────────
        g2 = W.Group(self.th, "Heatsink fans",
                     footer="The firmware forces the fans on whenever the stage "
                            "is cooling, and keeps them running for two minutes "
                            "after a run ends. This switch is the manual "
                            "override on top of that.")
        self.sw_fan = W.Switch(self.th, False)
        self.sw_fan.toggled.connect(self.on_fan_toggle)
        g2.add_row("Fans", self.sw_fan, sub="forced on while cooling")
        self.sl_fan = W.SliderField(g2, self.th, "Speed", 0, 100, 100, "%", 0,
                                    on_change=self.set_fan_speed)
        col.addWidget(g2)

        # ── stored setups ───────────────────────────────────────────────
        g3 = W.Group(self.th, "Stored setups")
        g3.add_row("Profiles", self._button("Open", 'plain', 'chevron',
                                            self.open_profiles),
                   sub="Multi-step temperature programs")
        g3.add_row("Presets", self._button("Open", 'plain', 'chevron',
                                           self.open_presets),
                   sub="Complete settings, saved by name")
        self.cal_status_row = g3.add_row(
            "Calibration", W.value_label(self.th, "reading from device…"),
            sub="Tap while calibrating to watch progress")
        self.cal_status_row.mousePressEvent = lambda e: self.open_cal_window()
        self.cal_status_row.setCursor(Qt.CursorShape.PointingHandCursor)
        col.addWidget(g3)
        col.addStretch(1)

        # ── the run controls ────────────────────────────────────────────
        # PINNED, not scrolled. iOS keeps a primary action on a bar above the
        # tab bar, and on an instrument that is not a style choice: Stop has to
        # be one click away no matter how far down the page you have scrolled.
        barw = QWidget()
        barw.setObjectName('bg')
        bar = QHBoxLayout(barw)
        bar.setContentsMargins(self.th.px(PAD), self.th.px(8),
                               self.th.px(PAD), self.th.px(10))
        bar.setSpacing(self.th.px(10))
        self.btn_run = self._button("Start", 'go', 'play', self.toggle_run)
        self.btn_run.setMinimumHeight(self.th.px(48))
        self.btn_freeze = self._button("Freeze", 'tinted', 'snowflake',
                                       self.do_freeze)
        self.btn_freeze.setMinimumHeight(self.th.px(48))
        self.btn_estop = self._button("", 'stop', 'xmark', self.do_estop)
        self.btn_estop.setMinimumHeight(self.th.px(48))
        self.btn_estop.setFixedWidth(self.th.px(70))
        self.btn_estop.setToolTip("Emergency stop - cuts the drive immediately")
        bar.addWidget(self.btn_run, 3)
        bar.addWidget(self.btn_freeze, 2)
        bar.addWidget(self.btn_estop, 0)

        wrap = QWidget()
        wrap.setObjectName('bg')
        wl = QVBoxLayout(wrap)
        wl.setContentsMargins(0, 0, 0, 0)
        wl.setSpacing(0)
        wl.addWidget(page, 1)
        wl.addWidget(barw)

        self._redraw_live([], [], [], [], [])
        return wrap

    # ────────────────────────────────────────────────────────────────────
    #  SERIES
    # ────────────────────────────────────────────────────────────────────
    def _page_series(self):
        page, col = self._scroll_page()

        intro = QLabel("Add tests - setpoint, rate and dwell. The app runs them "
                       "one after another, returns to base between them and "
                       "archives every result itself.")
        intro.setWordWrap(True)
        intro.setStyleSheet(f"color: {self.th['label2']}; "
                            f"font-size: {self.th.pt('footnote')}pt;")
        col.addWidget(intro)

        g = W.Group(self.th, "New test")
        self.ser_sp = W.SliderField(g, self.th, "Setpoint", -15, 110, 50.0,
                                    "°C", 1)
        self.ser_rate = W.SliderField(g, self.th, "Heat rate", 0.5, 80, 30.0,
                                      "°C/min", 1)
        self.ser_hold = W.SliderField(g, self.th, "Hold after reached", 0, 900,
                                      60, "s", 0)
        row = QWidget()
        rl = QHBoxLayout(row)
        rl.setContentsMargins(self.th.px(12), self.th.px(8), self.th.px(12),
                              self.th.px(12))
        rl.setSpacing(self.th.px(8))
        rl.addWidget(self._button("Add test", 'tinted', 'plus',
                                  self._on_series_add), 2)
        rl.addWidget(self._button("Ramps 10…70", 'plain', '',
                                  self._on_series_quickfill), 2)
        g.add_widget(row)
        col.addWidget(g)

        g2 = W.Group(self.th, "Between tests",
                     footer="The return rate is independent of Cool rate on "
                            "Control. It defaults to the maximum on purpose: a "
                            "slower return is slower than the stage cools by "
                            "itself, so the controller adds heat to brake it "
                            "and every return starts with a visible hump.")
        self.ser_base = W.SliderField(g2, self.th, "Base temperature", -15, 100,
                                      self.series_base_sp, "°C", 1,
                                      on_change=self._on_series_base_change)
        self.ser_return = W.SliderField(g2, self.th, "Return rate", 1, 80, 80.0,
                                        "°C/min", 1)
        self.seg_mode = W.Segmented(self.th, ["Test series", "Program"], 0)
        self.seg_mode.changed.connect(
            lambda i: self.series_mode.set('seria' if i == 0 else 'program'))
        g2.add_row("Mode", self.seg_mode,
                   sub="Series returns to base after each test; a program runs "
                       "the steps straight on from where the last one ended")
        self.sw_cool_test = W.Switch(self.th, False)
        self.sw_cool_test.toggled.connect(self.series_cool_as_test.set)
        g2.add_row("Descent is a test too", self.sw_cool_test,
                   sub="Collects cooling data at the test's own rate")
        col.addWidget(g2)

        g3 = W.Group(self.th, "Test list")
        self.series_list = QListWidget()
        self.series_list.setMinimumHeight(self.th.px(180))
        g3.add_widget(self.series_list, sep=False)
        row2 = QWidget()
        r2 = QHBoxLayout(row2)
        r2.setContentsMargins(self.th.px(12), self.th.px(6), self.th.px(12),
                              self.th.px(10))
        r2.setSpacing(self.th.px(8))
        r2.addWidget(self._button("Remove", 'plain', 'minus',
                                  self._on_series_remove))
        r2.addWidget(self._button("Clear", 'destructive', 'trash',
                                  self._on_series_clear))
        r2.addStretch(1)
        r2.addWidget(self._button("Save program", 'plain', 'export',
                                  self._series_save_prog))
        r2.addWidget(self._button("Load", 'plain', 'folder',
                                  self._series_load_prog))
        g3.add_widget(row2)
        col.addWidget(g3)

        g4 = W.Group(self.th, "Status")
        self.series_status_row = g4.add_row("Series inactive")
        col.addWidget(g4)

        self.btn_series_run = self._button("Start series", 'go', 'play',
                                           self._on_series_toggle)
        self.btn_series_run.setMinimumHeight(self.th.px(48))
        col.addWidget(self.btn_series_run)
        col.addStretch(1)

        self._series_refresh_list()
        return page

    # ────────────────────────────────────────────────────────────────────
    #  ARCHIVE
    # ────────────────────────────────────────────────────────────────────
    def _page_archive(self):
        page = QWidget()
        page.setObjectName('bg')
        outer = QVBoxLayout(page)
        outer.setContentsMargins(self.th.px(PAD), self.th.px(4),
                                 self.th.px(PAD), self.th.px(12))
        outer.setSpacing(self.th.px(12))

        # ── where the measurements live ─────────────────────────────────
        gdir = W.Group(self.th)
        right = QWidget()
        rl = QHBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.setSpacing(self.th.px(2))
        rl.addWidget(self._icon_button('folder', "Open in file manager",
                                       self.open_log_folder, 'label2', 19))
        rl.addWidget(self._button("Change", 'plain', '', self.choose_data_dir))
        rl.addWidget(self._button("New", 'plain', 'plus', self.create_data_dir))
        self.data_dir_row = gdir.add_row("Data folder", right,
                                         sub=str(self.log_dir))
        outer.addWidget(gdir)

        body = QHBoxLayout()
        body.setSpacing(self.th.px(12))

        # ── the list of runs ────────────────────────────────────────────
        left = QWidget()
        left.setFixedWidth(self.th.px(320))
        ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        ll.setSpacing(self.th.px(6))
        hdr = QHBoxLayout()
        cap = QLabel("SAVED RUNS")
        cap.setObjectName('groupHeader')
        hdr.addWidget(cap)
        hdr.addStretch(1)
        hdr.addWidget(self._button("All", 'plain', '', self._arch_select_all))
        hdr.addWidget(self._button("None", 'plain', '', self._arch_clear_sel))
        hdr.addWidget(self._icon_button('refresh', "Rescan the folder",
                                        self.refresh_arch, 'blue', 18))
        ll.addLayout(hdr)

        card = W.Card(self.th, pad=6)
        self.arch_list = QListWidget()
        self.arch_list.itemChanged.connect(self._on_arch_item)
        self.arch_list.setSelectionMode(
            QListWidget.SelectionMode.SingleSelection)
        card.box.addWidget(self.arch_list)
        ll.addWidget(card, 1)
        self.btn_arch_del = self._button("Delete selected", 'destructive',
                                         'trash', self._delete_selected)
        ll.addWidget(self.btn_arch_del)
        body.addWidget(left)

        # ── the chart and its controls ──────────────────────────────────
        rightc = QVBoxLayout()
        rightc.setSpacing(self.th.px(10))
        ccard = W.Card(self.th, pad=12)
        head, _, self.arch_title = self._card_header(
            "Comparison", "drag · scroll · right-click")
        ccard.box.addLayout(head)
        self.chart_a = ChartPanel(self.th, nrows=1,
                                  margins=(0.05, 0.05, 0.995, 0.985))
        self.chart_a.redraw = self._redraw_arch
        self.chart_a.setMinimumHeight(self.th.px(300))
        self.ax_a = self.chart_a.ax
        ccard.box.addWidget(self.chart_a, 1)

        self.arch_settings_lbl = QLabel("")
        self.arch_settings_lbl.setWordWrap(True)
        self.arch_settings_lbl.setStyleSheet(
            f"color: {self.th['label2']}; font-size: {self.th.pt('caption')}pt;")
        ccard.box.addWidget(self.arch_settings_lbl)
        rightc.addWidget(ccard, 1)

        ctl = QHBoxLayout()
        ctl.setSpacing(self.th.px(10))
        self.seg_x = W.Segmented(self.th, ["From start", "File time", "PC clock",
                                           "Ramp start", "At temp"], 0)
        self.seg_x.changed.connect(self._on_xmode_change)
        ctl.addWidget(self.seg_x, 1)
        self.sl_tref = W.Slider(0, 110, 0.5)
        self.sl_tref.setValueF(40.0, silent=True)
        self.sl_tref.setFixedWidth(self.th.px(120))
        self.sl_tref.valueChangedF.connect(self._on_tref)
        self.sl_tref.setEnabled(False)
        self.lbl_tref = W.value_label(self.th, "40.0 °C")
        self.lbl_tref.setFixedWidth(self.th.px(92))
        self.lbl_tref.setEnabled(False)
        ctl.addWidget(self.sl_tref)
        ctl.addWidget(self.lbl_tref)
        rightc.addLayout(ctl)

        ctl2 = QHBoxLayout()
        ctl2.setSpacing(self.th.px(9))
        cap2 = QLabel("CURVES")
        cap2.setObjectName('groupHeader')
        ctl2.addWidget(cap2)
        self.arch_switches = {}
        for key, label in (('temp', 'temp'), ('sa', 'setpoint'),
                           ('st', 'target'), ('t2', 'probe 2'), ('pwm', 'power')):
            sw = W.Switch(self.th, self.arch_show[key].get())
            sw.toggled.connect(
                lambda on, k=key: (self.arch_show[k].set(on), self._redraw_arch()))
            self.arch_switches[key] = sw
            lb = QLabel(label)
            lb.setStyleSheet(f"font-size: {self.th.pt('footnote')}pt;")
            ctl2.addWidget(lb)
            ctl2.addWidget(sw)
        ctl2.addStretch(1)
        rightc.addLayout(ctl2)

        exp = QHBoxLayout()
        exp.setSpacing(self.th.px(8))
        # Δ vs 1st rides with the export row, not with the curve switches: five
        # switches and their labels already fill that row at the smallest
        # window this app allows, and a sixth one clipped the captions.
        lbd = QLabel("Δ vs 1st")
        lbd.setStyleSheet(f"font-size: {self.th.pt('footnote')}pt; "
                          f"color: {self.th['orange']};")
        self.sw_delta = W.Switch(self.th, False)
        self.sw_delta.toggled.connect(
            lambda on: (self.arch_delta.set(on), self._redraw_arch()))
        exp.addWidget(lbd)
        exp.addWidget(self.sw_delta)
        exp.addStretch(1)
        exp.addWidget(self._button("Statistics", 'plain', 'info',
                                   self.show_arch_stats))
        exp.addWidget(self._button("CSV", 'plain', 'export', self.export_arch_csv))
        exp.addWidget(self._button("Image", 'plain', 'share', self.save_arch_chart))
        exp.addWidget(self._button("PDF report", 'tinted', 'doc',
                                   self.export_arch_pdf))
        rightc.addLayout(exp)
        body.addLayout(rightc, 1)
        outer.addLayout(body, 1)

        self.refresh_arch()
        self._redraw_arch()
        return page

    # ────────────────────────────────────────────────────────────────────
    #  TUNING
    # ────────────────────────────────────────────────────────────────────
    def _page_tuning(self):
        page, col = self._scroll_page()

        intro = QLabel(
            "Commissioning is done: the PID grid is calibrated and living in "
            "the board's Flash, the feed-forward model is fixed in firmware, "
            "and the Peltier is soldered in one orientation. What is left here "
            "is what you still need day to day.")
        intro.setWordWrap(True)
        intro.setStyleSheet(f"color: {self.th['label2']}; "
                            f"font-size: {self.th.pt('footnote')}pt;")
        col.addWidget(intro)

        g = W.Group(self.th, "Calibration",
                    footer="Self-tune re-measures the gains for the CURRENT "
                           "setpoint and rate only, and writes them to Flash.")
        self.cal_summary_row = g.add_row(
            "State", W.value_label(self.th, "reading from device…"))
        g.add_row("Grid", self._button("View table", 'plain', 'chevron',
                                       self.show_cal_table),
                  sub="Kp / Ki / Kd per temperature and ramp")
        g.add_row("Self-tune here", self._button("Run", 'tinted', 'play',
                                                 self.do_selftune))
        g.add_row("Full auto-calibration",
                  self._button("Set up", 'plain', 'chevron', self.do_autocal),
                  sub="Relay sweep across the whole range - hours, not minutes")
        col.addWidget(g)

        g2 = W.Group(self.th, "Thermocouple",
                     footer="A per-setup measurement value, not a commissioning "
                            "knob - it stays adjustable.")
        self.sl_off = W.SliderField(g2, self.th, "Calibration offset", -20, 20,
                                    0.0, "°C", 1,
                                    on_change=lambda v: self.send(f"OFFSET:{v:.1f}"))
        col.addWidget(g2)

        g3 = W.Group(self.th, "Peltier polarity",
                     footer="Fixed in the board's Flash and no longer detectable "
                            "at runtime - the wiring is soldered. Changing it "
                            "needs POL_DEFAULT in the firmware.")
        self.pol_row = g3.add_row("Polarity", W.value_label(self.th, "unknown"))
        col.addWidget(g3)

        g4 = W.Group(self.th, "Device flash",
                     footer="Settings held in the board's own memory.")
        row = QWidget()
        rl = QHBoxLayout(row)
        rl.setContentsMargins(self.th.px(12), self.th.px(8), self.th.px(12),
                              self.th.px(12))
        rl.setSpacing(self.th.px(8))
        rl.addWidget(self._button("Save to board", 'tinted', 'export',
                                  lambda: self.send("SAVE")), 1)
        rl.addWidget(self._button("Load from board", 'plain', 'folder',
                                  lambda: self.send("LOAD")), 1)
        g4.add_widget(row, sep=False)
        col.addWidget(g4)

        g5 = W.Group(self.th, "Calibration backup on this PC",
                     footer="Auto-loaded onto the board on every connection.")
        row2 = QWidget()
        rl2 = QHBoxLayout(row2)
        rl2.setContentsMargins(self.th.px(12), self.th.px(8), self.th.px(12),
                               self.th.px(12))
        rl2.setSpacing(self.th.px(8))
        rl2.addWidget(self._button("Back up", 'tinted', 'export',
                                   lambda: self.dump_calibration_to_pc(False)), 1)
        rl2.addWidget(self._button("Restore", 'plain', 'folder',
                                   self._manual_load_cal), 1)
        g5.add_widget(row2, sep=False)
        col.addWidget(g5)

        g6 = W.Group(self.th, "Reset",
                     footer="Clears every profile and the calibration on the "
                            "board. The PC backup above is not touched.")
        g6.add_row("Restore factory settings",
                   self._button("Reset", 'destructive', 'refresh', self.do_reset))
        col.addWidget(g6)
        col.addStretch(1)
        return page

    # ────────────────────────────────────────────────────────────────────
    #  DEVICE
    # ────────────────────────────────────────────────────────────────────
    def _page_device(self):
        page, col = self._scroll_page()

        g = W.Group(self.th, "Serial connection")
        self.conn_list = QListWidget()
        self.conn_list.setMinimumHeight(self.th.px(130))
        self.conn_list.itemDoubleClicked.connect(lambda _: self.conn_from_tab())
        g.add_widget(self.conn_list, sep=False)
        row = QWidget()
        rl = QHBoxLayout(row)
        rl.setContentsMargins(self.th.px(12), self.th.px(8), self.th.px(12),
                              self.th.px(12))
        rl.setSpacing(self.th.px(8))
        rl.addWidget(self._button("Refresh", 'plain', 'refresh',
                                  self.refresh_ports))
        rl.addStretch(1)
        rl.addWidget(self._button("Disconnect", 'destructive', '',
                                  self.disconnect))
        rl.addWidget(self._button("Connect", 'filled', 'link',
                                  self.conn_from_tab))
        g.add_widget(row)
        col.addWidget(g)

        g2 = W.Group(self.th, "Versions")
        self.fw_row = g2.add_row("Firmware", W.value_label(self.th, "—"))
        g2.add_row("Application", W.value_label(self.th, APP_BUILD))
        self.diag_row = g2.add_row(
            "Diagnostics", self._button("Open", 'plain', 'chevron',
                                        self.open_diag_window),
            sub="Everything the board reports over the link")
        col.addWidget(g2)

        g3 = W.Group(self.th, "Appearance",
                     footer="Text size changes immediately; spacing is measured "
                            "again the next time the app starts.")
        self.seg_appearance = W.Segmented(
            self.th, ["Light", "Dark"], 1 if self.th.dark else 0)
        self.seg_appearance.changed.connect(
            lambda i: self.set_appearance('dark' if i else 'light'))
        g3.add_row("Theme", self.seg_appearance)
        scales = [0.85, 1.0, 1.15, 1.3, 1.5]
        idx = min(range(len(scales)),
                  key=lambda i: abs(scales[i] - self.th.scale))
        self.seg_scale = W.Segmented(self.th, ["85%", "100%", "115%", "130%",
                                               "150%"], idx)
        self.seg_scale.changed.connect(lambda i: self.set_ui_scale(scales[i]))
        g3.add_row("Text size", self.seg_scale)
        col.addWidget(g3)

        g4 = W.Group(self.th, "Getting started")
        for i, line in enumerate((
                "Connect the ItsyBitsy (firmware v19 PC MODE or newer) over USB",
                "Pick the port above and connect - it also happens automatically",
                "The controls sync themselves with the board",
                "Set the target and the rates, then press Start on Control",
                "The chart is live and every sample is written to CSV")):
            g4.add_row(f"{i + 1}.", W.value_label(self.th, ""), sub=line)
        col.addWidget(g4)
        col.addStretch(1)

        self.refresh_ports()
        return page

    # ════════════════════════════════════════════════════════════════════
    #  APPEARANCE
    # ════════════════════════════════════════════════════════════════════
    def toggle_appearance(self):
        self.set_appearance('light' if self.th.dark else 'dark')
        if hasattr(self, 'seg_appearance'):
            self.seg_appearance.setIndex(1 if self.th.dark else 0, emit=False)

    def set_appearance(self, mode):
        """Flip light/dark without rebuilding a single widget.

        Everything that can be styled declaratively reads the stylesheet, and
        every custom-painted widget reads the palette live at paint time - so
        re-emitting the QSS and repainting is the whole operation. The charts
        need one extra step, because matplotlib holds its colours in the
        artists it has already created."""
        self.th.mode = mode
        self._save_setting('appearance', mode)
        icons.clear_cache()
        QApplication.instance().setStyleSheet(self.th.qss())
        self._apply_wordmark()
        self.header.appearance.setIcon(
            icons.icon('sun' if self.th.dark else 'moon', self.th.px(22),
                       self.th['blue']))
        self._retint_icons()
        for ch in (self.chart, self.chart_a):
            ch.retheme(self.th)
        self.setWindowIcon(QIcon(brand.medallion_pixmap(
            64, self.th['blue'], 'transparent')))
        for w in self.findChildren(QWidget):
            w.update()

    def set_ui_scale(self, scale):
        self.th.scale = float(scale)
        self._save_setting('ui_scale', round(float(scale), 3))
        QApplication.instance().setStyleSheet(self.th.qss())
        self._apply_wordmark()
        for w in self.findChildren(QWidget):
            w.updateGeometry()
            w.update()

    def _retint_icons(self):
        for b, name, kind, size in self._icon_btns:
            b.setIcon(icons.icon(name, self.th.px(size), self._tint(kind)))
        self._refresh_diag_indicator()

    # ════════════════════════════════════════════════════════════════════
    #  SETTINGS FILE
    # ════════════════════════════════════════════════════════════════════
    def _load_setting(self, key, default=None):
        try:
            if self.settings_file.exists():
                with open(self.settings_file, 'r', encoding='utf-8') as f:
                    return json.load(f).get(key, default)
        except Exception:
            pass
        return default

    def _save_setting(self, key, value):
        d = {}
        try:
            if self.settings_file.exists():
                with open(self.settings_file, 'r', encoding='utf-8') as f:
                    d = json.load(f)
        except Exception:
            d = {}
        d[key] = value
        try:
            self.cfg_dir.mkdir(parents=True, exist_ok=True)
            with open(self.settings_file, 'w', encoding='utf-8') as f:
                json.dump(d, f, indent=2)
        except Exception as e:
            print(f"settings not saved: {e}")

    def _load_data_dir(self):
        """The remembered data folder; if it is missing or unavailable, the
        default. We do NOT force-create a remembered path - if the user
        unplugged the drive, falling back quietly beats refusing to start."""
        p = self._load_setting('data_dir')
        if p:
            q = Path(p)
            if q.is_dir():
                return q
            try:
                q.mkdir(parents=True, exist_ok=True)
                return q
            except Exception:
                print(f"Data folder '{q}' unavailable - using {self.cfg_dir}")
        self.cfg_dir.mkdir(exist_ok=True)
        return self.cfg_dir

    # ════════════════════════════════════════════════════════════════════
    #  SERIAL
    # ════════════════════════════════════════════════════════════════════
    def send(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + '\n').encode())
            except Exception as e:
                print(f"send err: {e}")

    def _auto_connect(self):
        """Find the ItsyBitsy and connect to it without being asked."""
        if self.connected:
            return
        try:
            ports = list(serial.tools.list_ports.comports())
        except Exception:
            return
        if not ports:
            return

        def score(p):
            d = (p.description or '').lower()
            m = (p.manufacturer or '').lower() if hasattr(p, 'manufacturer') else ''
            s = 0
            for kw in ('itsybitsy', 'adafruit', 'usb serial', 'usb-serial',
                       'circuitpython'):
                if kw in d or kw in m:
                    s += 10
            if getattr(p, 'vid', None) == 0x239A:      # Adafruit
                s += 20
            return s

        best = max(ports, key=score)
        if score(best) > 0 or len(ports) == 1:
            self.connect(best.device)

    def connect(self, port):
        try:
            self.ser = serial.Serial(port, self.baud, timeout=0.5)
            self.port_name = port
            self.clear_buf()
            self._cfg_synced = False      # allow one slider synchronisation
            self.set_status(True, f"{port}")
            self.running = True
            threading.Thread(target=self.reader, daemon=True).start()
            # VER answers immediately, so it works even when the board has been
            # powered for a long time - unlike BUILD:, which is sent once in
            # setup() and which the app misses if it connects afterwards.
            self.after(1500, lambda: self.send("GET"))
            self.after(1600, lambda: self.send("VER"))
            self.after(2200, self._auto_load_calibration)
        except Exception as e:
            core.error(self, "Cannot open the port", f"{port}\n\n{e}")
            self.set_status(False, "")

    def disconnect(self):
        self.running = False
        if self.cyc_on:
            self.cyc_stop("disconnected")
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.set_status(False, "")
        self.dev_fw_build = None
        if hasattr(self, 'fw_row'):
            self.fw_row.right.setText("—")

    def clear_buf(self):
        for a in (self.t, self.temp, self.spt, self.spa, self.pwm,
                  self.kp, self.ki, self.kd, self.states):
            a.clear()
        self.t0 = None

    def reader(self):
        """Serial reader thread - parses telemetry and every protocol line."""
        if self.ser and self.ser.is_open:
            self.ser.reset_input_buffer()
        while self.running:
            try:
                if not self.ser or not self.ser.is_open:
                    break
                raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not raw:
                    continue

                if raw.startswith("CFG:"):
                    self._parse_cfg(raw[4:])
                    continue
                if raw.startswith("CALPLAN:"):
                    self._parse_calplan(raw[8:])
                    continue
                if raw.startswith("CALDUMP:"):
                    self._caldump_buf = []
                    self._caldump_active = True
                    continue
                if raw.startswith("PROF:") and self._caldump_active:
                    self._caldump_buf.append(raw[5:])
                    continue
                if raw == "CALDUMPEND":
                    self._caldump_active = False
                    self.call(self._finish_caldump_save)
                    continue
                if raw.startswith("CALSTAT:"):
                    self._parse_calstat(raw[8:])
                    continue
                if raw.startswith("CALWARN:"):
                    self._parse_calwarn(raw[8:])
                    continue
                if raw.startswith("ERR:"):
                    self._parse_err(raw[4:])
                    continue
                if raw.startswith("BUILD:"):
                    b = raw[6:].strip()
                    self.call(lambda b=b: self._set_fw_build(b))
                    continue

                # CSV telemetry: 9 fields, plus optional extras
                p = raw.split(',')
                is_csv = len(p) >= 9
                if is_csv:
                    try:
                        float(p[0])
                    except ValueError:
                        is_csv = False
                if not is_csv:
                    # Not telemetry and not a known prefix. Instead of dropping
                    # it silently, it goes to Diagnostics - so the app shows
                    # EVERYTHING the firmware says, which a Serial Monitor
                    # cannot do while the app holds the port.
                    low = raw.upper()
                    lvl = 'WARN' if any(k in low for k in
                                        ('FAIL', 'ERROR', '!!!', 'BLAD', 'BŁĄD')) \
                        else 'INFO'
                    self._log_diag(lvl, raw)
                    continue
                try:
                    d = dict(temp=float(p[1]), sa=float(p[2]), st=float(p[3]),
                             pwm=int(p[4]), kp=float(p[5]), ki=float(p[6]),
                             kd=float(p[7]), state=p[8].strip())
                except Exception:
                    continue

                d['temp2'] = None
                if len(p) >= 10:
                    try:
                        v2 = float(p[9])
                        d['temp2'] = v2 if v2 != 0 else None   # 0 = none/error
                    except Exception:
                        pass
                self._latest_temp2 = d['temp2']

                # The PID breakdown (fields 11-17, AUTO only): FF, P, I, D, the
                # raw result before clamping and slew, the applied reactScale,
                # and the estimated far-side temperature. It goes into the run
                # archive so that diagnosing oscillation or lag can rely on
                # numbers rather than on guessing from the temp/PWM trace.
                d['dbg'] = None
                if len(p) >= 16:
                    try:
                        d['dbg'] = dict(
                            ff=float(p[10]), p=float(p[11]), i=float(p[12]),
                            dd=float(p[13]), raw=float(p[14]), react=float(p[15]),
                            amb=(float(p[16]) if len(p) >= 17 else None))
                    except Exception:
                        pass

                # Time comes from the FIRMWARE, not from the PC clock: the
                # computer's clock drifted while the queue buffered and
                # understated the achieved rate.
                try:
                    fw_time = float(p[0])
                except Exception:
                    fw_time = 0
                if self.t0 is None:
                    self.t0 = fw_time
                now = fw_time - self.t0
                state = d['state']

                if self.cyc_on and state in ('AUTO', 'COOLDOWN', 'FREEZE',
                                             'FREEZE_READY'):
                    self.cyc_log(time.time() - self.cyc_t0 if self.cyc_t0 else 0,
                                 d['temp'], d['sa'], d['st'], d['pwm'],
                                 d['kp'], d['ki'], d['kd'], state,
                                 d.get('temp2'), d.get('dbg'))

                prev = self.last_state
                self.last_state = state
                self.cur_state = state
                if state.startswith('ST') or state.startswith('CAL'):
                    self._st_pid_update = (d['kp'], d['ki'], d['kd'])
                if self.cal_running and 'CAL' in prev and state == 'MAN':
                    self.cal_running = False
                    self.cal_current = self.cal_total
                    self.call(self._cal_finished)
                self.data_queue.put((now, d['temp'], d['st'], d['sa'],
                                     d['pwm'] * 100 / 255, d['kp'], d['ki'],
                                     d['kd'], state, prev))

            except serial.SerialException:
                self.running = False
                self.call(lambda: self.set_status(False, "connection lost"))
                break
            except Exception as e:
                if self.running:
                    print(f"reader err: {e}")
                time.sleep(0.3)

    # ── protocol parsing ────────────────────────────────────────────────
    def _parse_cfg(self, cfg):
        d = {}
        for part in cfg.split(','):
            if '=' in part:
                k, v = part.split('=', 1)
                d[k.strip()] = v.strip()
        self.call(lambda: self._apply_cfg(d))

    def _apply_cfg(self, d):
        try:
            # Sync the controls ONLY on the first CFG after connecting.
            # Afterwards the user's own settings must stand - a CFG arriving
            # after a STOP would otherwise quietly undo them.
            if not getattr(self, '_cfg_synced', False):
                for key, attr in (('SP', 'sl_sp'), ('RU', 'sl_ru'),
                                  ('RD', 'sl_rd'), ('TMAX', 'sl_tmax'),
                                  ('OFFSET', 'sl_off')):
                    if key in d and hasattr(self, attr):
                        getattr(self, attr).set(float(d[key]))
                self._cfg_synced = True
            if 'CAL' in d:
                self.dev_cal = (d['CAL'] == '1')
                self._update_cal_summary()
            if 'STATE' in d:
                self.cur_state = d['STATE']
            if 'POL' in d:
                self.dev_pol_swapped = (d['POL'] == '1')
            if 'POLSET' in d:
                self.dev_pol_set = (d['POLSET'] == '1')
            if 'CALMIN' in d:
                self.dev_cal_min = float(d['CALMIN'])
            if 'CALMAX' in d:
                self.dev_cal_max = float(d['CALMAX'])
            if 'FAN' in d:
                fan_val = int(float(d['FAN']))
                self.fan_on = fan_val > 0
                if fan_val > 0 and hasattr(self, 'sl_fan'):
                    self.sl_fan.set(fan_val)
                if hasattr(self, 'sw_fan'):
                    self.sw_fan.setCheckedSilently(self.fan_on)
            self._update_pol_indicator()
        except Exception as e:
            print(f"apply_cfg err: {e}")

    def _parse_calplan(self, txt):
        """CALPLAN:9,temps=20/30/…,ramps=relay - build the step list. In relay
        mode there is one test per temperature, not a temp x ramp grid."""
        try:
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
            plan = ([(t, 'relay') for t in temps] if relay_mode
                    else [(t, r) for t in temps for r in ramps])
            self.cal_plan = plan
            self.cal_total = total or len(plan)
            self.cal_current = 0
            self.cal_phase = None
            self.cal_running = True
            self.cal_t0 = time.time()
            self.cal_step_times = []
            self.cal_warnings = []
            self.cal_ramp_warnings = []
            self.call(self._refresh_cal_view)
        except Exception as e:
            print(f"calplan err: {e}")

    def _parse_calstat(self, txt):
        """CALSTAT:5/24,T=40,R=2 - progress."""
        try:
            parts = txt.split(',')
            cur, tot = parts[0].split('/')
            new_current = int(cur)
            self.cal_total = int(tot)
            for part in parts[1:]:
                if part.startswith('T='):
                    self.cal_cur_temp = float(part[2:])
                elif part.startswith('R='):
                    rv = part[2:].strip()
                    # In relay mode R= carries the step PHASE, not a ramp.
                    if rv in ('heating', 'stabil', 'relay'):
                        self.cal_phase = rv
                        self.cal_cur_ramp = 'relay'
                    elif rv.startswith('rampprep:') or rv.startswith('ramptest:'):
                        key, _, rate = rv.partition(':')
                        self.cal_phase = key
                        try:
                            self.cal_cur_ramp = float(rate)
                        except Exception:
                            self.cal_cur_ramp = rate
                    else:
                        self.cal_phase = None
                        try:
                            self.cal_cur_ramp = float(rv)
                        except Exception:
                            self.cal_cur_ramp = rv
            if new_current != self.cal_current:
                if self.cal_t0:
                    self.cal_step_times.append(time.time())
                self.cal_current = new_current
            self.cal_running = True
            self.call(self._refresh_cal_view)
        except Exception as e:
            print(f"calstat err: {e}")

    def _parse_calwarn(self, txt):
        """Two different warnings share the CALWARN message.

        1) T=90,cycles=1,amp=140,relay_fail - the relay test for that
        temperature did not catch oscillation and the firmware wrote BASE
        values instead of measured ones. amp is the excitation it gave up at;
        at the maximum (140) even the strongest gentle push failed to cross the
        setpoint both ways, which is a physical limit, not a matter of time.

        2) T=50,R=20,err=2.34,ramp_track_fail - the RAMPING test for one rate
        (after a successful relay) never got below the tracking threshold. That
        single cell keeps the relay profile, which is still a real measurement,
        so it must NOT mark the whole temperature as failed - hence a separate
        list."""
        try:
            d = {}
            for part in txt.split(','):
                if '=' in part:
                    k, v = part.split('=', 1)
                    d[k.strip()] = v.strip()
            temp = float(d.get('T', 'nan'))
            if temp != temp:                      # NaN
                return
            if 'R' in d and 'err' in d:
                try:
                    ramp = float(d['R'])
                except Exception:
                    ramp = None
                try:
                    err = float(d['err'])
                except Exception:
                    err = None
                self.cal_ramp_warnings.append((temp, ramp, err))
                self._log_diag('WARN', f"Calibration: ramp test {ramp} °C/min "
                                       f"@ {temp} °C did not keep up with the "
                                       f"active setpoint (err={err} °C)")
            else:
                cycles = int(d.get('cycles', '0'))
                amp = int(d['amp']) if 'amp' in d else None
                self.cal_warnings.append((temp, cycles, amp))
                self._log_diag('WARN', f"Calibration: relay test @ {temp} °C "
                                       f"caught no oscillation (cycles={cycles}, "
                                       f"amp={amp}) - base values used")
            self.call(self._refresh_cal_view)
        except Exception as e:
            print(f"calwarn err: {e}")

    def _parse_err(self, txt):
        """ERR:code=N,…,active=0/1 - a hardware or safety code. The firmware
        sends it only on an edge, once when it appears and once when it clears,
        so there is no risk of flooding the link."""
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
                try:
                    detail = " - " + decode_tc_fault(int(d['bits'], 16))
                except Exception:
                    pass
            elif code == 2 and 'val' in d:
                detail = f" - reading {d['val']} °C"
            elif code == 3:
                detail = (f" - temp {d.get('temp', '?')} °C, "
                          f"limit {d.get('limit', '?')} °C")
            text = base + detail
            if active:
                self.err_active[code] = text
                self._log_diag('ERR', text)
            else:
                self.err_active.pop(code, None)
                self._log_diag('INFO', f"CLEARED: {base}")
        except Exception as e:
            print(f"err parse err: {e}")

    # ── diagnostics plumbing ────────────────────────────────────────────
    def _log_diag(self, level, text):
        entry = (time.time(), level, text)
        self.diag_log.append(entry)
        if len(self.diag_log) > 500:
            del self.diag_log[:-500]
        if level in ('ERR', 'WARN'):
            self.diag_unseen += 1
        self.call(self._refresh_diag_indicator)
        if self.diag_win is not None:
            self.call(lambda e=entry: self.diag_win.append_entry(e))

    def _refresh_diag_indicator(self):
        if not hasattr(self, 'btn_diag'):
            return
        if self.err_active:
            col, icon = 'red', 'warning'
        elif self.diag_unseen > 0:
            col, icon = 'orange', 'warning'
        else:
            col, icon = 'label2', 'info'
        self._reicon(self.btn_diag, icon, col)
        self.btn_diag.setToolTip(
            f"Diagnostics — {len(self.err_active)} active alarm(s)"
            if self.err_active else
            f"Diagnostics — {self.diag_unseen} new" if self.diag_unseen
            else "Diagnostics")

    def _set_fw_build(self, build):
        self.dev_fw_build = build
        if hasattr(self, 'fw_row'):
            self.fw_row.right.setText(build)
            self.fw_row.right.setStyleSheet(f"color: {self.th['green']};")
        self._log_diag('INFO', f"Connected - firmware build {build}")

    def open_diag_window(self):
        self.diag_unseen = 0
        self._refresh_diag_indicator()
        if self.diag_win is not None:
            self.diag_win.raise_()
            return
        self.diag_win = DiagnosticsDialog(self, self.th, self)
        self.diag_win.show()

    # ════════════════════════════════════════════════════════════════════
    #  CALIBRATION
    # ════════════════════════════════════════════════════════════════════
    def _cal_step_stats(self):
        """(avg_step_s, elapsed_in_current_step_s). avg_step is None until at
        least one step has finished."""
        times = self.cal_step_times
        now = time.time()
        avg_step = None
        if len(times) >= 2:
            durations = [times[i] - times[i - 1] for i in range(1, len(times))]
            avg_step = sum(durations) / len(durations)
        if times:
            elapsed = now - times[-1]
        elif self.cal_t0:
            elapsed = now - self.cal_t0
        else:
            elapsed = 0
        return avg_step, elapsed

    def _cal_eta(self):
        """Seconds remaining, or None when there is not enough data yet. Zero
        ONLY when the calibration has really finished - cur == total used to
        give zero while the last point had merely started."""
        if not self.cal_t0 or self.cal_total < 1:
            return None
        if not self.cal_running:
            return 0
        avg_step, elapsed = self._cal_step_stats()
        if avg_step is None:
            if self.cal_current < 1:
                return None
            avg_step = (time.time() - self.cal_t0) / self.cal_current
        remaining = (max(0, self.cal_total - self.cal_current) * avg_step
                     + max(0, avg_step - elapsed))
        return max(0, remaining)

    def _cal_progress_fraction(self):
        """Grows smoothly through the current step instead of jumping to 100%
        the moment the last point starts."""
        if not self.cal_total:
            return 0.0
        if not self.cal_running and self.cal_current >= self.cal_total:
            return 1.0
        avg_step, elapsed = self._cal_step_stats()
        completed = max(0, self.cal_current - 1)
        step_frac = min(0.95, elapsed / avg_step) if avg_step else 0.0
        return min(1.0, (completed + step_frac) / self.cal_total)

    def _cal_finished(self):
        self._refresh_cal_view()
        self.cal_status_row.right.setText("done - saving to PC…")
        self.dev_cal = True
        self.send("GET")
        self.after(800, lambda: self.dump_calibration_to_pc(silent=False))

    def _refresh_cal_view(self):
        if self.cal_running and self.cal_total > 0:
            eta = self._cal_eta()
            eta_s = f" · ~{int(eta // 60)} min" if eta else ""
            self.cal_status_row.right.setText(
                f"{self.cal_current}/{self.cal_total}{eta_s}")
        elif self.cal_total and self.cal_current >= self.cal_total:
            self.cal_status_row.right.setText("done")
        if self.cal_win:
            try:
                self.cal_win.refresh()
            except Exception:
                pass
        # The firmware IGNORES SP/RU/RD while calibrating (sys == CAL), so an
        # accidental drag cannot disturb a relay measurement. Previously the
        # controls stayed live and silently did nothing, which looked like the
        # target had changed. Now they are visibly dead for the duration.
        for sl in ('sl_sp', 'sl_ru', 'sl_rd'):
            if hasattr(self, sl):
                getattr(self, sl).set_enabled(not self.cal_running)

    def _manual_load_cal(self):
        if not self.connected:
            core.warn(self, "Not connected", "Connect to the device first.")
            return
        if not self.cal_file.exists():
            core.info(self, "No backup",
                      "No saved calibration on this PC. Run a calibration "
                      "first, or back one up from the board.")
            return
        try:
            with open(self.cal_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            saved = data.get('saved', '?')
            nvalid = sum(1 for p in data.get('profiles', []) if p.get('valid'))
        except Exception:
            saved, nvalid = '?', 0
        if core.ask(self, "Restore calibration to the board?",
                    f"Saved: {saved}\nProfiles: {nvalid}\n\n"
                    "This overwrites what the board currently holds."):
            self.load_calibration_from_pc()

    def show_cal_table(self):
        if not self.connected:
            core.warn(self, "Not connected", "Connect to the device first.")
            return
        self._caldump_buf = []
        self._caldump_active = False
        self._caldump_purpose = 'view'
        self.send("DUMPCAL")

    # The firmware writes these when a relay test fails, so a cell landing
    # exactly on them is almost certainly a fallback, not a measurement.
    _CAL_BASE_KP, _CAL_BASE_KI, _CAL_BASE_KD = 10.0, 0.3, 0.8

    def _is_base_profile(self, p):
        try:
            return (abs(p['KpH'] - self._CAL_BASE_KP) < 0.05 and
                    abs(p['KiH'] - self._CAL_BASE_KI) < 0.01 and
                    abs(p['KdH'] - self._CAL_BASE_KD) < 0.01)
        except Exception:
            return False

    def dump_calibration_to_pc(self, silent=True):
        if not self.connected:
            core.warn(self, "Not connected", "Connect to the device first.")
            return
        self._caldump_purpose = 'save'
        try:
            self._pending_offset = self.sl_off.get()
        except Exception:
            self._pending_offset = 0.0
        self.send("DUMPCAL")

    def _finish_caldump_save(self):
        try:
            profiles = []
            for line in self._caldump_buf:
                parts = line.split(',')
                if len(parts) >= 8:
                    profiles.append({
                        'idx': int(parts[0]),
                        'KpH': float(parts[1]), 'KiH': float(parts[2]),
                        'KdH': float(parts[3]), 'KpC': float(parts[4]),
                        'KiC': float(parts[5]), 'KdC': float(parts[6]),
                        'valid': parts[7].strip() == '1'})
            data = {'version': 1,
                    'saved': datetime.now().isoformat(timespec='seconds'),
                    'offset': self._pending_offset or 0.0,
                    'profiles': profiles}
            with open(self.cal_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2)
            n_valid = sum(1 for p in profiles if p['valid'])
            if self._caldump_purpose == 'save':
                core.info(self, "Calibration backed up",
                          f"{n_valid} profiles and the offset were written to\n"
                          f"{self.cal_file}\n\n"
                          "They are restored automatically on the next connection.")
            elif self._caldump_purpose == 'view':
                CalTableDialog(self, self.th, profiles,
                               self._is_base_profile).exec()
        except Exception as e:
            print(f"calibration save error: {e}")
        self._caldump_purpose = None

    def _auto_load_calibration(self):
        if self.connected and self.cal_file.exists():
            self.load_calibration_from_pc()

    def load_calibration_from_pc(self):
        if not self.cal_file.exists():
            return False
        try:
            with open(self.cal_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
            profiles = data.get('profiles', [])
            if not profiles:
                return False
            self.send(f"OFFSET:{data.get('offset', 0.0):.1f}")

            def send_profiles(i=0):
                if i >= len(profiles):
                    self.send("SETCALDONE:1")
                    self.dev_cal = True
                    self._update_cal_summary()
                    return
                p = profiles[i]
                self.send(f"SETPROF:{p['idx']},{p['KpH']:.3f},{p['KiH']:.4f},"
                          f"{p['KdH']:.3f},{p['KpC']:.3f},{p['KiC']:.4f},"
                          f"{p['KdC']:.3f},{1 if p['valid'] else 0}")
                self.after(40, lambda: send_profiles(i + 1))

            send_profiles(0)
            return True
        except Exception as e:
            print(f"calibration load error: {e}")
            return False

    def do_autocal(self):
        if not self.connected:
            core.warn(self, "Not connected", "Connect to the device first.")
            return
        CalRangeDialog(self, self.th, self).exec()

    def start_autocal(self, temp_min, temp_max, ramps):
        self.send(f"CALRANGE:{temp_min:.0f},{temp_max:.0f}")
        time.sleep(0.1)
        # CRUCIAL - this is what defines the ramps the calibration will cover.
        self.send("SETCALRAMPS:" + ",".join(f"{r:.0f}" for r in ramps))
        time.sleep(0.1)
        self.cal_running = True
        self.cal_t0 = time.time()
        self.cal_current = 0
        self.send("AUTOCAL")
        self.cal_status_row.right.setText("starting…")
        self.after(600, self.open_cal_window)

    def open_cal_window(self):
        if not self.cal_plan and not self.cal_running:
            core.info(self, "Not calibrating",
                      "Start a calibration from the Tuning screen.")
            return
        if self.cal_win:
            self.cal_win.raise_()
            return
        self.cal_win = CalibrationDialog(self, self.th, self)
        self.cal_win.show()

    def _update_cal_summary(self):
        """One line saying whether the board is calibrated. What matters day to
        day is not the individual gains, it is whether the grid in Flash is
        valid."""
        if not hasattr(self, 'cal_summary_row'):
            return
        if self.dev_cal:
            self.cal_summary_row.right.setText("calibrated")
            self.cal_summary_row.right.setStyleSheet(f"color: {self.th['green']};")
            self.cal_status_row.right.setText("calibrated")
        else:
            self.cal_summary_row.right.setText("base gains")
            self.cal_summary_row.right.setStyleSheet(f"color: {self.th['orange']};")
            self.cal_status_row.right.setText("not calibrated")

    def _update_pol_indicator(self):
        if not hasattr(self, 'pol_row'):
            return
        if self.dev_pol_set:
            txt = "swapped" if self.dev_pol_swapped else "normal"
            col = self.th['orange'] if self.dev_pol_swapped else self.th['green']
        else:
            txt, col = "not set", self.th['label3']
        self.pol_row.right.setText(txt)
        self.pol_row.right.setStyleSheet(f"color: {col};")

    # ════════════════════════════════════════════════════════════════════
    #  STATUS AND RUN CONTROL
    # ════════════════════════════════════════════════════════════════════
    def set_status(self, connected, msg):
        self.connected = connected
        if connected:
            self.header.chip.set((msg or "CONNECTED").upper(), 'green')
        else:
            self.header.chip.set((msg or "OFFLINE").upper(), 'red')

    def toggle_run(self):
        self.do_stop() if self.is_running else self.do_start()

    def _update_run_button(self, running):
        self.is_running = running
        if not hasattr(self, 'btn_run'):
            return
        self.btn_run.setText("Stop" if running else "Start")
        self.btn_run.setProperty('kind', 'stop' if running else 'go')
        self._reicon(self.btn_run, 'stop' if running else 'play',
                     'stop' if running else 'go')
        self._restyle(self.btn_run)

    def do_start(self):
        if not self.connected:
            core.warn(self, "Not connected", "Connect to the device first.")
            return
        # Every Start counts a fresh average from zero, so runs can go back to
        # back without stale numbers on the cards.
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
        self.reach_lbl.setText("starting…")

        self.send(f"SP:{self.sl_sp.get():.1f}")
        self.send(f"RU:{self.sl_ru.get():.1f}")
        self.send(f"RD:{self.sl_rd.get():.1f}")
        self.send(f"TMAX:{self.sl_tmax.get():.0f}")
        # Kp/Ki/Kd are deliberately NOT pushed from here. They used to be sent
        # from the manual sliders on every Start, which quietly overwrote what
        # the board had just interpolated from its calibration grid for this
        # setpoint and rate - which is the whole point of having calibrated it.
        self.send(f"OFFSET:{self.sl_off.get():.1f}")
        time.sleep(0.05)
        self.send("START")
        self._update_run_button(True)

    def do_stop(self):
        # A manual stop also aborts an automatic series, otherwise the app
        # would "resurrect" it with the next step in the middle of a manual
        # intervention.
        if self.series_running:
            self._series_abort("manual stop")
        self.send("STOP")
        self.send("AUTOCALSTOP")
        self._update_run_button(False)

    def do_estop(self):
        self.send("ESTOP")
        self.send("AUTOCALSTOP")
        self._update_run_button(False)

    def on_fan_toggle(self, on):
        if not self.connected:
            core.warn(self, "Not connected", "Connect to the device first.")
            self.sw_fan.setCheckedSilently(False)
            return
        self.fan_on = on
        if on:
            spd = int(self.sl_fan.get()) or 100
            if spd != self.sl_fan.get():
                self.sl_fan.set(100)
            self.send(f"FAN:{spd}")
        else:
            self.send("FANOFF")

    def set_fan_speed(self, v):
        spd = int(v)
        self.send(f"FAN:{spd}")
        self.fan_on = spd > 0
        if hasattr(self, 'sw_fan'):
            self.sw_fan.setCheckedSilently(self.fan_on)

    def do_freeze(self):
        if not self.connected:
            core.warn(self, "Not connected", "Connect to the device first.")
            return
        if core.ask(self, "Freeze the stage?",
                    "Ramps gently down to 20 °C and HOLDS it there, keeping "
                    "the cooling active so it cannot re-melt.\n\n"
                    "You will see 'solid' when it is ready. Press Stop when "
                    "the sample has been swapped."):
            self.send("FREEZE")
            self.reach_lbl.setText("freezing…")

    def do_reset(self):
        if not self.connected:
            core.warn(self, "Not connected", "Connect to the device first.")
            return
        if core.ask(self, "Restore factory settings?",
                    "This clears every profile and the calibration held on the "
                    "board."):
            self.send("RESET")

    def do_selftune(self):
        if not self.connected:
            core.warn(self, "Not connected", "Connect to the device first.")
            return
        if core.ask(self, "Start self-tune?",
                    "Takes about two minutes and needs the device running."):
            self.send("SELFTUNE")

    # ── presets ─────────────────────────────────────────────────────────
    def _gather_settings(self):
        s = {}
        for key, attr in (('sp', 'sl_sp'), ('ru', 'sl_ru'), ('rd', 'sl_rd'),
                          ('tmax', 'sl_tmax'), ('off', 'sl_off'),
                          ('fan', 'sl_fan')):
            if hasattr(self, attr):
                try:
                    s[key] = getattr(self, attr).get()
                except Exception:
                    pass
        return s

    def _load_presets(self):
        if not self.presets_file.exists():
            return {}
        try:
            with open(self.presets_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception:
            return {}

    def _save_presets(self, presets):
        try:
            with open(self.presets_file, 'w', encoding='utf-8') as f:
                json.dump(presets, f, indent=2)
            return True
        except Exception as e:
            print(f"presets save err: {e}")
            return False

    def open_presets(self):
        PresetDialog(self, self.th, self).exec()

    def open_profiles(self):
        ProfileDialog(self, self.th, self).exec()

    def apply_preset(self, settings):
        for key, attr, cmd, dec in (
                ('sp', 'sl_sp', 'SP', 1), ('ru', 'sl_ru', 'RU', 1),
                ('rd', 'sl_rd', 'RD', 1), ('tmax', 'sl_tmax', 'TMAX', 0),
                ('off', 'sl_off', 'OFFSET', 1), ('fan', 'sl_fan', 'FAN', 0)):
            if key in settings and hasattr(self, attr):
                val = settings[key]
                try:
                    getattr(self, attr).set(val)
                    if self.connected:
                        self.send(f"{cmd}:{val:.{dec}f}")
                except Exception as e:
                    print(f"apply preset {key}: {e}")
        if 'fan' in settings and hasattr(self, 'sw_fan'):
            self.fan_on = settings['fan'] > 0
            self.sw_fan.setCheckedSilently(self.fan_on)

    # ════════════════════════════════════════════════════════════════════
    #  PORTS
    # ════════════════════════════════════════════════════════════════════
    def refresh_ports(self):
        self.conn_list.clear()
        self._ports = list(serial.tools.list_ports.comports())
        for p in self._ports:
            it = QListWidgetItem(f"{p.device}     {p.description or '?'}")
            it.setIcon(icons.icon('link', self.th.px(18),
                                  self.th.solid('label2', 'card')))
            self.conn_list.addItem(it)
        if self._ports:
            self.conn_list.setCurrentRow(0)

    def conn_from_tab(self):
        row = self.conn_list.currentRow()
        if 0 <= row < len(getattr(self, '_ports', [])):
            self.connect(self._ports[row].device)

    # ════════════════════════════════════════════════════════════════════
    #  TICK - the heartbeat of the whole app
    # ════════════════════════════════════════════════════════════════════
    # ── SLEEP / STALL GUARD + HEARTBEAT ─────────────────────────────────
    # Two halves of the same safety story, one on each side of the USB cable.
    #
    # PC SIDE (here): tick() runs every ~250 ms. If the wall clock jumps by far
    # more than that, this process was NOT running - the machine slept,
    # hibernated, or froze hard enough that a measurement was no longer being
    # supervised or logged. The safe thing is to end the run, not to carry on
    # with a hole in the data.
    #
    # BOARD SIDE: the app arms the firmware watchdog (PCWD:1) when a run starts
    # and pings it every second. If the PC vanishes, the board stops itself
    # after PC_TIMEOUT_MS - the case this half cannot cover, because a sleeping
    # PC runs no code at all.
    def _guard_tick(self):
        now = time.time()
        last = getattr(self, '_last_tick_wall', None)
        self._last_tick_wall = now
        if self.connected and now - getattr(self, '_last_ping', 0) >= PING_EVERY_S:
            self._last_ping = now
            self.send("PING:1")
        if last is None:
            return
        gap = now - last
        if gap < STALL_LIMIT_S:
            return
        if not (self.cyc_on or self.series_running):
            print(f"Tick gap {gap:.1f}s (idle) - ignored")
            return
        print(f"!!! Tick gap {gap:.1f}s during a run - safe stop")
        try:
            if self.series_running:
                self._series_abort(f"PC asleep for {gap:.0f}s")
        except Exception:
            pass
        try:
            self.send("STOP")
            self._update_run_button(False)
        except Exception:
            pass
        try:
            self.cyc_stop(f"interrupted - PC asleep {gap:.0f}s")
        except Exception:
            pass
        core.warn(self, "Measurement interrupted",
                  f"The computer stopped running this program for {gap:.0f} s "
                  "(sleep, hibernation or a hard freeze).\n\n"
                  "The run was ended safely: the Peltier is off and the fans "
                  "are on their cool-down. Everything collected up to that "
                  "moment has been saved.")

    def tick(self):
        try:
            self._guard_tick()
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
                if len(self.t) > self.maxlen:
                    for a in (self.t, self.temp, self.spt, self.spa, self.pwm,
                              self.kp, self.ki, self.kd, self.states):
                        del a[0]

                # Run start
                if state == 'AUTO' and prev != 'AUTO' and not self.cyc_on:
                    self._cyc_start(temp)
                    self.reach_start_t = now2
                    self.reach_start_temp = temp
                    self.reach_target = st
                    self.reach_done = False
                    self.reach_in_tol_t = None
                    self.reach_time = None
                    self.reach_avg_rate = None
                    self.last_setpoint_target = st
                    self._ramp_reset(now2, temp, st)
                elif self.cyc_on and state == 'MAN' and prev in (
                        'AUTO', 'COOLDOWN', 'FREEZE', 'FREEZE_READY'):
                    self.cyc_stop("done")

                # A target change while running starts a new approach
                if state == 'AUTO' and self.last_setpoint_target is not None:
                    if abs(st - self.last_setpoint_target) > 0.5:
                        self.reach_start_t = now2
                        self.reach_start_temp = temp
                        self.reach_target = st
                        self.reach_done = False
                        self.reach_in_tol_t = None
                        self.last_setpoint_target = st
                        self._ramp_reset(now2, temp, st)

                # End of the RAMP PHASE = the generator (spA) reached target.
                # Computed BEFORE the approach check, so the two stay
                # independent - see the comment at self.ramp_t0.
                if (state == 'AUTO' and not self.ramp_done
                        and self.ramp_t0 is not None and abs(sa - st) <= 0.05):
                    self.ramp_done = True
                    self.ramp_secs = now2 - self.ramp_t0
                    if self.ramp_secs > 1.0 and self.ramp_temp0 is not None:
                        self.ramp_rate = ((temp - self.ramp_temp0) /
                                          (self.ramp_secs / 60.0))
                    self.ramp_lag = st - temp

                # Reached = |error| <= REACH_TOL_C held for REACH_STABLE_S.
                if (state == 'AUTO' and not self.reach_done
                        and self.reach_target is not None
                        and self.reach_start_t is not None):
                    if abs(temp - self.reach_target) > REACH_TOL_C:
                        self.reach_in_tol_t = None      # fell out - start over
                    elif self.reach_in_tol_t is None:
                        self.reach_in_tol_t = now2
                    if (self.reach_in_tol_t is not None
                            and now2 - self.reach_in_tol_t >= REACH_STABLE_S):
                        self.reach_done = True
                        self.reach_time = now2 - self.reach_start_t
                        delta = self.reach_target - self.reach_start_temp
                        if self.reach_time > 0:
                            self.reach_avg_rate = abs(delta) / (self.reach_time / 60.0)
                        self.reach_dir = "HEAT" if delta > 0 else "COOL"
                        self._last_reach_summary = {
                            'target': self.reach_target,
                            'time_s': self.reach_time,
                            'avg_rate': self.reach_avg_rate,
                            'dir': self.reach_dir}
        except Exception as e:
            print(f"tick err: {e}")

        if self.t:
            try:
                self.update_cards()
            except Exception as e:
                print(f"cards err: {e}")

        if self.series_running:
            try:
                self._series_tick()
            except Exception as e:
                print(f"series err: {e}")

    def _ramp_reset(self, t0, temp0, target):
        """Start counting a NEW ramp phase - see the comment at self.ramp_t0."""
        self.ramp_t0 = t0
        self.ramp_temp0 = temp0
        self.ramp_done = False
        self.ramp_secs = None
        self.ramp_rate = None
        self.ramp_lag = None
        # The COMMANDED rate comes from whichever control matches the direction.
        try:
            if target is not None and temp0 is not None and target < temp0:
                self.ramp_cmd_rate = self.sl_rd.get()
            else:
                self.ramp_cmd_rate = self.sl_ru.get()
        except Exception:
            self.ramp_cmd_rate = None

    # ── the readouts ────────────────────────────────────────────────────
    def update_cards(self):
        if not self.t:
            return
        temp = self.temp[-1]; spt = self.spt[-1]; pwm = self.pwm[-1]
        self.cards['temp'].set(f"{temp:.2f}")
        t2 = self._latest_temp2
        self.cards['temp2'].set(f"{t2:.2f}" if t2 is not None else "--")
        self.cards['sp'].set(f"{spt:.1f}")

        # AVG RATE is the rate of the RAMP ITSELF, not of the approach tail.
        # During the ramp it counts from the ramp's start; once the ramp
        # finishes it FREEZES at what the ramp achieved, so you can read
        # straight off whether the ramp kept up with the commanded rate.
        avg_rate = 0.0
        if self.ramp_done and self.ramp_rate is not None:
            avg_rate = self.ramp_rate
        elif (self.ramp_t0 is not None and self.ramp_temp0 is not None
                and self.cur_state == 'AUTO'):
            elapsed = self.t[-1] - self.ramp_t0
            if elapsed > 2:
                avg_rate = (temp - self.ramp_temp0) / (elapsed / 60.0)
        self.cards['rate'].set(f"{avg_rate:+.1f}")
        cmd = self.ramp_cmd_rate
        if cmd and abs(cmd) > 0.1 and abs(avg_rate) > 0.1:
            frac = abs(avg_rate) / abs(cmd)
            col = ('green' if frac >= 0.95 else
                   'yellow' if frac >= 0.85 else 'red')
            self.cards['rate'].unit.setText(f"°C/min · {frac * 100:.0f}% of cmd")
            self.cards['rate'].unit.setStyleSheet(
                "QLabel { color: %s; font-size: %dpt; }"
                % (self.th[col], self.th.pt('footnote')))
        else:
            self.cards['rate'].unit.setText("°C/min")
            self.cards['rate'].unit.setStyleSheet("")

        diff = spt - temp
        self.cards['pwm'].set(f"{pwm:.0f}")
        label = "% · heating" if diff > 0.3 else (
            "% · cooling" if diff < -0.3 else "% · holding")
        colr = 'red' if diff > 0.3 else ('cyan' if diff < -0.3 else 'label3')
        self.cards['pwm'].unit.setText(label)
        self.cards['pwm'].unit.setStyleSheet(
            "QLabel { color: %s; font-size: %dpt; }"
            % (self.th[colr], self.th.pt('footnote')))

        self._update_reach_label()

        # Paused - do not refresh, so a frozen chart can be inspected.
        if self.chart_paused:
            return
        t, tm, st, sa, pw = self.t, self.temp, self.spt, self.spa, self.pwm
        if self.chart_window > 0 and len(t) > 1:
            cutoff = t[-1] - self.chart_window
            i0 = 0
            for i in range(len(t) - 1, -1, -1):
                if t[i] < cutoff:
                    i0 = i
                    break
            t, tm, st, sa, pw = (t[i0:], tm[i0:], st[i0:], sa[i0:], pw[i0:])
        self._live_args = (t, tm, st, sa, pw)
        self._redraw_live(t, tm, st, sa, pw)

    def _update_reach_label(self):
        if self.cur_state == 'FREEZE_READY':
            self.reach_lbl.setText("stage solid — ready to swap the sample")
            return
        if self.cur_state == 'FREEZE':
            self.reach_lbl.setText("freezing → hold 20 °C")
            return
        if self.reach_done and self.reach_time is not None:
            d = (self.reach_dir or '').lower()
            # SPLIT into two numbers instead of one misleading average: how long
            # the RAMP itself ran and at what rate against the command, and
            # separately how long the APPROACH tail took after it. Two different
            # problems, fixed in two different places.
            if self.ramp_rate is not None and self.ramp_secs is not None:
                cmd = self.ramp_cmd_rate
                pct = (f" ({abs(self.ramp_rate) / abs(cmd) * 100:.0f}% of "
                       f"{abs(cmd):.0f})" if cmd and abs(cmd) > 0.1 else "")
                tail = max(0.0, self.reach_time - self.ramp_secs)
                lag = (f" · {self.ramp_lag:+.2f} °C left"
                       if self.ramp_lag is not None else "")
                self.reach_lbl.setText(
                    f"{d} · ramp {abs(self.ramp_rate):.1f} °C/min{pct} in "
                    f"{self.ramp_secs:.0f}s{lag} · approach +{tail:.0f}s")
            else:
                rate = (f"{self.reach_avg_rate:.2f}"
                        if self.reach_avg_rate else "?")
                self.reach_lbl.setText(
                    f"{d} reached in {core.fmt_hms(self.reach_time)} · "
                    f"avg {rate} °C/min")
            return
        if (self.cur_state == 'AUTO' and self.reach_start_t is not None
                and not self.reach_done and self.t):
            el = self.t[-1] - self.reach_start_t
            self.reach_lbl.setText(
                f"reaching {self.reach_target:.1f} °C · {core.fmt_hms(el)}")
            return
        self.reach_lbl.setText("")

    # ── the live chart ──────────────────────────────────────────────────
    def _redraw_live(self, t, temp, spt, spa, pwm):
        cs = self.chart.cs
        self.ax1.clear()
        self.ax2.clear()
        if t:
            self.ax1.plot(t, spt, color=cs['orange'], lw=1.3, ls='--',
                          label='target', alpha=0.8)
            # The ACTIVE setpoint is the ramp generator's output - watching it
            # against the temperature is how you see tracking, which is the
            # whole point of a ramped measurement.
            self.ax1.plot(t, spa, color=cs['cyan'], lw=1.5, ls=':',
                          label='setpoint')
            self.ax1.plot(t, temp, color=cs['blue'], lw=2.2, label='temperature')
        style_axes(self.ax1, cs, yunit='°C', show_x=False)
        if t:
            # 'best' rather than a fixed corner: a heating run fills the top
            # left by the end and a descent fills the bottom right, so any
            # fixed placement collides with the trace half the time.
            style_legend(self.ax1, cs, loc='best', ncol=3)

        if t:
            self.ax2.fill_between(t, 0, pwm, color=cs['green'], alpha=0.28)
            self.ax2.plot(t, pwm, color=cs['green'], lw=1.5)
        style_axes(self.ax2, cs, xunit='s', yunit='power %')
        self.ax2.set_ylim(-105, 105)
        self.ax2.set_yticks([-100, 0, 100])

        if not t:
            # Empty axes default to a 0..1 grid, which reads as data that is
            # simply flat. Better to say plainly that nothing has arrived yet.
            for ax in (self.ax1, self.ax2):
                ax.set_xticks([]); ax.set_yticks([])
                ax.grid(False)
            self.ax1.text(0.5, 0.5, "waiting for the first sample",
                          ha='center', va='center', color=cs['dim2'],
                          fontsize=11, transform=self.ax1.transAxes)

        # A manual zoom must survive the ~4 Hz redraw - see ChartNav.
        self.chart.nav.restore()
        self.chart.draw()

    def _nav_redraw(self):
        if self._live_args:
            self._redraw_live(*self._live_args)
        else:
            self._redraw_live([], [], [], [], [])

    def toggle_pause(self):
        self.chart_paused = not self.chart_paused
        self.btn_pause.setText("Resume" if self.chart_paused else "Pause")
        self._reicon(self.btn_pause,
                     'play' if self.chart_paused else 'pause')

    def set_chart_window(self, secs):
        self.chart_window = secs

    def save_live_chart(self):
        dest = core.save_path(self, "Save chart", "live_chart.png",
                              "PNG image (*.png);;PDF (*.pdf);;SVG (*.svg)",
                              self.log_dir)
        if not dest:
            return
        try:
            self.chart.export(dest, size=(9.5, 5.4))
            core.info(self, "Saved", dest)
        except Exception as e:
            core.error(self, "Save error", str(e))

    # ════════════════════════════════════════════════════════════════════
    #  RUN FILES
    # ════════════════════════════════════════════════════════════════════
    # IMPORTANT DISTINCTION: screen blanking breaks nothing - the process keeps
    # running, the port keeps reading, every CSV row is flushed immediately.
    # What breaks things is SUSPENDING THE WHOLE SYSTEM: USB is re-enumerated
    # from scratch, the port can disappear, and the SERIES state machine (which
    # runs here, in the app, not in the firmware) stops switching legs - the
    # board holds the last commanded setpoint while the series stalls and the
    # log gains a hole.
    #
    # So for the duration of a measurement we ask the system not to sleep. This
    # is an ordinary per-process API: it changes no system settings, needs no
    # administrator rights, and stops the moment the lock is released.
    def _wake_lock(self, on):
        try:
            if sys.platform.startswith('win'):
                import ctypes
                ES_CONTINUOUS = 0x80000000
                ES_SYSTEM_REQUIRED = 0x00000001
                ES_DISPLAY_REQUIRED = 0x00000002
                # ES_DISPLAY_REQUIRED keeps the screen awake too. Blanking on
                # its own is harmless, but on most machines it is the step right
                # before sleep - and the point is that a measurement in progress
                # is never interrupted by idle policy.
                flags = ((ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_DISPLAY_REQUIRED)
                         if on else ES_CONTINUOUS)
                ok = ctypes.windll.kernel32.SetThreadExecutionState(flags)
                if on and not ok:
                    print("WARNING: could not block system sleep")
                    return
            elif sys.platform == 'darwin':
                import subprocess
                if on:
                    if getattr(self, '_wl_proc', None) is None:
                        self._wl_proc = subprocess.Popen(
                            ['caffeinate', '-d', '-i', '-s', '-w',
                             str(os.getpid())])
                else:
                    p = getattr(self, '_wl_proc', None)
                    if p is not None:
                        try:
                            p.terminate()
                        except Exception:
                            pass
                        self._wl_proc = None
            else:
                import subprocess
                if on:
                    if getattr(self, '_wl_proc', None) is None:
                        self._wl_proc = subprocess.Popen(
                            ['systemd-inhibit',
                             '--what=sleep:idle:handle-lid-switch',
                             '--who=LACHI', '--why=measurement in progress',
                             'sleep', 'infinity'])
                else:
                    p = getattr(self, '_wl_proc', None)
                    if p is not None:
                        try:
                            p.terminate()
                        except Exception:
                            pass
                        self._wl_proc = None
            print("WAKE LOCK: %s" % ("held" if on else "released"))
        except Exception as e:
            # No caffeinate or systemd-inhibit, or an exotic system. The
            # measurement should happen anyway, so we only report it.
            print(f"WAKE LOCK unavailable ({e}) - make sure the machine does "
                  "not fall asleep")

    def _cyc_start(self, temp0):
        self.cyc_on = True
        self._wake_lock(True)
        # Arm the board-side watchdog: from now on the firmware expects a PING
        # at least every PC_TIMEOUT_MS and stops itself if the PC goes quiet.
        self.send("PCWD:1")
        self._last_ping = 0.0
        self.cyc_t0 = time.time()
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.cyc_ts = ts
        self.cyc_fn = self.log_dir / f"_tmp_cykl_{ts}.csv"
        self.cyc_file = open(self.cyc_fn, 'w', newline='', encoding='utf-8')
        self.cyc_wr = csv.writer(self.cyc_file)
        self.cyc_wr.writerow(CSV_COLS)
        self.cyc_rows = 0
        print(f"CYC START T={temp0:.1f}")

    def cyc_log(self, t, temp, sa, st, pwm, kp, ki, kd, state, temp2=None,
                dbg=None):
        if not self.cyc_wr:
            return
        try:
            t2str = f"{temp2:.2f}" if temp2 is not None else ""
            if dbg:
                dbgvals = [f"{dbg['ff']:.2f}", f"{dbg['p']:.2f}",
                           f"{dbg['i']:.2f}", f"{dbg['dd']:.2f}",
                           f"{dbg['raw']:.2f}", f"{dbg['react']:.2f}",
                           ("" if dbg.get('amb') is None else f"{dbg['amb']:.2f}")]
            else:
                dbgvals = [""] * 7
            # ONE call to now() - two would give inconsistent milliseconds.
            n = datetime.now()
            pcnow = n.strftime("%Y-%m-%d %H:%M:%S.") + f"{n.microsecond // 1000:03d}"
            self.cyc_wr.writerow([f"{t:.2f}", f"{temp:.2f}", f"{sa:.2f}",
                                  f"{st:.2f}", pwm, f"{pwm * 100 / 255:.1f}",
                                  f"{kp:.3f}", f"{ki:.4f}", f"{kd:.3f}", state,
                                  t2str, *dbgvals, pcnow])
            self.cyc_file.flush()
            self.cyc_rows += 1
        except Exception:
            pass

    def cyc_stop(self, reason=""):
        if self.cyc_file:
            try:
                self.cyc_file.close()
            except Exception:
                pass
        had_data = self.cyc_on and self.cyc_rows > 0
        tmp_path = self.cyc_fn
        self.cyc_on = False
        self.cyc_file = None
        self.cyc_wr = None
        # The sleep lock is released only when a series is NOT in flight -
        # between legs cyc_stop and _cyc_start fire back to back, and it would
        # be a shame to let the machine sleep in that gap.
        if not self.series_running:
            self._wake_lock(False)
            self.send("PCWD:0")
        print(f"CYC STOP: {reason} ({self.cyc_rows} samples)")

        if self.series_skip_archive:
            # The "return to base" leg of a series - an approach to the starting
            # position, not a test, so there is nothing to archive.
            self.series_skip_archive = False
            if tmp_path and tmp_path.exists():
                try:
                    tmp_path.unlink()
                except Exception:
                    pass
            return
        if had_data and tmp_path and tmp_path.exists():
            hint = self.series_name_hint
            self.series_name_hint = None
            if hint:
                # SERIES: save straight away under a readable name, with no
                # modal window - nobody is standing at the computer to close it.
                self.call(lambda: self.save_cycle_as(tmp_path, hint))
            else:
                self.call(lambda: self._ask_save_name(tmp_path))
        elif tmp_path and tmp_path.exists():
            try:
                tmp_path.unlink()
            except Exception:
                pass

    def _ask_save_name(self, tmp_path):
        SaveCycleDialog(self, self.th, self, tmp_path).exec()

    def save_cycle_as(self, tmp_path, name):
        """Save under the user's description; a timestamp is added only when
        that would overwrite an existing file."""
        clean = re.sub(r'[^\w\-\s]', '', name.strip()).strip()
        safe = re.sub(r'\s+', '_', clean) or "cykl"
        dest = self.log_dir / f"c_{safe}.csv"
        if dest.exists():
            dest = self.log_dir / f"c_{safe}_{datetime.now():%m%d_%H%M}.csv"
        try:
            Path(tmp_path).rename(dest)
            print(f"Run saved: {dest.name}")
        except Exception as e:
            print(f"Save error: {e}")
        try:
            self.refresh_arch()
        except Exception:
            pass

    def discard_cycle(self, tmp_path):
        try:
            if Path(tmp_path).exists():
                Path(tmp_path).unlink()
        except Exception:
            pass

    # ════════════════════════════════════════════════════════════════════
    #  MEASUREMENT SERIES
    # ════════════════════════════════════════════════════════════════════
    # An automatic sequence of setpoint/rate tests with no hand on the keyboard
    # between them. Each test is:
    #   1) ramp to the setpoint at the given rate, until "reached" (the same
    #      criterion the Control cards use)
    #   2) hold there for hold_s seconds, where oscillation and lag show up
    #   3) archive under a readable name, without asking
    #   4) return to base before the next test, so every start is from the same
    #      point
    def series_add_step(self, sp, rate, hold_s):
        self.series_steps.append({'sp': float(sp), 'rate': float(rate),
                                  'hold_s': float(hold_s)})
        self._series_refresh_list()

    def series_remove_step(self, idx):
        if 0 <= idx < len(self.series_steps):
            del self.series_steps[idx]
            self._series_refresh_list()

    def _series_refresh_list(self):
        if not hasattr(self, 'series_list'):
            return
        self.series_list.clear()
        for i, s in enumerate(self.series_steps):
            self.series_list.addItem(
                f"{i + 1}.   {s['sp']:.1f} °C    {s['rate']:.1f} °C/min    "
                f"hold {s['hold_s']:.0f} s")

    def _on_series_add(self):
        self.series_add_step(self.ser_sp.get(), self.ser_rate.get(),
                             self.ser_hold.get())

    def _on_series_remove(self):
        row = self.series_list.currentRow()
        if row >= 0:
            self.series_remove_step(row)

    def _on_series_clear(self):
        self.series_steps = []
        self._series_refresh_list()

    def _on_series_base_change(self, v):
        self.series_base_sp = float(v)

    def _on_series_quickfill(self):
        sp = self.ser_sp.get()
        for rate in (10, 20, 30, 40, 50, 60, 70):
            self.series_add_step(sp, rate, 60)

    def _on_series_toggle(self):
        if self.series_running:
            self._series_abort("manual stop")
            self.send("STOP")
            self._update_run_button(False)
        else:
            self.series_start()

    def series_start(self):
        if not self.connected:
            core.warn(self, "Not connected", "Connect to the device first.")
            return
        if not self.series_steps:
            core.warn(self, "Empty series", "Add at least one test.")
            return
        if self.series_running:
            return
        self.series_running = True
        self.series_idx = 0
        # The return between tests uses its OWN fast rate regardless of what
        # Cool rate on Control says; the original is restored afterwards.
        self._series_saved_rd = self.sl_rd.get()
        self._series_status(f"Series start: {len(self.series_steps)} tests")
        self._series_launch_heat(self.series_idx)
        self._series_button(True)

    def _series_button(self, running):
        self.btn_series_run.setText("Stop series" if running else "Start series")
        self.btn_series_run.setProperty('kind', 'stop' if running else 'go')
        self._reicon(self.btn_series_run, 'stop' if running else 'play',
                     'stop' if running else 'go')
        self._restyle(self.btn_series_run)

    def _series_restore_rd(self):
        if self._series_saved_rd is not None:
            self.sl_rd.set(self._series_saved_rd)
            if self.connected:
                self.send(f"RD:{self._series_saved_rd:.1f}")
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
        self._series_status(f"Series aborted ({reason})" if reason
                            else "Series aborted")
        self._series_button(False)

    def _series_finish(self):
        self.series_running = False
        self.series_leg = None
        self.series_phase = None
        if not self.cyc_on:
            self._wake_lock(False)
        self._series_restore_rd()
        self._series_status(f"Series finished - {len(self.series_steps)} tests, "
                            f"files in {self.log_dir.name}")
        self._series_button(False)

    def _series_status(self, text):
        print(f"SERIES: {text}")
        if hasattr(self, 'series_status_row'):
            self.series_status_row.title.setText(text)

    def _series_save_prog(self):
        if not self.series_steps:
            core.info(self, "Empty program", "Add steps first.")
            return
        dest = core.save_path(self, "Save program",
                              datetime.now().strftime("program_%Y-%m-%d.json"),
                              "Program (*.json)", self.log_dir)
        if not dest:
            return
        try:
            with open(dest, 'w', encoding='utf-8') as f:
                json.dump({'tryb': self.series_mode.get(),
                           'baza': self.series_base_sp,
                           'kroki': self.series_steps}, f, indent=2)
            self._series_status(f"Program saved: {Path(dest).name}")
        except Exception as e:
            core.error(self, "Save program", str(e))

    def _series_load_prog(self):
        src = core.open_path(self, "Load program", "Program (*.json)",
                             self.log_dir)
        if not src:
            return
        try:
            with open(src, 'r', encoding='utf-8') as f:
                d = json.load(f)
            clean = [dict(sp=float(s['sp']), rate=float(s['rate']),
                          hold_s=float(s.get('hold_s', 60)))
                     for s in (d.get('kroki') or [])]
            if not clean:
                core.warn(self, "Program", "The file contains no steps.")
                return
            self.series_steps = clean
            if d.get('tryb') in ('seria', 'program'):
                self.series_mode.set(d['tryb'])
                self.seg_mode.setIndex(0 if d['tryb'] == 'seria' else 1,
                                       emit=False)
            if d.get('baza') is not None:
                self.series_base_sp = float(d['baza'])
                self.ser_base.set(self.series_base_sp)
            self._series_refresh_list()
            self._series_status(f"Program loaded: {len(clean)} steps")
        except Exception as e:
            core.error(self, "Load program", str(e))

    def _series_roll_cycle(self, hint):
        """Close the CURRENT run file under `hint` and immediately open a new
        one, WITHOUT stopping the controller. Previously every leg was closed
        with STOP, because only an AUTO->MAN transition closed the file - and
        that forced a break in control (see _series_switch_leg)."""
        self.series_name_hint = hint
        self.cyc_stop("end of series leg")
        self._cyc_start(self.temp[-1] if self.temp else 0.0)

    def _series_switch_leg(self, sp, ru=None, rd=None):
        """Move to a new target WITHOUT STOP/START - the controller stays in AUTO.

        WHY: previously the app went STOP -> 600 ms -> START between legs. On
        STOP the firmware drops to MAN and zeroes the power; on START it sets
        the active setpoint to the CURRENT reading. In that gap the reading had
        already fallen, because the stage has low thermal inertia and cools the
        instant ~60 PWM units are removed. Measured on real logs across five
        transitions: a 1.4-1.7 s gap during which the temperature fell 5.6-7.0
        °C, so the descent ramp began from ~44-47 °C instead of 50 - exactly the
        "setpoint line much lower" and the visible hole between legs.

        The firmware accepts SP/RU/RD during AUTO (they only change the target
        and the rates) and START does anything only when sys == MAN. So it is
        enough not to leave AUTO: spA transitions smoothly, with no zeroed
        power, no break and no setpoint jump.
        """
        if ru is not None:
            self.send(f"RU:{ru:.1f}")
        if rd is not None:
            self.send(f"RD:{rd:.1f}")
        self.send(f"SP:{sp:.1f}")
        # The approach statistics restart for every leg. NOTE: do_start zeroes
        # reach_start_t because a MAN->AUTO transition follows and sets it. Here
        # there will be NO such transition, so it has to be set by hand -
        # otherwise the detection in tick() (which needs reach_start_t) would
        # never fire and the leg would hang until its timeout.
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
        # compares telemetry against this value, and for a moment after sending
        # SP the telemetry still carries the OLD setpoint. Writing the new
        # target here would trigger a false "setpoint changed" and reset the
        # counters we just set. The series steers the setpoint itself anyway.
        self.last_setpoint_target = None
        self._last_reach_summary = None
        self._ramp_reset(now, cur, sp)
        self._update_run_button(True)

    def _series_launch_heat(self, idx):
        if not self.connected:
            # The series runs UNATTENDED - no modal "not connected" box left
            # hanging in mid-air, just a readable status and a clean abort.
            self._series_abort("lost the connection")
            return
        step = self.series_steps[idx]
        self.sl_sp.set(step['sp'])
        self.sl_ru.set(step['rate'])
        self.do_start()
        self.series_leg = 'heat'
        self.series_phase = 'ramping'
        self.series_phase_t0 = time.time()
        self._series_status(
            f"Test {idx + 1}/{len(self.series_steps)}: {step['sp']:.1f} °C at "
            f"{step['rate']:.1f} °C/min — heating")

    def _series_launch_cool(self):
        if not self.connected:
            self._series_abort("lost the connection")
            return
        return_rate = self.ser_return.get()
        cool_is_test = bool(self.series_cool_as_test.get())
        if cool_is_test and self.series_idx < len(self.series_steps):
            # The descent as a FULL TEST: the power model was calibrated from
            # HEATING data only, because the descents used to run at a fixed
            # fast return rate. Running them at the test's own rate gives a
            # matching set of cooling runs to calibrate that branch the same way.
            return_rate = self.series_steps[self.series_idx]['rate']
        self.sl_rd.set(return_rate)
        self.sl_sp.set(self.series_base_sp)
        self._series_switch_leg(self.series_base_sp, rd=return_rate)
        self.series_leg = 'cool'
        self.series_phase = 'ramping'
        self.series_phase_t0 = time.time()
        # The return used to be discarded as "not data". It is our ONLY downward
        # ramp, so it is archived exactly like the heating - real data beats a
        # tidy folder.
        self.series_skip_archive = False
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        kind = "cooltest" if cool_is_test else "cool"
        self.series_name_hint = (f"seria_{kind}_toSP{self.series_base_sp:.0f}"
                                 f"_R{return_rate:.0f}_{ts}")
        lbl = "Descent test" if cool_is_test else "Return"
        self._series_status(f"{lbl} to {self.series_base_sp:.1f} °C at "
                            f"{return_rate:.0f} °C/min")

    def _series_tick(self):
        if not self.series_running or self.series_idx >= len(self.series_steps):
            return
        now = time.time()
        step = self.series_steps[self.series_idx]

        if self.series_leg == 'heat' and self.series_phase == 'ramping':
            if self.reach_done:
                self.series_phase = 'holding'
                self.series_phase_t0 = now
                self._series_status(
                    f"Test {self.series_idx + 1}/{len(self.series_steps)}: "
                    f"reached {step['sp']:.1f} °C — holding {step['hold_s']:.0f}s")
            elif now - self.series_phase_t0 > SERIES_HEAT_TIMEOUT_S:
                self._series_status(f"Test {self.series_idx + 1}: approach "
                                    "timed out — ending this test")
                self._series_end_heat_leg(tag="TIMEOUT")

        elif self.series_leg == 'heat' and self.series_phase == 'holding':
            elapsed = now - self.series_phase_t0
            self._series_status(
                f"Test {self.series_idx + 1}/{len(self.series_steps)}: holding "
                f"{step['sp']:.1f} °C — {max(0, step['hold_s'] - elapsed):.0f}s left")
            if elapsed >= step['hold_s']:
                self._series_end_heat_leg(tag="OK")

        elif self.series_leg == 'cool' and self.series_phase == 'ramping':
            # The descent used to END here, at the moment of reaching, without a
            # single hold sample. Now it gets a hold like the heating, so the
            # tail is visible in the log: whether the temperature settles on the
            # target and whether it crosses it.
            if self.reach_done:
                self.series_phase = 'holding'
                self.series_phase_t0 = now
                self._series_status(
                    f"Descent {self.series_idx + 1}/{len(self.series_steps)}: "
                    f"reached {self.series_base_sp:.1f} °C — holding "
                    f"{step['hold_s']:.0f}s")
            elif now - self.series_phase_t0 > SERIES_COOL_TIMEOUT_S:
                self._series_status(f"Descent {self.series_idx + 1}: approach "
                                    "timed out — ending")
                self._series_end_cool_leg()

        elif self.series_leg == 'cool' and self.series_phase == 'holding':
            elapsed = now - self.series_phase_t0
            self._series_status(
                f"Descent {self.series_idx + 1}/{len(self.series_steps)}: "
                f"holding {self.series_base_sp:.1f} °C — "
                f"{max(0, step['hold_s'] - elapsed):.0f}s left")
            if elapsed >= step['hold_s']:
                self._series_end_cool_leg()

    def _series_end_heat_leg(self, tag="OK"):
        # THE BUG THIS GUARD FIXES (reported as "10/40/70 instead of the whole
        # list"): tick() runs every 250 ms, but the next step was scheduled with
        # a 600 ms delay - and the condition that leads here stayed TRUE for
        # that whole window. So tick() called this two or three times before the
        # phase actually changed, each call scheduling its own advance, and
        # series_idx jumped by 2-3 instead of 1. Setting the 'ending' sentinel
        # synchronously makes the condition stop matching at once.
        self.series_phase = 'ending'
        step = self.series_steps[self.series_idx]
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        prog = (self.series_mode.get() == 'program')
        if prog:
            # In program mode the name carries the step number - otherwise
            # consecutive steps with the same setpoint are indistinguishable.
            hint = (f"prog{self.series_idx + 1:02d}_SP{step['sp']:.0f}"
                    f"_R{step['rate']:.0f}_{tag}_{ts}")
        else:
            hint = f"seria_SP{step['sp']:.0f}_R{step['rate']:.0f}_{tag}_{ts}"
        nxt = self.series_idx + 1

        if prog:
            # PROGRAM: no return to base - the next step starts exactly where
            # this one finished, smoothly, without STOP/START.
            if nxt < len(self.series_steps) and self.connected:
                self._series_roll_cycle(hint)
                self.series_idx = nxt
                nstep = self.series_steps[nxt]
                self.sl_sp.set(nstep['sp'])
                cur = self.temp[-1] if self.temp else nstep['sp']
                down = nstep['sp'] < cur - 0.5
                (self.sl_rd if down else self.sl_ru).set(nstep['rate'])
                self._series_switch_leg(
                    nstep['sp'],
                    rd=(nstep['rate'] if down else None),
                    ru=(None if down else nstep['rate']))
                self.series_leg = 'heat'    # 'heat' = the leg the step commands
                self.series_phase = 'ramping'
                self.series_phase_t0 = time.time()
                self._series_status(
                    f"Step {nxt + 1}/{len(self.series_steps)}: "
                    f"{'descent' if down else 'approach'} to {nstep['sp']:.1f} °C "
                    f"at {nstep['rate']:.0f} °C/min")
            else:
                self.series_name_hint = hint
                self.send("STOP")
                self._update_run_button(False)
                self.after(600, self._series_advance)
        elif abs(self.series_base_sp - step['sp']) > 0.5:
            # Close the heating file and open the descent file immediately,
            # WITHOUT leaving AUTO - so the descent starts where heating ended.
            self._series_roll_cycle(hint)
            self.series_leg = 'cool'
            self._series_launch_cool()
        else:
            # Target equals base, so there is no descent leg - finish here.
            self.series_name_hint = hint
            self.send("STOP")
            self._update_run_button(False)
            self.after(600, self._series_advance)

    def _series_end_cool_leg(self):
        # The same sentinel guard as above - and this was the ACTUAL source of
        # the skipping, because every test had base != setpoint and so always
        # went through the cool leg.
        self.series_phase = 'ending'
        nxt = self.series_idx + 1
        if nxt < len(self.series_steps) and self.connected:
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
                f"Test {nxt + 1}/{len(self.series_steps)}: {step['sp']:.1f} °C "
                f"at {step['rate']:.1f} °C/min — heating")
        else:
            self.send("STOP")
            self._update_run_button(False)
            self.after(600, self._series_advance)

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

    # ════════════════════════════════════════════════════════════════════
    #  ARCHIVE
    # ════════════════════════════════════════════════════════════════════
    def _cycle_display_name(self, path):
        s = Path(path).stem
        if s.startswith('cykl_'):
            s = s[5:]
        elif s.startswith('c_'):
            s = s[2:]
        return s.replace('_', ' ')

    def _arch_files(self):
        return sorted([f for f in self.log_dir.glob("*.csv")
                       if (f.name.startswith("cykl_") or f.name.startswith("c_"))
                       and not f.name.startswith("_tmp")],
                      key=lambda f: f.stat().st_mtime, reverse=True)

    def refresh_arch(self):
        if not hasattr(self, 'arch_list'):
            return
        keep = {p for p, v in self.arch_vars.items() if v.get()}
        self.arch_list.blockSignals(True)
        self.arch_list.clear()
        self.arch_vars = {}
        files = self._arch_files()
        if not files:
            it = QListWidgetItem("No saved runs yet")
            it.setFlags(Qt.ItemFlag.NoItemFlags)
            self.arch_list.addItem(it)
            self.arch_list.blockSignals(False)
            self.data_dir_row.sub.setText(str(self.log_dir))
            return

        today = datetime.now().strftime("%Y-%m-%d")
        groups = {}
        for f in files:
            day = datetime.fromtimestamp(f.stat().st_mtime).strftime("%Y-%m-%d")
            groups.setdefault(day, []).append(f)

        i = 0
        for day, day_files in groups.items():
            hdr = QListWidgetItem(
                f"{'Today' if day == today else day}   ({len(day_files)})")
            hdr.setFlags(Qt.ItemFlag.NoItemFlags)
            f = hdr.font()
            f.setPointSize(max(7, self.th.pt('caption')))
            f.setWeight(QFont.Weight.DemiBold)
            hdr.setFont(f)
            hdr.setForeground(QColor(self.th.solid('label2', 'card')))
            self.arch_list.addItem(hdr)
            for path in day_files:
                key = str(path)
                it = QListWidgetItem(self._cycle_display_name(path))
                it.setFlags(it.flags() | Qt.ItemFlag.ItemIsUserCheckable)
                it.setCheckState(Qt.CheckState.Checked if key in keep
                                 else Qt.CheckState.Unchecked)
                it.setData(Qt.ItemDataRole.UserRole, key)
                # The dot is the only thing tying a row to its curve.
                it.setIcon(self._dot_icon(
                    self.th[ARCH_COLOR_KEYS[i % len(ARCH_COLOR_KEYS)]]))
                it.setToolTip(path.name)
                self.arch_list.addItem(it)
                self.arch_vars[key] = Var(key in keep)
                i += 1
        self.arch_list.blockSignals(False)
        self.data_dir_row.sub.setText(str(self.log_dir))

    def _dot_icon(self, color):
        d = self.th.px(12)
        px = QPixmap(d * 2, d * 2)
        px.setDevicePixelRatio(2)
        px.fill(Qt.GlobalColor.transparent)
        q = QPainter(px)
        q.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        q.setPen(Qt.PenStyle.NoPen)
        q.setBrush(QColor(color))
        q.drawEllipse(1, 1, d - 2, d - 2)
        q.end()
        return QIcon(px)

    def _on_arch_item(self, item):
        key = item.data(Qt.ItemDataRole.UserRole)
        if key in self.arch_vars:
            self.arch_vars[key].set(item.checkState() == Qt.CheckState.Checked)
            self._redraw_arch()

    def _set_all_checks(self, state):
        self.arch_list.blockSignals(True)
        for i in range(self.arch_list.count()):
            it = self.arch_list.item(i)
            key = it.data(Qt.ItemDataRole.UserRole)
            if key:
                it.setCheckState(state)
                self.arch_vars[key].set(state == Qt.CheckState.Checked)
        self.arch_list.blockSignals(False)
        self._redraw_arch()

    def _arch_select_all(self):
        self._set_all_checks(Qt.CheckState.Checked)

    def _arch_clear_sel(self):
        self._set_all_checks(Qt.CheckState.Unchecked)

    def _delete_selected(self):
        it = self.arch_list.currentItem()
        key = it.data(Qt.ItemDataRole.UserRole) if it else None
        if not key:
            core.info(self, "Nothing selected", "Click a run in the list first.")
            return
        name = self._cycle_display_name(Path(key))
        if core.ask(self, "Delete this run?", f"{name}\n\nThis cannot be undone."):
            try:
                Path(key).unlink()
                self.refresh_arch()
                self._redraw_arch()
            except Exception as e:
                core.error(self, "Delete error", str(e))

    def _on_xmode_change(self, i):
        self.arch_xmode.set(['t0', 'abs', 'pc', 'ramp', 'temp'][i])
        en = self.arch_xmode.get() == 'temp'
        self.sl_tref.setEnabled(en)
        self.lbl_tref.setEnabled(en)
        self._redraw_arch()

    def _on_tref(self, v):
        self.arch_tref.set(v)
        self.lbl_tref.setText(f"{v:.1f} °C")
        if self.arch_xmode.get() == 'temp':
            self._redraw_arch()

    # ── loading ─────────────────────────────────────────────────────────
    def _cycle_settings(self, path):
        """Target, ramp and gains as recorded in the file."""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                rows = [csv_row(r) for r in csv.DictReader(f)]
        except Exception:
            return None
        valid = [r for r in rows
                 if (r.get('time_s') or '').replace('.', '').replace('-', '').isdigit()]
        if not valid:
            return None
        s = {}
        try:
            sps = [float(r['setpoint_target']) for r in valid
                   if r.get('setpoint_target')]
            s['target'] = max(set(sps), key=sps.count) if sps else None
        except Exception:
            s['target'] = None
        try:
            s['kp'] = float(valid[0].get('Kp', 0))
            s['ki'] = float(valid[0].get('Ki', 0))
            s['kd'] = float(valid[0].get('Kd', 0))
        except Exception:
            s['kp'] = s['ki'] = s['kd'] = None
        # The ramp is estimated from the slope of the active setpoint at the
        # start - which is the commanded rate, not the achieved one.
        try:
            t0 = float(valid[0]['time_s'])
            sa0 = float(valid[0]['setpoint_active'])
            ramp = None
            for r in valid:
                tt = float(r['time_s'])
                if tt - t0 >= 5:
                    dt_min = (tt - t0) / 60.0
                    if dt_min > 0:
                        ramp = abs(float(r['setpoint_active']) - sa0) / dt_min
                    break
            s['ramp'] = ramp
        except Exception:
            s['ramp'] = None
        return s

    def _load_cycle_data(self, path):
        """(t, temp, target, pwm) from a run file, comment tolerant."""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = [csv_row(r) for r in csv.DictReader(f)]
        except Exception:
            return None
        t, temp, spt, pwm, temp2, sa_list, pc_raw = [], [], [], [], [], [], []
        for r in data:
            cz = r.get('time_s', '')
            if not cz or cz.startswith('#'):
                continue
            try:
                tt = float(cz)
                tm = float(r.get('temperature_C', 'nan'))
                sp = float(r.get('setpoint_target', 'nan'))
            except (ValueError, TypeError):
                continue
            t.append(tt); temp.append(tm); spt.append(sp)
            try:
                sa_list.append(float(r.get('setpoint_active', 'nan')))
            except Exception:
                sa_list.append(None)
            try:
                pwm.append(float(r.get('PWM_%', r.get('PWM', 0))))
            except Exception:
                pwm.append(0)
            try:
                v = r.get('temperature2_C', '')
                temp2.append(float(v) if v else None)
            except Exception:
                temp2.append(None)
            pc_raw.append(r.get('pc_time', '') or '')
        if not t:
            return None
        self._last_temp2 = temp2
        self._last_sa = sa_list
        self._last_pc = self._pc_seconds(pc_raw, t, path)
        return t, temp, spt, pwm

    def _pc_seconds(self, pc_raw, t, path):
        """The PC clock column as epoch seconds. Old files have no such column,
        so the axis is rebuilt from the file's modification time: mtime is the
        moment of CLOSING, so start = mtime - run length."""
        out, ok = [], False
        for sraw in pc_raw:
            v = None
            if sraw:
                for fmt, n in (("%Y-%m-%d %H:%M:%S.%f", 23),
                               ("%Y-%m-%d %H:%M:%S", 19)):
                    try:
                        v = datetime.strptime(sraw[:n], fmt).timestamp()
                        ok = True
                        break
                    except Exception:
                        v = None
            out.append(v)
        if ok:
            base = next((i for i, v in enumerate(out) if v is not None), None)
            if base is not None:
                t0 = out[base] - t[base]
                out = [v if v is not None else t0 + t[i]
                       for i, v in enumerate(out)]
            return out
        try:
            end = Path(path).stat().st_mtime
            t0 = end - (t[-1] - t[0])
            return [t0 + (x - t[0]) for x in t]
        except Exception:
            return [None] * len(t)

    def _compute_stats(self, data):
        t, temp, spt, pwm = data
        st = {}
        st['tmin'] = min(temp)
        st['tmax'] = max(temp)
        st['duration'] = t[-1] - t[0] if len(t) > 1 else 0
        st['target'] = spt[-1] if spt else 0

        idx_max = temp.index(st['tmax'])
        rise_time = t[idx_max] - t[0] if idx_max > 0 else 0
        st['avg_rise'] = ((st['tmax'] - temp[0]) / (rise_time / 60.0)
                          if rise_time > 5 else 0)

        target = st['target']
        st['overshoot'] = max(0, st['tmax'] - target) if target else 0

        # Settling = the first sample after which it STAYS within ±1 °C
        st['settle_time'] = None
        if target:
            for i, tm in enumerate(temp):
                if abs(tm - target) <= 1.0:
                    rest = temp[i:]
                    if sum(1 for x in rest if abs(x - target) <= 1.0) >= len(rest) * 0.8:
                        st['settle_time'] = t[i] - t[0]
                        break

        n = len(temp)
        tail = temp[int(n * 0.8):] if n > 5 else temp
        st['steady_error'] = (statistics.mean(abs(x - target) for x in tail)
                              if target and tail else 0)
        devs = [abs(temp[i] - spt[i]) for i in range(len(temp))]
        st['max_dev'] = max(devs) if devs else 0
        # Noise in the settled tail is a measure of MEASUREMENT quality, not of
        # control quality - it is the thermocouple and its wiring.
        st['noise_std'] = statistics.stdev(tail) if len(tail) > 2 else 0
        return st

    def _arch_t_offset(self, t, temp, mode, tref):
        """The x-axis zero for one run, per the selected alignment."""
        if mode == 'abs':
            return 0.0
        if mode == 'temp':
            # The FIRST crossing of tref, interpolated between samples, so runs
            # that started from different temperatures overlay at the same
            # THERMAL point rather than the same time point.
            for i in range(1, len(temp)):
                a, b = temp[i - 1], temp[i]
                if (a - tref) * (b - tref) <= 0 and a != b:
                    f = (tref - a) / (b - a)
                    return t[i - 1] + f * (t[i] - t[i - 1])
                if a == tref:
                    return t[i - 1]
            return t[0]
        if mode == 'ramp':
            # Zero = where the ramp REALLY starts, detected from the active
            # setpoint: while it stands still we are still pre-start. Aligns
            # runs with different run-ups before the actual ramp.
            sa = self._last_sa or []
            for i in range(1, min(len(sa), len(t))):
                if (sa[i] is not None and sa[0] is not None
                        and abs(sa[i] - sa[0]) > 0.05):
                    return t[i]
            for i in range(1, len(temp)):
                if abs(temp[i] - temp[0]) > 0.3:
                    return t[i]
            return t[0]
        return t[0]                                    # 't0'

    def _redraw_arch(self):
        if not hasattr(self, 'ax_a'):
            return
        cs = self.chart_a.cs
        selected = [p for p, v in self.arch_vars.items() if v.get()]
        self.ax_a.clear()
        # The PWM axis is created on demand; the old one is removed on every
        # redraw or they would pile up.
        if self._ax_pwm is not None:
            try:
                self._ax_pwm.remove()
            except Exception:
                pass
            self._ax_pwm = None

        mode = self.arch_xmode.get()
        show = {k: v.get() for k, v in self.arch_show.items()}
        tref = self.arch_tref.get()

        if not selected:
            self.ax_a.text(0.5, 0.5, "Pick one or more runs on the left",
                           ha='center', va='center', color=cs['dim2'],
                           fontsize=11, transform=self.ax_a.transAxes)
            style_axes(self.ax_a, cs, grid=False)
            self.ax_a.set_xticks([]); self.ax_a.set_yticks([])
            self.chart_a.nav.restore()
            self.chart_a.draw()
            self.arch_settings_lbl.setText("")
            self.arch_title.setText("drag · scroll · right-click")
            return

        files = self._arch_files()
        file_order = {str(f): i for i, f in enumerate(files)}
        multi = len(selected) > 1

        series = []
        for path in selected:
            d = self._load_cycle_data(path)
            if not d:
                continue
            t, temp, spt, pwm = d
            series.append(dict(path=path, t=t, temp=temp, spt=spt, pwm=pwm,
                               sa=list(self._last_sa or []),
                               t2=list(self._last_temp2 or []),
                               pc=list(self._last_pc or [])))
        if not series:
            self.chart_a.draw()
            return
        # Order = the order of the list on the left. It matters for the
        # difference mode: the reference must be the trace the user sees first.
        series.sort(key=lambda z: file_order.get(z['path'], 10 ** 6))

        if mode == 'pc':
            # Common axis = the PC clock. Zero comes from the EARLIEST run, so
            # the numbers stay small while the labels show the real time of day.
            starts = [s['pc'][0] for s in series if s['pc'] and s['pc'][0] is not None]
            self._pc_zero = min(starts) if starts else 0.0
            for s in series:
                s['off'] = 0.0
        else:
            for s in series:
                s['off'] = self._arch_t_offset(s['t'], s['temp'], mode, tref)

        spans = []
        for s in series:
            if mode == 'pc' and s['pc'] and s['pc'][0] is not None:
                spans.append(s['pc'][-1] - s['pc'][0])
            else:
                spans.append(s['t'][-1] - s['t'][0])
        use_min = (max(spans) if spans else 0) > 180
        tdiv = 60.0 if use_min else 1.0

        # ── comparing runs against each other ───────────────────────────
        delta_mode = bool(self.arch_delta.get() and len(series) > 1)
        ref = series[0] if delta_mode else None
        ref_x, ref_name = None, ""
        if delta_mode:
            ref_name = self._cycle_display_name(Path(ref['path']))
            if mode == 'pc' and ref['pc'] and ref['pc'][0] is not None:
                ref_x = [((v or 0) - self._pc_zero) / tdiv for v in ref['pc']]
            else:
                ref_x = [(x - ref['off']) / tdiv for x in ref['t']]
            # In difference mode the setpoints only clutter the picture.
            show = dict(show); show['sa'] = False; show['st'] = False

        def _interp(xs, ys, x):
            if not xs:
                return 0.0
            if x <= xs[0]:
                return ys[0]
            if x >= xs[-1]:
                return ys[-1]
            lo, hi = 0, len(xs) - 1
            while lo < hi - 1:
                mid = (lo + hi) // 2
                if xs[mid] <= x:
                    lo = mid
                else:
                    hi = mid
            dx = xs[hi] - xs[lo]
            if dx == 0:
                return ys[lo]
            return ys[lo] + (x - xs[lo]) / dx * (ys[hi] - ys[lo])

        ax2 = None
        for s in series:
            ci = file_order.get(s['path'], 0) % len(ARCH_COLOR_KEYS)
            col = cs[ARCH_COLOR_KEYS[ci]]
            if mode == 'pc' and s['pc'] and s['pc'][0] is not None:
                tx = [((v or 0) - self._pc_zero) / tdiv for v in s['pc']]
            else:
                tx = [(x - s['off']) / tdiv for x in s['t']]
            name = self._cycle_display_name(Path(s['path']))
            base = f"{name} · " if multi else ""

            if show.get('st'):
                self.ax_a.plot(tx, s['spt'], color=cs['orange'], lw=1.2, ls='--',
                               alpha=0.55,
                               label=(base + 'target') if not multi else None)
            if show.get('sa') and s['sa'] and any(v is not None for v in s['sa']):
                xs = [tx[i] for i in range(min(len(s['sa']), len(tx)))
                      if s['sa'][i] is not None]
                ys = [v for v in s['sa'][:len(tx)] if v is not None]
                if ys:
                    self.ax_a.plot(xs, ys, color=(cs['cyan'] if not multi else col),
                                   lw=1.1, ls=':', alpha=0.8,
                                   label=(base + 'setpoint') if not multi else None)
            if show.get('temp'):
                if delta_mode and ref is not None:
                    if s is ref:
                        self.ax_a.axhline(0, color=cs['dim2'], lw=1.0, ls='--',
                                          alpha=0.7)
                        continue
                    dy = [s['temp'][i] - _interp(ref_x, ref['temp'], tx[i])
                          for i in range(len(tx))]
                    self.ax_a.plot(tx, dy, color=col, lw=1.8,
                                   label=f"{name} − {ref_name}")
                else:
                    self.ax_a.plot(tx, s['temp'], color=col,
                                   lw=(2 if not multi else 1.8),
                                   label=(name if multi else 'temperature'))
            if show.get('t2') and s['t2'] and any(v is not None for v in s['t2']):
                xs = [tx[i] for i in range(min(len(s['t2']), len(tx)))
                      if s['t2'][i] is not None]
                ys = [v for v in s['t2'][:len(tx)] if v is not None]
                if ys:
                    self.ax_a.plot(xs, ys, color=cs['purple'], lw=1.5, alpha=0.85,
                                   label=(base + 'probe 2'))
            if show.get('pwm'):
                if ax2 is None:
                    ax2 = self.ax_a.twinx()
                    self._ax_pwm = ax2
                    ax2.tick_params(colors=cs['dim'], labelsize=8, length=0)
                    for sp in ax2.spines.values():
                        sp.set_visible(False)
                    ax2.text(0.995, 0.955, 'power %', transform=ax2.transAxes,
                             ha='right', va='top', color=cs['dim2'], fontsize=8)
                ax2.plot(tx, s['pwm'], color=col, lw=0.9, ls='-.', alpha=0.5)

        if mode == 'temp':
            self.ax_a.axhline(tref, color=cs['dim2'], lw=0.8, ls='--', alpha=0.6)
            self.ax_a.axvline(0, color=cs['dim2'], lw=0.8, ls='--', alpha=0.6)

        if mode == 'pc':
            import matplotlib.ticker as mt
            z = getattr(self, '_pc_zero', 0.0)

            def _fmt(v, _pos):
                try:
                    return datetime.fromtimestamp(z + v * tdiv).strftime("%H:%M:%S")
                except Exception:
                    return ""
            self.ax_a.xaxis.set_major_formatter(mt.FuncFormatter(_fmt))

        unit = 'min' if use_min else 's'
        note = {'t0': '0 = start of the run', 'abs': "the file's own time",
                'ramp': '0 = start of the ramp',
                'temp': f'0 = crossing {tref:.1f} °C',
                'pc': 'wall clock'}.get(mode, '')
        style_axes(self.ax_a, cs,
                   xunit=('h:min:s' if mode == 'pc' else unit),
                   yunit=(f'ΔT vs {ref_name} [°C]' if delta_mode
                          else 'temperature [°C]'))
        if note:
            self.ax_a.text(0.008, 0.885, note, transform=self.ax_a.transAxes,
                           ha='left', va='top', color=cs['dim2'], fontsize=8)
        style_legend(self.ax_a, cs, loc='best')

        # Headline: statistics for one run, or a count when comparing.
        if not multi:
            d = self._load_cycle_data(selected[0])
            if d:
                t, temp, _s, _p = d
                dur = t[-1] - t[0] if len(t) > 1 else 0
                idx_max = temp.index(max(temp))
                rise = t[idx_max] - t[0] if idx_max > 0 else 0
                avg = (max(temp) - temp[0]) / (rise / 60.0) if rise > 5 else 0
                self.arch_title.setText(
                    f"{min(temp):.1f}–{max(temp):.1f} °C · {core.fmt_hms(dur)} "
                    f"· avg rise {avg:.2f} °C/min")
        else:
            self.arch_title.setText(f"comparing {len(selected)} runs")

        self.chart_a.nav.restore()
        self.chart_a.draw()

        if not multi:
            cset = self._cycle_settings(selected[0])
            if cset:
                def fmt(v, suf=''):
                    return f"{v:.1f}{suf}" if v is not None else "?"
                self.arch_settings_lbl.setText(
                    f"target {fmt(cset['target'], ' °C')}   ·   ramp "
                    f"~{fmt(cset['ramp'], ' °C/min')}   ·   Kp {fmt(cset['kp'])}  "
                    f"Ki {fmt(cset['ki'])}  Kd {fmt(cset['kd'])}")
            else:
                self.arch_settings_lbl.setText("")
        else:
            self.arch_settings_lbl.setText(
                f"{len(selected)} runs selected — settings are shown for a "
                "single selection")

    # ── archive actions ─────────────────────────────────────────────────
    def _selected_arch_path(self):
        for p, v in self.arch_vars.items():
            if v.get():
                return Path(p)
        return None

    def show_arch_stats(self):
        path = self._selected_arch_path()
        if not path:
            core.info(self, "Nothing selected", "Tick a run in the list first.")
            return
        data = self._load_cycle_data(path)
        if not data:
            core.error(self, "Unreadable", "Could not load that run.")
            return
        StatsDialog(self, self.th, self._cycle_display_name(path),
                    self._compute_stats(data)).exec()

    def export_arch_csv(self):
        """One selected - an ordinary save-as. More than one - pick a folder
        and copy them all under their original names."""
        import shutil
        sel = [Path(p) for p, v in self.arch_vars.items() if v.get()]
        if not sel:
            core.info(self, "Nothing selected", "Tick a run in the list first.")
            return
        try:
            if len(sel) == 1:
                dest = core.save_path(self, "Save measurement CSV", sel[0].name,
                                      "CSV (*.csv)", self.log_dir)
                if not dest:
                    return
                shutil.copy(sel[0], dest)
                core.info(self, "Saved", dest)
            else:
                folder = core.choose_dir(self, f"Where to put {len(sel)} files?",
                                         self.log_dir)
                if not folder:
                    return
                done, failed = 0, []
                for f in sel:
                    try:
                        shutil.copy(f, Path(folder) / f.name)
                        done += 1
                    except Exception as e:
                        failed.append(f"{f.name}: {e}")
                msg = f"{done} of {len(sel)} files copied to\n{folder}"
                if failed:
                    msg += "\n\nFailed:\n" + "\n".join(failed[:5])
                core.info(self, "Exported", msg)
        except Exception as e:
            core.error(self, "Export error", str(e))

    def save_arch_chart(self):
        if not any(v.get() for v in self.arch_vars.values()):
            core.info(self, "Nothing selected", "Tick at least one run first.")
            return
        dest = core.save_path(self, "Save chart", "comparison.png",
                              "PNG image (*.png);;PDF (*.pdf);;SVG (*.svg)",
                              self.log_dir)
        if not dest:
            return
        try:
            # ALWAYS exported on white - the chart is rebuilt once in the print
            # palette, saved, then rebuilt in the screen palette, so what is on
            # screen is untouched and what lands in the file is printable.
            self.chart_a.export(dest, size=(9.5, 5.4))
            core.info(self, "Saved", dest)
        except Exception as e:
            core.error(self, "Save error", str(e))

    def export_arch_pdf(self):
        path = self._selected_arch_path()
        if not path:
            core.info(self, "Nothing selected", "Tick a run in the list first.")
            return
        data = self._load_cycle_data(path)
        if not data:
            core.error(self, "Unreadable", "Could not load that run.")
            return
        dest = core.save_path(self, "Save PDF report", f"{path.stem}_report.pdf",
                              "PDF report (*.pdf)", self.log_dir)
        if not dest:
            return
        try:
            self._build_pdf_report(path, data, dest)
            core.info(self, "Report saved", dest)
        except Exception as e:
            core.error(self, "PDF error", str(e))

    def _build_pdf_report(self, path, data, dest):
        """An A4 report built with matplotlib alone - no extra dependency, and
        it is on white by construction."""
        from matplotlib.backends.backend_pdf import PdfPages
        from matplotlib.figure import Figure

        t, temp, spt, pwm = data
        st = self._compute_stats(data)
        t0 = t[0]
        tx = [x - t0 for x in t]

        with PdfPages(dest) as pdf:
            fig = Figure(figsize=(8.27, 11.69))            # A4 portrait
            fig.patch.set_facecolor('white')
            fig.text(0.5, 0.96, f"{APP_NAME} — run report", ha='center',
                     fontsize=16, fontweight='bold')
            fig.text(0.5, 0.935,
                     f"{self._cycle_display_name(path)}  ·  generated "
                     f"{datetime.now():%Y-%m-%d %H:%M}",
                     ha='center', fontsize=9, color='gray')

            ax1 = fig.add_axes([0.1, 0.55, 0.82, 0.32])
            ax1.plot(tx, spt, color='#D2691E', lw=1.2, ls='--', label='target',
                     alpha=0.75)
            ax1.plot(tx, temp, color='#1F6FB4', lw=1.8, label='temperature')
            ax1.set_xlabel('time [s]', fontsize=9)
            ax1.set_ylabel('temperature [°C]', fontsize=9)
            ax1.legend(fontsize=9, loc='best')
            ax1.grid(True, alpha=0.3)
            ax1.set_title('Temperature profile', fontsize=11, loc='left')

            ax2 = fig.add_axes([0.1, 0.40, 0.82, 0.10])
            ax2.fill_between(tx, pwm, color='#2E7D32', alpha=0.5)
            ax2.set_xlabel('time [s]', fontsize=8)
            ax2.set_ylabel('power [%]', fontsize=8)
            ax2.grid(True, alpha=0.3)

            settle = (f"{st['settle_time']:.0f} s"
                      if st['settle_time'] is not None else "not reached")
            lines = [("STATISTICS", ""),
                     ("Temperature range", f"{st['tmin']:.1f} – {st['tmax']:.1f} °C"),
                     ("Target", f"{st['target']:.1f} °C"),
                     ("Duration", core.fmt_hms(st['duration'])),
                     ("Average rise rate", f"{st['avg_rise']:.2f} °C/min"),
                     ("Overshoot", f"{st['overshoot']:.2f} °C"),
                     ("Settling time (±1 °C)", settle),
                     ("Steady-state error", f"{st['steady_error']:.3f} °C"),
                     ("Max deviation from ramp", f"{st['max_dev']:.2f} °C"),
                     ("Noise σ", f"±{st['noise_std']:.3f} °C")]
            y = 0.32
            for label, val in lines:
                if not val:
                    fig.text(0.1, y, label, fontsize=11, fontweight='bold')
                else:
                    fig.text(0.12, y, label, fontsize=9, color='#333333')
                    fig.text(0.55, y, val, fontsize=9, fontweight='bold')
                y -= 0.025
            pdf.savefig(fig)

    # ── data folder ─────────────────────────────────────────────────────
    def _set_data_dir(self, newdir):
        newdir = Path(newdir)
        if newdir == self.log_dir:
            return
        # Not while a run is being written: the temporary file is already open
        # in the old folder and archiving would go nowhere.
        if self.cyc_on:
            core.warn(self, "Measurement in progress",
                      "I will not change the folder while a run is being "
                      "saved. Stop the measurement and try again.")
            return
        try:
            newdir.mkdir(parents=True, exist_ok=True)
            probe = newdir / ".lachi_write_test"
            probe.write_text("ok", encoding='utf-8')
            probe.unlink()
        except Exception as e:
            core.error(self, "Data folder", f"I cannot write to\n{newdir}\n\n{e}")
            return
        self.log_dir = newdir
        self._save_setting('data_dir', str(newdir))
        self.refresh_arch()
        self._redraw_arch()

    def choose_data_dir(self):
        p = core.choose_dir(self, "Folder for measurement data", self.log_dir)
        if p:
            self._set_data_dir(p)

    def create_data_dir(self):
        parent = core.choose_dir(self, "Where to create the new folder?",
                                 self.log_dir)
        if not parent:
            return
        name = core.ask_text(self, "New folder", "Folder name:",
                             datetime.now().strftime("Measurements_%Y-%m-%d"))
        if not name:
            return
        safe = re.sub(r'[<>:"/\\|?*]', '_', name).strip().strip('.')
        if not safe:
            core.warn(self, "New folder", "That name is empty.")
            return
        self._set_data_dir(Path(parent) / safe)

    def open_log_folder(self):
        if not core.reveal(self.log_dir):
            core.info(self, "Data folder", str(self.log_dir))

    # ════════════════════════════════════════════════════════════════════
    #  SHUTDOWN
    # ════════════════════════════════════════════════════════════════════
    def closeEvent(self, e):
        # Always hand the system back its right to sleep - otherwise the lock
        # would outlive the closed program until logout.
        try:
            self._wake_lock(False)
        except Exception:
            pass
        try:
            self.disconnect()
        except Exception:
            pass
        e.accept()


def main():
    # High-DPI: Qt6 scales by itself, but rounding the device pixel ratio makes
    # a 150% display land on a half-integer factor and everything goes soft.
    # PassThrough keeps the fractional factor and the text stays sharp.
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)

    app = QApplication(sys.argv)
    app.setApplicationName(APP_NAME)

    # Inter if it is shipped beside the app; otherwise the stack in theme.py
    # falls through to the platform's own UI font.
    from PyQt6.QtGui import QFontDatabase
    assets = Path(__file__).with_name('lachi') / 'assets'
    if assets.is_dir():
        for f in assets.glob('*.[ot]tf'):
            QFontDatabase.addApplicationFont(str(f))

    win = Lachi()
    app.setStyleSheet(win.th.qss())
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
