# DigitShowSide - Real-time Measurement and Control Software
# Copyright (C) 2026 Makoto KUNO
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from __future__ import annotations
import os
import sys
import time
from typing import TYPE_CHECKING
import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtGui import QFont, QFontDatabase
from PySide6.QtWidgets import QComboBox, QHBoxLayout, QLabel, QVBoxLayout, QWidget
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def _make_mono_font(point_size: int) -> QFont:
    font = QFontDatabase.systemFont(QFontDatabase.SystemFont.GeneralFont)
    font.setStyleHint(QFont.StyleHint.Monospace)
    font.setFixedPitch(True)
    font.setPointSize(point_size)
    return font
if TYPE_CHECKING:
    from win_main import MainWindow
from utl_context import DSS_CHDEF_AI_MAX, DSS_CHDEF_PARAM_MAX, DSS_FONT_PT_MD, DSS_FONT_PT_SM, DSS_MB_AI_REGISTER_COUNT, DSS_MB_AO_REGISTER_COUNT, DSS_PREVIEW_CHART_MAX_POINTS, DSS_PREVIEW_NOT_SAVING_POINTS, DSS_TIM_PREVIEW_DATASAVING_INI_MSEC, DSS_TIM_PREVIEW_NOT_SAVING_MSEC, dss_active_parameter_labels, dss_active_physical_labels, dss_active_raw_labels, get_context
from utl_origami import OrigamiBuffer

class PlotPanel(QObject):
    PREVIEW_WINDOW_SEC = 60.0
    REFRESH_INTERVAL_MS = 100
    x_changed = Signal(int, str)
    y_changed = Signal(int, str)

    def __init__(self, default_y_label: str, parent_window: MainWindow, plot_index: int=0):
        super().__init__()
        self._parent = parent_window
        self._plot_index = plot_index
        self._origami = OrigamiBuffer(n_ai=DSS_CHDEF_AI_MAX, n_ao=DSS_MB_AO_REGISTER_COUNT, n_param=DSS_CHDEF_PARAM_MAX, buf_max=DSS_PREVIEW_CHART_MAX_POINTS, not_saving_points=DSS_PREVIEW_NOT_SAVING_POINTS)
        self._origami.reset(saving=False)
        self._build_ui(default_y_label)
        self._curve = None
        self._refresh_timer = QTimer(self)
        self._refresh_timer.setSingleShot(True)
        self._refresh_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._refresh_timer.timeout.connect(self.refresh)

    def _build_ui(self, default_y_label: str) -> None:
        n_par = len(dss_active_parameter_labels(get_context().Control.mode))
        y_options = [f'raw_{i:02d}' for i in range(DSS_MB_AI_REGISTER_COUNT)] + [f'phy_{i:02d}' for i in range(DSS_MB_AI_REGISTER_COUNT)] + [f'par_{i:02d}' for i in range(n_par)]
        x_options = ['time', *y_options]
        label_font = _make_mono_font(DSS_FONT_PT_MD)
        timing_font = _make_mono_font(DSS_FONT_PT_SM)
        panel = QWidget()
        panel_layout = QHBoxLayout(panel)
        panel_layout.setContentsMargins(0, 0, 0, 0)
        panel_layout.setSpacing(4)
        selectors = QWidget()
        sel_layout = QVBoxLayout(selectors)
        sel_layout.setContentsMargins(0, 0, 0, 0)
        x_axis_label = QLabel('X-axis')
        x_axis_label.setFont(label_font)
        sel_layout.addWidget(x_axis_label)
        self._x_combo = QComboBox()
        self._x_combo.setFont(label_font)
        self._x_combo.setMaximumWidth(100)
        self._x_combo.addItems(x_options)
        sel_layout.addWidget(self._x_combo)
        y_axis_label = QLabel('Y-axis')
        y_axis_label.setFont(label_font)
        sel_layout.addWidget(y_axis_label)
        self._y_combo = QComboBox()
        self._y_combo.setFont(label_font)
        self._y_combo.setMaximumWidth(100)
        self._y_combo.addItems(y_options)
        sel_layout.addWidget(self._y_combo)
        sel_layout.addStretch(1)
        self._timing_label = QLabel('point: --\nrate: --.-[sec]\nproc: ---.-[ms]')
        self._timing_label.setFont(timing_font)
        self._timing_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        sel_layout.addWidget(self._timing_label)
        panel_layout.addWidget(selectors)
        self._date_axis = pg.DateAxisItem(orientation='bottom')
        self._plot = pg.PlotWidget(axisItems={'bottom': self._date_axis})
        self._plot.setLabel('bottom', 'Time')
        self._plot.setLabel('left', default_y_label)
        self._plot.showGrid(x=True, y=True)
        self._plot.setAntialiasing(True)
        self._plot.hideButtons()
        self._plot.setMouseEnabled(x=False, y=False)
        self._plot.setMenuEnabled(False)
        self._plot.wheelEvent = lambda _ev: None
        panel_layout.addWidget(self._plot, stretch=1)
        self._x_combo.currentTextChanged.connect(self._on_x_changed)
        self._y_combo.currentTextChanged.connect(self._on_y_changed)
        self.widget = panel

    def _on_x_changed(self, text: str) -> None:
        self.refresh()
        self.x_changed.emit(self._plot_index, text)

    def _on_y_changed(self, text: str) -> None:
        self.refresh()
        self.y_changed.emit(self._plot_index, text)

    def push_sample(self, t: float, ai_raw: np.ndarray, ai_phy: np.ndarray, ao_raw: np.ndarray, params: np.ndarray) -> None:
        self._origami.store(t, ai_raw, ai_phy, ao_raw, params)
        if not self._refresh_timer.isActive():
            self._refresh_timer.start(self.REFRESH_INTERVAL_MS)

    def start_saving(self) -> None:
        self._origami.reset(saving=True)
        self.refresh()

    def stop_saving(self) -> None:
        self._origami.reset(saving=False)
        self.refresh()

    def refresh(self) -> None:
        t0 = time.perf_counter()
        x_key = self._x_combo.currentText()
        y_key = self._y_combo.currentText()
        x_kind, x_idx = _parse_key(x_key)
        y_kind, y_idx = _parse_key(y_key)
        x_data, y_data = self._origami.read_xy(x_kind, x_idx, y_kind, y_idx)
        self._plot.setLabel('bottom', _axis_label(x_key))
        self._plot.setLabel('left', _axis_label(y_key))
        if x_kind == 'time':
            if self._origami.is_saving:
                self._plot.enableAutoRange(axis='x', enable=True)
            else:
                self._plot.enableAutoRange(axis='x', enable=False)
                if x_data.size:
                    latest_t = float(x_data[-1])
                    self._plot.setXRange(latest_t - self.PREVIEW_WINDOW_SEC, latest_t, padding=0.0)
        else:
            self._plot.enableAutoRange(axis='x', enable=True)
        self._plot.enableAutoRange(axis='y', enable=True)
        if self._curve is None:
            self._curve = self._plot.plot([], [], pen=pg.mkPen(width=1))
        assert self._curve is not None
        self._curve.setData(x_data, y_data)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        n_points = int(x_data.size)
        timer_msec = self._origami.current_timer_msec(DSS_TIM_PREVIEW_DATASAVING_INI_MSEC if self._origami.is_saving else DSS_TIM_PREVIEW_NOT_SAVING_MSEC)
        period_sec = timer_msec / 1000.0 if timer_msec > 0 else 0.0
        self._timing_label.setText(f'point: {n_points}\nrate: {period_sec:.1f}[sec]\nproc: {elapsed_ms:.1f}[ms]')
        self._refresh_timer.start(self.REFRESH_INTERVAL_MS)

    def current_timer_msec(self) -> int:
        base = DSS_TIM_PREVIEW_DATASAVING_INI_MSEC if self._origami.is_saving else DSS_TIM_PREVIEW_NOT_SAVING_MSEC
        return self._origami.current_timer_msec(base)

def _parse_key(text: str) -> tuple[str, int]:
    if text == 'time':
        return ('time', -1)
    try:
        prefix, idx_str = text.split('_', 1)
        return (prefix, int(idx_str))
    except ValueError:
        return ('time', -1)

def _axis_label(key: str) -> str:
    if key == 'time':
        return 'Time'
    try:
        prefix, idx_str = key.split('_', 1)
        idx = int(idx_str)
        mode = get_context().Control.mode
        if prefix == 'raw' and idx < len(dss_active_raw_labels(mode)):
            return f'{idx:02d}:{dss_active_raw_labels(mode)[idx]}(i16)'
        if prefix == 'phy' and idx < len(dss_active_physical_labels(mode)):
            return f'{idx:02d}:{dss_active_physical_labels(mode)[idx]}'
        if prefix == 'par' and idx < len(dss_active_parameter_labels(mode)):
            return f'{idx:02d}:{dss_active_parameter_labels(mode)[idx]}'
    except ValueError:
        pass
    return key
