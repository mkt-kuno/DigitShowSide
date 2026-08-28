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
import platform
import subprocess
import sys
import time
from collections.abc import Callable, Mapping
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING
import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QMetaObject, Qt, QThread, QTimer, Slot
from PySide6.QtGui import QAction, QCloseEvent, QFont, QFontDatabase, QIcon
from PySide6.QtWidgets import QApplication, QBoxLayout, QComboBox, QDialog, QFileDialog, QFrame, QGridLayout, QHBoxLayout, QHeaderView, QLabel, QLayout, QLineEdit, QMainWindow, QMenuBar, QMessageBox, QPushButton, QSizePolicy, QTreeWidget, QTreeWidgetItem, QVBoxLayout, QWidget
from dlg_webserver import WebServerDialog
from utl_context import DSS_APP_NAME, DSS_APP_VERSION, DSS_CCH_FLOAT_ERR_RGB, DSS_CCH_FLOAT_WARN_RGB, DSS_CHDEF_PARAM_MAX, DSS_FONT_PT_MD, DSS_FONT_PT_XL, DSS_LOG_LATEST_LINES, DSS_MB_AI_REGISTER_COUNT, DSS_MB_AO_REGISTER_COUNT, DSS_TIM_CONTROL_MS, DSS_VOLTAGE_OUT_LABELS, ControlMode, ControlType, dss_active_parameter_labels, dss_active_physical_labels, dss_active_raw_labels, dss_cch_int16_rgb_for, get_context
from utl_logger import LogRecord, get_logger, parse_level, shutdown_logger
from web_server import ControlApiServer
pg.setConfigOptions(useOpenGL=True)
CONTROL_TYPE_OPTIONS = ['None', 'PreCon', 'Step', 'External']
SAMPLING_TIME_OPTIONS = ['200 ms', '500 ms', '1 sec', '2 sec', '5 sec', '10 sec', '20 sec', '30 sec', '1 min', '2 min', '5 min', '10 min', '20 min', '30 min']
SAMPLING_TIME_DEFAULT_INDEX = 2
CONTROL_TYPE_TO_ENUM: dict[str, ControlType] = {'None': ControlType.NONE, 'PreCon': ControlType.PRECONSOLIDATION, 'Step': ControlType.STEP, 'External': ControlType.EXTERNAL}
SETTINGS_PANEL_WIDTH = 290

def parse_sampling_time_ms(text: str) -> int:
    parts = text.strip().split()
    if len(parts) != 2:
        return 1000
    try:
        v = float(parts[0])
    except ValueError:
        return 1000
    unit = parts[1].lower()
    if unit == 'ms':
        return max(1, int(v))
    if unit == 'sec':
        return max(1, int(v * 1000))
    if unit == 'min':
        return max(1, int(v * 60000))
    return 1000
import contextlib
from dlg_calibration import CalibrationDialog
from dlg_env_var import EnvVarDialog
from dlg_step_ctrl import StepCtrlDialog
from dlg_version import VersionDialog
from dlg_voltage_output import VoltageOutputDialog
from fle_tsv_writer import TsvWriter, default_filename, format_timestamp_local
from pnl_plot import PlotPanel
from trs_modbus import ModbusWorker, _pad_int_array
from utl_config import read_control_mode_from_config, restore_from_config, save_calibration_to_config, save_control_mode_to_config, save_plot_selection_to_config
from utl_prevent_sleep import PreventSleep
from utl_serial import find_arduino_like_port
from utl_single_instance import SingleInstanceGuard
MotorPreConsolidationDialog = None
TorsionalPreConsolidationDialog = None
try:
    from dlg_motor_pre_consolidation import MotorPreConsolidationDialog as _MotorPreConsolidationDialog
    MotorPreConsolidationDialog = _MotorPreConsolidationDialog
except ImportError:
    pass
try:
    from dlg_torsional_pre_consolidation import TorsionalPreConsolidationDialog as _TorsionalPreConsolidationDialog
    TorsionalPreConsolidationDialog = _TorsionalPreConsolidationDialog
except ImportError:
    pass

def resolve_control_mode(argv: list[str] | None=None, environ: Mapping[str, str] | None=None, config_mode: str | None=None) -> ControlMode:
    args = list(sys.argv if argv is None else argv)
    env = os.environ if environ is None else environ
    requested: str | None = None
    for i, arg in enumerate(args):
        if arg == '--mode' and i + 1 < len(args):
            requested = args[i + 1]
        elif arg.startswith('--mode='):
            requested = arg.split('=', 1)[1]
    if requested is None:
        requested = env.get('DSS_CONTROL_MODE') or None
    if requested is None:
        requested = (config_mode or '').strip().upper() or None
    if requested is None:
        return ControlMode.MOTOR
    try:
        return ControlMode[requested.strip().upper()]
    except KeyError:
        raise ValueError(f'unknown control mode: {requested!r} (expected motor|torsional)') from None

def format_window_title(mode: ControlMode) -> str:
    return f'{DSS_APP_NAME} v{DSS_APP_VERSION} ({mode.name.title()} Mode)'
SpecimenDialog = None
MotorSpecimenDialog = None
TorsionalSpecimenDialog = None
try:
    from dlg_motor_specimen import MotorSpecimenDialog as _MotorSpecimenDialog
    MotorSpecimenDialog = _MotorSpecimenDialog
except Exception:
    pass
try:
    from dlg_torsional_specimen import TorsionalSpecimenDialog as _TorsionalSpecimenDialog
    TorsionalSpecimenDialog = _TorsionalSpecimenDialog
except Exception:
    pass
if TYPE_CHECKING:
    from ctl_motor import MotorController
    from ctl_torsional import TorsionalController
_APP_INSTANCE_GUARD: SingleInstanceGuard = SingleInstanceGuard()
_APP_SLEEP_GUARD: PreventSleep = PreventSleep()

class MainWindow(QMainWindow):

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(format_window_title(get_context().Control.mode))
        self.setMinimumSize(1900, 1000)
        self._modbus_thread: QThread | None = None
        self._modbus_worker: ModbusWorker | None = None
        self._calibration_dialog: CalibrationDialog | None = None
        self._voltage_out_dialog: VoltageOutputDialog | None = None
        self._specimen_dialog: QDialog | None = None
        self._pre_con_dialog: QDialog | None = None
        self._step_ctrl_dialog: StepCtrlDialog | None = None
        self._version_dialog: VersionDialog | None = None
        self._env_var_dialog: EnvVarDialog | None = None
        self._webserver_dialog: WebServerDialog | None = None
        self._webserver: ControlApiServer | None = None
        self._calibration = np.zeros((DSS_MB_AI_REGISTER_COUNT, 3), dtype=np.float64)
        self._calibration[:, 1] = 1.0
        restore_from_config(self.appdata_path())
        ctx_local = get_context()
        for i in range(DSS_MB_AI_REGISTER_COUNT):
            self._calibration[i, 0] = float(ctx_local.AIO.AI[i].Cal.a)
            self._calibration[i, 1] = float(ctx_local.AIO.AI[i].Cal.b)
            self._calibration[i, 2] = float(ctx_local.AIO.AI[i].Cal.c)
        self._latest_raw_values = np.zeros(DSS_MB_AI_REGISTER_COUNT, dtype=np.float64)
        self._latest_phy_values = np.zeros(DSS_MB_AI_REGISTER_COUNT, dtype=np.float32)
        self._latest_ao_mV: list[int] = [0] * DSS_MB_AO_REGISTER_COUNT
        self._latest_param = np.zeros(DSS_CHDEF_PARAM_MAX, dtype=np.float32)
        self._plot_panels: list[PlotPanel] = []
        self._save_t0 = 0.0
        self._is_saving = False
        self._is_controlling = False
        self._last_elapsed = 0
        self._tsv_writer: TsvWriter | None = None
        self._tsv_path: Path | None = None
        self._sampling_timer = QTimer(self)
        self._sampling_timer.setSingleShot(False)
        self._sampling_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._sampling_timer.timeout.connect(self._on_sampling_tick)
        self._control_timer = QTimer(self)
        self._control_timer.setSingleShot(False)
        self._control_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._control_timer.setInterval(DSS_TIM_CONTROL_MS)
        self._control_timer.timeout.connect(self._on_control_tick)
        self._build_menu_bar()
        self._build_central_widget()
        self._restore_plot_selection()
        self._setup_modbus()
        self._elapsed_timer = QTimer(self)
        self._elapsed_timer.setInterval(200)
        self._elapsed_timer.timeout.connect(self._update_elapsed)
        self._elapsed_timer.start()
        self._apply_sampling_time_interval()
        self._refresh_button_states()
        self._refresh_mode_label()
        self._init_web_server()

    def _build_menu_bar(self) -> None:
        menu_bar: QMenuBar = self.menuBar()
        analog_in = menu_bar.addMenu('&AnalogIn')
        analog_in.addAction(self._make_action('Calibration &Value', self.show_calibration_value))
        analog_out = menu_bar.addMenu('&AnalogOut')
        analog_out.addAction(self._make_action('&Voltage Output', self.show_voltage_output))
        specimen = menu_bar.addMenu('&Specimen')
        specimen.addAction(self._make_action('&Config', self.show_specimen_config))
        control = menu_bar.addMenu('&Control')
        control.addAction(self._make_action('Pre&Consolidation', self.show_pre_consolidation))
        control.addAction(self._make_action('Step&Control', self.show_step_ctrl))
        other = menu_bar.addMenu('&Other')
        other.addAction(self._make_action('&Version', self.show_version))
        other.addAction(self._make_action('&EnvironmentVariables', self.show_environment_variables))
        other.addAction(self._make_action('&Web Server Info', self.show_webserver))
        other.addAction(self._make_action('Open &Appdata/Log Folder', lambda: self.open_in_explorer(self.log_dir())))
        other.addAction(self._make_action('Open &TempData', lambda: self.open_in_explorer(self.tempdata_path())))
        other.addAction(self._make_action('&Change Mode', self.change_mode_via_restart))

    def _build_central_widget(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 4, 8, 8)
        root.setSpacing(4)
        ctx = get_context()
        self._raw_value_edits = self._build_value_group(root, 'Raw Value (int16_t = -32768 to +32767)', dss_active_raw_labels(ctx.Control.mode), '0', suffix='(i16)')
        self._physical_value_edits = self._build_value_group(root, 'Physical Value', dss_active_physical_labels(ctx.Control.mode), '0.0000', suffix='')
        self._parameter_edits = self._build_value_group(root, 'Parameter', dss_active_parameter_labels(ctx.Control.mode), '0.0000', suffix='')
        self._voltage_out_edits = self._build_value_group(root, 'Voltage Out', DSS_VOLTAGE_OUT_LABELS, '0.0000', suffix='')
        bottom_row = QWidget()
        bottom_layout = QHBoxLayout(bottom_row)
        bottom_layout.setContentsMargins(0, 0, 0, 0)
        bottom_layout.setSpacing(8)
        plot_outer, plot_outer_layout = self._build_simple_header('Plot', bottom_layout)
        plot_body = QWidget()
        plot_layout = QHBoxLayout(plot_body)
        plot_layout.setContentsMargins(2, 2, 2, 2)
        plot_layout.setSpacing(8)
        plot_layout.addWidget(self._build_plot_panel('00:LoadCell(i16)'))
        plot_layout.addWidget(self._build_plot_panel('00:q(kPa)'))
        plot_outer_layout.addWidget(plot_body)
        bottom_layout.addWidget(plot_outer, stretch=3)
        left_stack = QWidget()
        left_stack_layout = QVBoxLayout(left_stack)
        left_stack_layout.setContentsMargins(0, 0, 0, 0)
        left_stack_layout.setSpacing(8)
        log_widget = QWidget()
        log_layout = QVBoxLayout(log_widget)
        log_layout.setContentsMargins(0, 4, 0, 0)
        self._log_viewer = self._build_log_viewer()
        log_layout.addWidget(self._log_viewer)
        get_logger().record_logged.connect(self._on_log_record)
        left_stack_layout.addWidget(log_widget, stretch=1)
        control_group = QFrame()
        control_group.setFixedHeight(155)
        control_group.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        control_group.setFrameShape(QFrame.Shape.Box)
        control_layout = QVBoxLayout(control_group)
        control_layout.setContentsMargins(2, 2, 2, 2)
        control_layout.setSpacing(0)
        self._mode_label = QLabel('Mode: None')
        control_layout.addWidget(self._mode_label)
        control_layout.addStretch(1)
        left_stack_layout.addWidget(control_group, stretch=0)
        save_widget = QWidget()
        save_layout = QHBoxLayout(save_widget)
        save_layout.setContentsMargins(0, 0, 0, 0)
        save_layout.addWidget(QLabel('FilePath'))
        self._save_filename_edit = QLineEdit()
        self._save_filename_edit.setReadOnly(True)
        save_layout.addWidget(self._save_filename_edit, stretch=1)
        self._elapsed_edit = QLineEdit('0')
        self._elapsed_edit.setReadOnly(True)
        self._elapsed_edit.setMaximumWidth(60)
        save_layout.addWidget(self._elapsed_edit)
        save_layout.addWidget(QLabel('[sec]'))
        left_stack_layout.addWidget(save_widget, stretch=0)
        bottom_layout.addWidget(left_stack, stretch=2)
        right_stack = QWidget()
        right_stack_layout = QVBoxLayout(right_stack)
        right_stack_layout.setContentsMargins(0, 0, 0, 0)
        right_stack_layout.setSpacing(4)
        right_stack_layout.setSizeConstraint(QLayout.SizeConstraint.SetFixedSize)
        self._build_current_settings(right_stack_layout)
        self._build_basic_settings(right_stack_layout)
        buttons_grid = QWidget()
        buttons_grid.setFixedWidth(SETTINGS_PANEL_WIDTH)
        buttons_grid_layout = QGridLayout(buttons_grid)
        buttons_grid_layout.setContentsMargins(0, 0, 0, 0)
        buttons_grid_layout.setSpacing(4)
        buttons_grid_layout.setRowStretch(0, 1)
        buttons_grid_layout.setRowStretch(1, 1)
        for text, row, col in [('Start Control', 0, 0), ('Stop Control', 0, 1), ('Start Saving', 1, 0), ('Stop Saving', 1, 1)]:
            btn = QPushButton(text)
            btn.setMinimumHeight(40)
            btn.setFont(self._make_mono_font(DSS_FONT_PT_MD))
            buttons_grid_layout.addWidget(btn, row, col)
            if text == 'Start Control':
                self._start_control_btn = btn
                btn.clicked.connect(self._start_control)
            elif text == 'Stop Control':
                self._stop_control_btn = btn
                btn.clicked.connect(self._stop_control)
            elif text == 'Start Saving':
                self._start_saving_btn = btn
                btn.clicked.connect(self._start_saving)
            elif text == 'Stop Saving':
                self._stop_saving_btn = btn
                btn.clicked.connect(self._stop_saving)
        right_stack_layout.addWidget(buttons_grid, stretch=1)
        bottom_layout.addWidget(right_stack)
        root.addWidget(bottom_row, stretch=1)

    def _build_current_settings(self, parent_layout: QVBoxLayout) -> None:
        outer, outer_layout = self._build_simple_header('Current Settings', parent_layout)
        outer.setFixedWidth(SETTINGS_PANEL_WIDTH)
        label_font = self._make_mono_font(DSS_FONT_PT_MD)
        grid_host = QWidget()
        grid = QGridLayout(grid_host)
        grid.setContentsMargins(4, 6, 4, 6)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)
        control_type_label = QLabel('ControlType')
        control_type_label.setFont(label_font)
        grid.addWidget(control_type_label, 0, 0)
        self._current_control_type = QLabel('00:None')
        self._current_control_type.setFont(label_font)
        self._current_control_type.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        grid.addWidget(self._current_control_type, 0, 1)
        sampling_time_label = QLabel('SamplingTime')
        sampling_time_label.setFont(label_font)
        grid.addWidget(sampling_time_label, 1, 0)
        self._current_sampling_time = QLabel(SAMPLING_TIME_OPTIONS[SAMPLING_TIME_DEFAULT_INDEX])
        self._current_sampling_time.setFont(label_font)
        self._current_sampling_time.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        grid.addWidget(self._current_sampling_time, 1, 1)
        outer_layout.addWidget(grid_host)

    def _build_basic_settings(self, parent_layout: QVBoxLayout) -> None:
        outer, outer_layout = self._build_simple_header('Basic Settings', parent_layout)
        outer.setFixedWidth(SETTINGS_PANEL_WIDTH)
        label_font = self._make_mono_font(DSS_FONT_PT_MD)
        body = QWidget()
        basic_layout = QVBoxLayout(body)
        basic_layout.setContentsMargins(6, 8, 6, 8)
        basic_layout.setSpacing(10)
        row1 = QHBoxLayout()
        row1.addStretch(1)
        control_type_label = QLabel('ControlType')
        control_type_label.setFont(label_font)
        row1.addWidget(control_type_label)
        self._control_type_combo = QComboBox()
        self._control_type_combo.setFont(label_font)
        self._control_type_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        self._control_type_combo.addItems(CONTROL_TYPE_OPTIONS)
        row1.addWidget(self._control_type_combo)
        apply1 = QPushButton('Apply')
        apply1.setFont(label_font)
        apply1.setFixedWidth(60)
        apply1.clicked.connect(self._apply_control_type)
        row1.addWidget(apply1)
        basic_layout.addLayout(row1)
        row2 = QHBoxLayout()
        row2.addStretch(1)
        sampling_time_label = QLabel('SamplingTime')
        sampling_time_label.setFont(label_font)
        row2.addWidget(sampling_time_label)
        self._sampling_time_combo = QComboBox()
        self._sampling_time_combo.setFont(label_font)
        self._sampling_time_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        self._sampling_time_combo.addItems(SAMPLING_TIME_OPTIONS)
        self._sampling_time_combo.setCurrentIndex(SAMPLING_TIME_DEFAULT_INDEX)
        row2.addWidget(self._sampling_time_combo)
        apply2 = QPushButton('Apply')
        apply2.setFont(label_font)
        apply2.setFixedWidth(60)
        apply2.clicked.connect(self._apply_sampling_time)
        row2.addWidget(apply2)
        basic_layout.addLayout(row2)
        outer_layout.addWidget(body)

    def _build_log_viewer(self) -> QTreeWidget:
        tree = QTreeWidget()
        tree.setColumnCount(3)
        tree.setHeaderHidden(True)
        tree.setRootIsDecorated(False)
        tree.setAlternatingRowColors(True)
        tree.setUniformRowHeights(True)
        tree.header().setStretchLastSection(True)
        tree.header().setSectionResizeMode(0, QHeaderView.ResizeMode.Fixed)
        tree.header().setSectionResizeMode(1, QHeaderView.ResizeMode.Fixed)
        tree.header().setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        tree.setColumnWidth(0, 140)
        tree.setColumnWidth(1, 40)
        tree.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        tree.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        return tree

    def _build_plot_panel(self, default_axis_label: str) -> QWidget:
        panel = PlotPanel((_default_y_label := default_axis_label), self, plot_index=len(self._plot_panels))
        panel.x_changed.connect(self._on_plot_xy_changed)
        panel.y_changed.connect(self._on_plot_xy_changed)
        self._plot_panels.append(panel)
        return panel.widget

    def _restore_plot_selection(self) -> None:
        from utl_config import _read_existing_payload
        data = _read_existing_payload(self.appdata_path())
        if not isinstance(data, dict):
            return
        plot = data.get('plot')
        if not isinstance(plot, dict):
            return
        for idx, key in enumerate(('a', 'b')):
            if idx >= len(self._plot_panels):
                break
            entry = plot.get(key)
            if not isinstance(entry, dict):
                continue
            panel = self._plot_panels[idx]
            x = entry.get('x')
            y = entry.get('y')
            if isinstance(x, str):
                i = panel._x_combo.findText(x)
                if i >= 0:
                    panel._x_combo.setCurrentIndex(i)
            if isinstance(y, str):
                i = panel._y_combo.findText(y)
                if i >= 0:
                    panel._y_combo.setCurrentIndex(i)

    def _on_plot_xy_changed(self, plot_index: int, _text: str) -> None:
        selections: list[tuple[str | None, str | None]] = []
        for _i, panel in enumerate(self._plot_panels):
            selections.append((panel._x_combo.currentText(), panel._y_combo.currentText()))
        save_plot_selection_to_config(selections, self.appdata_path())

    def _build_value_group(self, parent_layout: QVBoxLayout, title: str, labels: list[str], initial_value: str, suffix: str) -> list[QLabel]:
        _outer, outer_layout = self._build_simple_header(title, parent_layout)
        grid_host = QWidget()
        outer_layout.addWidget(grid_host)
        grid = QGridLayout(grid_host)
        grid.setContentsMargins(4, 0, 4, 0)
        grid.setVerticalSpacing(0)
        value_font = self._make_mono_font(DSS_FONT_PT_XL)
        label_font = self._make_mono_font(DSS_FONT_PT_MD)
        value_labels: list[QLabel] = []
        for i, name in enumerate(labels):
            row, col = divmod(i, 8)
            cell = QWidget()
            cell_layout = QVBoxLayout(cell)
            cell_layout.setContentsMargins(0, 0, 0, 0)
            cell_layout.setSpacing(0)
            label_text = f'{i:02d}:{name}' if not suffix else f'{i:02d}:{name}{suffix}'
            label = QLabel(label_text)
            label.setFont(label_font)
            cell_layout.addWidget(label)
            value = QLabel(initial_value)
            value.setFrameShape(QFrame.Shape.Box)
            value.setStyleSheet('padding: 0px;')
            value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            value.setMinimumWidth(100)
            value.setFont(value_font)
            cell_layout.addWidget(value)
            grid.addWidget(cell, row, col)
            value_labels.append(value)
        return value_labels

    def _build_simple_header(self, title: str, parent_layout: QBoxLayout) -> tuple[QWidget, QVBoxLayout]:
        outer = QWidget()
        outer_layout = QVBoxLayout(outer)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.setSpacing(0)
        header = QWidget()
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(6)
        left_bar = QFrame()
        left_bar.setFrameShape(QFrame.Shape.HLine)
        left_bar.setFrameShadow(QFrame.Shadow.Sunken)
        left_bar.setFixedWidth(12)
        header_layout.addWidget(left_bar)
        title_label = QLabel(title)
        title_label.setFont(self._make_mono_font(DSS_FONT_PT_MD))
        header_layout.addWidget(title_label)
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setFrameShadow(QFrame.Shadow.Sunken)
        header_layout.addWidget(line, stretch=1)
        outer_layout.addWidget(header)
        parent_layout.addWidget(outer)
        return (outer, outer_layout)

    def _make_action(self, text: str, slot: Callable[[], None]) -> QAction:
        action = QAction(text, self)
        action.triggered.connect(slot)
        return action

    def _make_mono_font(self, point_size: int) -> QFont:
        font = QFontDatabase.systemFont(QFontDatabase.SystemFont.GeneralFont)
        font.setStyleHint(QFont.StyleHint.Monospace)
        font.setFixedPitch(True)
        font.setPointSize(point_size)
        return font

    def _set_value_label_color(self, label: QLabel, rgb: tuple[int, int, int] | None) -> None:
        if rgb is None:
            label.setStyleSheet('padding: 0px;')
        else:
            r, g, b = rgb
            label.setStyleSheet(f'padding: 0px; color: rgb({r}, {g}, {b});')

    def _setup_modbus(self) -> None:
        result = find_arduino_like_port()
        if result is None:
            self._log('ERR', 'No Arduino-like USB serial device found (check VID/PID: 2341 / 2E8A / 0483 / 1A86 / 10C4 / 0403)')
            return
        _, port, name = result
        self._log('INFO', f'Detected {name} on {port.device}')
        self._modbus_thread = QThread(self)
        self._modbus_worker = ModbusWorker(port.device)
        self._modbus_worker.moveToThread(self._modbus_thread)
        self._modbus_thread.started.connect(self._modbus_worker.start)
        self._modbus_worker.new_values.connect(self._on_modbus_values)
        self._modbus_worker.new_values.connect(self._apply_physical_values)
        self._modbus_worker.new_values.connect(self._push_raw_to_calibration_dialog)
        self._modbus_worker.new_values.connect(self._on_modbus_sample_for_plot)
        self._modbus_worker.voltage_changed.connect(self._on_voltage_changed)
        self._modbus_worker.error.connect(self._log_modbus_error)
        self._modbus_worker.status.connect(self._log_modbus_status)
        self._modbus_thread.start()

    def _teardown_modbus(self) -> None:
        if self._is_saving:
            self._stop_saving()
        if self._is_controlling:
            self._stop_control()
        if self._modbus_worker is not None:
            QMetaObject.invokeMethod(self._modbus_worker, 'stop', Qt.ConnectionType.BlockingQueuedConnection)
        if self._modbus_thread is not None:
            self._modbus_thread.quit()
            self._modbus_thread.wait(2000)

    @Slot(int, int)
    def _on_voltage_changed(self, channel: int, mv: int) -> None:
        if 0 <= channel < len(self._voltage_out_edits):
            self._voltage_out_edits[channel].setText(f'{mv / 1000.0:.4f}')
        if 0 <= channel < len(self._latest_ao_mV):
            self._latest_ao_mV[channel] = mv

    @Slot(list)
    def _on_modbus_values(self, values: list[int | float]) -> None:
        arr = _pad_int_array(np.asarray(values, dtype=np.float64), DSS_MB_AI_REGISTER_COUNT, np.float64)
        raw_i16 = arr.astype(np.int16)
        abs_arr = np.abs(arr)
        for i in range(DSS_MB_AI_REGISTER_COUNT):
            edit = self._raw_value_edits[i]
            edit.setText(str(int(raw_i16[i])))
            self._set_value_label_color(edit, dss_cch_int16_rgb_for(abs_arr[i]))
        self._latest_raw_values = arr
        ctype = get_context().Control.type
        text = next((t for t, e in CONTROL_TYPE_TO_ENUM.items() if e == ctype), CONTROL_TYPE_OPTIONS[0])
        self._current_control_type.setText(f'{int(ctype):02d}:{text}')
        self._current_sampling_time.setText(self._sampling_time_text_for(self._sampling_timer.interval()))

    @Slot(list)
    def _apply_physical_values(self, values: list[int | float] | None=None) -> None:
        if values is None:
            arr = self._latest_raw_values
        else:
            arr = _pad_int_array(np.asarray(values, dtype=np.float64), DSS_MB_AI_REGISTER_COUNT, np.float64)
        x = arr.astype(np.float32, copy=False)
        a = self._calibration[:, 0].astype(np.float32, copy=False)
        b = self._calibration[:, 1].astype(np.float32, copy=False)
        c = self._calibration[:, 2].astype(np.float32, copy=False)
        phy = a * x * x + b * x + c
        for i in range(DSS_MB_AI_REGISTER_COUNT):
            self._physical_value_edits[i].setText(f'{float(phy[i]):.4f}')
        self._latest_phy_values = phy.astype(np.float32, copy=False)
        self._refresh_parameter_from_ctx()

    @Slot(int)
    def _on_calibration_coeff_changed(self, idx: int) -> None:
        if idx < 0 or idx >= DSS_MB_AI_REGISTER_COUNT:
            return
        dlg = self._calibration_dialog
        if dlg is None:
            return
        if hasattr(dlg, 'coefficients_valid') and (not dlg.coefficients_valid(idx)):
            return
        a, b, c = dlg._get_coeff(idx)
        self._calibration[idx] = (a, b, c)
        ctx = get_context()
        ctx.AIO.AI[idx].Cal.a = float(a)
        ctx.AIO.AI[idx].Cal.b = float(b)
        ctx.AIO.AI[idx].Cal.c = float(c)
        save_calibration_to_config(self.appdata_path())
        self._apply_physical_values()
        for panel in self._plot_panels:
            panel.refresh()

    @Slot(list)
    def _push_raw_to_calibration_dialog(self, values: list[int | float]) -> None:
        dialog = self._calibration_dialog
        if dialog is not None and dialog.isVisible():
            dialog.update_raw_values(values)

    @Slot(list)
    def _on_modbus_sample_for_plot(self, values: list[int | float]) -> None:
        ai_raw = _pad_int_array(np.asarray(values, dtype=np.float32), DSS_MB_AI_REGISTER_COUNT, np.float32)
        ai_phy = self._latest_phy_values.astype(np.float32, copy=False)
        ao_raw = np.asarray(self._latest_ao_mV, dtype=np.float32)
        params = self._latest_param.astype(np.float32, copy=False)
        t = time.time()
        for panel in self._plot_panels:
            panel.push_sample(t, ai_raw, ai_phy, ao_raw, params)

    def _current_sampling_text(self) -> str:
        return self._sampling_time_combo.currentText()

    def _apply_sampling_time_interval(self) -> None:
        ms = parse_sampling_time_ms(self._current_sampling_text())
        self._sampling_timer.setInterval(ms)

    def _apply_sampling_time(self) -> None:
        self._apply_sampling_time_interval()
        self._current_sampling_time.setText(self._current_sampling_text())
        self._log('INFO', f'SamplingTime = {self._current_sampling_text()}')

    def _apply_control_type(self) -> None:
        text = self._control_type_combo.currentText()
        ctype = CONTROL_TYPE_TO_ENUM.get(text, ControlType.NONE)
        self._current_control_type.setText(f'{int(ctype):02d}:{text}')
        ctx = get_context()
        ctx.Control.type = ctype
        self._refresh_mode_label()
        self._refresh_button_states()

    def _sampling_time_text_for(self, ms: int) -> str:
        for opt in SAMPLING_TIME_OPTIONS:
            if parse_sampling_time_ms(opt) == ms:
                return opt
        return f'{ms} msec'

    def _on_sampling_tick(self) -> None:
        if not self._is_saving:
            return
        if self._tsv_writer is None or not self._tsv_writer.is_open:
            return
        self._compute_and_append_sample()

    def _active_controller_class(self) -> type[MotorController] | type[TorsionalController]:
        from ctl_motor import MotorController
        from ctl_torsional import TorsionalController
        if get_context().Control.mode == ControlMode.TORSIONAL:
            return TorsionalController
        return MotorController

    def _refresh_parameter_from_ctx(self) -> None:
        ctx = get_context()
        ctx.AIO.AI.raw[:] = self._latest_raw_values.astype(np.float32, copy=False)
        ctx.AIO.AI.phy[:] = self._latest_phy_values
        try:
            self._active_controller_class()().calculate_param()
        except Exception as exc:
            self._log('ERR', f'calculate_param failed: {exc}')
            return
        self._latest_param = np.asarray(ctx.AIO.param, copy=True)
        err_mask = np.isnan(self._latest_param)
        warn_mask = ~err_mask & np.isinf(self._latest_param)
        n = min(len(self._parameter_edits), DSS_CHDEF_PARAM_MAX)
        for i in range(n):
            edit = self._parameter_edits[i]
            edit.setText(f'{float(self._latest_param[i]):.4f}')
            if err_mask[i]:
                rgb: tuple[int, int, int] | None = DSS_CCH_FLOAT_ERR_RGB
            elif warn_mask[i]:
                rgb = DSS_CCH_FLOAT_WARN_RGB
            else:
                rgb = None
            self._set_value_label_color(edit, rgb)

    def _compute_and_append_sample(self) -> None:
        if self._tsv_writer is None:
            return
        ao_mV = np.asarray(self._latest_ao_mV, dtype=np.uint16)
        now = datetime.now()
        self._tsv_writer.append_row(timestamp=now, ai_raw=self._latest_raw_values, ai_phy=self._latest_phy_values, ao_raw=ao_mV, par=self._latest_param[:DSS_CHDEF_PARAM_MAX])

    def _start_saving(self) -> None:
        if self._is_saving:
            return
        default_dir = Path.home() / 'Desktop'
        default_dir.mkdir(parents=True, exist_ok=True)
        default_name = default_filename()
        default_path = default_dir / default_name
        path, _ = QFileDialog.getSaveFileName(self, 'Start Saving (TSV)', str(default_path), 'TSV Files (*.tsv);;All Files (*)')
        if not path:
            return
        if not path.lower().endswith('.tsv'):
            path = path + '.tsv'
        try:
            writer = TsvWriter(path)
            writer.open()
        except OSError as exc:
            QMessageBox.warning(self, 'Save Error', str(exc))
            return
        self._tsv_writer = writer
        self._tsv_path = Path(path)
        self._save_filename_edit.setText(str(self._tsv_path))
        self._is_saving = True
        self._save_t0 = time.monotonic()
        self._last_elapsed = 0
        self._elapsed_edit.setText('0')
        self._apply_sampling_time_interval()
        self._sampling_timer.start()
        for panel in self._plot_panels:
            panel.start_saving()
        self._log('INFO', f'Saving started: {self._tsv_path}')
        self._refresh_button_states()

    def _stop_saving(self) -> None:
        if not self._is_saving:
            return
        self._sampling_timer.stop()
        for panel in self._plot_panels:
            panel.stop_saving()
        if self._tsv_writer is not None:
            try:
                self._tsv_writer.close()
            except Exception as exc:
                self._log('ERR', f'TSV close failed: {exc}')
        self._log('INFO', f'Saving stopped: {self._tsv_path}')
        self._tsv_writer = None
        self._tsv_path = None
        self._save_filename_edit.clear()
        self._last_elapsed = 0
        self._elapsed_edit.setText('0')
        self._is_saving = False
        self._refresh_button_states()

    def _start_control(self) -> None:
        if self._is_controlling:
            return
        ctx = get_context()
        if ctx.Control.type == ControlType.NONE:
            self._log('WARN', 'ControlType is None; cannot start control')
            return
        if ctx.Control.type == ControlType.EXTERNAL:
            self._log('INFO', 'ControlType=EXTERNAL: WebServer mode')
            ctx.Control.mode = ControlMode.MOTOR
            ctx.Control.watch.reset()
            ctx.Control.watch.start()
            self._is_controlling = True
            self._control_timer.start()
            self._refresh_button_states()
            self._refresh_mode_label()
            return
        ctx.Control.watch.reset()
        ctx.Control.watch.start()
        ctx.Flag.control = True
        try:
            self._active_controller_class()().start()
        except Exception as exc:
            self._log('ERR', f'Controller start failed: {exc}')
        self._request_analog_output_update()
        self._is_controlling = True
        self._control_timer.start()
        self._log('INFO', f'Control started: type={int(ctx.Control.type)}')
        self._refresh_button_states()
        self._refresh_mode_label()

    def _stop_control(self) -> None:
        if not self._is_controlling:
            return
        self._control_timer.stop()
        ctx = get_context()
        ctx.Flag.control = False
        if ctx.Control.type != ControlType.EXTERNAL:
            try:
                self._active_controller_class()().stop()
            except Exception as exc:
                self._log('ERR', f'Controller stop failed: {exc}')
            self._request_analog_output_update()
        ctx.Control.watch.stop()
        ctx.Control.watch.reset()
        self._is_controlling = False
        self._log('INFO', 'Control stopped')
        self._refresh_button_states()
        self._refresh_mode_label()

    def _on_control_tick(self) -> None:
        ctx = get_context()
        if not ctx.Flag.control and ctx.Control.type != ControlType.EXTERNAL:
            return
        dt_sec = self._control_timer.interval() / 1000.0
        if ctx.Control.mode == ControlMode.MOTOR:
            run_ao_update = self._tick_motor_control(dt_sec)
        elif ctx.Control.mode == ControlMode.TORSIONAL:
            run_ao_update = self._tick_torsional_control(dt_sec)
        else:
            run_ao_update = True
        if not run_ao_update:
            return
        self._request_analog_output_update()

    def _tick_motor_control(self, dt_sec: float) -> bool:
        if get_context().Control.type == ControlType.EXTERNAL:
            try:
                from ctl_motor import MotorController
                MotorController().calculate_param()
            except Exception as exc:
                self._log('ERR', f'calculate_param failed: {exc}')
            return True
        try:
            from ctl_motor import CtrlResult, MotorController
            ret = MotorController().step(dt_sec=dt_sec)
            if ret == CtrlResult.STOP:
                self._stop_control()
                return False
        except Exception as exc:
            self._log('ERR', f'Motor step failed: {exc}')
            self._stop_control()
            return False
        return True

    def _tick_torsional_control(self, dt_sec: float) -> bool:
        if get_context().Control.type == ControlType.EXTERNAL:
            try:
                from ctl_torsional import TorsionalController
                TorsionalController().calculate_param()
            except Exception as exc:
                self._log('ERR', f'calculate_param failed: {exc}')
            return True
        try:
            from ctl_motor import CtrlResult
            from ctl_torsional import TorsionalController
            ret = TorsionalController().step(dt_sec=dt_sec)
            if ret == CtrlResult.STOP:
                self._stop_control()
                return False
        except Exception as exc:
            self._log('ERR', f'Torsional step failed: {exc}')
            self._stop_control()
            return False
        return True

    def _request_analog_output_update(self) -> None:
        from utl_context import DSS_CHDEF_AO_MAX
        changed = False
        for ch in range(DSS_CHDEF_AO_MAX):
            raw_v = float(get_context().AIO.AO[ch].raw)
            if raw_v > 10.0:
                raw_v = 10.0
            elif raw_v < 0.0:
                raw_v = 0.0
            get_context().AIO.AO[ch].raw = raw_v
            mv = round(raw_v * 1000.0)
            self._request_voltage_out(ch, mv)
            if 0 <= ch < len(self._voltage_out_edits):
                self._voltage_out_edits[ch].setText(f'{raw_v:.4f}')
            if 0 <= ch < len(self._latest_ao_mV):
                self._latest_ao_mV[ch] = mv
                changed = True
        if changed:
            self._refresh_mode_label()

    def _refresh_button_states(self) -> None:
        self._start_saving_btn.setEnabled(not self._is_saving)
        self._stop_saving_btn.setEnabled(self._is_saving)
        self._start_control_btn.setEnabled(not self._is_controlling)
        self._stop_control_btn.setEnabled(self._is_controlling)

    def _refresh_mode_label(self) -> None:
        get_context()
        if not self._is_controlling:
            self._mode_label.setText('Mode: None')
        else:
            info = self._active_controller_class()().info_string()
            self._mode_label.setText(info or 'Mode: Active')

    @Slot()
    def _update_elapsed(self) -> None:
        if not self._is_saving:
            return
        elapsed = time.monotonic() - self._save_t0
        seconds = int(elapsed)
        if seconds != self._last_elapsed:
            self._last_elapsed = seconds
            self._elapsed_edit.setText(str(seconds))

    def _read_current_raw_values(self) -> np.ndarray:
        out = np.zeros(DSS_MB_AI_REGISTER_COUNT, dtype=np.int32)
        for i in range(DSS_MB_AI_REGISTER_COUNT):
            with contextlib.suppress(ValueError):
                out[i] = int(self._raw_value_edits[i].text())
        return out

    @Slot(str)
    def _log_modbus_error(self, msg: str) -> None:
        self._log('ERR', msg)

    @Slot(str)
    def _log_modbus_status(self, msg: str) -> None:
        self._log('INFO', msg)

    def _log(self, level: str, msg: str) -> None:
        get_logger().log(parse_level(level), msg)

    @Slot(object)
    def _on_log_record(self, record: LogRecord) -> None:
        timestamp = format_timestamp_local(record.ts)
        item = QTreeWidgetItem([timestamp, record.level.label, record.msg])
        self._log_viewer.addTopLevelItem(item)
        while self._log_viewer.topLevelItemCount() > DSS_LOG_LATEST_LINES:
            self._log_viewer.takeTopLevelItem(0)

    def closeEvent(self, event: QCloseEvent) -> None:
        if self._is_saving or self._is_controlling:
            parts: list[str] = []
            if self._is_saving:
                parts.append('Saving is in progress.')
            if self._is_controlling:
                parts.append('Control is running.')
            message = ' '.join(parts) + ' Are you sure you want to quit?'
            reply = QMessageBox.question(self, 'Confirm Quit', message, QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No, QMessageBox.StandardButton.No)
            if reply != QMessageBox.StandardButton.Yes:
                event.ignore()
                return
        if self._webserver is not None:
            self._webserver.stop()
        self._teardown_modbus()
        get_logger().close()
        super().closeEvent(event)

    def show_version(self) -> None:
        if self._version_dialog is None:
            self._version_dialog = VersionDialog(self)
        self._version_dialog.show()
        self._version_dialog.raise_()
        self._version_dialog.activateWindow()

    def show_calibration_value(self) -> None:
        if self._calibration_dialog is None:
            self._calibration_dialog = CalibrationDialog(self)
            self._calibration_dialog.init_from_state([(float(a), float(b), float(c)) for a, b, c in self._calibration.tolist()])
            self._calibration_dialog.coefficients_changed.connect(self._on_calibration_coeff_changed)
        self._calibration_dialog.update_raw_values(self._read_current_raw_values())
        self._calibration_dialog.show()
        self._calibration_dialog.raise_()
        self._calibration_dialog.activateWindow()

    def show_voltage_output(self) -> None:
        if self._voltage_out_dialog is None:
            self._voltage_out_dialog = VoltageOutputDialog(self)
        if hasattr(self._voltage_out_dialog, '_refresh_from_ctx'):
            self._voltage_out_dialog._refresh_from_ctx()
        self._voltage_out_dialog.show()
        self._voltage_out_dialog.raise_()
        self._voltage_out_dialog.activateWindow()

    def show_specimen_config(self) -> None:
        ctx = get_context()
        existing = self._specimen_dialog
        want_motor = ctx.Control.mode == ControlMode.MOTOR
        target_cls: type | None
        if want_motor:
            target_cls = MotorSpecimenDialog if MotorSpecimenDialog is not None else SpecimenDialog
        else:
            target_cls = TorsionalSpecimenDialog if TorsionalSpecimenDialog is not None else SpecimenDialog
        if existing is not None and type(existing) is not target_cls:
            with contextlib.suppress(Exception):
                existing.close()
            existing = None
            self._specimen_dialog = None
        if existing is None and target_cls is not None:
            existing = target_cls(self)
            if hasattr(existing, 'specimen_changed'):
                existing.specimen_changed.connect(self._on_specimen_changed)
            self._specimen_dialog = existing
        if existing is None:
            return
        if hasattr(existing, 'refresh_from_ctx'):
            existing.refresh_from_ctx()
        assert isinstance(existing, QDialog)
        existing.show()
        existing.raise_()
        existing.activateWindow()

    def show_pre_consolidation(self) -> None:
        ctx = get_context()
        want_torsional = ctx.Control.mode == ControlMode.TORSIONAL
        target_cls: type | None = TorsionalPreConsolidationDialog if want_torsional else MotorPreConsolidationDialog
        if target_cls is None:
            mode_name = 'TORSIONAL' if want_torsional else 'MOTOR'
            self._log('ERR', f'PreConsolidation dialog ({mode_name}) failed to import')
            return
        existing = self._pre_con_dialog
        if existing is not None and type(existing) is not target_cls:
            with contextlib.suppress(Exception):
                existing.close()
            existing = None
            self._pre_con_dialog = None
        if existing is None and target_cls is not None:
            existing = target_cls(self)
            if hasattr(existing, 'preconsolidation_changed'):
                existing.preconsolidation_changed.connect(self._on_preconsolidation_changed)
            self._pre_con_dialog = existing
        if existing is None:
            return
        if hasattr(existing, '_refresh_from_ctx'):
            existing._refresh_from_ctx()
        assert isinstance(existing, QDialog)
        existing.show()
        existing.raise_()
        existing.activateWindow()

    def show_step_ctrl(self) -> None:
        if self._step_ctrl_dialog is None:
            self._step_ctrl_dialog = StepCtrlDialog(self)
        self._step_ctrl_dialog.show()
        self._step_ctrl_dialog.raise_()
        self._step_ctrl_dialog.activateWindow()

    def change_mode_via_restart(self) -> None:
        if self._is_controlling:
            QMessageBox.warning(self, 'Change Mode', 'Control is running. Stop control before changing mode.')
            return
        ctx = get_context()
        target = ControlMode.TORSIONAL if ctx.Control.mode == ControlMode.MOTOR else ControlMode.MOTOR
        reply = QMessageBox.question(self, 'Change Mode', f'Restart in {target.name} mode?', QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No, QMessageBox.StandardButton.No)
        if reply != QMessageBox.StandardButton.Yes:
            return
        if not save_control_mode_to_config(target.name, self.appdata_path()):
            self._log('ERR', f'Failed to save control_mode={target.name} to config')
            return
        self._log('INFO', f'Restarting in {target.name} mode...')
        subprocess.Popen([sys.executable, *sys.argv])
        app = QApplication.instance()
        assert app is not None
        app.quit()

    def _on_specimen_changed(self) -> None:
        self._log('INFO', 'Specimen data updated')

    def _on_preconsolidation_changed(self) -> None:
        self._log('INFO', 'TorsionalPreConsolidation updated')

    def _request_voltage_out(self, channel: int, value_mv: int) -> None:
        if self._modbus_worker is None:
            self._log('ERR', 'Modbus not connected')
            return
        self._modbus_worker.write_register_request.emit(channel, value_mv)

    def show_environment_variables(self) -> None:
        if self._env_var_dialog is None:
            self._env_var_dialog = EnvVarDialog(self)
        self._env_var_dialog.refresh()
        self._env_var_dialog.show()
        self._env_var_dialog.raise_()
        self._env_var_dialog.activateWindow()

    def _init_web_server(self) -> None:
        try:
            from utl_origami import set_default_buffer
            if self._plot_panels:
                first_panel = self._plot_panels[0]
                origami = getattr(first_panel, '_origami', None)
                if origami is not None:
                    set_default_buffer(origami)
        except Exception as exc:
            self._log('WARN', f'OrigamiBuffer singleton setup failed: {exc}')
        server = ControlApiServer(parent=self)
        server.control_start_requested.connect(self._start_control)
        server.control_stop_requested.connect(self._stop_control)
        server.analog_output_update_requested.connect(self._request_analog_output_update)
        ok = server.start()
        if ok:
            self._log('INFO', f'WebServer started on 0.0.0.0:{server.port()}')
        else:
            self._log('WARN', 'WebServer failed to start after 5 retries')
        self._webserver = server

    def show_webserver(self) -> None:
        if self._webserver is None:
            from PySide6.QtWidgets import QMessageBox
            QMessageBox.warning(self, 'WebServer', 'WebServer is not initialized')
            return
        if self._webserver_dialog is None:
            self._webserver_dialog = WebServerDialog(self._webserver, self)
        self._webserver_dialog.show()
        self._webserver_dialog.raise_()
        self._webserver_dialog.activateWindow()

    def appdata_path(self) -> Path:
        base = os.environ.get('APPDATA') or str(Path.home() / 'AppData' / 'Roaming')
        return Path(base) / DSS_APP_NAME

    def tempdata_path(self) -> Path:
        return Path(os.environ.get('TEMP') or os.environ.get('TMP') or str(Path.home())) / DSS_APP_NAME

    def log_dir(self) -> Path:
        resolved = get_logger().log_dir()
        if resolved is not None:
            return resolved
        return self.appdata_path() / 'Log'

    def open_in_explorer(self, path: Path) -> None:
        path.mkdir(parents=True, exist_ok=True)
        if platform.system() == 'Windows':
            subprocess.Popen(['explorer', str(path)])
        elif platform.system() == 'Darwin':
            subprocess.Popen(['open', str(path)])
        else:
            subprocess.Popen(['xdg-open', str(path)])

def main() -> int:
    try:
        mode = resolve_control_mode(config_mode=read_control_mode_from_config())
    except ValueError as exc:
        print(f'DigitShowSide: {exc}', file=sys.stderr)
        print('usage: DigitShowSide [--mode motor|torsional]', file=sys.stderr)
        return 2
    if not _APP_INSTANCE_GUARD.try_acquire():
        return 0
    app = QApplication(sys.argv)
    app.setApplicationName(DSS_APP_NAME)
    app.setApplicationVersion(DSS_APP_VERSION)
    _icon_path = Path(__file__).resolve().parent / 'app_icon.png'
    if _icon_path.is_file():
        app.setWindowIcon(QIcon(str(_icon_path)))
    get_logger().install()
    app.aboutToQuit.connect(shutdown_logger)
    app.aboutToQuit.connect(_APP_INSTANCE_GUARD.release)
    app.aboutToQuit.connect(_APP_SLEEP_GUARD.release)
    _APP_SLEEP_GUARD.acquire()
    get_context().Control.mode = mode
    w = MainWindow()
    w.resize(1900, 1000)
    w.show()
    return app.exec()
if __name__ == '__main__':
    sys.exit(main())
