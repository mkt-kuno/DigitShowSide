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
import contextlib
import json
import os
import sys
from collections.abc import Sequence
from datetime import datetime
from pathlib import Path
from typing import ClassVar, Protocol
import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QFontDatabase
from PySide6.QtWidgets import QCheckBox, QComboBox, QDialog, QFileDialog, QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMessageBox, QPushButton, QScrollArea, QSpinBox, QVBoxLayout, QWidget
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ctl_motor_step_ctrl_class import ArgSpec, MotorStepControlClass
from ctl_torsional_step_ctrl_class import TorsionalStepControlClass
from utl_context import DSS_CHDEF_STEPCTRL_ARGS_MAX, DSS_CHDEF_STEPCTRL_STEP_MAX, DSS_FONT_PT_MD, DSS_TIM_CONTROL_MS, ControlMode, get_context

class _StepControlBaseProto(Protocol):
    CTRL_NUM: ClassVar[int]
    NAME: ClassVar[str]
    ARGS: ClassVar[tuple[ArgSpec, ...]]
    MAX_ARGS: ClassVar[int]

    @classmethod
    def all_subclasses(cls) -> Sequence[type[_StepControlBaseProto]]:
        ...

    @classmethod
    def get_class_by_ctrl_num(cls, ctrl_num: int) -> type[_StepControlBaseProto] | None:
        ...

def _active_step_control_base() -> type[_StepControlBaseProto]:
    if get_context().Control.mode == ControlMode.TORSIONAL:
        return TorsionalStepControlClass
    return MotorStepControlClass

class StepCtrlDialog(QDialog):

    def __init__(self, parent: QWidget | None=None) -> None:
        super().__init__(parent)
        self.setModal(True)
        self.setWindowTitle('Step Control')
        self.resize(820, 720)
        self.setMinimumWidth(820)
        self._arg_slots: list[QFrame] = []
        self._arg_edits: list[QLineEdit] = []
        self._arg_unit_labels: list[QLabel] = []
        self._arg_desc_labels: list[QLabel] = []
        self._cur_step_spin: QSpinBox | None = None
        self._cur_ctrl_name_label: QLabel | None = None
        self._prev_step_label: QLabel | None = None
        self._prev_ctrl_name_label: QLabel | None = None
        self._next_step_label: QLabel | None = None
        self._next_ctrl_name_label: QLabel | None = None
        self._target_step_spin: QSpinBox | None = None
        self._ctrl_combo: QComboBox | None = None
        self._edit_check: QCheckBox | None = None
        self._edit_label: QLabel | None = None
        self._step_poll_timer: QTimer | None = None
        self._refreshing: bool = False
        self._editing_step: int = self._clamp_step(get_context().Control.current_step)
        self._last_known_current_step: int = self._editing_step
        self._build_ui()
        self._init_step_poll_timer()
        self._refresh_all()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(6)
        font = QFont()
        font.setPointSize(12)
        self.setFont(font)
        input_font = QFont()
        input_font.setPointSize(14)
        mono_font = self._make_mono_font(DSS_FONT_PT_MD)
        bold_font = QFont(font.family(), 11)
        bold_font.setBold(True)
        top_btn_row = QHBoxLayout()
        top_btn_row.addStretch(1)
        read_btn = QPushButton('Import')
        read_btn.clicked.connect(self._on_read_file)
        top_btn_row.addWidget(read_btn)
        save_btn = QPushButton('Export')
        save_btn.clicked.connect(self._on_save_file)
        top_btn_row.addWidget(save_btn)
        root.addLayout(top_btn_row)
        top_row = QHBoxLayout()
        top_row.setSpacing(8)
        prev_group, self._prev_step_label, self._prev_ctrl_name_label = self._build_preview_group('Previous Step Preview', mono_font)
        top_row.addWidget(prev_group, stretch=1)
        cur_step_group = QGroupBox('Current Step Editor')
        cur_step_grid = QGridLayout(cur_step_group)
        cur_step_grid.setHorizontalSpacing(6)
        cur_step_grid.setVerticalSpacing(4)
        cur_step_grid.addWidget(QLabel('Step No.'), 0, 0)
        self._cur_step_spin = QSpinBox()
        self._cur_step_spin.setRange(0, DSS_CHDEF_STEPCTRL_STEP_MAX - 1)
        self._cur_step_spin.setValue(0)
        self._cur_step_spin.setEnabled(False)
        self._cur_step_spin.setMaximumWidth(110)
        self._cur_step_spin.valueChanged.connect(self._on_cur_step_changed)
        cur_step_grid.addWidget(self._cur_step_spin, 0, 1, 1, 3)
        self._edit_check = QCheckBox()
        self._edit_check.toggled.connect(self._on_edit_check_toggle)
        cur_step_grid.addWidget(self._edit_check, 0, 4)
        self._edit_label = QLabel('Edit')
        cur_step_grid.addWidget(self._edit_label, 0, 5)
        cur_step_grid.addWidget(QLabel('Control No.'), 1, 0)
        self._cur_ctrl_name_label = QLabel('')
        self._cur_ctrl_name_label.setFont(mono_font)
        cur_step_grid.addWidget(self._cur_ctrl_name_label, 1, 1, 1, 5)
        top_row.addWidget(cur_step_group, stretch=1)
        next_group, self._next_step_label, self._next_ctrl_name_label = self._build_preview_group('Next Step Preview', mono_font)
        top_row.addWidget(next_group, stretch=1)
        root.addLayout(top_row)
        ctrl_args_group = QGroupBox('Control Arguments Editor')
        ctrl_args_grid = QGridLayout(ctrl_args_group)
        ctrl_args_grid.setHorizontalSpacing(6)
        ctrl_args_grid.setVerticalSpacing(4)
        ctrl_args_grid.addWidget(QLabel('Target Step No.'), 0, 0)
        self._target_step_spin = QSpinBox()
        self._target_step_spin.setRange(0, DSS_CHDEF_STEPCTRL_STEP_MAX - 1)
        self._target_step_spin.setValue(0)
        self._target_step_spin.setFont(input_font)
        self._target_step_spin.setMaximumWidth(110)
        self._target_step_spin.valueChanged.connect(self._on_target_step_changed)
        ctrl_args_grid.addWidget(self._target_step_spin, 0, 1)
        reload_btn = QPushButton('Reload')
        reload_btn.clicked.connect(self._on_load)
        ctrl_args_grid.addWidget(reload_btn, 0, 2)
        apply_btn = QPushButton('Apply')
        apply_btn.clicked.connect(self._on_update)
        ctrl_args_grid.addWidget(apply_btn, 0, 3)
        ctrl_args_grid.addWidget(QLabel('Control No.'), 1, 0)
        self._ctrl_combo = QComboBox()
        self._ctrl_combo.setFont(input_font)
        self._ctrl_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        for sub_cls in _active_step_control_base().all_subclasses():
            self._ctrl_combo.addItem(f'{sub_cls.CTRL_NUM}: {sub_cls.NAME}', userData=sub_cls.CTRL_NUM)
        self._ctrl_combo.currentIndexChanged.connect(self._on_ctrl_changed)
        ctrl_args_grid.addWidget(self._ctrl_combo, 1, 1, 1, 3)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumHeight(280)
        slots_host = QWidget()
        slots_layout = QVBoxLayout(slots_host)
        slots_layout.setContentsMargins(4, 4, 4, 4)
        slots_layout.setSpacing(6)
        for i in range(DSS_CHDEF_STEPCTRL_ARGS_MAX):
            row_frame = self._build_arg_row(i, input_font, bold_font, mono_font)
            slots_layout.addWidget(row_frame)
            self._arg_slots.append(row_frame)
        slots_layout.addStretch(1)
        scroll.setWidget(slots_host)
        ctrl_args_grid.addWidget(scroll, 2, 0, 1, 4)
        root.addWidget(ctrl_args_group)

    def _build_arg_row(self, index: int, input_font: QFont, bold_font: QFont, mono_font: QFont) -> QFrame:
        row_frame = QFrame()
        row_frame.setFrameShape(QFrame.Shape.StyledPanel)
        row = QHBoxLayout()
        row.setContentsMargins(4, 2, 4, 2)
        row.setSpacing(6)
        row_frame.setLayout(row)
        title = QLabel(f'Args[{index:02d}]')
        title.setFont(bold_font)
        title.setMinimumWidth(70)
        row.addWidget(title)
        edit = QLineEdit('0.0')
        edit.setFont(input_font)
        edit.setMaximumWidth(140)
        edit.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        row.addWidget(edit)
        self._arg_edits.append(edit)
        unit_label = QLabel('[-]')
        unit_label.setFont(mono_font)
        unit_label.setMinimumWidth(80)
        row.addWidget(unit_label)
        self._arg_unit_labels.append(unit_label)
        desc_label = QLabel('')
        desc_label.setWordWrap(True)
        desc_label.setFont(mono_font)
        row.addWidget(desc_label, stretch=1)
        self._arg_desc_labels.append(desc_label)
        return row_frame

    @staticmethod
    def _build_preview_group(title: str, mono_font: QFont) -> tuple[QGroupBox, QLabel, QLabel]:
        group = QGroupBox(title)
        grid = QGridLayout(group)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(4)
        grid.addWidget(QLabel('Step No.'), 0, 0)
        step_label = QLabel('')
        step_label.setFont(mono_font)
        grid.addWidget(step_label, 0, 1)
        grid.addWidget(QLabel('Control No.'), 1, 0)
        ctrl_name_label = QLabel('')
        ctrl_name_label.setFont(mono_font)
        grid.addWidget(ctrl_name_label, 1, 1)
        return (group, step_label, ctrl_name_label)

    @staticmethod
    def _make_mono_font(point_size: int) -> QFont:
        font = QFontDatabase.systemFont(QFontDatabase.SystemFont.GeneralFont)
        font.setStyleHint(QFont.StyleHint.Monospace)
        font.setFixedPitch(True)
        font.setPointSize(point_size)
        return font

    def _clamp_step(self, step: int) -> int:
        return max(0, min(step, DSS_CHDEF_STEPCTRL_STEP_MAX - 1))

    def _current_ctrl_num(self) -> int:
        if self._ctrl_combo is None:
            return 0
        data = self._ctrl_combo.currentData()
        return int(data) if data is not None else 0

    def _refresh_args_from_ctx(self, step: int) -> None:
        ctx = get_context()
        if step < 0 or step >= len(ctx.Control.Step):
            return
        args = ctx.Control.Step[step].args
        ctrl_num = int(ctx.Control.Step[step].ctrl)
        for i in range(DSS_CHDEF_STEPCTRL_ARGS_MAX):
            self._arg_edits[i].setText(f'{float(args[i]):.4f}')
        if self._ctrl_combo is not None:
            idx = self._ctrl_combo.findData(ctrl_num)
            if idx >= 0:
                self._ctrl_combo.setCurrentIndex(idx)
        self._refresh_args_enabled_state(ctrl_num)

    def _refresh_args_enabled_state(self, ctrl_num: int) -> None:
        sub_cls = _active_step_control_base().get_class_by_ctrl_num(ctrl_num)
        max_args = sub_cls.MAX_ARGS if sub_cls else 0
        args: tuple[ArgSpec, ...] = sub_cls.ARGS if sub_cls else ()
        for i, slot in enumerate(self._arg_slots):
            enabled = i < max_args
            slot.setEnabled(enabled)
            if i < len(args):
                spec = args[i]
                self._arg_unit_labels[i].setText(spec.unit or '-')
                self._arg_desc_labels[i].setText(spec.description)
            else:
                self._arg_unit_labels[i].setText('-')
                self._arg_desc_labels[i].setText('')

    def _commit_args_to_ctx(self, step: int) -> None:
        ctx = get_context()
        if step < 0 or step >= len(ctx.Control.Step):
            return
        try:
            vals = np.array([float(self._arg_edits[i].text() or '0') for i in range(DSS_CHDEF_STEPCTRL_ARGS_MAX)], dtype=np.float32)
            ctrl_num = self._current_ctrl_num()
        except ValueError as exc:
            QMessageBox.warning(self, 'Invalid Input', str(exc))
            return
        ctx.Control.Step[step].args[:] = vals
        ctx.Control.Step[step].ctrl = ctrl_num

    def _refresh_all(self) -> None:
        if self._refreshing:
            return
        self._refreshing = True
        try:
            ctx = get_context()
            assert self._target_step_spin is not None
            self._target_step_spin.setValue(self._editing_step)
            self._refresh_args_from_ctx(self._editing_step)
            cur_step = self._clamp_step(ctx.Control.current_step)
            self._refresh_current_step_display(cur_step)
            self._last_known_current_step = cur_step
        finally:
            self._refreshing = False

    def _refresh_current_step_display(self, cur_step: int) -> None:
        if self._cur_step_spin is not None:
            self._cur_step_spin.blockSignals(True)
            try:
                self._cur_step_spin.setValue(cur_step)
            finally:
                self._cur_step_spin.blockSignals(False)
        if self._cur_ctrl_name_label is not None:
            ctrl_num = int(get_context().Control.Step[cur_step].ctrl)
            sub_cls = _active_step_control_base().get_class_by_ctrl_num(ctrl_num)
            self._cur_ctrl_name_label.setText(f'{sub_cls.CTRL_NUM}: {sub_cls.NAME}' if sub_cls else '?')
        self._refresh_previews(cur_step)

    def _refresh_previews(self, cur_step: int) -> None:
        prev_step = cur_step - 1
        next_step = cur_step + 1
        if self._prev_step_label is not None and self._prev_ctrl_name_label is not None:
            self._refresh_preview(self._prev_step_label, self._prev_ctrl_name_label, prev_step)
        if self._next_step_label is not None and self._next_ctrl_name_label is not None:
            self._refresh_preview(self._next_step_label, self._next_ctrl_name_label, next_step)

    @staticmethod
    def _refresh_preview(step_label: QLabel, ctrl_name_label: QLabel, step: int) -> None:
        if step < 0 or step >= DSS_CHDEF_STEPCTRL_STEP_MAX:
            step_label.setText('-----')
            ctrl_name_label.setText('-----')
            return
        ctx = get_context()
        step_label.setText(str(step))
        ctrl_num = int(ctx.Control.Step[step].ctrl)
        sub_cls = _active_step_control_base().get_class_by_ctrl_num(ctrl_num)
        ctrl_name_label.setText(f'{sub_cls.CTRL_NUM}: {sub_cls.NAME}' if sub_cls else '?')

    def _init_step_poll_timer(self) -> None:
        timer = QTimer(self)
        timer.setInterval(DSS_TIM_CONTROL_MS)
        timer.setTimerType(Qt.TimerType.PreciseTimer)
        timer.timeout.connect(self._on_step_poll_tick)
        timer.start()
        self._step_poll_timer = timer

    def _on_step_poll_tick(self) -> None:
        cur_step = self._clamp_step(get_context().Control.current_step)
        if cur_step == self._last_known_current_step:
            return
        self._refresh_current_step_display(cur_step)
        self._last_known_current_step = cur_step

    def _on_target_step_changed(self, new_value: int) -> None:
        if self._refreshing:
            return
        if self._target_step_spin is None:
            return
        new_step = self._clamp_step(new_value)
        if new_step == self._editing_step:
            return
        self._refreshing = True
        try:
            self._editing_step = new_step
            self._refresh_args_from_ctx(new_step)
        finally:
            self._refreshing = False

    def _on_load(self) -> None:
        if self._refreshing:
            return
        self._refreshing = True
        try:
            self._refresh_args_from_ctx(self._editing_step)
        finally:
            self._refreshing = False

    def _on_ctrl_changed(self, _index: int) -> None:
        if self._refreshing:
            return
        self._refresh_args_enabled_state(self._current_ctrl_num())

    def _on_update(self) -> None:
        step = self._editing_step
        self._commit_args_to_ctx(step)
        self._refresh_all()
        self._log_to_parent(f'StepCtrl: applied step {step}')

    def _reset_cycle_state(self) -> None:
        from ctl_motor_step_ctrl_cyclic_strain import MotorStepControl_CyclicStrain
        from ctl_motor_step_ctrl_cyclic_stress import MotorStepControl_CyclicStress
        from ctl_torsional_step_ctrl_class import reset_all_cycle_states
        MotorStepControl_CyclicStress.reset_cycle()
        MotorStepControl_CyclicStrain.reset_cycle()
        reset_all_cycle_states()

    def _on_cur_step_changed(self, new_value: int) -> None:
        if self._refreshing:
            return
        if self._cur_step_spin is None or self._edit_check is None:
            return
        if not self._edit_check.isChecked():
            return
        new_step = self._clamp_step(new_value)
        cur_step = self._clamp_step(get_context().Control.current_step)
        if new_step == cur_step:
            return
        ctx = get_context()
        ctx.Control.current_step = new_step
        self._reset_cycle_state()
        ctx.Control.watch.reset()
        self._refresh_all()

    def _on_edit_check_toggle(self, checked: bool) -> None:
        if self._cur_step_spin is not None:
            self._cur_step_spin.setEnabled(checked)

    def _on_read_file(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, 'Load Step Control', '', 'JSON Files (*.json);;All Files (*)')
        if not path:
            return
        try:
            with Path(path).open(encoding='utf-8') as fp:
                data = json.load(fp)
        except (OSError, json.JSONDecodeError) as exc:
            QMessageBox.warning(self, 'LoadError', str(exc))
            return
        if not isinstance(data, dict) or data.get('type') != 'StepControl':
            QMessageBox.warning(self, 'LoadError', 'Not a StepControl JSON file')
            return
        ctx = get_context()
        for step in range(DSS_CHDEF_STEPCTRL_STEP_MAX):
            key = f'{step:04d}'
            row = data.get(key)
            if not isinstance(row, dict):
                continue
            if 'ctrl' in row:
                raw_ctrl = row['ctrl']
                if isinstance(raw_ctrl, float) and (not raw_ctrl.is_integer()):
                    self._log_to_parent(f'StepCtrl load: step {step:04d} has non-integer ctrl, skipped')
                    continue
                try:
                    ctx.Control.Step[step].ctrl = int(raw_ctrl)
                except (TypeError, ValueError):
                    self._log_to_parent(f'StepCtrl load: step {step:04d} has invalid ctrl, skipped')
            for par in range(DSS_CHDEF_STEPCTRL_ARGS_MAX):
                pkey = f'args{par:02d}'
                if pkey in row:
                    with contextlib.suppress(TypeError, ValueError):
                        ctx.Control.Step[step].args[par] = float(row[pkey])
        self._refresh_all()
        self._log_to_parent(f'StepCtrl loaded: {path}')

    def _on_save_file(self) -> None:
        ctx = get_context()
        default_name = datetime.now().strftime('%Y%m%d_%H%M%S') + '.ctl.json'
        path, _ = QFileDialog.getSaveFileName(self, 'Save Step Control', default_name, 'JSON Files (*.json);;All Files (*)')
        if not path:
            return
        out = {'type': 'StepControl'}
        for step in range(DSS_CHDEF_STEPCTRL_STEP_MAX):
            row = ctx.Control.Step[step]
            out[f'{step:04d}'] = {'ctrl': int(row.ctrl), **{f'args{i:02d}': float(row.args[i]) for i in range(DSS_CHDEF_STEPCTRL_ARGS_MAX)}}
        try:
            with Path(path).open('w', encoding='utf-8') as fp:
                json.dump(out, fp, indent=2)
        except OSError as exc:
            QMessageBox.warning(self, 'SaveError', str(exc))
            return
        self._log_to_parent(f'StepCtrl saved: {path}')

    def _log_to_parent(self, msg: str) -> None:
        parent = self.parent()
        log = getattr(parent, '_log', None)
        if log is not None:
            log('INFO', msg)
