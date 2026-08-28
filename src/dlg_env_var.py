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
import math
import os
import sys
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QCheckBox, QDialog, QFrame, QGridLayout, QHBoxLayout, QLabel, QLineEdit, QMessageBox, QPushButton, QVBoxLayout, QWidget
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utl_context import DSS_AO_CH_EP_AXIS, DSS_AO_CH_EP_CELL, DSS_AO_CH_MOTOR_SPEED, DSS_AO_CH_TORSIONAL_SPEED, DSS_VOLTAGE_OUT_LABELS, CDSBPyContext, get_context
SYSENV_MAX: int = 16
ACTIVE_LIMIT: int = 14

@dataclass(slots=True)
class _Spec:
    label: str
    read: Callable[[CDSBPyContext], float]
    write: Callable[[CDSBPyContext, float], None] | None = None

def _build_specs() -> list[_Spec]:
    ao_speed_label = f'DA{DSS_AO_CH_MOTOR_SPEED:02d}:{DSS_VOLTAGE_OUT_LABELS[DSS_AO_CH_MOTOR_SPEED]}'
    ao_cell_label = f'DA{DSS_AO_CH_EP_CELL:02d}:{DSS_VOLTAGE_OUT_LABELS[DSS_AO_CH_EP_CELL]}'
    ao_axis_label = f'DA{DSS_AO_CH_EP_AXIS:02d}:{DSS_VOLTAGE_OUT_LABELS[DSS_AO_CH_EP_AXIS]}'
    ao_tor_label = f'DA{DSS_AO_CH_TORSIONAL_SPEED:02d}:{DSS_VOLTAGE_OUT_LABELS[DSS_AO_CH_TORSIONAL_SPEED]}'

    def ao_getter(idx: int, sub: str) -> Callable[[CDSBPyContext], float]:
        return lambda ctx: float(getattr(ctx.AIO.AO[idx].Cal, sub))

    def ao_setter(idx: int, sub: str) -> Callable[[CDSBPyContext, float], None]:
        return lambda ctx, v: setattr(ctx.AIO.AO[idx].Cal, sub, v)

    def struct_getter(path: str) -> Callable[[CDSBPyContext], float]:
        return lambda ctx: float(_walk_attr(ctx, path))

    def struct_setter(path: str) -> Callable[[CDSBPyContext, float], None]:
        return lambda ctx, v: _set_walk(ctx, path, v)
    specs: list[_Spec] = [_Spec(f'{ao_speed_label}\tax(gradient)', ao_getter(DSS_AO_CH_MOTOR_SPEED, 'a'), ao_setter(DSS_AO_CH_MOTOR_SPEED, 'a')), _Spec(f'{ao_speed_label}\t+b(intercept)', ao_getter(DSS_AO_CH_MOTOR_SPEED, 'b'), ao_setter(DSS_AO_CH_MOTOR_SPEED, 'b')), _Spec(f'{ao_cell_label}\tax(gradient)', ao_getter(DSS_AO_CH_EP_CELL, 'a'), ao_setter(DSS_AO_CH_EP_CELL, 'a')), _Spec(f'{ao_cell_label}\t+b(intercept)', ao_getter(DSS_AO_CH_EP_CELL, 'b'), ao_setter(DSS_AO_CH_EP_CELL, 'b')), _Spec(f'{ao_axis_label}\tax(gradient)', ao_getter(DSS_AO_CH_EP_AXIS, 'a'), ao_setter(DSS_AO_CH_EP_AXIS, 'a')), _Spec(f'{ao_axis_label}\t+b(intercept)', ao_getter(DSS_AO_CH_EP_AXIS, 'b'), ao_setter(DSS_AO_CH_EP_AXIS, 'b')), _Spec(f'{ao_tor_label}\tax(gradient)', ao_getter(DSS_AO_CH_TORSIONAL_SPEED, 'a'), ao_setter(DSS_AO_CH_TORSIONAL_SPEED, 'a')), _Spec(f'{ao_tor_label}\t+b(intercept)', ao_getter(DSS_AO_CH_TORSIONAL_SPEED, 'b'), ao_setter(DSS_AO_CH_TORSIONAL_SPEED, 'b')), _Spec('Error in Compressive Control of Deviator Stress (kPa)', struct_getter('Control.ErrorStress.com'), struct_setter('Control.ErrorStress.com')), _Spec('Error in Extensive Control of Deviator Stress (kPa)', struct_getter('Control.ErrorStress.ext'), struct_setter('Control.ErrorStress.ext')), _Spec('Error in Control of Cell Pressure (kPa)', struct_getter('Control.ErrorStress.air'), struct_setter('Control.ErrorStress.air')), _Spec('Error in Control of Axial Strain(%)', struct_getter('Control.ErrorStress.ea'), struct_setter('Control.ErrorStress.ea')), _Spec('Default Specimen Diameter (mm)\t only apply on start up', struct_getter('SpecimenData.default_diameter'), struct_setter('SpecimenData.default_diameter')), _Spec('Default Specimen Height   (mm)\t only apply on start up', struct_getter('SpecimenData.default_height'), struct_setter('SpecimenData.default_height')), _Spec('none', lambda ctx: 0.0, None), _Spec('none', lambda ctx: 0.0, None)]
    return specs

def _walk_attr(root: object, dotted: str) -> Any:
    obj = root
    for part in dotted.split('.'):
        obj = getattr(obj, part)
    return obj

def _set_walk(root: object, dotted: str, value: float) -> None:
    parts = dotted.split('.')
    obj = root
    for part in parts[:-1]:
        obj = getattr(obj, part)
    setattr(obj, parts[-1], value)

class EnvVarDialog(QDialog):

    def __init__(self, parent: QWidget | None=None) -> None:
        super().__init__(parent)
        self.setModal(True)
        self.setWindowTitle('Environmental Variables')
        self.resize(760, 560)
        self.setMinimumWidth(760)
        font = QFont()
        font.setPointSize(10)
        self.setFont(font)
        self._input_font = QFont()
        self._input_font.setPointSize(11)
        self._specs: list[_Spec] = _build_specs()
        assert len(self._specs) == SYSENV_MAX
        self._current_labels: list[QLabel] = []
        self._value_edits: list[QLineEdit] = []
        self._update_buttons: list[QPushButton] = []
        self._build_ui()
        self.refresh()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)
        top = QHBoxLayout()
        caution = QLabel('Caution! Changing these values during control may cause unexpected behaviour or force termination of the application.')
        caution.setWordWrap(True)
        top.addWidget(caution, stretch=1)
        self._accept_check = QCheckBox('Accept Risks')
        self._accept_check.toggled.connect(self._refresh_enabled)
        top.addWidget(self._accept_check, alignment=Qt.AlignmentFlag.AlignTop)
        root.addLayout(top)
        grid = QGridLayout()
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(4)
        for col, header in enumerate(('Name', 'Current', 'Value', '')):
            lbl = QLabel(header)
            grid.addWidget(lbl, 0, col)
        for i, spec in enumerate(self._specs):
            row = i + 1
            name_lbl = QLabel(spec.label)
            grid.addWidget(name_lbl, row, 0)
            cur_lbl = QLabel('0')
            cur_lbl.setFrameShape(QFrame.Shape.Box)
            cur_lbl.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            cur_lbl.setMinimumWidth(110)
            grid.addWidget(cur_lbl, row, 1)
            edit = QLineEdit('0')
            edit.setFont(self._input_font)
            edit.setMaximumWidth(140)
            edit.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            edit.returnPressed.connect(lambda *_, idx=i: self._on_update(idx))
            grid.addWidget(edit, row, 2)
            upd = QPushButton('Apply')
            upd.setFixedWidth(80)
            upd.clicked.connect(lambda *_, idx=i: self._on_update(idx))
            grid.addWidget(upd, row, 3)
            self._current_labels.append(cur_lbl)
            self._value_edits.append(edit)
            self._update_buttons.append(upd)
        grid.setColumnStretch(0, 2)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(2, 1)
        grid.setColumnStretch(3, 0)
        root.addLayout(grid)
        self._refresh_enabled()

    def refresh(self) -> None:
        ctx = get_context()
        for i, spec in enumerate(self._specs):
            val = float(spec.read(ctx))
            txt = f'{val:g}'
            self._current_labels[i].setText(txt)
            self._value_edits[i].setText(txt)

    def _refresh_enabled(self) -> None:
        ctx = get_context()
        in_control = bool(ctx.Flag.control)
        accepted = self._accept_check.isChecked()
        allow = not in_control or accepted
        for i in range(SYSENV_MAX):
            is_active = i < ACTIVE_LIMIT and self._specs[i].write is not None
            self._value_edits[i].setEnabled(allow and is_active)
            self._update_buttons[i].setEnabled(allow and is_active)

    def _on_update(self, idx: int) -> None:
        spec = self._specs[idx]
        if spec.write is None:
            return
        edit = self._value_edits[idx]
        text = edit.text()
        try:
            new_val = float(text) if text.strip() else 0.0
        except ValueError:
            QMessageBox.warning(self, 'Invalid Input', f"Row {idx}: '{text}' is not a valid number")
            return
        if not math.isfinite(new_val):
            QMessageBox.warning(self, 'Invalid Input', f"Row {idx}: '{text}' is not finite")
            return
        try:
            spec.write(get_context(), new_val)
        except Exception as exc:
            QMessageBox.warning(self, 'Update Error', str(exc))
            return
        self.refresh()
        self._log_to_parent(f'EnvVar[{idx}] updated to {new_val}')

    def _log_to_parent(self, msg: str) -> None:
        parent = self.parent()
        log = getattr(parent, '_log', None)
        if log is not None:
            log('INFO', msg)
