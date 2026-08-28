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

import os
import sys
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QDialog, QGridLayout, QLabel, QLineEdit, QMessageBox, QPushButton, QVBoxLayout, QWidget
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utl_context import DSS_AO_CH_EP_AXIS, DSS_AO_CH_EP_CELL, DSS_MB_AO_MAX_MV, DSS_MB_AO_MIN_MV, DSS_VOLTAGE_OUT_LABELS, get_context

class VoltageOutputDialog(QDialog):

    def __init__(self, parent: QWidget | None=None) -> None:
        super().__init__(parent)
        self.setModal(True)
        self.setWindowTitle('Voltage Output')
        self.resize(420, 380)
        self._edits: list[QLineEdit] = []
        self._buttons: list[QPushButton] = []
        self._build_ui()
        self._refresh_from_ctx()

    def _refresh_from_ctx(self) -> None:
        ctx = get_context()
        for i, edit in enumerate(self._edits):
            edit.setText(f'{float(ctx.AIO.AO.row(i).raw):.3f}')

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(6)
        font = QFont()
        font.setPointSize(12)
        self.setFont(font)
        input_font = QFont()
        input_font.setPointSize(14)
        info = QLabel('Output: 0-10 V (0-10,000 mV).')
        info.setWordWrap(True)
        root.addWidget(info)
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(4)
        ch_header = QLabel('Channel')
        v_header = QLabel('Voltage (V)')
        v_header.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        grid.addWidget(ch_header, 0, 0)
        grid.addWidget(v_header, 0, 1)
        grid.addWidget(QLabel(''), 0, 2)
        for i, name in enumerate(DSS_VOLTAGE_OUT_LABELS):
            row = i + 1
            label = QLabel(f'{i:02d}:{name}')
            edit = QLineEdit('0')
            edit.setFont(input_font)
            edit.setMaximumWidth(140)
            edit.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            edit.returnPressed.connect(lambda *_, idx=i: self._on_apply(idx))
            btn = QPushButton('Apply')
            btn.setFixedWidth(70)
            btn.clicked.connect(lambda *_, idx=i: self._on_apply(idx))
            grid.addWidget(label, row, 0)
            grid.addWidget(edit, row, 1)
            grid.addWidget(btn, row, 2)
            self._edits.append(edit)
            self._buttons.append(btn)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        root.addLayout(grid)
        root.addStretch(1)

    def _on_apply(self, idx: int) -> None:
        text = self._edits[idx].text().strip()
        if not text:
            QMessageBox.warning(self, 'Invalid Input', f'CH{idx:02d}: value is empty.')
            return
        try:
            v = float(text)
        except ValueError:
            QMessageBox.warning(self, 'Invalid Input', f"CH{idx:02d}: '{text}' is not a valid number.")
            return
        raw_v = max(DSS_MB_AO_MIN_MV / 1000.0, min(DSS_MB_AO_MAX_MV / 1000.0, v))
        if raw_v != v:
            self._log_to_parent(f'CH{idx:02d}: input {v} V clamped to {raw_v:.3f} V')
        ctx = get_context()
        if idx == DSS_AO_CH_EP_CELL:
            a = float(ctx.AIO.AO.row(idx).Cal.a)
            b = float(ctx.AIO.AO.row(idx).Cal.b)
            kpa = (raw_v - b) / a if a != 0.0 else 0.0
            from ctl_motor import MotorController
            MotorController().set_ep_cell_pressure_kpa(kpa)
        elif idx == DSS_AO_CH_EP_AXIS:
            a = float(ctx.AIO.AO.row(idx).Cal.a)
            b = float(ctx.AIO.AO.row(idx).Cal.b)
            n_val = (raw_v - b) / a if a != 0.0 else 0.0
            from ctl_motor import MotorController
            MotorController().set_ep_axial_pressure_n(n_val)
        else:
            ctx.AIO.AO[idx].raw = raw_v
        request = getattr(self.parent(), '_request_analog_output_update', None)
        if request is not None:
            request()

    def _log_to_parent(self, msg: str) -> None:
        log = getattr(self.parent(), '_log', None)
        if log is not None:
            log('INFO', msg)
