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
import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMessageBox, QPushButton, QVBoxLayout, QWidget
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utl_context import DSS_PRECON_TOR_DEFAULT_AXIS_SPEED_MAX_RPM, DSS_PRECON_TOR_DEFAULT_CELL_RATE_KPA_PER_MIN, DSS_PRECON_TOR_DEFAULT_CELL_TARGET_KPA, DSS_PRECON_TOR_DEFAULT_Q_AT_MAX_SPEED_KPA, get_context

def _format(value: float) -> str:
    return f'{float(value):g}'

class TorsionalPreConsolidationDialog(QDialog):
    preconsolidation_changed = Signal()

    def __init__(self, parent: QWidget | None=None) -> None:
        super().__init__(parent)
        self.setModal(True)
        self.setWindowTitle('Control Parameters in Pre-Consolidation (Torsional)')
        self.resize(560, 280)
        self._edits: dict[str, QLineEdit] = {}
        self._build_ui()
        self._refresh_from_ctx()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)
        font = QFont()
        font.setPointSize(12)
        self.setFont(font)
        input_font = QFont()
        input_font.setPointSize(14)
        group = QGroupBox('Settings of pre-consolidation (Torsional)')
        grid = QGridLayout(group)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)
        fields = (('axis_speed_max_rpm', 'Max Axial Motor Speed (rpm)', _format(DSS_PRECON_TOR_DEFAULT_AXIS_SPEED_MAX_RPM)), ('q_at_max_speed_kpa', 'q at Max Motor Speed (kPa)', _format(DSS_PRECON_TOR_DEFAULT_Q_AT_MAX_SPEED_KPA)), ('cell_target_kpa', 'Target Cell Pressure, σr (kPa)', _format(DSS_PRECON_TOR_DEFAULT_CELL_TARGET_KPA)), ('cell_rate_kpa_per_min', 'Cell Pressure Rate (kPa/min)', _format(DSS_PRECON_TOR_DEFAULT_CELL_RATE_KPA_PER_MIN)))
        for row, (key, label_text, default) in enumerate(fields):
            label = QLabel(label_text)
            edit = QLineEdit(default)
            edit.setFont(input_font)
            edit.setMinimumWidth(200)
            edit.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            grid.addWidget(label, row, 0, alignment=Qt.AlignmentFlag.AlignVCenter)
            grid.addWidget(edit, row, 1, alignment=Qt.AlignmentFlag.AlignVCenter)
            self._edits[key] = edit
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        root.addWidget(group)
        root.addStretch(1)
        root.addLayout(self._build_footer())

    def _build_footer(self) -> QHBoxLayout:
        h = QHBoxLayout()
        h.addStretch(1)
        refresh_btn = QPushButton('Refresh')
        refresh_btn.setFixedWidth(90)
        refresh_btn.clicked.connect(self._on_refresh)
        h.addWidget(refresh_btn)
        apply_btn = QPushButton('Apply')
        apply_btn.setFixedWidth(90)
        apply_btn.clicked.connect(self._on_apply)
        h.addWidget(apply_btn)
        cancel_btn = QPushButton('Cancel')
        cancel_btn.setFixedWidth(90)
        cancel_btn.clicked.connect(self.reject)
        h.addWidget(cancel_btn)
        return h

    def _refresh_from_ctx(self) -> None:
        ctx = get_context()
        pct = ctx.Control.PreConsolidationTorsional
        self._edits['axis_speed_max_rpm'].setText(_format(float(pct.axis_speed_max_rpm)))
        self._edits['q_at_max_speed_kpa'].setText(_format(float(pct.q_at_max_speed_kpa)))
        self._edits['cell_target_kpa'].setText(_format(float(pct.cell_target_kpa)))
        self._edits['cell_rate_kpa_per_min'].setText(_format(float(pct.cell_rate_kpa_per_min)))

    def _commit_to_ctx(self) -> bool:
        ctx = get_context()
        pct = ctx.Control.PreConsolidationTorsional
        for key, label, attr in (('axis_speed_max_rpm', 'Max Axial Motor Speed', 'axis_speed_max_rpm'), ('q_at_max_speed_kpa', 'q at Max Motor Speed', 'q_at_max_speed_kpa'), ('cell_target_kpa', 'Target Cell Pressure', 'cell_target_kpa'), ('cell_rate_kpa_per_min', 'Cell Pressure Rate', 'cell_rate_kpa_per_min')):
            val = self._parse_float(self._edits[key], label)
            if val is None:
                return False
            setattr(pct, attr, val)
        return True

    def _parse_float(self, edit: QLineEdit, label: str) -> float | None:
        text = edit.text().strip()
        if not text:
            return 0.0
        try:
            value = float(np.float64(text))
        except (ValueError, OverflowError):
            QMessageBox.warning(self, 'Invalid Input', f"{label}: '{text}' is not a valid number.")
            return None
        if not np.isfinite(value):
            QMessageBox.warning(self, 'Invalid Input', f"{label}: '{text}' is not finite.")
            return None
        return value

    def _on_apply(self) -> None:
        if not self._commit_to_ctx():
            return
        self._refresh_from_ctx()
        self._log_to_parent('PreConsolidationTorsional: applied')
        self.preconsolidation_changed.emit()

    def _on_refresh(self) -> None:
        self._refresh_from_ctx()
        self._log_to_parent('PreConsolidationTorsional: refreshed')

    def _log_to_parent(self, msg: str) -> None:
        parent = self.parent()
        log = getattr(parent, '_log', None)
        if log is not None:
            log('INFO', msg)
