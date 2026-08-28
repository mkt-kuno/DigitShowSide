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
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMessageBox, QPushButton, QVBoxLayout, QWidget
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utl_context import ControlMode, get_context
LABEL_TARGET_MOTOR = 'Target Deviator Stress, q (kPa)'
LABEL_TARGET_TORSIONAL = 'Target Shear Stress, τ (kPa)'
LABEL_ERROR_MOTOR = 'q error at max motor speed (kPa)'
LABEL_ERROR_TORSIONAL = 'τ error at max torsional speed (kPa)'
LABEL_SPEED_MOTOR = 'Max Motor Speed (rpm)'
LABEL_SPEED_TORSIONAL = 'Max Torsional Speed (rpm)'
DEFAULTS_MOTOR = ('0', '10', '1000')
DEFAULTS_TORSIONAL = ('0', '10', '1000')

class MotorPreConsolidationDialog(QDialog):

    def __init__(self, parent: QWidget | None=None) -> None:
        super().__init__(parent)
        self.setModal(True)
        self.setWindowTitle('Control Parameters in Pre-Consolidation Process')
        self.resize(440, 240)
        self._edits: list[QLineEdit | None] = [None, None, None]
        self._labels: list[QLabel | None] = [None, None, None]
        self._build_ui()
        self._init_defaults_if_needed()
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
        group = QGroupBox('Settings of pre-consolidation')
        grid = QGridLayout(group)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(6)
        initial_labels = [LABEL_TARGET_MOTOR, LABEL_ERROR_MOTOR, LABEL_SPEED_MOTOR]
        defaults = list(DEFAULTS_MOTOR)
        for i, (lab, default) in enumerate(zip(initial_labels, defaults, strict=False)):
            label = QLabel(lab)
            edit = QLineEdit(default)
            edit.setFont(input_font)
            edit.setMinimumWidth(180)
            edit.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            grid.addWidget(label, i, 0, alignment=Qt.AlignmentFlag.AlignVCenter)
            grid.addWidget(edit, i, 1, alignment=Qt.AlignmentFlag.AlignVCenter)
            self._labels[i] = label
            self._edits[i] = edit
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        root.addWidget(group)
        btn_row = QHBoxLayout()
        btn_row.addStretch(1)
        btn_update = QPushButton('Apply')
        btn_update.setFixedWidth(90)
        btn_update.clicked.connect(self._on_apply)
        btn_row.addWidget(btn_update)
        root.addLayout(btn_row)

    def _init_defaults_if_needed(self) -> None:
        ctx = get_context()
        pc = ctx.Control.PreConsolidation
        if float(pc.target) == 0.0 and float(pc.error) == 0.0 and (float(pc.motor_speed) == 0.0):
            pc.error = 10.0
            pc.motor_speed = 1000.0

    def _refresh_from_ctx(self) -> None:
        ctx = get_context()
        pc = ctx.Control.PreConsolidation
        assert self._edits[0] is not None
        assert self._edits[1] is not None
        assert self._edits[2] is not None
        assert self._labels[0] is not None
        assert self._labels[1] is not None
        assert self._labels[2] is not None
        self._edits[0].setText(f'{float(pc.target):.4f}')
        self._edits[1].setText(f'{float(pc.error):.4f}')
        self._edits[2].setText(f'{float(pc.motor_speed):.4f}')
        is_motor = ctx.Control.mode == ControlMode.MOTOR
        is_torsional = not is_motor
        self._labels[0].setText(LABEL_TARGET_MOTOR if is_motor else LABEL_TARGET_TORSIONAL)
        self._labels[1].setText(LABEL_ERROR_MOTOR if is_motor else LABEL_ERROR_TORSIONAL)
        self._labels[2].setText(LABEL_SPEED_MOTOR if is_motor else LABEL_SPEED_TORSIONAL)
        self._edits[1].setEnabled(is_motor)
        self._edits[2].setEnabled(is_motor)
        _ = is_torsional

    def _commit_to_ctx(self) -> None:
        ctx = get_context()
        assert self._edits[0] is not None
        assert self._edits[1] is not None
        assert self._edits[2] is not None
        is_motor = ctx.Control.mode == ControlMode.MOTOR
        target = self._parse_float(self._edits[0], 'Target Stress')
        if target is None:
            return
        ctx.Control.PreConsolidation.target = target
        if is_motor:
            err = self._parse_float(self._edits[1], 'q error at max motor speed')
            if err is None:
                return
            speed = self._parse_float(self._edits[2], 'Max Motor Speed')
            if speed is None:
                return
            ctx.Control.PreConsolidation.error = err
            ctx.Control.PreConsolidation.motor_speed = speed
        self._log_to_parent(f'PreConsolidation: target={target}, error={ctx.Control.PreConsolidation.error}, max_speed={ctx.Control.PreConsolidation.motor_speed}')

    def _on_apply(self) -> None:
        self._commit_to_ctx()
        self._refresh_from_ctx()

    def _parse_float(self, edit: QLineEdit, label: str) -> float | None:
        text = edit.text().strip()
        if not text:
            return 0.0
        try:
            value = float(text)
        except ValueError:
            QMessageBox.warning(self, 'Invalid Input', f"{label}: '{text}' is not a valid number.")
            return None
        if not math.isfinite(value):
            QMessageBox.warning(self, 'Invalid Input', f"{label}: '{text}' is not finite.")
            return None
        return value

    def _log_to_parent(self, msg: str) -> None:
        parent = self.parent()
        log = getattr(parent, '_log', None)
        if log is not None:
            log('INFO', msg)
