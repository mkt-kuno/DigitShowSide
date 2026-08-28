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

import json
import math
import os
import sys
from datetime import date
from pathlib import Path
import numpy as np
import numpy.typing as npt
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QDialog, QFileDialog, QFrame, QGridLayout, QHBoxLayout, QLabel, QLineEdit, QMessageBox, QPushButton, QSizePolicy, QVBoxLayout, QWidget
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utl_context import DSS_MB_AI_REGISTER_COUNT, dss_active_physical_labels, dss_active_raw_labels, get_context

class CalibrationDialog(QDialog):
    coefficients_changed = Signal(int)

    def __init__(self, parent: QWidget | None=None) -> None:
        super().__init__(parent)
        self.setModal(True)
        self.setWindowTitle('Calibration Value')
        self.resize(1180, 620)
        self.setMinimumWidth(960)
        self._raw_values: list[int] = [0] * DSS_MB_AI_REGISTER_COUNT
        self._a_edits: list[QLineEdit] = []
        self._b_edits: list[QLineEdit] = []
        self._c_edits: list[QLineEdit] = []
        self._y_labels: list[QLabel] = []
        self._tare_buttons: list[QPushButton] = []
        self._build_ui()
        self._refresh_all()

    def _build_ui(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)
        font = QFont()
        font.setPointSize(12)
        self.setFont(font)
        input_font = QFont()
        input_font.setPointSize(14)
        grid_widget = QWidget()
        grid = QGridLayout(grid_widget)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(2)
        header_x = QLabel('x : Raw Value')
        header_arrow = QLabel('→')
        header_arrow.setAlignment(Qt.AlignmentFlag.AlignCenter)
        header_y = QLabel('y : Physical Value')
        header_a = QLabel('ax² +')
        header_a.setAlignment(Qt.AlignmentFlag.AlignCenter)
        header_b = QLabel('bx +')
        header_b.setAlignment(Qt.AlignmentFlag.AlignCenter)
        header_c = QLabel('c')
        header_c.setAlignment(Qt.AlignmentFlag.AlignCenter)
        header_equals = QLabel('=')
        header_equals.setAlignment(Qt.AlignmentFlag.AlignCenter)
        header_y_val = QLabel('y')
        header_y_val.setAlignment(Qt.AlignmentFlag.AlignCenter)
        grid.addWidget(header_x, 0, 0, alignment=Qt.AlignmentFlag.AlignTop)
        grid.addWidget(header_arrow, 0, 1, alignment=Qt.AlignmentFlag.AlignTop)
        grid.addWidget(header_y, 0, 2, alignment=Qt.AlignmentFlag.AlignTop)
        grid.addWidget(header_a, 0, 3, alignment=Qt.AlignmentFlag.AlignTop)
        grid.addWidget(header_b, 0, 4, alignment=Qt.AlignmentFlag.AlignTop)
        grid.addWidget(header_c, 0, 5, alignment=Qt.AlignmentFlag.AlignTop)
        grid.addWidget(header_equals, 0, 6, alignment=Qt.AlignmentFlag.AlignTop)
        grid.addWidget(header_y_val, 0, 7, alignment=Qt.AlignmentFlag.AlignTop)
        header_tare_all = QPushButton('Tare All')
        header_tare_all.setMinimumWidth(120)
        header_tare_all.clicked.connect(self._on_tare_all)
        grid.addWidget(header_tare_all, 0, 8, alignment=Qt.AlignmentFlag.AlignTop)
        for i in range(DSS_MB_AI_REGISTER_COUNT):
            row = i + 1
            x_label = QLabel(f'{i:02d}:{dss_active_raw_labels(get_context().Control.mode)[i]}(i16)')
            x_arrow = QLabel('→')
            x_arrow.setAlignment(Qt.AlignmentFlag.AlignCenter)
            y_label = QLabel(f'{i:02d}:{dss_active_physical_labels(get_context().Control.mode)[i]}')
            a_edit = QLineEdit('0')
            a_edit.setFont(input_font)
            a_edit.setMaximumWidth(90)
            a_edit.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            b_edit = QLineEdit('1')
            b_edit.setFont(input_font)
            b_edit.setMaximumWidth(90)
            b_edit.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            c_edit = QLineEdit('0')
            c_edit.setFont(input_font)
            c_edit.setMaximumWidth(90)
            c_edit.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            equals = QLabel('=')
            equals.setAlignment(Qt.AlignmentFlag.AlignCenter)
            y_disp = QLabel('0.0000')
            y_disp.setFont(input_font)
            y_disp.setFrameShape(QFrame.Shape.Box)
            y_disp.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            y_disp.setMinimumWidth(100)
            tare_btn = QPushButton('Tare')
            tare_btn.setMinimumWidth(90)
            grid.addWidget(x_label, row, 0, alignment=Qt.AlignmentFlag.AlignTop)
            grid.addWidget(x_arrow, row, 1, alignment=Qt.AlignmentFlag.AlignTop)
            grid.addWidget(y_label, row, 2, alignment=Qt.AlignmentFlag.AlignTop)
            grid.addWidget(a_edit, row, 3, alignment=Qt.AlignmentFlag.AlignTop)
            grid.addWidget(b_edit, row, 4, alignment=Qt.AlignmentFlag.AlignTop)
            grid.addWidget(c_edit, row, 5, alignment=Qt.AlignmentFlag.AlignTop)
            grid.addWidget(equals, row, 6, alignment=Qt.AlignmentFlag.AlignTop)
            grid.addWidget(y_disp, row, 7, alignment=Qt.AlignmentFlag.AlignTop)
            grid.addWidget(tare_btn, row, 8, alignment=Qt.AlignmentFlag.AlignTop)
            self._a_edits.append(a_edit)
            self._b_edits.append(b_edit)
            self._c_edits.append(c_edit)
            self._y_labels.append(y_disp)
            self._tare_buttons.append(tare_btn)
            a_edit.textChanged.connect(lambda *_, idx=i: self._on_coeff_edited(idx))
            b_edit.textChanged.connect(lambda *_, idx=i: self._on_coeff_edited(idx))
            c_edit.textChanged.connect(lambda *_, idx=i: self._on_coeff_edited(idx))
            tare_btn.clicked.connect(lambda *_, idx=i: self._on_tare(idx))
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(2, 1)
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(6)
        right_widget.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        btn_load = QPushButton('Import')
        btn_load.setMinimumWidth(140)
        btn_load.clicked.connect(self._on_load)
        right_layout.addWidget(btn_load)
        btn_save = QPushButton('Export')
        btn_save.setMinimumWidth(140)
        btn_save.clicked.connect(self._on_save)
        right_layout.addWidget(btn_save)
        root.addWidget(grid_widget, stretch=1)
        root.addWidget(right_widget, 0, Qt.AlignmentFlag.AlignTop)

    def _get_coeff(self, idx: int) -> tuple[float, float, float]:

        def f(edit: QLineEdit) -> float:
            try:
                return float(edit.text() or '0')
            except ValueError:
                return 0.0
        return (f(self._a_edits[idx]), f(self._b_edits[idx]), f(self._c_edits[idx]))

    def _compute_y(self, idx: int, x: int) -> float:
        a, b, c = self._get_coeff(idx)
        return a * x * x + b * x + c

    def _refresh_y(self, idx: int) -> None:
        x = self._raw_values[idx] if idx < len(self._raw_values) else 0
        self._y_labels[idx].setText(f'{self._compute_y(idx, x):.4f}')

    def _refresh_all(self) -> None:
        for i in range(DSS_MB_AI_REGISTER_COUNT):
            self._refresh_y(i)

    def update_raw_values(self, values: npt.ArrayLike) -> None:
        arr = np.asarray(values, dtype=np.int32)
        n = arr.shape[0]
        if n < DSS_MB_AI_REGISTER_COUNT:
            arr = np.pad(arr, (0, DSS_MB_AI_REGISTER_COUNT - n))
        elif n > DSS_MB_AI_REGISTER_COUNT:
            arr = arr[:DSS_MB_AI_REGISTER_COUNT]
        self._raw_values = arr.tolist()
        self._refresh_all()

    def _on_coeff_edited(self, idx: int) -> None:
        if not self.coefficients_valid(idx):
            self._y_labels[idx].setText('---')
            return
        self._refresh_y(idx)
        self.coefficients_changed.emit(idx)

    @staticmethod
    def _parse_coeff_text(text: str) -> float | None:
        s = text.strip()
        if not s:
            return 0.0
        try:
            v = float(s)
        except ValueError:
            return None
        return v if math.isfinite(v) else None

    def coefficients_valid(self, idx: int) -> bool:
        return all((self._parse_coeff_text(edit.text()) is not None for edit in (self._a_edits[idx], self._b_edits[idx], self._c_edits[idx])))

    def _set_coeff_silently(self, idx: int, a: float, b: float, c: float) -> None:
        for edit, val in ((self._a_edits[idx], a), (self._b_edits[idx], b), (self._c_edits[idx], c)):
            edit.blockSignals(True)
            try:
                edit.setText(f'{val}')
            finally:
                edit.blockSignals(False)
        self._refresh_y(idx)
        self.coefficients_changed.emit(idx)

    def init_from_state(self, calibration: list[tuple[float, float, float]]) -> None:
        for i in range(DSS_MB_AI_REGISTER_COUNT):
            if i < len(calibration):
                a, b, c = calibration[i]
            else:
                a, b, c = (0.0, 1.0, 0.0)
            self._set_coeff_silently(i, a, b, c)

    def _on_tare(self, idx: int) -> None:
        if not self.coefficients_valid(idx):
            self._log_to_parent(f'CH{idx:02d}: coefficient is invalid, tare skipped')
            return
        x = self._raw_values[idx] if idx < len(self._raw_values) else 0
        a, b, _c = self._get_coeff(idx)
        self._set_coeff_silently(idx, a, b, -a * x * x - b * x)

    def _on_tare_all(self) -> None:
        for i in range(DSS_MB_AI_REGISTER_COUNT):
            self._on_tare(i)

    def _on_save(self) -> None:
        default_name = f'{date.today():%Y%m%d}_calibration.json'
        path, _ = QFileDialog.getSaveFileName(self, 'Save Calibration', default_name, 'JSON Files (*.json);;All Files (*)')
        if not path:
            return
        invalid = [i for i in range(DSS_MB_AI_REGISTER_COUNT) if not self.coefficients_valid(i)]
        if invalid:
            QMessageBox.warning(self, 'Save Error', f"Invalid coefficient at CH {', '.join((f'{i:02d}' for i in invalid))}")
            return
        data: dict[str, object] = {'type': 'Calibration'}
        for i in range(DSS_MB_AI_REGISTER_COUNT):
            a, b, c = self._get_coeff(i)
            data[f'{i:02d}'] = {'a': a, 'b': b, 'c': c}
        try:
            with Path(path).open('w', encoding='utf-8') as fp:
                json.dump(data, fp, indent=2)
        except OSError as e:
            QMessageBox.warning(self, 'Save Error', str(e))
            return
        self._log_to_parent(f'Calibration saved: {path}')

    def _on_load(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, 'Load Calibration', '', 'JSON Files (*.json);;All Files (*)')
        if not path:
            return
        try:
            with Path(path).open(encoding='utf-8') as fp:
                data = json.load(fp)
        except (OSError, json.JSONDecodeError) as e:
            QMessageBox.warning(self, 'Load Error', str(e))
            return
        if not isinstance(data, dict) or data.get('type') != 'Calibration':
            QMessageBox.warning(self, 'Load Error', 'Not a Calibration JSON file')
            return

        def json_or_current(val: object, fallback: float) -> float:
            try:
                f = float(val)
            except (TypeError, ValueError):
                return fallback
            return f if math.isfinite(f) else fallback
        for i in range(DSS_MB_AI_REGISTER_COUNT):
            ch = data.get(f'{i:02d}')
            if not isinstance(ch, dict):
                continue
            cur_a, cur_b, cur_c = self._get_coeff(i)
            a = json_or_current(ch.get('a'), cur_a)
            b = json_or_current(ch.get('b'), cur_b)
            c = json_or_current(ch.get('c'), cur_c)
            self._set_coeff_silently(i, a, b, c)
        self._log_to_parent(f'Calibration loaded: {path}')

    def _log_to_parent(self, msg: str) -> None:
        parent = self.parent()
        log = getattr(parent, '_log', None)
        if log is not None:
            log('INFO', msg)
