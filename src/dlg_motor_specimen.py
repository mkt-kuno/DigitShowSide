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
import json
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Any
import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QDialog, QFileDialog, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMessageBox, QPushButton, QSizePolicy, QVBoxLayout, QWidget
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utl_context import DSS_AI_CH_LCDPT, DSS_AI_CH_LDT1, DSS_AI_CH_LDT2, DSS_AI_CH_V_DISP, DSS_STAGE_AFTER, DSS_STAGE_BEFORE, DSS_STAGE_INITIAL, DSS_STAGE_PRESENT, get_context
STAGE_HEADERS: tuple[tuple[int, str], ...] = ((DSS_STAGE_PRESENT, 'Present'), (DSS_STAGE_INITIAL, 'Initial'), (DSS_STAGE_BEFORE, 'Before\nconsolidation'), (DSS_STAGE_AFTER, 'After\nconsolidation'))
STAGE_HAS_COPY: dict[int, bool] = {DSS_STAGE_PRESENT: False, DSS_STAGE_INITIAL: True, DSS_STAGE_BEFORE: True, DSS_STAGE_AFTER: True}
ROW_DEFS: tuple[tuple[str, str], ...] = (('diameter', 'Diameter (mm)'), ('height', 'Height (mm)'), ('volume', 'Volume *(mm3)'), ('area', 'Area *(mm2)'), ('ldt_1', 'LDT1 (mm)'), ('ldt_2', 'LDT2 (mm)'))
_AUTO_FIELDS: frozenset[str] = frozenset({'volume', 'area'})
_USER_FIELDS: frozenset[str] = frozenset({'diameter', 'height', 'ldt_1', 'ldt_2'})
_LEGACY_JSON_KEYS: dict[str, str] = {'ldt_1': 'cg_1', 'ldt_2': 'cg_2'}

def _is_cell_editable(stage_idx: int, field: str) -> bool:
    if field in _AUTO_FIELDS:
        return False
    if stage_idx == DSS_STAGE_PRESENT:
        return False
    return field in _USER_FIELDS
ZERO_ADJUST_CHANNELS: tuple[int, ...] = (DSS_AI_CH_V_DISP, DSS_AI_CH_LCDPT, DSS_AI_CH_LDT1, DSS_AI_CH_LDT2)

def _format(value: float) -> str:
    return f'{float(value):g}'

class MotorSpecimenDialog(QDialog):
    specimen_changed = Signal()

    def __init__(self, parent: QWidget | None=None) -> None:
        super().__init__(parent)
        self.setModal(True)
        self.setWindowTitle('Specimen Data')
        self.resize(860, 720)
        self.setMinimumWidth(860)
        self._stage_edits: dict[tuple[int, str], QLineEdit] = {}
        self._build_ui()
        self.refresh_from_ctx()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)
        font = QFont()
        font.setPointSize(12)
        self.setFont(font)
        input_font = QFont()
        input_font.setPointSize(14)
        root.addWidget(self._build_apparatus_group(input_font))
        root.addWidget(self._build_stage_group(input_font), stretch=1)
        root.addWidget(self._build_update_group())
        root.addStretch(0)
        root.addLayout(self._build_footer())

    def _build_apparatus_group(self, input_font: QFont) -> QGroupBox:
        gb = QGroupBox('Parameters of Test Apparatus [only available for torsional shear]')
        layout = QGridLayout(gb)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(4)
        layout.setColumnStretch(1, 0)
        layout.setColumnStretch(2, 0)
        layout.setColumnStretch(3, 1)
        self._mem_young_edit = self._add_labeled_edit(layout, 0, "Young's Modulus of membrane (kPa)", input_font, default='0', read_only=True)
        self._mem_thick_edit = self._add_labeled_edit(layout, 1, 'Thickness of membrane (mm)', input_font, default='0.3', read_only=True)
        self._cap_weight_edit = self._add_labeled_edit(layout, 2, 'Cap Weight (N)', input_font, default='0', read_only=True)
        update_btn = QPushButton('Apply')
        update_btn.setEnabled(False)
        layout.addWidget(update_btn, 0, 4, 3, 1)
        return gb

    def _add_labeled_edit(self, grid: QGridLayout, row: int, label_text: str, font: QFont, default: str='0', read_only: bool=False) -> QLineEdit:
        label = QLabel(label_text)
        grid.addWidget(label, row, 0)
        edit = QLineEdit(default)
        edit.setFont(font)
        edit.setMaximumWidth(180)
        edit.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        if read_only:
            edit.setReadOnly(True)
            edit.setFrame(False)
        grid.addWidget(edit, row, 1)
        return edit

    def _build_stage_group(self, input_font: QFont) -> QGroupBox:
        gb = QGroupBox("Input Specimen's Data")
        layout = QGridLayout(gb)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(4)
        layout.addWidget(QLabel(''), 0, 0)
        for col, (_, header) in enumerate(STAGE_HEADERS, start=1):
            lbl = QLabel(header)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            layout.addWidget(lbl, 0, col)
        for r, (field, label) in enumerate(ROW_DEFS, start=1):
            layout.addWidget(QLabel(label), r, 0)
            for col, (idx, _) in enumerate(STAGE_HEADERS, start=1):
                edit = QLineEdit('0')
                edit.setFont(input_font)
                edit.setMaximumWidth(140)
                edit.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                cell_editable = _is_cell_editable(idx, field)
                edit.setReadOnly(not cell_editable)
                if not cell_editable:
                    edit.setFrame(False)
                if cell_editable:
                    edit.returnPressed.connect(self._on_typed_commit)
                layout.addWidget(edit, r, col)
                self._stage_edits[idx, field] = edit
        last_row = len(ROW_DEFS) + 1
        note = QLabel('(*: Automatically calculated)')
        layout.addWidget(note, last_row + 1, 0)
        for col, (idx, _) in enumerate(STAGE_HEADERS, start=1):
            if STAGE_HAS_COPY[idx]:
                btn = QPushButton('copy to present')
                btn.setFixedWidth(140)
                btn.clicked.connect(lambda *_, i=idx: self._on_copy_to_present(i))
                layout.addWidget(btn, last_row + 1, col, alignment=Qt.AlignmentFlag.AlignCenter)
        layout.setColumnStretch(0, 0)
        for c in range(1, len(STAGE_HEADERS) + 1):
            layout.setColumnStretch(c, 1)
        gb.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        return gb

    def _build_update_group(self) -> QGroupBox:
        gb = QGroupBox('Update Reference Specimen Size and Initialize Strains')
        layout = QGridLayout(gb)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(4)
        before_btn = QPushButton('Before Consolidation')
        before_btn.clicked.connect(self._on_before_consolidation)
        layout.addWidget(before_btn, 0, 0)
        before_lbl = QLabel('Assuming isotropic deformation (where the volumetric strain is three times the axial strain), the reference size of the specimen is updated based on the axial displacement data.')
        before_lbl.setWordWrap(True)
        layout.addWidget(before_lbl, 0, 1)
        after_btn = QPushButton('After Consolidation')
        after_btn.clicked.connect(self._on_after_consolidation)
        layout.addWidget(after_btn, 1, 0)
        after_lbl = QLabel('Update reference specimen size from current specimen strains.')
        after_lbl.setWordWrap(True)
        layout.addWidget(after_lbl, 1, 1)
        layout.setColumnStretch(1, 1)
        return gb

    def _build_footer(self) -> QHBoxLayout:
        h = QHBoxLayout()
        h.addStretch(1)
        load_btn = QPushButton('Import')
        load_btn.clicked.connect(self._on_load)
        h.addWidget(load_btn)
        save_btn = QPushButton('Export')
        save_btn.clicked.connect(self._on_save)
        h.addWidget(save_btn)
        return h

    def _on_typed_commit(self) -> None:
        try:
            self.commit_to_ctx()
        except ValueError as e:
            QMessageBox.warning(self, 'Invalid Input', str(e))
            return
        self.refresh_from_ctx()

    def refresh_from_ctx(self) -> None:
        ctx = get_context()
        sd = ctx.SpecimenData
        self._mem_young_edit.setText(_format(sd.membrane_modulus))
        self._mem_thick_edit.setText(_format(sd.membrane_thickness))
        self._cap_weight_edit.setText(_format(sd.cap_weight))
        for idx, _ in STAGE_HEADERS:
            row_view = sd.Stage.row(idx)
            for field, _ in ROW_DEFS:
                self._stage_edits[idx, field].setText(_format(getattr(row_view, field)))

    def commit_to_ctx(self) -> None:
        ctx = get_context()
        sd = ctx.SpecimenData
        for idx, _ in STAGE_HEADERS:
            for field, _ in ROW_DEFS:
                if not _is_cell_editable(idx, field):
                    continue
                val = self._parse_float(self._stage_edits[idx, field])
                setattr(sd.Stage[idx], field, val)
        self._recalc_volumes_areas()

    def _recalc_volumes_areas(self) -> None:
        ctx = get_context()
        stage = ctx.SpecimenData.Stage
        d = stage.get_field('diameter').astype(np.float64)
        h = stage.get_field('height').astype(np.float64)
        area = np.pi * d * d / 4.0
        volume = area * h
        stage.set_field('area', area.astype(np.float32))
        stage.set_field('volume', volume.astype(np.float32))

    def _on_copy_to_present(self, src_idx: int) -> None:
        try:
            self.commit_to_ctx()
        except ValueError as e:
            QMessageBox.warning(self, 'Invalid Input', str(e))
            return
        ctx = get_context()
        ctx.SpecimenData.Stage._a[DSS_STAGE_PRESENT] = ctx.SpecimenData.Stage._a[src_idx]
        self.refresh_from_ctx()
        self._log_to_parent(f'SpecimenData: stage {self._stage_name(src_idx)} copied to Present')
        self.specimen_changed.emit()

    def _on_before_consolidation(self) -> None:
        ctx = get_context()
        aio_phy = ctx.AIO.AI.phy
        ai_cal_c = ctx.AIO.AI._a['Cal']['c']
        lvdt, lcdpt, ldt1, ldt2 = (float(aio_phy[ch]) for ch in ZERO_ADJUST_CHANNELS)
        stage = ctx.SpecimenData.Stage
        present_h = float(stage[DSS_STAGE_PRESENT].height)
        present_v = float(stage[DSS_STAGE_PRESENT].volume)
        present_l1 = float(stage[DSS_STAGE_PRESENT].ldt_1)
        present_l2 = float(stage[DSS_STAGE_PRESENT].ldt_2)
        before_h = present_h - lvdt
        before_v = present_v * (1.0 - 3.0 * lvdt / present_h)
        before_a = before_v / before_h
        before = stage[DSS_STAGE_BEFORE]
        before.height = before_h
        before.volume = before_v
        before.area = before_a
        before.diameter = float(np.sqrt(4.0 * before_a / np.pi))
        before.ldt_1 = present_l1 - ldt1
        before.ldt_2 = present_l2 - ldt2
        ai_cal_c[list(ZERO_ADJUST_CHANNELS)] -= np.array([lvdt, lcdpt, ldt1, ldt2], dtype=np.float32)
        stage._a[DSS_STAGE_PRESENT] = stage._a[DSS_STAGE_BEFORE]
        self.refresh_from_ctx()
        self._log_to_parent('SpecimenData: Before Consolidation applied')
        self.specimen_changed.emit()

    def _on_after_consolidation(self) -> None:
        ctx = get_context()
        aio_phy = ctx.AIO.AI.phy
        ai_cal_c = ctx.AIO.AI._a['Cal']['c']
        lvdt, lcdpt, ldt1, ldt2 = (float(aio_phy[ch]) for ch in ZERO_ADJUST_CHANNELS)
        stage = ctx.SpecimenData.Stage
        present = stage[DSS_STAGE_PRESENT]
        present_h = float(present.height)
        present_v = float(present.volume)
        present_a = float(present.area)
        present_d = float(present.diameter)
        present_l1 = float(present.ldt_1)
        present_l2 = float(present.ldt_2)
        new_h = present_h - lvdt
        new_v = present_v - lcdpt
        new_a = new_v / new_h
        after = stage[DSS_STAGE_AFTER]
        after.height = new_h
        after.volume = new_v
        after.area = new_a
        after.diameter = present_d * float(np.sqrt(new_a / present_a))
        after.ldt_1 = present_l1 - ldt1
        after.ldt_2 = present_l2 - ldt2
        ai_cal_c[list(ZERO_ADJUST_CHANNELS)] -= np.array([lvdt, lcdpt, ldt1, ldt2], dtype=np.float32)
        stage._a[DSS_STAGE_PRESENT] = stage._a[DSS_STAGE_AFTER]
        self.refresh_from_ctx()
        self._log_to_parent('SpecimenData: After Consolidation applied')
        self.specimen_changed.emit()

    def _on_save(self) -> None:
        try:
            self.commit_to_ctx()
        except ValueError as e:
            QMessageBox.warning(self, 'Invalid Input', str(e))
            return
        ts = datetime.now().strftime('%Y%m%d_specimen')
        path, _ = QFileDialog.getSaveFileName(self, 'Save Specimen Data', f'{ts}', 'JSON Files (*.json);;All Files (*)')
        if not path:
            return
        if not path.lower().endswith('.json'):
            path = f'{path}.json'
        ctx = get_context()
        sd = ctx.SpecimenData
        data = {'type': 'SpecimenData', 'membrane_youngs_modulus': float(sd.membrane_modulus), 'membrane_thickness': float(sd.membrane_thickness), 'cap_weight': float(sd.cap_weight)}
        for stage_label, stage_idx in (('present', DSS_STAGE_PRESENT), ('initial', DSS_STAGE_INITIAL), ('before', DSS_STAGE_BEFORE), ('after', DSS_STAGE_AFTER)):
            row = sd.Stage.row(stage_idx)
            data[stage_label] = {field: float(getattr(row, field)) for field, _ in ROW_DEFS}
        try:
            with Path(path).open('w', encoding='utf-8') as fp:
                json.dump(data, fp, indent=2)
        except OSError as e:
            QMessageBox.warning(self, 'Save Error', str(e))
            return
        self._log_to_parent(f'Specimen saved: {path}')

    def _on_load(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, 'Load Specimen Data', '', 'JSON Files (*.json);;All Files (*)')
        if not path:
            return
        try:
            with Path(path).open(encoding='utf-8') as fp:
                data = json.load(fp)
        except (OSError, json.JSONDecodeError) as e:
            QMessageBox.warning(self, 'Load Error', str(e))
            return
        if not isinstance(data, dict) or data.get('type') != 'SpecimenData':
            QMessageBox.warning(self, 'Load Error', 'Not a SpecimenData JSON file')
            return
        try:
            modulus, thickness, cap_wt, stages = self._parse_specimen_json(data)
        except (TypeError, ValueError) as e:
            QMessageBox.warning(self, 'Load Error', str(e))
            return
        ctx = get_context()
        sd = ctx.SpecimenData
        sd.membrane_modulus = modulus
        sd.membrane_thickness = thickness
        sd.cap_weight = cap_wt
        for stage_idx, values in stages.items():
            dst = sd.Stage[stage_idx]
            for field, val in values.items():
                setattr(dst, field, val)
        self._recalc_volumes_areas()
        self.refresh_from_ctx()
        self._log_to_parent(f'Specimen loaded: {path}')
        self.specimen_changed.emit()

    def _parse_specimen_json(self, data: dict[str, Any]) -> tuple[float, float, float, dict[int, dict[str, float]]]:
        modulus = float(data.get('membrane_youngs_modulus', 0.0))
        thickness = float(data.get('membrane_thickness', 0.0))
        cap_wt = float(data.get('cap_weight', 0.0))
        stages: dict[int, dict[str, float]] = {}
        for stage_label, stage_idx in (('present', DSS_STAGE_PRESENT), ('initial', DSS_STAGE_INITIAL), ('before', DSS_STAGE_BEFORE), ('after', DSS_STAGE_AFTER)):
            src = data.get(stage_label)
            if not isinstance(src, dict):
                continue
            values: dict[str, float] = {}
            for field, _ in ROW_DEFS:
                key = field if field in src else _LEGACY_JSON_KEYS.get(field, field)
                if src.get(key) is None:
                    continue
                val = float(src[key])
                if not np.isfinite(val):
                    raise ValueError(f"'{field}' is not finite")
                values[field] = val
            stages[stage_idx] = values
        return (modulus, thickness, cap_wt, stages)

    @staticmethod
    def _parse_float(edit: QLineEdit) -> float:
        text = edit.text()
        if not text.strip():
            return 0.0
        try:
            value = float(text)
        except ValueError as exc:
            raise ValueError(f"'{text}' is not a valid number") from exc
        if not np.isfinite(value):
            raise ValueError(f"'{text}' is not finite")
        return value

    @staticmethod
    def _stage_name(idx: int) -> str:
        return {DSS_STAGE_PRESENT: 'Present', DSS_STAGE_INITIAL: 'Initial', DSS_STAGE_BEFORE: 'Before', DSS_STAGE_AFTER: 'After'}.get(idx, str(idx))

    def _log_to_parent(self, msg: str) -> None:
        parent = self.parent()
        log = getattr(parent, '_log', None)
        if log is not None:
            log('INFO', msg)
