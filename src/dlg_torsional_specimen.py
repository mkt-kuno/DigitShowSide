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
from utl_context import DSS_AI_CH_LCDPT, DSS_AI_CH_V_DISP, DSS_STAGE_AFTER, DSS_STAGE_BEFORE, DSS_STAGE_INITIAL, DSS_STAGE_PRESENT, get_context
STAGE_HEADERS: tuple[tuple[int, str], ...] = ((DSS_STAGE_PRESENT, 'present'), (DSS_STAGE_INITIAL, 'initial'), (DSS_STAGE_BEFORE, 'before consolidation'), (DSS_STAGE_AFTER, 'after consolidation'))
STAGE_HAS_COPY: dict[int, bool] = {DSS_STAGE_PRESENT: False, DSS_STAGE_INITIAL: True, DSS_STAGE_BEFORE: True, DSS_STAGE_AFTER: True}
ROW_DEFS: tuple[tuple[str, str], ...] = (('diameter_in', 'Inner Diameter (mm)'), ('diameter_out', 'Outer Diameter (mm)'), ('height', 'Height (mm)'), ('volume', 'Volume (mm3)'), ('dia_in_membrane', 'Diameter of inner membrane (mm)'), ('dia_out_membrane', 'Diameter of outer membrane (mm)'), ('height_in_membrane', 'Height of inner membrane (mm)'), ('height_out_membrane', 'Height of outer membrane (mm)'))
_AUTO_FIELDS: frozenset[str] = frozenset({'volume'})
_USER_FIELDS: frozenset[str] = frozenset({'diameter_in', 'diameter_out', 'height', 'dia_in_membrane', 'dia_out_membrane', 'height_in_membrane', 'height_out_membrane'})
_ZERO_ADJUST_CHANNELS: tuple[int, ...] = (DSS_AI_CH_V_DISP, DSS_AI_CH_LCDPT)
_STAGE_JSON_KEYS: tuple[tuple[str, int], ...] = (('present', DSS_STAGE_PRESENT), ('initial', DSS_STAGE_INITIAL), ('before', DSS_STAGE_BEFORE), ('after', DSS_STAGE_AFTER))

def _is_cell_editable(stage_idx: int, field: str) -> bool:
    if field in _AUTO_FIELDS:
        return False
    if stage_idx == DSS_STAGE_PRESENT:
        return False
    return field in _USER_FIELDS

def _format(value: float) -> str:
    return f'{float(value):g}'

class TorsionalSpecimenDialog(QDialog):
    specimen_changed = Signal()

    def __init__(self, parent: QWidget | None=None) -> None:
        super().__init__(parent)
        self.setModal(True)
        self.setWindowTitle('Specimen Data')
        self.resize(720, 640)
        self.setMinimumWidth(720)
        self._stage_edits: dict[tuple[int, str], QLineEdit] = {}
        self._apparatus_edits: dict[str, QLineEdit] = {}
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
        root.addLayout(self._build_footer())
        root.addStretch(0)

    def _add_labeled_edit(self, grid: QGridLayout, row: int, col: int, label_text: str, font: QFont, field_key: str, default: str='0') -> None:
        label = QLabel(label_text)
        grid.addWidget(label, row, col)
        edit = QLineEdit(default)
        edit.setFont(font)
        edit.setMaximumWidth(120)
        edit.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        edit.returnPressed.connect(self._on_update)
        grid.addWidget(edit, row, col + 1)
        self._apparatus_edits[field_key] = edit

    def _build_apparatus_group(self, input_font: QFont) -> QGroupBox:
        gb = QGroupBox('Parameters of Test Apparatus')
        layout = QGridLayout(gb)
        layout.setHorizontalSpacing(8)
        layout.setVerticalSpacing(4)
        self._add_labeled_edit(layout, 0, 0, 'Referential diameter of inner membrane (mm)', input_font, 'r_dia_in_m')
        self._add_labeled_edit(layout, 1, 0, 'Referential diameter of outer membrane (mm)', input_font, 'r_dia_out_m')
        self._add_labeled_edit(layout, 2, 0, 'Referential height of inner membrane (mm)', input_font, 'r_height_in_m')
        self._add_labeled_edit(layout, 3, 0, 'Referential height of outer membrane (mm)', input_font, 'r_height_out_m')
        self._add_labeled_edit(layout, 0, 2, "Young's Modulus of membrane (kPa)", input_font, 'membrane_modulus')
        self._add_labeled_edit(layout, 1, 2, 'Thickness of membrane (mm)', input_font, 'membrane_thickness')
        self._add_labeled_edit(layout, 2, 2, 'Cap Weight (N)', input_font, 'cap_weight')
        self._add_labeled_edit(layout, 3, 2, 'Rod Area (mm2)', input_font, 'rod_area')
        layout.setColumnStretch(1, 0)
        layout.setColumnStretch(3, 0)
        layout.setColumnStretch(4, 1)
        return gb

    def _build_stage_group(self, input_font: QFont) -> QGroupBox:
        gb = QGroupBox('')
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
                edit.setMaximumWidth(110)
                edit.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                cell_editable = _is_cell_editable(idx, field)
                edit.setReadOnly(not cell_editable)
                if not cell_editable:
                    edit.setFrame(False)
                if cell_editable:
                    edit.returnPressed.connect(self._on_update)
                layout.addWidget(edit, r, col)
                self._stage_edits[idx, field] = edit
        btn_row = len(ROW_DEFS) + 1
        for col, (idx, _) in enumerate(STAGE_HEADERS, start=1):
            if STAGE_HAS_COPY[idx]:
                btn = QPushButton('-> present..')
                btn.setFixedWidth(110)
                btn.clicked.connect(lambda *_, i=idx: self._on_to_present(i))
                layout.addWidget(btn, btn_row, col, alignment=Qt.AlignmentFlag.AlignCenter)
        update_btn = QPushButton('Update variants')
        update_btn.clicked.connect(self._on_update)
        layout.addWidget(update_btn, btn_row, len(STAGE_HEADERS) + 1)
        note = QLabel('Volume is automatically calculated.')
        layout.addWidget(note, btn_row + 1, 0, 1, 2)
        layout.setColumnStretch(0, 1)
        for c in range(1, len(STAGE_HEADERS) + 1):
            layout.setColumnStretch(c, 0)
        gb.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        return gb

    def _build_footer(self) -> QHBoxLayout:
        h = QHBoxLayout()
        init_group = QGroupBox('Initialize Standard Specimen Data')
        init_layout = QGridLayout(init_group)
        before_btn = QPushButton('Before Consolidation')
        before_btn.clicked.connect(self._on_before_consolidation)
        init_layout.addWidget(before_btn, 0, 0)
        after_btn = QPushButton('After Consolidation')
        after_btn.clicked.connect(self._on_after_consolidation)
        init_layout.addWidget(after_btn, 1, 0)
        h.addWidget(init_group)
        h.addStretch(1)
        save_btn = QPushButton('Save to file')
        save_btn.clicked.connect(self._on_save)
        h.addWidget(save_btn)
        import_btn = QPushButton('Import')
        import_btn.clicked.connect(self._on_import)
        h.addWidget(import_btn)
        close_btn = QPushButton('Close')
        close_btn.clicked.connect(self.reject)
        h.addWidget(close_btn)
        return h

    def refresh_from_ctx(self) -> None:
        ctx = get_context()
        st = ctx.SpecimenTorsional
        apparatus: dict[str, float] = {'r_dia_in_m': float(st.r_dia_in_m), 'r_dia_out_m': float(st.r_dia_out_m), 'r_height_in_m': float(st.r_height_in_m), 'r_height_out_m': float(st.r_height_out_m), 'membrane_modulus': float(st.membrane_modulus), 'membrane_thickness': float(st.membrane_thickness), 'cap_weight': float(st.cap_weight), 'rod_area': float(st.rod_area)}
        for key, val in apparatus.items():
            self._apparatus_edits[key].setText(_format(val))
        for idx, _ in STAGE_HEADERS:
            row_view = st.Stage.row(idx)
            for field, _ in ROW_DEFS:
                self._stage_edits[idx, field].setText(_format(getattr(row_view, field)))

    def commit_to_ctx(self) -> None:
        ctx = get_context()
        st = ctx.SpecimenTorsional
        apparatus_fields: tuple[tuple[str, str], ...] = (('r_dia_in_m', 'Referential diameter of inner membrane'), ('r_dia_out_m', 'Referential diameter of outer membrane'), ('r_height_in_m', 'Referential height of inner membrane'), ('r_height_out_m', 'Referential height of outer membrane'), ('membrane_modulus', "Young's Modulus of membrane"), ('membrane_thickness', 'Thickness of membrane'), ('cap_weight', 'Cap Weight'), ('rod_area', 'Rod Area'))
        for key, label in apparatus_fields:
            setattr(st, key, self._parse_float(self._apparatus_edits[key], label))
        for idx, _ in STAGE_HEADERS:
            for field, _ in ROW_DEFS:
                if not _is_cell_editable(idx, field):
                    continue
                val = self._parse_float(self._stage_edits[idx, field], field)
                setattr(st.Stage[idx], field, val)
        st.recalc_volumes()

    def _on_update(self) -> None:
        try:
            self.commit_to_ctx()
        except ValueError as e:
            QMessageBox.warning(self, 'Invalid Input', str(e))
            return
        self.refresh_from_ctx()
        self._log_to_parent('SpecimenTorsional: updated')
        self.specimen_changed.emit()

    def _on_to_present(self, src_idx: int) -> None:
        try:
            self.commit_to_ctx()
        except ValueError as e:
            QMessageBox.warning(self, 'Invalid Input', str(e))
            return
        ctx = get_context()
        ctx.SpecimenTorsional.Stage._a[DSS_STAGE_PRESENT] = ctx.SpecimenTorsional.Stage._a[src_idx]
        self.refresh_from_ctx()
        self._log_to_parent(f'SpecimenTorsional: stage {src_idx} copied to Present')
        self.specimen_changed.emit()

    def _current_phys(self) -> tuple[float, float, float, float]:
        ctx = get_context()
        st = ctx.SpecimenTorsional
        present = st.Stage.row(DSS_STAGE_PRESENT)
        phy = ctx.AIO.AI.phy
        ext_mm = float(phy[DSS_AI_CH_V_DISP])
        bw2_mm3 = float(phy[DSS_AI_CH_LCDPT])
        h0 = float(present.height)
        v0 = float(present.volume)
        height = h0 - ext_mm
        volume = v0 - bw2_mm3
        shrink = 1.0
        if v0 > 0.0 and h0 > 0.0 and (1.0 - ext_mm / h0 > 0.0):
            shrink = float(np.sqrt((1.0 - bw2_mm3 / v0) / (1.0 - ext_mm / h0)))
        diameter_in = float(present.diameter_in) * shrink
        diameter_out = float(present.diameter_out) * shrink
        return (height, volume, diameter_in, diameter_out)

    def _zero_adjust_displacements(self) -> None:
        ctx = get_context()
        phy = ctx.AIO.AI.phy
        cal_c = ctx.AIO.AI._a['Cal']['c']
        cal_c[list(_ZERO_ADJUST_CHANNELS)] -= np.array([float(phy[ch]) for ch in _ZERO_ADJUST_CHANNELS], dtype=np.float32)

    def _on_before_consolidation(self) -> None:
        try:
            self.commit_to_ctx()
        except ValueError as e:
            QMessageBox.warning(self, 'Invalid Input', str(e))
            return
        ctx = get_context()
        st = ctx.SpecimenTorsional
        height_now, _, _, _ = self._current_phys()
        initial = st.Stage.row(DSS_STAGE_INITIAL)
        h1 = float(initial.height)
        v1 = float(initial.volume)
        if h1 <= 0.0 or v1 <= 0.0 or h1 <= height_now:
            QMessageBox.warning(self, 'Invalid Data', 'Initial Height/Volume must be positive.')
            return
        be_consol_ez = 1.0 - height_now / h1
        be_consol_ev = be_consol_ez * 3.0
        before = st.Stage.row(DSS_STAGE_BEFORE)
        before.height = height_now
        before.volume = v1 * (1.0 - be_consol_ev)
        ratio = float(np.sqrt((1.0 - be_consol_ev) / (1.0 - be_consol_ez)))
        before.diameter_in = float(initial.diameter_in) * ratio
        before.diameter_out = float(initial.diameter_out) * ratio
        before.dia_in_membrane = float(before.diameter_in) - float(st.membrane_thickness) / 2.0
        before.dia_out_membrane = float(before.diameter_out) + float(st.membrane_thickness) / 2.0
        before.height_in_membrane = height_now
        before.height_out_membrane = height_now
        self._zero_adjust_displacements()
        self.refresh_from_ctx()
        self._on_to_present(DSS_STAGE_BEFORE)
        self._log_to_parent('SpecimenTorsional: Before Consolidation applied')

    def _on_after_consolidation(self) -> None:
        try:
            self.commit_to_ctx()
        except ValueError as e:
            QMessageBox.warning(self, 'Invalid Input', str(e))
            return
        ctx = get_context()
        st = ctx.SpecimenTorsional
        height_now, volume_now, diameter_in_now, diameter_out_now = self._current_phys()
        if height_now <= 0.0 or volume_now <= 0.0:
            QMessageBox.warning(self, 'Invalid Data', 'Current Height/Volume must be positive.')
            return
        after = st.Stage.row(DSS_STAGE_AFTER)
        after.height = height_now
        after.volume = volume_now
        after.diameter_in = diameter_in_now
        after.diameter_out = diameter_out_now
        after.dia_in_membrane = diameter_in_now - float(st.membrane_thickness) / 2.0
        after.dia_out_membrane = diameter_out_now + float(st.membrane_thickness) / 2.0
        after.height_in_membrane = height_now
        after.height_out_membrane = height_now
        self._zero_adjust_displacements()
        self.refresh_from_ctx()
        self._on_to_present(DSS_STAGE_AFTER)
        self._log_to_parent('SpecimenTorsional: After Consolidation applied')

    def _on_save(self) -> None:
        try:
            self.commit_to_ctx()
        except ValueError as e:
            QMessageBox.warning(self, 'Invalid Input', str(e))
            return
        ts = datetime.now().strftime('%Y%m%d_specimen')
        path, _ = QFileDialog.getSaveFileName(self, 'Save Specimen Data', f'{ts}.tsj.json', 'Torsional Specimen JSON (*.tsj.json);;JSON Files (*.json);;All Files (*)')
        if not path:
            return
        if not path.lower().endswith('.json'):
            path = f'{path}.tsj.json'
        ctx = get_context()
        st = ctx.SpecimenTorsional
        data: dict[str, Any] = {'type': 'SpecimenDataTorsional', 'referential_diameter_inner_membrane': float(st.r_dia_in_m), 'referential_diameter_outer_membrane': float(st.r_dia_out_m), 'referential_height_inner_membrane': float(st.r_height_in_m), 'referential_height_outer_membrane': float(st.r_height_out_m), 'membrane_youngs_modulus': float(st.membrane_modulus), 'membrane_thickness': float(st.membrane_thickness), 'rod_area': float(st.rod_area), 'cap_weight': float(st.cap_weight)}
        for stage_label, stage_idx in _STAGE_JSON_KEYS:
            row_view = st.Stage.row(stage_idx)
            data[stage_label] = {field: float(getattr(row_view, field)) for field, _ in ROW_DEFS}
        try:
            with Path(path).open('w', encoding='utf-8') as fp:
                json.dump(data, fp, indent=2)
        except OSError as e:
            QMessageBox.warning(self, 'Save Error', str(e))
            return
        self._log_to_parent(f'SpecimenTorsional saved: {path}')

    def _on_import(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, 'Import Specimen Data', '', 'Torsional Specimen JSON (*.tsj.json);;JSON Files (*.json);;All Files (*)')
        if not path:
            return
        try:
            with Path(path).open(encoding='utf-8') as fp:
                data = json.load(fp)
        except (OSError, json.JSONDecodeError) as e:
            QMessageBox.warning(self, 'Import Error', str(e))
            return
        if not isinstance(data, dict) or data.get('type') != 'SpecimenDataTorsional':
            QMessageBox.warning(self, 'Import Error', 'Not a SpecimenDataTorsional JSON file')
            return
        ctx = get_context()
        st = ctx.SpecimenTorsional
        try:
            self._apply_import_data(st, data)
        except (TypeError, ValueError) as e:
            QMessageBox.warning(self, 'Import Error', str(e))
            return
        st.recalc_volumes()
        self.refresh_from_ctx()
        self._log_to_parent(f'SpecimenTorsional imported: {path}')
        self.specimen_changed.emit()

    @staticmethod
    def _apply_import_data(st: Any, data: dict[str, Any]) -> None:
        scalar_map: tuple[tuple[str, str], ...] = (('referential_diameter_inner_membrane', 'r_dia_in_m'), ('referential_diameter_outer_membrane', 'r_dia_out_m'), ('referential_height_inner_membrane', 'r_height_in_m'), ('referential_height_outer_membrane', 'r_height_out_m'), ('membrane_youngs_modulus', 'membrane_modulus'), ('membrane_thickness', 'membrane_thickness'), ('rod_area', 'rod_area'), ('cap_weight', 'cap_weight'))
        for json_key, attr in scalar_map:
            if data.get(json_key) is not None:
                setattr(st, attr, float(data[json_key]))
        for stage_label, stage_idx in _STAGE_JSON_KEYS:
            src = data.get(stage_label)
            if not isinstance(src, dict):
                continue
            dst = st.Stage.row(stage_idx)
            for field, _ in ROW_DEFS:
                if src.get(field) is not None:
                    val = float(src[field])
                    if not np.isfinite(val):
                        raise ValueError(f"'{field}' is not finite")
                    setattr(dst, field, val)

    @staticmethod
    def _parse_float(edit: QLineEdit, label: str) -> float:
        text = edit.text()
        if not text.strip():
            return 0.0
        try:
            value = float(text)
        except ValueError as exc:
            raise ValueError(f"{label}: '{text}' is not a valid number") from exc
        if not np.isfinite(value):
            raise ValueError(f"{label}: '{text}' is not finite")
        return value

    def _log_to_parent(self, msg: str) -> None:
        parent = self.parent()
        log = getattr(parent, '_log', None)
        if log is not None:
            log('INFO', msg)
