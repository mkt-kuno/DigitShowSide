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
import sys
from datetime import datetime
from pathlib import Path
from typing import IO
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import contextlib
from utl_context import DSS_APP_NAME as APP_NAME, DSS_APP_VERSION as APP_VERSION, DSS_CHDEF_PARAM_MAX, DSS_MB_AI_REGISTER_COUNT, DSS_MB_AO_REGISTER_COUNT, DSS_VOLTAGE_OUT_LABELS, dss_active_parameter_labels, dss_active_physical_labels, dss_active_raw_labels, get_context
HEADER_LINE_COUNT: int = 16
HEADER_START_MARK: str = '# START_HEADER'
HEADER_END_MARK: str = '# END_HEADER'
DATA_COLUMN_NAMES: tuple[str, ...] = ('timestamp', *tuple((f'ai_raw_{i:02d}' for i in range(DSS_MB_AI_REGISTER_COUNT))), *tuple((f'ai_phy_{i:02d}' for i in range(DSS_MB_AI_REGISTER_COUNT))), *tuple((f'ao_raw_{i:02d}' for i in range(DSS_MB_AO_REGISTER_COUNT))), *tuple((f'par_{i:02d}' for i in range(DSS_CHDEF_PARAM_MAX))))
EXPECTED_COLUMN_COUNT: int = 1 + 2 * DSS_MB_AI_REGISTER_COUNT + DSS_MB_AO_REGISTER_COUNT + DSS_CHDEF_PARAM_MAX

def format_timestamp_local(dt: datetime) -> str:
    return dt.strftime('%Y-%m-%d %H.%M.%S') + f'.{dt.microsecond // 1000:03d}'

def default_filename(now: datetime | None=None) -> str:
    dt = now if now is not None else datetime.now()
    return dt.strftime('%Y%m%d_%H%M%S') + '.tsv'

def _package_version(dist_name: str) -> str:
    try:
        from importlib.metadata import version as _v
        return _v(dist_name)
    except Exception:
        try:
            import importlib
            mod = importlib.import_module(dist_name)
            return str(getattr(mod, '__version__', '(unknown)'))
        except Exception:
            return '(unknown)'

def _pyside_version() -> str:
    try:
        import PySide6
        return str(PySide6.__version__)
    except Exception:
        return '(unknown)'

def _open_shared_append(path: Path) -> IO[str]:
    return path.open('a', encoding='utf-8', newline='\n', buffering=1)

def _csv_escape_label(label: str) -> str:
    if any((c in label for c in (',', '"', '\r', '\n'))):
        return '"' + label.replace('"', '""') + '"'
    return label

def _build_header_lines(*, sep: str='\t', quote_labels: bool=False) -> list[str]:
    ctx = get_context()

    def _calib_row_ai(field: str) -> list[float]:
        return [float(getattr(ctx.AIO.AI[i].Cal, field)) for i in range(DSS_MB_AI_REGISTER_COUNT)]

    def _calib_row_ao(field: str) -> list[float]:
        return [float(getattr(ctx.AIO.AO[i].Cal, field)) for i in range(DSS_MB_AO_REGISTER_COUNT)]

    def _tsv_row(prefix: str, values: list[float]) -> str:
        return prefix + sep + sep.join((f'{v:.6g}' for v in values))

    def _label_combined() -> str:
        mode = ctx.Control.mode
        all_labels = dss_active_raw_labels(mode) + dss_active_physical_labels(mode) + DSS_VOLTAGE_OUT_LABELS + dss_active_parameter_labels(mode)
        formatted = [_csv_escape_label(label) for label in all_labels] if quote_labels else all_labels
        return '# label' + sep + sep.join(formatted)
    lines: list[str] = [f'# {APP_NAME} v{APP_VERSION}', f'# Python: {platform.python_version()}', f'# PySide: {_pyside_version()}', f"# pyqtgraph: {_package_version('pyqtgraph')}", f"# pymodbus: {_package_version('pymodbus')}", f"# pyserial: {_package_version('pyserial')}", f"# numpy: {_package_version('numpy')}", _tsv_row('# AI.Cal.a', _calib_row_ai('a')), _tsv_row('# AI.Cal.b', _calib_row_ai('b')), _tsv_row('# AI.Cal.c', _calib_row_ai('c')), _tsv_row('# AO.Cal.a', _calib_row_ao('a')), _tsv_row('# AO.Cal.b', _calib_row_ao('b')), _label_combined()]
    target = HEADER_LINE_COUNT - 2
    if len(lines) < target:
        pad = target - len(lines)
        lines.extend(['#'] * pad)
    if len(lines) != target:
        raise RuntimeError(f'TSV header content size mismatch: got {len(lines)}, expected {target}')
    return lines

class TsvWriter:

    def __init__(self, path: str | os.PathLike[str], *, csv: bool=False) -> None:
        self._path = Path(path)
        self._sep = ',' if csv else '\t'
        self._quote_labels = csv
        self._fp: IO[str] | None = None
        self._closed = True

    @property
    def is_open(self) -> bool:
        return self._fp is not None and (not self._closed)

    def open(self, start_dt: datetime | None=None) -> None:
        if self.is_open:
            return
        start_dt = start_dt if start_dt is not None else datetime.now()
        is_new = not self._path.exists() or self._path.stat().st_size == 0
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._fp = _open_shared_append(self._path)
        self._closed = False
        if is_new:
            self._write_header(start_dt)
        self._fp.flush()

    def close(self) -> None:
        if self._fp is None:
            self._closed = True
            return
        with contextlib.suppress(Exception):
            self._fp.flush()
        with contextlib.suppress(Exception):
            self._fp.close()
        self._fp = None
        self._closed = True

    def _write_header(self, start_dt: datetime) -> None:
        assert self._fp is not None
        lines = [HEADER_START_MARK]
        lines.extend(_build_header_lines(sep=self._sep, quote_labels=self._quote_labels))
        lines.append(HEADER_END_MARK)
        if len(lines) != HEADER_LINE_COUNT:
            raise RuntimeError(f'TSV header size mismatch: got {len(lines)}, expected {HEADER_LINE_COUNT}')
        self._fp.write('\n'.join(lines) + '\n')
        self._fp.write(self._sep.join(DATA_COLUMN_NAMES) + '\n')

    def append_row(self, timestamp: datetime, ai_raw: np.ndarray, ai_phy: np.ndarray, ao_raw: np.ndarray, par: np.ndarray) -> None:
        if not self.is_open:
            raise RuntimeError('TsvWriter is not open')
        assert self._fp is not None
        ts = format_timestamp_local(timestamp)
        ai_raw_f32 = np.asarray(ai_raw, dtype=np.float32)
        ai_raw_strs = ['nan' if np.isnan(float(v)) else f'{float(v):.6g}' for v in ai_raw_f32]
        ai_phy_f32 = np.asarray(ai_phy, dtype=np.float32)
        ao_raw_u16 = np.asarray(ao_raw, dtype=np.uint16)
        par_f32 = np.asarray(par, dtype=np.float32)
        parts: list[str] = [ts]
        parts.extend(ai_raw_strs)
        parts.extend((f'{float(v):.6g}' for v in ai_phy_f32))
        parts.extend((str(int(v)) for v in ao_raw_u16))
        parts.extend((f'{float(v):.6g}' for v in par_f32))
        if len(parts) != EXPECTED_COLUMN_COUNT:
            raise RuntimeError(f'Tabular row column count mismatch: got {len(parts)}, expected {EXPECTED_COLUMN_COUNT}')
        self._fp.write(self._sep.join(parts) + '\n')
        self._fp.flush()
__all__ = ['DATA_COLUMN_NAMES', 'EXPECTED_COLUMN_COUNT', 'HEADER_END_MARK', 'HEADER_LINE_COUNT', 'HEADER_START_MARK', 'TsvWriter', 'default_filename', 'format_timestamp_local']
