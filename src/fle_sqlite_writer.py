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
import sqlite3
import sys
from datetime import datetime
from pathlib import Path
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utl_context import DSS_CHDEF_PARAM_MAX, DSS_MB_AI_REGISTER_COUNT, DSS_MB_AO_REGISTER_COUNT, DSS_VOLTAGE_OUT_LABELS, dss_active_parameter_labels, dss_active_physical_labels, dss_active_raw_labels, get_context

def _build_data_col_names() -> list[str]:
    cols = ['time']
    cols.extend((f'raw_{ch:02d}' for ch in range(DSS_MB_AI_REGISTER_COUNT)))
    cols.extend((f'phy_{ch:02d}' for ch in range(DSS_MB_AI_REGISTER_COUNT)))
    cols.extend((f'par_{ch:02d}' for ch in range(DSS_CHDEF_PARAM_MAX)))
    cols.extend((f'out_{ch:02d}' for ch in range(DSS_MB_AO_REGISTER_COUNT)))
    return cols

def _build_label_col_names() -> list[str]:
    cols: list[str] = []
    cols.extend((f'raw_{ch:02d}' for ch in range(DSS_MB_AI_REGISTER_COUNT)))
    cols.extend((f'phy_{ch:02d}' for ch in range(DSS_MB_AI_REGISTER_COUNT)))
    cols.extend((f'par_{ch:02d}' for ch in range(DSS_CHDEF_PARAM_MAX)))
    cols.extend((f'out_{ch:02d}' for ch in range(DSS_MB_AO_REGISTER_COUNT)))
    return cols
_DATA_COLS: list[str] = _build_data_col_names()
_LABEL_COLS: list[str] = _build_label_col_names()
_TOTAL_DATA_COLS: int = len(_DATA_COLS)
_TOTAL_LABEL_COLS: int = len(_LABEL_COLS)
_CREATE_DATA_SQL: str = f"CREATE TABLE IF NOT EXISTS data (id INTEGER PRIMARY KEY AUTOINCREMENT,localtime TIMESTAMP DEFAULT (strftime('%Y-%m-%d %H:%M:%f', 'now', 'localtime')),{', '.join((f'{c} REAL' for c in _DATA_COLS))})"
_CREATE_LABEL_SQL: str = f"CREATE TABLE IF NOT EXISTS label (id INTEGER PRIMARY KEY,{', '.join((f'{c} TEXT' for c in _LABEL_COLS))})"
_INSERT_DATA_SQL: str = f"INSERT INTO data ({', '.join(_DATA_COLS)}) VALUES ({', '.join(['?'] * _TOTAL_DATA_COLS)})"
_INSERT_LABEL_SQL: str = f"INSERT OR REPLACE INTO label (id, {', '.join(_LABEL_COLS)}) VALUES (0, {', '.join(['?'] * _TOTAL_LABEL_COLS)})"

class SqliteWriter:
    _BATCH_SIZE: int = 100

    def __init__(self, path: str | os.PathLike[str]) -> None:
        self._path = Path(path)
        self._conn: sqlite3.Connection | None = None
        self._closed = True
        self._t0: datetime | None = None
        self._pending: int = 0

    @property
    def is_open(self) -> bool:
        return self._conn is not None and (not self._closed)

    def open(self, start_dt: datetime | None=None) -> None:
        if self.is_open:
            return
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._conn = sqlite3.connect(self._path)
        self._conn.execute('PRAGMA journal_mode=WAL')
        self._conn.execute('PRAGMA synchronous=NORMAL')
        self._conn.execute(_CREATE_DATA_SQL)
        self._conn.execute(_CREATE_LABEL_SQL)
        mode = get_context().Control.mode
        label_values: list[str] = [*dss_active_raw_labels(mode), *dss_active_physical_labels(mode), *dss_active_parameter_labels(mode), *DSS_VOLTAGE_OUT_LABELS]
        self._conn.execute(_INSERT_LABEL_SQL, label_values)
        self._conn.commit()
        self._t0 = start_dt
        self._pending = 0
        self._closed = False

    def close(self) -> None:
        if not self.is_open:
            return
        assert self._conn is not None
        if self._pending > 0:
            self._conn.commit()
            self._pending = 0
        self._conn.close()
        self._conn = None
        self._closed = True

    def append_row(self, timestamp: datetime, ai_raw: np.ndarray, ai_phy: np.ndarray, ao_raw: np.ndarray, par: np.ndarray) -> None:
        if not self.is_open:
            raise RuntimeError('SqliteWriter is not open')
        assert self._conn is not None
        if self._t0 is None:
            self._t0 = timestamp
        elapsed = (timestamp - self._t0).total_seconds()
        values: list[float] = [float(elapsed)]
        values.extend(np.asarray(ai_raw, dtype=np.float64).tolist())
        values.extend(np.asarray(ai_phy, dtype=np.float64).tolist())
        values.extend(np.asarray(par, dtype=np.float64).tolist())
        values.extend(np.asarray(ao_raw, dtype=np.float64).tolist())
        self._conn.execute(_INSERT_DATA_SQL, values)
        self._pending += 1
        if self._pending >= self._BATCH_SIZE:
            self._conn.commit()
            self._pending = 0
__all__ = ['SqliteWriter']
