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
from datetime import datetime
from pathlib import Path
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utl_context import DSS_CHDEF_PARAM_MAX, DSS_MB_AI_REGISTER_COUNT, DSS_MB_AO_REGISTER_COUNT

def _build_structured_dtype() -> np.dtype:
    names: list[str] = ['time']
    names.extend((f'raw_{ch:02d}' for ch in range(DSS_MB_AI_REGISTER_COUNT)))
    names.extend((f'phy_{ch:02d}' for ch in range(DSS_MB_AI_REGISTER_COUNT)))
    names.extend((f'par_{ch:02d}' for ch in range(DSS_CHDEF_PARAM_MAX)))
    names.extend((f'out_{ch:02d}' for ch in range(DSS_MB_AO_REGISTER_COUNT)))
    return np.dtype([(n, '<f8') for n in names])

class NpyWriter:
    _ROW_DTYPE: np.dtype = _build_structured_dtype()

    def __init__(self, path: str | os.PathLike[str]) -> None:
        self._path = Path(path)
        self._rows: list[np.ndarray] = []
        self._closed = True
        self._t0: datetime | None = None

    @property
    def is_open(self) -> bool:
        return not self._closed

    def open(self, start_dt: datetime | None=None) -> None:
        if self.is_open:
            return
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._rows = []
        self._t0 = start_dt
        self._closed = False

    def close(self) -> None:
        if not self.is_open:
            return
        arr = np.array(self._rows, dtype=self._ROW_DTYPE)
        with self._path.open('wb') as fp:
            np.save(fp, arr, allow_pickle=False)
        self._closed = True

    def append_row(self, timestamp: datetime, ai_raw: np.ndarray, ai_phy: np.ndarray, ao_raw: np.ndarray, par: np.ndarray) -> None:
        if not self.is_open:
            raise RuntimeError('NpyWriter is not open')
        if self._t0 is None:
            self._t0 = timestamp
        elapsed = (timestamp - self._t0).total_seconds()
        row = np.zeros((), dtype=self._ROW_DTYPE)
        row['time'] = np.float64(elapsed)
        ai_raw_f64 = np.asarray(ai_raw, dtype=np.float64)
        ai_phy_f64 = np.asarray(ai_phy, dtype=np.float64)
        ao_raw_f64 = np.asarray(ao_raw, dtype=np.float64)
        par_f64 = np.asarray(par, dtype=np.float64)
        for ch in range(DSS_MB_AI_REGISTER_COUNT):
            row[f'raw_{ch:02d}'] = ai_raw_f64[ch]
            row[f'phy_{ch:02d}'] = ai_phy_f64[ch]
        for ch in range(DSS_CHDEF_PARAM_MAX):
            row[f'par_{ch:02d}'] = par_f64[ch]
        for ch in range(DSS_MB_AO_REGISTER_COUNT):
            row[f'out_{ch:02d}'] = ao_raw_f64[ch]
        self._rows.append(row)
__all__ = ['NpyWriter']
