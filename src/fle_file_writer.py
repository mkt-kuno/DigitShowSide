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
from typing import Protocol
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fle_csv_writer import CsvWriter
from fle_npy_writer import NpyWriter
from fle_npz_writer import NpzWriter
from fle_sqlite_writer import SqliteWriter
from fle_tsv_writer import TsvWriter

class FileWriter(Protocol):

    @property
    def is_open(self) -> bool:
        ...

    def open(self, start_dt: datetime | None=None) -> None:
        ...

    def append_row(self, timestamp: datetime, ai_raw: np.ndarray, ai_phy: np.ndarray, ao_raw: np.ndarray, par: np.ndarray) -> None:
        ...

    def close(self) -> None:
        ...

def open_writer(path: str | os.PathLike[str]) -> FileWriter:
    ext = Path(path).suffix.lower()
    if ext == '.tsv':
        return TsvWriter(path)
    if ext == '.csv':
        return CsvWriter(path)
    if ext == '.npy':
        return NpyWriter(path)
    if ext == '.npz':
        return NpzWriter(path)
    if ext in ('.sqlite', '.sqlite3', '.db'):
        return SqliteWriter(path)
    raise ValueError(f'Unsupported file extension: {ext!r}')
__all__ = ['FileWriter', 'open_writer']
