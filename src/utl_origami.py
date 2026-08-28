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
import time
import numpy as np

class OrigamiBuffer:

    def __init__(self, n_ai: int, n_ao: int, n_param: int, buf_max: int, not_saving_points: int) -> None:
        if buf_max < 2:
            raise ValueError('buf_max must be >= 2')
        if not_saving_points < 2:
            raise ValueError('not_saving_points must be >= 2')
        self._n_ai = int(n_ai)
        self._n_ao = int(n_ao)
        self._n_param = int(n_param)
        self._buf_max = int(buf_max)
        self._not_saving_points = int(not_saving_points)
        self._times: np.ndarray = np.empty(0, dtype=np.float64)
        self._ai_raw: np.ndarray = np.empty((0, self._n_ai), dtype=np.float32)
        self._ai_phy: np.ndarray = np.empty((0, self._n_ai), dtype=np.float32)
        self._ao_raw: np.ndarray = np.empty((0, self._n_ao), dtype=np.float32)
        self._params: np.ndarray = np.empty((0, self._n_param), dtype=np.float32)
        self._saving: bool = False
        self._num_folding: int = 0
        self._write_count: int = 0
        self._sample_counter: int = 0
        self._save_start_epoch: float | None = None

    def reset(self, saving: bool) -> None:
        self._times = np.empty(0, dtype=np.float64)
        self._ai_raw = np.empty((0, self._n_ai), dtype=np.float32)
        self._ai_phy = np.empty((0, self._n_ai), dtype=np.float32)
        self._ao_raw = np.empty((0, self._n_ao), dtype=np.float32)
        self._params = np.empty((0, self._n_param), dtype=np.float32)
        self._saving = bool(saving)
        self._num_folding = 0
        self._write_count = 0
        self._sample_counter = 0
        self._save_start_epoch = time.time() if saving else None

    def store(self, t: float, ai_raw: np.ndarray, ai_phy: np.ndarray, ao_raw: np.ndarray, params: np.ndarray) -> None:
        self._sample_counter += 1
        mask = 1 << self._num_folding
        if self._sample_counter & mask - 1 != 0:
            return
        if self._saving and self._write_count >= self._buf_max:
            self._fold()
        self._times = np.append(self._times, float(t))
        self._ai_raw = np.concatenate((self._ai_raw, ai_raw[None, :].astype(np.float32, copy=False)), axis=0)
        self._ai_phy = np.concatenate((self._ai_phy, ai_phy[None, :].astype(np.float32, copy=False)), axis=0)
        self._ao_raw = np.concatenate((self._ao_raw, ao_raw[None, :].astype(np.float32, copy=False)), axis=0)
        self._params = np.concatenate((self._params, params[None, :].astype(np.float32, copy=False)), axis=0)
        self._write_count += 1
        if not self._saving and self._write_count > self._not_saving_points:
            excess = self._write_count - self._not_saving_points
            self._times = self._times[excess:]
            self._ai_raw = self._ai_raw[excess:]
            self._ai_phy = self._ai_phy[excess:]
            self._ao_raw = self._ao_raw[excess:]
            self._params = self._params[excess:]
            self._write_count = self._not_saving_points

    @property
    def is_saving(self) -> bool:
        return self._saving

    @property
    def num_folding(self) -> int:
        return self._num_folding

    @property
    def write_count(self) -> int:
        return self._write_count

    def current_timer_msec(self, base_msec: int) -> int:
        if base_msec <= 0:
            raise ValueError('base_msec must be > 0')
        return int(base_msec) * (1 << self._num_folding)

    def read_xy(self, x_kind: str, x_idx: int, y_kind: str, y_idx: int) -> tuple[np.ndarray, np.ndarray]:
        n = self._write_count
        if n == 0:
            empty = np.empty(0, dtype=np.float64)
            return (empty, empty.copy())
        x_full = self._col(x_kind, x_idx, n)
        y_full = self._col(y_kind, y_idx, n)
        valid = np.isfinite(x_full) & np.isfinite(y_full)
        if not valid.any():
            empty = np.empty(0, dtype=np.float64)
            return (empty, empty.copy())
        return (x_full[valid].astype(np.float64, copy=False), y_full[valid].astype(np.float64, copy=False))

    def _fold(self) -> None:
        self._num_folding += 1
        self._times = self._times[::2]
        self._ai_raw = self._ai_raw[::2]
        self._ai_phy = self._ai_phy[::2]
        self._ao_raw = self._ao_raw[::2]
        self._params = self._params[::2]
        self._write_count = int(self._times.shape[0])

    def _col(self, kind: str, idx: int, n: int) -> np.ndarray:
        if kind == 'time':
            result: np.ndarray = self._times[:n]
        elif kind == 'elapsed':
            if self._save_start_epoch is None:
                result = np.arange(n, dtype=np.float64) * 0.117
            else:
                result = self._times[:n] - self._save_start_epoch
        elif kind == 'raw':
            i = max(0, min(idx, self._n_ai - 1))
            result = self._ai_raw[:n, i].astype(np.float64, copy=False)
        elif kind == 'phy':
            i = max(0, min(idx, self._n_ai - 1))
            result = self._ai_phy[:n, i].astype(np.float64, copy=False)
        elif kind == 'ao':
            i = max(0, min(idx, self._n_ao - 1))
            result = self._ao_raw[:n, i].astype(np.float64, copy=False)
        elif kind == 'par':
            i = max(0, min(idx, self._n_param - 1))
            result = self._params[:n, i].astype(np.float64, copy=False)
        else:
            result = np.full(n, np.nan, dtype=np.float64)
        return result
_default_buffer: OrigamiBuffer | None = None

def set_default_buffer(buf: OrigamiBuffer) -> None:
    global _default_buffer
    _default_buffer = buf

def get_default_buffer() -> OrigamiBuffer | None:
    return _default_buffer
