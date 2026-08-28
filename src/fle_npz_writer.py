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
from utl_context import DSS_CHDEF_PARAM_MAX, DSS_MB_AI_REGISTER_COUNT, DSS_MB_AO_REGISTER_COUNT, DSS_VOLTAGE_OUT_LABELS, ControlMode, dss_active_parameter_labels, dss_active_physical_labels, dss_active_raw_labels, get_context

class NpzWriter:

    def __init__(self, path: str | os.PathLike[str]) -> None:
        self._path = Path(path)
        self._time: list[float] = []
        self._raw: list[np.ndarray] = []
        self._phy: list[np.ndarray] = []
        self._par: list[np.ndarray] = []
        self._out: list[np.ndarray] = []
        self._closed = True
        self._t0: datetime | None = None
        self._mode: ControlMode = ControlMode.MOTOR

    @property
    def is_open(self) -> bool:
        return not self._closed

    def open(self, start_dt: datetime | None=None) -> None:
        if self.is_open:
            return
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._time = []
        self._raw = []
        self._phy = []
        self._par = []
        self._out = []
        self._t0 = start_dt
        self._mode = get_context().Control.mode
        self._closed = False

    def close(self) -> None:
        if not self.is_open:
            return
        n = len(self._time)
        if n > 0:
            time_arr = np.asarray(self._time, dtype=np.float64)
            raw_arr = np.stack(self._raw, axis=0)
            phy_arr = np.stack(self._phy, axis=0)
            par_arr = np.stack(self._par, axis=0)
            out_arr = np.stack(self._out, axis=0)
        else:
            time_arr = np.empty(0, dtype=np.float64)
            raw_arr = np.empty((0, DSS_MB_AI_REGISTER_COUNT), dtype=np.float64)
            phy_arr = np.empty((0, DSS_MB_AI_REGISTER_COUNT), dtype=np.float64)
            par_arr = np.empty((0, DSS_CHDEF_PARAM_MAX), dtype=np.float64)
            out_arr = np.empty((0, DSS_MB_AO_REGISTER_COUNT), dtype=np.float64)
        raw_labels = np.array(dss_active_raw_labels(self._mode), dtype='U64')
        phy_labels = np.array(dss_active_physical_labels(self._mode), dtype='U64')
        par_labels = np.array(dss_active_parameter_labels(self._mode), dtype='U64')
        out_labels = np.array(DSS_VOLTAGE_OUT_LABELS, dtype='U64')
        with self._path.open('wb') as fp:
            np.savez_compressed(fp, time=time_arr, raw=raw_arr, phy=phy_arr, par=par_arr, out=out_arr, raw_labels=raw_labels, phy_labels=phy_labels, par_labels=par_labels, out_labels=out_labels)
        self._closed = True

    def append_row(self, timestamp: datetime, ai_raw: np.ndarray, ai_phy: np.ndarray, ao_raw: np.ndarray, par: np.ndarray) -> None:
        if not self.is_open:
            raise RuntimeError('NpzWriter is not open')
        if self._t0 is None:
            self._t0 = timestamp
        elapsed = (timestamp - self._t0).total_seconds()
        self._time.append(float(elapsed))
        self._raw.append(np.asarray(ai_raw, dtype=np.float64))
        self._phy.append(np.asarray(ai_phy, dtype=np.float64))
        self._par.append(np.asarray(par, dtype=np.float64))
        self._out.append(np.asarray(ao_raw, dtype=np.float64))
__all__ = ['NpzWriter']
