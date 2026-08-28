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
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, ClassVar
import numpy as np
DSS_MB_BAUDRATE: int = 38400
DSS_MB_PARITY: str = 'N'
DSS_MB_STOPBITS: int = 1
DSS_MB_BYTESIZE: int = 8
DSS_MB_SLAVE_ID: int = 1
DSS_MB_AI_START_ADDRESS: int = 0
DSS_MB_AI_REGISTER_COUNT: int = 16
DSS_MB_F32_INPUT_START: int = 5000
DSS_MB_F32_INPUT_PROBE_REGS: int = 2
DSS_MB_F32_INPUT_REGS_TOTAL: int = 32
DSS_MB_AO_START_ADDRESS: int = 0
DSS_MB_AO_REGISTER_COUNT: int = 8
DSS_MB_AO_MIN_MV: int = 0
DSS_MB_AO_MAX_MV: int = 10000
DSS_TIM_POLL_MS: int = 100
DSS_TIM_CONTROL_MS: int = 200
DSS_TIM_PREVIEW_DATASAVING_INI_MSEC: int = 100
DSS_TIM_PREVIEW_NOT_SAVING_MSEC: int = 100
DSS_CHDEF_AI_MAX: int = 16
DSS_CHDEF_AO_MAX: int = 8
DSS_CHDEF_PARAM_MAX: int = 32
DSS_CHDEF_STEPCTRL_STEP_MAX: int = 1024
DSS_CHDEF_STEPCTRL_ARGS_MAX: int = 16
DSS_PREVIEW_NOT_SAVING_POINTS: int = 600
DSS_PREVIEW_CHART_MAX_POINTS: int = 65536
DSS_PREVIEW_HTTP_MAX_POINTS: int = 8192
DSS_APP_NAME: str = 'DigitShowSide'
DSS_APP_VERSION: str = '1.0.0'
DSS_APP_GITHUB_URL: str = 'https://github.com/mkt-kuno/DigitShowSide'
DSS_LOG_DIRNAME: str = 'Log'
DSS_LOG_FILENAME: str = 'log.txt'
DSS_LOG_ROTATE_MAX_SIZE: int = 2 * 1024 * 1024
DSS_LOG_ROTATE_MAX_FILES: int = 128
DSS_LOG_LATEST_LINES: int = 5
DSS_LOG_FLUSH_INTERVAL_MS: int = 1000
DSS_AI_CH_VLC: int = 0
DSS_AI_CH_V_DISP: int = 1
DSS_AI_CH_LDT1: int = 2
DSS_AI_CH_LDT2: int = 3
DSS_AI_CH_TORQUE_LC: int = 4
DSS_AI_CH_CG1: int = 5
DSS_AI_CH_CG2: int = 6
DSS_AI_CH_CG3: int = 7
DSS_AI_CH_HCDPT: int = 8
DSS_AI_CH_LCDPT: int = 9
DSS_AI_CH_TOR_DISP: int = 10
DSS_AI_CAL_A_TOR_DISP_RAD_PER_V2: float = 0.0
DSS_AI_CAL_B_TOR_DISP_RAD_PER_V: float = 1.5707963
DSS_AI_CAL_C_TOR_DISP_RAD: float = -1.5707963
DSS_AI_CAL_A_TORQUE_LC_NCM_PER_V2: float = 0.0
DSS_AI_CAL_B_TORQUE_LC_NCM_PER_V: float = 1.0
DSS_AI_CAL_C_TORQUE_LC_NCM: float = 0.0
DSS_TORS_SZQ_GEOM_FACTOR: float = 1000000.0
DSS_TORS_TORQUE_M_DIVISOR: float = 1000000.0
DSS_TORS_NCM_TO_NM: float = 100.0
DSS_TORS_EP_GAIN_ESP: float = 0.9
DSS_TORS_EP_GAIN_CNS_CYCLIC: float = 0.3
DSS_TORS_EP_GAIN_CREEP: float = 0.3
DSS_TORS_EP_GAIN_CONST_P: float = 0.1
DSS_TORS_PRECON_EP_GAIN: float = 0.2
DSS_TORS_EXIT_BAND_FACTOR: float = 2.0
DSS_AO_CH_MOTOR_ONOFF: int = 0
DSS_AO_CH_MOTOR_UPDOWN: int = 1
DSS_AO_CH_MOTOR_SPEED: int = 2
DSS_AO_CH_EP_CELL: int = 3
DSS_AO_CH_EP_AXIS: int = 4
DSS_AO_CH_TORSIONAL_ONOFF: int = 5
DSS_AO_CH_TORSIONAL_CWCCW: int = 6
DSS_AO_CH_TORSIONAL_SPEED: int = 7
DSS_AO_DEF_VLT_MOTOR_ON: float = 5.0
DSS_AO_DEF_VLT_MOTOR_OFF: float = 0.0
DSS_AO_DEF_VLT_MOTOR_UP: float = 5.0
DSS_AO_DEF_VLT_MOTOR_DOWN: float = 0.0
DSS_AO_DEF_VLT_TORSIONAL_ON: float = 5.0
DSS_AO_DEF_VLT_TORSIONAL_OFF: float = 0.0
DSS_AO_DEF_VLT_TORSIONAL_CW: float = 5.0
DSS_AO_DEF_VLT_TORSIONAL_CCW: float = 0.0
DSS_AI_RAW_DTYPE = np.float32
DSS_AO_RAW_DTYPE = np.uint16
DSS_PARAM_DTYPE = np.float32
DSS_FONT_PT_XL: int = 22
DSS_FONT_PT_LG: int = 18
DSS_FONT_PT_MD: int = 12
DSS_FONT_PT_SM: int = 8
DSS_STAGE_PRESENT: int = 0
DSS_STAGE_INITIAL: int = 1
DSS_STAGE_BEFORE: int = 2
DSS_STAGE_AFTER: int = 3
DSS_SPECIMEN_DEFAULT_DIAMETER_MM: float = 50.0
DSS_SPECIMEN_DEFAULT_HEIGHT_MM: float = 100.0
DSS_SPECIMEN_DEFAULT_MEMBRANE_MODULUS: float = 0.0
DSS_SPECIMEN_DEFAULT_MEMBRANE_THICKNESS: float = 0.3
DSS_SPECIMEN_DEFAULT_CAP_WEIGHT: float = 0.0
DSS_SPECIMEN_TORSIONAL_DEFAULT_INNER_DIAMETER_MM: float = 60.0
DSS_SPECIMEN_TORSIONAL_DEFAULT_OUTER_DIAMETER_MM: float = 100.0
DSS_SPECIMEN_TORSIONAL_DEFAULT_HEIGHT_MM: float = 150.0
DSS_SPECIMEN_TORSIONAL_DEFAULT_REF_INNER_DIAMETER_MM: float = 59.85
DSS_SPECIMEN_TORSIONAL_DEFAULT_REF_OUTER_DIAMETER_MM: float = 100.15
DSS_SPECIMEN_TORSIONAL_DEFAULT_REF_HEIGHT_MM: float = 150.0
DSS_SPECIMEN_TORSIONAL_DEFAULT_MEMBRANE_MODULUS_KPA: float = 1400.0
DSS_SPECIMEN_TORSIONAL_DEFAULT_MEMBRANE_THICKNESS_MM: float = 0.3
DSS_SPECIMEN_TORSIONAL_DEFAULT_ROD_AREA_MM2: float = 0.0
DSS_SPECIMEN_TORSIONAL_DEFAULT_CAP_WEIGHT_N: float = 0.0
DSS_PRECON_DEFAULT_TARGET_KPA: float = 0.0
DSS_PRECON_DEFAULT_ERROR_KPA: float = 10.0
DSS_PRECON_DEFAULT_MOTOR_SPEED_RPM: float = 1000.0
DSS_PRECON_TOR_DEFAULT_AXIS_SPEED_MAX_RPM: float = 100.0
DSS_PRECON_TOR_DEFAULT_Q_AT_MAX_SPEED_KPA: float = 1.0
DSS_PRECON_TOR_DEFAULT_CELL_TARGET_KPA: float = 0.0
DSS_PRECON_TOR_DEFAULT_CELL_RATE_KPA_PER_MIN: float = 0.0
DSS_ERROR_STRESS_AIR_KPA: float = 0.5
DSS_ERROR_STRESS_COM_KPA: float = 0.5
DSS_ERROR_STRESS_EXT_KPA: float = -0.5
DSS_ERROR_STRESS_EA_PCT: float = 0.05
DSS_ERROR_STRESS_TORQUE_NM: float = 0.5
DSS_ERROR_STRESS_ANGLE_RAD: float = 0.05
DSS_AO_CAL_A_EP_CELL_V_PER_KPA: float = 0.01275
DSS_AO_CAL_A_EP_AXIS_V_PER_N: float = 0.00511
DSS_AO_CAL_A_MOTOR_SPEED_V_PER_RPM: float = 0.003333333
DSS_AO_CAL_A_DEFAULT: float = 0.0
DSS_AO_CAL_B_DEFAULT: float = 0.0
DSS_AO_CAL_A_TORSIONAL_SPEED_V_PER_RPM: float = 0.003333333
DSS_AO_CAL_B_TORSIONAL_SPEED_V: float = 0.0
DSS_CCH_INT16_ERR_THRESHOLD: int = int(np.floor(np.iinfo(np.int16).max * 0.95))
DSS_CCH_INT16_WARN_THRESHOLD: int = int(np.floor(np.iinfo(np.int16).max * 0.8))
DSS_CCH_INT16_WARN_RGB: tuple[int, int, int] = (255, 255, 0)
DSS_CCH_INT16_ERR_RGB: tuple[int, int, int] = (255, 0, 0)
DSS_CCH_FLOAT_WARN_RGB: tuple[int, int, int] = (255, 255, 0)
DSS_CCH_FLOAT_ERR_RGB: tuple[int, int, int] = (255, 0, 0)

def dss_cch_int16_rgb_for(raw: float | np.floating) -> tuple[int, int, int] | None:
    abs_raw = abs(float(raw))
    if abs_raw > DSS_CCH_INT16_ERR_THRESHOLD:
        return DSS_CCH_INT16_ERR_RGB
    if abs_raw > DSS_CCH_INT16_WARN_THRESHOLD:
        return DSS_CCH_INT16_WARN_RGB
    return None

def dss_cch_float_rgb_for(val: float | np.floating) -> tuple[int, int, int] | None:
    f = float(val)
    if np.isnan(f):
        return DSS_CCH_FLOAT_ERR_RGB
    if np.isinf(f):
        return DSS_CCH_FLOAT_WARN_RGB
    return None
_TSV_FORBIDDEN_CHARS = ('\t', '\n', '\r', '"')

def _validate_labels(name: str, labels: list[str], expected_len: int) -> list[str]:
    if len(labels) != expected_len:
        raise ValueError(f'{name}: expected {expected_len} labels, got {len(labels)}')
    for i, v in enumerate(labels):
        if not isinstance(v, str):
            raise TypeError(f'{name}[{i}]: not a string ({type(v).__name__})')
        if not v:
            raise ValueError(f'{name}[{i}]: empty string')
        for ch in _TSV_FORBIDDEN_CHARS:
            if ch in v:
                raise ValueError(f'{name}[{i}]={v!r}: contains forbidden char {ch!r}')
    return labels
DSS_RAW_LABELS: list[str] = _validate_labels('DSS_RAW_LABELS', ['LoadCell', 'LVDT', 'LDT1', 'LDT2', 'none', 'none', 'none', 'none', 'HCDPT', 'LCDPT', 'none', 'none', 'none', 'none', 'none', 'none'], DSS_MB_AI_REGISTER_COUNT)
DSS_PHYSICAL_LABELS: list[str] = _validate_labels('DSS_PHYSICAL_LABELS', ['Load(N)', 'ExtDisp(mm)', 'LDT1(mm)', 'LDT2(mm)', 'none', 'none', 'none', 'none', 'EffCellP(kPa)', 'VolChange(mm3)', 'none', 'none', 'none', 'none', 'none', 'none'], DSS_MB_AI_REGISTER_COUNT)
DSS_VOLTAGE_OUT_LABELS: list[str] = _validate_labels('DSS_VOLTAGE_OUT_LABELS', ['Motor ON/OFF', 'Motor UP/DOWN', 'Motor Speed', 'EP Cell Pressure', 'EP Axis Pressure', 'Torsional ON/OFF', 'Torsional CW/CCW', 'Torsional Speed'], DSS_MB_AO_REGISTER_COUNT)
DSS_PARAMETER_LABELS: list[str] = _validate_labels('DSS_PARAMETER_LABELS', ['q(kPa)', "p'(kPa)", "sigma'(a)(kPa)", "sigma'(r)(kPa)", 'AxialStrain(%)', 'RadialStrain(%)', 'VolumetricStrain(%)', 'LDT1(mm)', 'LDT2(mm)', 'LocalAxialStrain(%)', 'LDT1LocAxStrain(%)', 'LDT2LocAxStrain(%)', 'none', 'none', 'none', 'none', 'CurrentDiameter(mm)', 'CurrentHeight(mm)', 'CurrentArea(mm2)', 'CurrentVolume(mm3)', 'RefDiameter(mm)', 'RefHeight(mm)', 'RefArea(mm2)', 'RefVolume(mm3)', 'ControlType', 'StepCtrl_StepNo', 'StepCtrl_CtrlNo', 'StepCtrl_StepTime', 'StepCtrl_CycleNo', 'none', 'none', 'none'], DSS_CHDEF_PARAM_MAX)
DSS_RAW_LABELS_TORSIONAL: list[str] = _validate_labels('DSS_RAW_LABELS_TORSIONAL', ['LoadCell', 'LVDT', 'LDT1', 'LDT2', 'Torque Loadcell', 'ClipGauge1', 'ClipGauge2', 'ClipGauge3', 'HCDPT', 'LCDPT', 'Tor Disp', 'none', 'none', 'none', 'none', 'none'], DSS_MB_AI_REGISTER_COUNT)
DSS_PHYSICAL_LABELS_TORSIONAL: list[str] = _validate_labels('DSS_PHYSICAL_LABELS_TORSIONAL', ['Load(N)', 'ExtDisp(mm)', 'LDT1(mm)', 'LDT2(mm)', 'Torque[Ncm]', 'ClipGauge1(mm)', 'ClipGauge2(mm)', 'ClipGauge3(mm)', 'EffCellP(kPa)', 'VolChange(mm3)', 'Tor Disp(rad)', 'none', 'none', 'none', 'none', 'none'], DSS_MB_AI_REGISTER_COUNT)
DSS_PARAMETER_LABELS_TORSIONAL: list[str] = _validate_labels('DSS_PARAMETER_LABELS_TORSIONAL', ["sigma'(z)(kPa)", "sigma'(r)(kPa)", "sigma'(q)(kPa)", 'tau(zq)(kPa)', 'VolumetricStrain(%)', 'AxialStrain(%)', 'LDT1(mm)', 'LDT2(mm)', 'ClipGauge1(mm)', 'ClipGauge2(mm)', 'ClipGauge3(mm)', "p'(kPa)", 'q(kPa)', "sigma'(1)(kPa)", "sigma'(2)(kPa)", "sigma'(3)(kPa)", 'gamma1(zq)(%)', 'gamma2(zq)(%)', 'InnerCellP(kPa)', 'OuterCellP(kPa)', 'InnerDiameter(mm)', 'OuterDiameter(mm)', 'Height(mm)', 'Volume(mm3)', 'ControlType', 'StepCtrl_StepNo', 'StepCtrl_CtrlNo', 'StepCtrl_StepTime', 'StepCtrl_CycleNo', 'none', 'none', 'none'], DSS_CHDEF_PARAM_MAX)

class CyclicState(IntEnum):
    INIT = 0
    FIRST_QUARTER = 1
    SECOND_QUARTER = 2
    THIRD_QUARTER = 3
    FOURTH_QUARTER = 4

class ControlMode(IntEnum):
    MOTOR = 0
    TORSIONAL = 1

def dss_active_raw_labels(mode: ControlMode) -> list[str]:
    if mode == ControlMode.TORSIONAL:
        return DSS_RAW_LABELS_TORSIONAL
    return DSS_RAW_LABELS

def dss_active_physical_labels(mode: ControlMode) -> list[str]:
    if mode == ControlMode.TORSIONAL:
        return DSS_PHYSICAL_LABELS_TORSIONAL
    return DSS_PHYSICAL_LABELS

def dss_active_parameter_labels(mode: ControlMode) -> list[str]:
    if mode == ControlMode.TORSIONAL:
        return DSS_PARAMETER_LABELS_TORSIONAL
    return DSS_PARAMETER_LABELS

class ControlType(IntEnum):
    NONE = 0
    PRECONSOLIDATION = 1
    STEP = 15
    EXTERNAL = 30

class StructRow:
    __slots__ = ('_a',)

    @classmethod
    def _from_view(cls, view: np.ndarray) -> StructRow:
        inst = cls.__new__(cls)
        object.__setattr__(inst, '_a', view)
        return inst

    def _names(self) -> tuple[str, ...] | None:
        names = self._a.dtype.names
        return tuple(names) if names is not None else None

    def get(self, name: str) -> Any:
        names = self._names()
        if names is None or name not in names:
            raise AttributeError(f'{type(self).__name__}.{name}')
        return _unwrap(self._a[name])

    def set(self, name: str, value: Any) -> None:
        names = self._names()
        if names is None or name not in names:
            raise AttributeError(f'{type(self).__name__}.{name}')
        self._a[name] = value

    def as_dict(self) -> dict[str, Any]:
        names = self._names() or ()
        return {n: _unwrap(self._a[n]) for n in names}

    def __getattr__(self, name: str) -> Any:
        if name.startswith('_'):
            raise AttributeError(name)
        names = self._names()
        if names is not None and name in names:
            return _unwrap(self._a[name])
        raise AttributeError(f'{type(self).__name__}.{name}')

    def __setattr__(self, name: str, value: Any) -> None:
        if name == '_a':
            object.__setattr__(self, name, value)
            return
        names = self._names()
        if names is not None and name in names:
            self._a[name] = value
            return
        object.__setattr__(self, name, value)

    def __repr__(self) -> str:
        names = self._names() or ()
        items = ', '.join((f'{n}={self.get(n)!r}' for n in names))
        return f'{type(self).__name__}({items})'

def _unwrap(v: np.ndarray) -> Any:
    if v.ndim > 0:
        return v
    if v.dtype.names is not None and len(v.dtype.names) > 0:
        return StructRow._from_view(v)
    return v.item()

class Struct:
    _DTYPE: ClassVar[np.dtype | None] = None
    __slots__ = ('_a',)

    def __init__(self, **kw: Any) -> None:
        if self._DTYPE is None:
            raise NotImplementedError(f'{type(self).__name__}._DTYPE is not set')
        object.__setattr__(self, '_a', np.zeros((), dtype=self._DTYPE))
        names = self._a.dtype.names
        for k, v in kw.items():
            if k in names:
                self._a[k] = v

    @classmethod
    def _from_view(cls, view: np.ndarray) -> Struct:
        inst = cls.__new__(cls)
        object.__setattr__(inst, '_a', view)
        return inst

    def _names(self) -> tuple[str, ...] | None:
        names = self._a.dtype.names
        return tuple(names) if names is not None else None

    def get(self, name: str) -> Any:
        names = self._names()
        if names is None or name not in names:
            raise AttributeError(f'{type(self).__name__}.{name}')
        return _unwrap(self._a[name])

    def set(self, name: str, value: Any) -> None:
        names = self._names()
        if names is None or name not in names:
            raise AttributeError(f'{type(self).__name__}.{name}')
        self._a[name] = value

    def as_dict(self) -> dict[str, Any]:
        names = self._names() or ()
        return {n: _unwrap(self._a[n]) for n in names}

    def __getattr__(self, name: str) -> Any:
        if name.startswith('_'):
            raise AttributeError(name)
        names = self._names()
        if names is not None and name in names:
            return _unwrap(self._a[name])
        raise AttributeError(f'{type(self).__name__}.{name}')

    def __setattr__(self, name: str, value: Any) -> None:
        if name == '_a':
            object.__setattr__(self, name, value)
            return
        names = self._names()
        if names is not None and name in names:
            self._a[name] = value
            return
        object.__setattr__(self, name, value)

    def __repr__(self) -> str:
        names = self._names() or ()
        items = ', '.join((f'{n}={self.get(n)!r}' for n in names))
        return f'{type(self).__name__}({items})'

class StructArray:
    _DTYPE: ClassVar[np.dtype | None] = None
    _ROW: ClassVar[type | None] = None
    __slots__ = ('_a',)

    def __init_subclass__(cls, **kwargs: Any) -> None:
        super().__init_subclass__(**kwargs)
        if cls._ROW is None and cls._DTYPE is not None:
            cls._ROW = type(cls.__name__ + 'Row', (StructRow,), {'__slots__': (), '_DTYPE': cls._DTYPE})

    def __init__(self, n: int, **kw: Any) -> None:
        dtype = type(self)._DTYPE
        if dtype is None:
            raise NotImplementedError(f'{type(self).__name__}._DTYPE is not set')
        object.__setattr__(self, '_a', np.zeros(n, dtype=dtype))
        dtype_names: tuple[str, ...] | None = dtype.names
        names = dtype_names if dtype_names is not None else ()
        for k, v in kw.items():
            if k in names:
                self._a[k] = v

    def __len__(self) -> int:
        return int(self._a.shape[0])

    def __getitem__(self, i: int) -> StructRow:
        row_cls = type(self)._ROW
        assert row_cls is not None
        result: StructRow = row_cls._from_view(self._a[i])
        return result

    def get_field(self, name: str) -> np.ndarray[Any, np.dtype[Any]]:
        dtype = type(self)._DTYPE
        names: tuple[str, ...] | None = dtype.names if dtype is not None else None
        if names is None or name not in names:
            raise AttributeError(f'{type(self).__name__}.{name}')
        result: np.ndarray[Any, np.dtype[Any]] = self._a[name]
        return result

    def set_field(self, name: str, value: Any) -> None:
        dtype = type(self)._DTYPE
        names: tuple[str, ...] | None = dtype.names if dtype is not None else None
        if names is None or name not in names:
            raise AttributeError(f'{type(self).__name__}.{name}')
        self._a[name] = value

    def row(self, i: int) -> StructRow:
        row_cls = type(self)._ROW
        assert row_cls is not None
        result: StructRow = row_cls._from_view(self._a[i])
        return result

    def __getattr__(self, name: str) -> Any:
        if name.startswith('_'):
            raise AttributeError(name)
        dtype = type(self)._DTYPE
        dtype_names: tuple[str, ...] | None = dtype.names if dtype is not None else None
        if dtype_names is not None and name in dtype_names:
            return self._a[name]
        raise AttributeError(f'{type(self).__name__}.{name}')

    def __setattr__(self, name: str, value: Any) -> None:
        if name == '_a':
            super().__setattr__(name, value)
            return
        dtype = type(self)._DTYPE
        dtype_names = dtype.names if dtype is not None else None
        if dtype_names is not None and name in dtype_names:
            self._a[name] = value
            return
        super().__setattr__(name, value)

    def __repr__(self) -> str:
        return f'{type(self).__name__}(n={len(self)}, dtype={type(self)._DTYPE})'

@dataclass(slots=True)
class StopWatch:
    start_sec: float | None = None
    elapsed_sec: float = 0.0
    interval_sec: float = 0.0
    _last_sec: float | None = None
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def reset(self) -> None:
        with self._lock:
            self.start_sec = None
            self.elapsed_sec = 0.0
            self.interval_sec = 0.0
            self._last_sec = None

    def start(self) -> None:
        with self._lock:
            now = time.perf_counter()
            self.start_sec = now
            self._last_sec = now

    def stop(self) -> None:
        with self._lock:
            self.start_sec = None
            self._last_sec = None

    def update_elapsed_and_interval(self) -> None:
        with self._lock:
            now = time.perf_counter()
            if self.start_sec is None:
                return
            self.elapsed_sec = now - self.start_sec
            if self._last_sec is not None:
                self.interval_sec = now - self._last_sec
            self._last_sec = now

    def get_elapsed_sec(self) -> float:
        with self._lock:
            return self.elapsed_sec

class AIData(StructArray):
    _DTYPE = np.dtype([('raw', 'f4'), ('phy', 'f4'), ('Cal', [('a', 'f4'), ('b', 'f4'), ('c', 'f4')])])

    def __init__(self) -> None:
        super().__init__(DSS_CHDEF_AI_MAX)
        self._a['Cal']['a'] = 0.0
        self._a['Cal']['b'] = 1.0
        self._a['Cal']['c'] = 0.0

    def apply_calibration(self) -> None:
        x = self._a['raw']
        self._a['phy'] = self._a['Cal']['a'] * x * x + self._a['Cal']['b'] * x + self._a['Cal']['c']

class AOData(StructArray):
    _DTYPE = np.dtype([('raw', 'f4'), ('Cal', [('a', 'f4'), ('b', 'f4')])])

    def __init__(self) -> None:
        super().__init__(DSS_CHDEF_AO_MAX)
        self._a['Cal']['a'] = DSS_AO_CAL_A_DEFAULT
        self._a['Cal'][DSS_AO_CH_EP_CELL]['a'] = DSS_AO_CAL_A_EP_CELL_V_PER_KPA
        self._a['Cal'][DSS_AO_CH_EP_AXIS]['a'] = DSS_AO_CAL_A_EP_AXIS_V_PER_N
        self._a['Cal'][DSS_AO_CH_MOTOR_SPEED]['a'] = DSS_AO_CAL_A_MOTOR_SPEED_V_PER_RPM
        self._a['Cal'][DSS_AO_CH_TORSIONAL_SPEED]['a'] = DSS_AO_CAL_A_TORSIONAL_SPEED_V_PER_RPM
        self._a['Cal'][DSS_AO_CH_TORSIONAL_SPEED]['b'] = DSS_AO_CAL_B_TORSIONAL_SPEED_V

@dataclass(slots=True)
class AIO:
    AI: AIData = field(default_factory=AIData)
    AO: AOData = field(default_factory=AOData)
    param: np.ndarray = field(default_factory=lambda: np.zeros(DSS_CHDEF_PARAM_MAX, dtype=DSS_PARAM_DTYPE))

@dataclass(slots=True)
class Modbus:
    is_thread_end: bool = False
    ai_raw: np.ndarray = field(default_factory=lambda: np.zeros(DSS_CHDEF_AI_MAX, dtype=DSS_AI_RAW_DTYPE))
    ao_raw: np.ndarray = field(default_factory=lambda: np.zeros(DSS_CHDEF_AO_MAX, dtype=DSS_AO_RAW_DTYPE))
    port: str = ''
    baudrate: int = DSS_MB_BAUDRATE
    data_bits: int = DSS_MB_BYTESIZE
    parity: str = DSS_MB_PARITY
    stop_bits: int = DSS_MB_STOPBITS
    slave_id: int = DSS_MB_SLAVE_ID
    usb_cdc_direct: bool = False
    float_input_reg: bool = True
    device: object = None

@dataclass(slots=True)
class CurrentSpecimen:
    height: float = 0.0
    area: float = 0.0
    volume: float = 0.0
    diameter: float = 0.0

@dataclass(slots=True)
class Current:
    Specimen: CurrentSpecimen = field(default_factory=CurrentSpecimen)
    e_sa: float = 0.0
    e_sr: float = 0.0
    p: float = 0.0
    e_p: float = 0.0
    q: float = 0.0
    ea: float = 0.0
    er: float = 0.0
    ev: float = 0.0
    tau: float = 0.0
    gamma: float = 0.0
    torque: float = 0.0
    rotation: float = 0.0

@dataclass(slots=True)
class Flag:
    set_board: bool = False
    save_data: bool = False
    control: bool = False

class SpecimenStages(StructArray):
    _DTYPE = np.dtype([('diameter', 'f4'), ('area', 'f4'), ('height', 'f4'), ('volume', 'f4'), ('ldt_1', 'f4'), ('ldt_2', 'f4')])

    def __init__(self) -> None:
        super().__init__(4)
        self._a['ldt_1'] = np.nan
        self._a['ldt_2'] = np.nan

@dataclass(slots=True)
class SpecimenData:
    default_diameter: float = DSS_SPECIMEN_DEFAULT_DIAMETER_MM
    default_height: float = DSS_SPECIMEN_DEFAULT_HEIGHT_MM
    Stage: SpecimenStages = field(default_factory=SpecimenStages)
    membrane_modulus: float = DSS_SPECIMEN_DEFAULT_MEMBRANE_MODULUS
    membrane_thickness: float = DSS_SPECIMEN_DEFAULT_MEMBRANE_THICKNESS
    cap_weight: float = DSS_SPECIMEN_DEFAULT_CAP_WEIGHT

    def __post_init__(self) -> None:
        d = float(self.default_diameter)
        h = float(self.default_height)
        radius = d / 2.0
        area = np.float32(np.pi * radius * radius)
        volume = np.float32(area * h)
        for i in range(len(self.Stage)):
            self.Stage._a[i]['diameter'] = np.float32(d)
            self.Stage._a[i]['height'] = np.float32(h)
            self.Stage._a[i]['area'] = area
            self.Stage._a[i]['volume'] = volume

class SpecimenStagesTorsional(StructArray):
    _DTYPE = np.dtype([('diameter_in', 'f4'), ('diameter_out', 'f4'), ('height', 'f4'), ('volume', 'f4'), ('dia_in_membrane', 'f4'), ('dia_out_membrane', 'f4'), ('height_in_membrane', 'f4'), ('height_out_membrane', 'f4')])

    def __init__(self) -> None:
        super().__init__(4)

@dataclass(slots=True)
class SpecimenDataTorsional:
    Stage: SpecimenStagesTorsional = field(default_factory=SpecimenStagesTorsional)
    membrane_modulus: float = DSS_SPECIMEN_TORSIONAL_DEFAULT_MEMBRANE_MODULUS_KPA
    membrane_thickness: float = DSS_SPECIMEN_TORSIONAL_DEFAULT_MEMBRANE_THICKNESS_MM
    rod_area: float = DSS_SPECIMEN_TORSIONAL_DEFAULT_ROD_AREA_MM2
    cap_weight: float = DSS_SPECIMEN_TORSIONAL_DEFAULT_CAP_WEIGHT_N
    r_dia_in_m: float = DSS_SPECIMEN_TORSIONAL_DEFAULT_REF_INNER_DIAMETER_MM
    r_dia_out_m: float = DSS_SPECIMEN_TORSIONAL_DEFAULT_REF_OUTER_DIAMETER_MM
    r_height_in_m: float = DSS_SPECIMEN_TORSIONAL_DEFAULT_REF_HEIGHT_MM
    r_height_out_m: float = DSS_SPECIMEN_TORSIONAL_DEFAULT_REF_HEIGHT_MM

    def __post_init__(self) -> None:
        d_in = DSS_SPECIMEN_TORSIONAL_DEFAULT_INNER_DIAMETER_MM
        d_out = DSS_SPECIMEN_TORSIONAL_DEFAULT_OUTER_DIAMETER_MM
        h = DSS_SPECIMEN_TORSIONAL_DEFAULT_HEIGHT_MM
        volume = float(np.pi) / 4.0 * (d_out ** 2 - d_in ** 2) * h
        self.Stage._a['diameter_in'] = np.float32(d_in)
        self.Stage._a['diameter_out'] = np.float32(d_out)
        self.Stage._a['height'] = np.float32(h)
        self.Stage._a['volume'] = np.float32(volume)
        self.Stage._a['dia_in_membrane'] = np.float32(d_in)
        self.Stage._a['dia_out_membrane'] = np.float32(d_out)
        self.Stage._a['height_in_membrane'] = np.float32(h)
        self.Stage._a['height_out_membrane'] = np.float32(h)

    def recalc_volumes(self) -> None:
        d_out = self.Stage.get_field('diameter_out').astype(np.float64)
        d_in = self.Stage.get_field('diameter_in').astype(np.float64)
        h = self.Stage.get_field('height').astype(np.float64)
        volume = float(np.pi) * (d_out ** 2 - d_in ** 2) / 4.0 * h
        self.Stage.set_field('volume', volume.astype(np.float32))

class MotorVoltage(Struct):
    _DTYPE = np.dtype([('on', 'f4'), ('off', 'f4'), ('up', 'f4'), ('down', 'f4')])

    def __init__(self) -> None:
        super().__init__(on=DSS_AO_DEF_VLT_MOTOR_ON, off=DSS_AO_DEF_VLT_MOTOR_OFF, up=DSS_AO_DEF_VLT_MOTOR_UP, down=DSS_AO_DEF_VLT_MOTOR_DOWN)

class TorsionalVoltage(Struct):
    _DTYPE = np.dtype([('on', 'f4'), ('off', 'f4'), ('cw', 'f4'), ('ccw', 'f4')])

    def __init__(self) -> None:
        super().__init__(on=DSS_AO_DEF_VLT_TORSIONAL_ON, off=DSS_AO_DEF_VLT_TORSIONAL_OFF, cw=DSS_AO_DEF_VLT_TORSIONAL_CW, ccw=DSS_AO_DEF_VLT_TORSIONAL_CCW)

class PreConsolidation(Struct):
    _DTYPE = np.dtype([('target', 'f4'), ('error', 'f4'), ('motor_speed', 'f4')])

    def __init__(self) -> None:
        super().__init__()
        self.target = DSS_PRECON_DEFAULT_TARGET_KPA
        self.error = DSS_PRECON_DEFAULT_ERROR_KPA
        self.motor_speed = DSS_PRECON_DEFAULT_MOTOR_SPEED_RPM

class PreConsolidationTorsional(Struct):
    _DTYPE = np.dtype([('axis_speed_max_rpm', 'f4'), ('q_at_max_speed_kpa', 'f4'), ('cell_target_kpa', 'f4'), ('cell_rate_kpa_per_min', 'f4')])

    def __init__(self) -> None:
        super().__init__()
        self.axis_speed_max_rpm = DSS_PRECON_TOR_DEFAULT_AXIS_SPEED_MAX_RPM
        self.q_at_max_speed_kpa = DSS_PRECON_TOR_DEFAULT_Q_AT_MAX_SPEED_KPA
        self.cell_target_kpa = DSS_PRECON_TOR_DEFAULT_CELL_TARGET_KPA
        self.cell_rate_kpa_per_min = DSS_PRECON_TOR_DEFAULT_CELL_RATE_KPA_PER_MIN

class ErrorStress(Struct):
    _DTYPE = np.dtype([('com', 'f4'), ('ext', 'f4'), ('air', 'f4'), ('ea', 'f4'), ('torque', 'f4'), ('angle', 'f4')])

    def __init__(self) -> None:
        super().__init__()
        self.air = DSS_ERROR_STRESS_AIR_KPA
        self.com = DSS_ERROR_STRESS_COM_KPA
        self.ext = DSS_ERROR_STRESS_EXT_KPA
        self.ea = DSS_ERROR_STRESS_EA_PCT
        self.torque = DSS_ERROR_STRESS_TORQUE_NM
        self.angle = DSS_ERROR_STRESS_ANGLE_RAD

class ControlSteps(StructArray):
    _DTYPE = np.dtype([('ctrl', 'i4'), ('args', 'f4', DSS_CHDEF_STEPCTRL_ARGS_MAX)])

    def __init__(self) -> None:
        super().__init__(DSS_CHDEF_STEPCTRL_STEP_MAX)

@dataclass(slots=True)
class Cyclic:
    num: int = 0
    state: CyclicState = CyclicState.INIT

@dataclass(slots=True)
class Control:
    mode: ControlMode = ControlMode.MOTOR
    type: ControlType = ControlType.NONE
    watch: StopWatch = field(default_factory=StopWatch)
    MotorVoltage: MotorVoltage = field(default_factory=MotorVoltage)
    TorsionalVoltage: TorsionalVoltage = field(default_factory=TorsionalVoltage)
    PreConsolidation: PreConsolidation = field(default_factory=PreConsolidation)
    PreConsolidationTorsional: PreConsolidationTorsional = field(default_factory=PreConsolidationTorsional)
    ErrorStress: ErrorStress = field(default_factory=ErrorStress)
    Cyclic: Cyclic = field(default_factory=Cyclic)
    current_step: int = 0
    Step: ControlSteps = field(default_factory=ControlSteps)

class CDSBPyContext:
    _instance: ClassVar[CDSBPyContext | None] = None
    _init_lock: ClassVar[threading.Lock] = threading.Lock()
    Modbus: Modbus
    AIO: AIO
    Current: Current
    Flag: Flag
    SpecimenData: SpecimenData
    SpecimenTorsional: SpecimenDataTorsional
    PreConsolidationTorsional: PreConsolidationTorsional
    Control: Control
    __slots__ = ('AIO', 'Control', 'Current', 'Flag', 'Modbus', 'PreConsolidationTorsional', 'SpecimenData', 'SpecimenTorsional')

    def __new__(cls) -> CDSBPyContext:
        if cls._instance is None:
            with cls._init_lock:
                if cls._instance is None:
                    inst = super().__new__(cls)
                    inst.Modbus = Modbus()
                    inst.AIO = AIO()
                    inst.Current = Current()
                    inst.Flag = Flag()
                    inst.SpecimenData = SpecimenData()
                    inst.SpecimenTorsional = SpecimenDataTorsional()
                    inst.PreConsolidationTorsional = PreConsolidationTorsional()
                    inst.Control = Control()
                    cls._instance = inst
        return cls._instance

    @classmethod
    def reset(cls) -> None:
        with cls._init_lock:
            cls._instance = None

def get_context() -> CDSBPyContext:
    return CDSBPyContext()
