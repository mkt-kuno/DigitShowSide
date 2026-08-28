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
import threading
from pathlib import Path
from typing import Any
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import contextlib
from utl_context import DSS_CHDEF_AI_MAX, DSS_CHDEF_AO_MAX, CDSBPyContext, get_context
CONFIG_FILENAME: str = 'config.json'
CONFIG_LOCK: threading.Lock = threading.Lock()

def _config_path(appdata_dir: Path | None=None) -> Path:
    if appdata_dir is None:
        base = os.environ.get('APPDATA') or str(Path.home() / 'AppData' / 'Roaming')
        appdata_dir = Path(base) / 'DigitShowSide'
    appdata_dir.mkdir(parents=True, exist_ok=True)
    return appdata_dir / CONFIG_FILENAME

def _restore_ai_calibration(ctx: CDSBPyContext, ai: object) -> None:
    if not isinstance(ai, dict):
        return
    for ch in range(DSS_CHDEF_AI_MAX):
        v = ai.get(f'{ch:02d}')
        if not isinstance(v, dict):
            continue
        try:
            ctx.AIO.AI[ch].Cal.a = float(v.get('a', 0.0))
            ctx.AIO.AI[ch].Cal.b = float(v.get('b', 1.0))
            ctx.AIO.AI[ch].Cal.c = float(v.get('c', 0.0))
        except (TypeError, ValueError) as _exc:
            continue

def _restore_ao_calibration(ctx: CDSBPyContext, ao: object) -> None:
    if not isinstance(ao, dict):
        return
    for ch in range(DSS_CHDEF_AO_MAX):
        v = ao.get(f'{ch:02d}')
        if not isinstance(v, dict):
            continue
        try:
            ctx.AIO.AO[ch].Cal.a = float(v.get('a', 0.0))
            ctx.AIO.AO[ch].Cal.b = float(v.get('b', 0.0))
        except (TypeError, ValueError) as _exc:
            continue

def _restore_specimen_data(ctx: CDSBPyContext, sd: object) -> None:
    if not isinstance(sd, dict):
        return
    d_diam = sd.get('default_diameter')
    d_height = sd.get('default_height')
    if d_diam is not None:
        with contextlib.suppress(TypeError, ValueError):
            ctx.SpecimenData.default_diameter = float(d_diam)
    if d_height is not None:
        with contextlib.suppress(TypeError, ValueError):
            ctx.SpecimenData.default_height = float(d_height)
    d = float(ctx.SpecimenData.default_diameter)
    h = float(ctx.SpecimenData.default_height)
    r = d / 2.0
    area = np.float32(np.pi * r * r)
    volume = np.float32(area * h)
    for i in range(len(ctx.SpecimenData.Stage)):
        ctx.SpecimenData.Stage._a[i]['diameter'] = np.float32(d)
        ctx.SpecimenData.Stage._a[i]['height'] = np.float32(h)
        ctx.SpecimenData.Stage._a[i]['area'] = area
        ctx.SpecimenData.Stage._a[i]['volume'] = volume

def restore_from_config(appdata_dir: Path | None=None) -> bool:
    path = _config_path(appdata_dir)
    if not path.exists():
        return False
    try:
        with open(path, encoding='utf-8') as fp:
            data = json.load(fp)
    except (OSError, json.JSONDecodeError):
        return False
    if not isinstance(data, dict):
        return False
    ctx = get_context()
    try:
        calibration = data.get('calibration', {})
        _restore_ai_calibration(ctx, calibration.get('ai', {}))
        _restore_ao_calibration(ctx, calibration.get('ao', {}))
        _restore_specimen_data(ctx, data.get('specimen_data', {}))
    except Exception:
        return False
    else:
        return True

def _build_payload() -> dict[str, Any]:
    ctx = get_context()
    out: dict[str, Any] = {}
    ai = {}
    for ch in range(DSS_CHDEF_AI_MAX):
        ai[f'{ch:02d}'] = {'a': float(ctx.AIO.AI[ch].Cal.a), 'b': float(ctx.AIO.AI[ch].Cal.b), 'c': float(ctx.AIO.AI[ch].Cal.c)}
    ao = {}
    for ch in range(DSS_CHDEF_AO_MAX):
        ao[f'{ch:02d}'] = {'a': float(ctx.AIO.AO[ch].Cal.a), 'b': float(ctx.AIO.AO[ch].Cal.b)}
    out['calibration'] = {'ai': ai, 'ao': ao}
    out['specimen_data'] = {'default_diameter': float(ctx.SpecimenData.default_diameter), 'default_height': float(ctx.SpecimenData.default_height)}
    out['control_mode'] = ctx.Control.mode.name
    return out

def save_calibration_to_config(appdata_dir: Path | None=None) -> bool:
    payload = _build_payload()
    return _write_payload(payload, appdata_dir)

def save_plot_selection_to_config(plot_selections: list[tuple[str | None, str | None]], appdata_dir: Path | None=None) -> bool:
    payload = _read_existing_payload(appdata_dir) or _build_payload()
    keys = ('a', 'b')
    for i, (x, y) in enumerate(plot_selections[:len(keys)]):
        key = keys[i]
        if x is not None:
            payload.setdefault('plot', {}).setdefault(key, {})['x'] = str(x)
        if y is not None:
            payload.setdefault('plot', {}).setdefault(key, {})['y'] = str(y)
    return _write_payload(payload, appdata_dir)

def read_control_mode_from_config(appdata_dir: Path | None=None) -> str | None:
    payload = _read_existing_payload(appdata_dir)
    if not payload:
        return None
    value = payload.get('control_mode')
    if not isinstance(value, str):
        return None
    return value

def save_control_mode_to_config(mode: str, appdata_dir: Path | None=None) -> bool:
    if mode not in ('MOTOR', 'TORSIONAL'):
        return False
    payload = _read_existing_payload(appdata_dir) or _build_payload()
    payload['control_mode'] = mode
    return _write_payload(payload, appdata_dir)

def _read_existing_payload(appdata_dir: Path | None=None) -> dict[str, Any] | None:
    path = _config_path(appdata_dir)
    if not path.exists():
        return None
    try:
        with open(path, encoding='utf-8') as fp:
            data = json.load(fp)
    except (OSError, json.JSONDecodeError):
        return None
    return data if isinstance(data, dict) else None

def _write_payload(payload: dict[str, Any], appdata_dir: Path | None) -> bool:
    path = _config_path(appdata_dir)
    with CONFIG_LOCK:
        try:
            tmp = path.with_suffix(path.suffix + '.tmp')
            with open(tmp, 'w', encoding='utf-8') as fp:
                json.dump(payload, fp, indent=2, ensure_ascii=False)
            os.replace(tmp, path)
        except OSError:
            return False
        else:
            return True
__all__ = ['CONFIG_FILENAME', 'read_control_mode_from_config', 'restore_from_config', 'save_calibration_to_config', 'save_control_mode_to_config', 'save_plot_selection_to_config']
