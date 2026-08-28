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
import math
import sys
from collections.abc import Callable
from typing import Any
import numpy as np
from PySide6.QtHttpServer import QHttpServer, QHttpServerRequest, QHttpServerResponder, QHttpServerResponse
from PySide6.QtNetwork import QHttpHeaders
from ctl_motor import MotorController
from utl_context import DSS_APP_VERSION, DSS_CHDEF_PARAM_MAX, DSS_CHDEF_STEPCTRL_ARGS_MAX, DSS_CHDEF_STEPCTRL_STEP_MAX, DSS_MB_AI_REGISTER_COUNT, DSS_MB_AO_REGISTER_COUNT, DSS_PREVIEW_HTTP_MAX_POINTS, DSS_VOLTAGE_OUT_LABELS, CDSBPyContext, ControlType, dss_active_parameter_labels, dss_active_physical_labels, dss_active_raw_labels, get_context
from utl_origami import get_default_buffer
DSS_HTTP_BODY_MAX: int = 4096
_JSON_MIME: bytes = b'application/json; charset=utf-8'
_Method = QHttpServerRequest.Method
_StatusCode = QHttpServerResponder.StatusCode

def _fmt(value: float | int | None) -> float | None:
    if value is None:
        return None
    f = float(value)
    if math.isnan(f) or math.isinf(f):
        return None
    return f

def _finite_list(arr: np.ndarray[Any, np.dtype[Any]]) -> list[Any]:
    a = np.asarray(arr, dtype=np.float64)
    mask = np.isfinite(a)
    if bool(mask.all()):
        return list(a.tolist())
    obj = a.astype(object)
    obj[~mask] = None
    return list(obj.tolist())

def _decimate(arr: np.ndarray[Any, np.dtype[Any]], length: int | None) -> np.ndarray[Any, np.dtype[Any]]:
    if length is None:
        return arr[:DSS_PREVIEW_HTTP_MAX_POINTS]
    n = int(arr.shape[0])
    if length <= 0 or n <= length:
        return arr
    idx = np.linspace(0, n - 1, length).astype(np.intp)
    return arr[idx]

class ControlApiHandler:

    def __init__(self, ctx: CDSBPyContext | None=None, recorder: Callable[[str, str, str, int, str], None] | None=None, request_control_start: Callable[[], None] | None=None, request_control_stop: Callable[[], None] | None=None, request_analog_output_update: Callable[[], None] | None=None) -> None:
        self._ctx_obj = ctx
        self._recorder = recorder
        self._request_control_start = request_control_start
        self._request_control_stop = request_control_stop
        self._request_analog_output_update = request_analog_output_update
        self._pending: QHttpServerResponse | None = None
        self._callbacks: list[Callable[..., Any]] = []

    def _ctx(self) -> CDSBPyContext:
        if self._ctx_obj is None:
            self._ctx_obj = get_context()
        return self._ctx_obj

    def setup_router(self, server: QHttpServer) -> None:
        routes: tuple[tuple[str, Callable[[QHttpServerRequest], QHttpServerResponse]], ...] = (('/v1', self._v1_root), ('/v1/', self._v1_root), ('/v1/preview', self._v1_preview), ('/v1/health', self._v1_health), ('/v2/health', self._v2_health), ('/v2/control/type', self._v2_control_type), ('/v2/control/step', self._v2_control_step), ('/v2/control/step_all', self._v2_control_step_all), ('/v2/control/args', self._v2_control_args), ('/v2/control/motor', self._v2_control_motor), ('/v2/control/start', self._v2_control_start), ('/v2/control/stop', self._v2_control_stop))
        for rule, handler in routes:
            callback = self._wrap(handler)
            self._callbacks.append(callback)
            server.route(rule, callback)
        after = self._after_request
        self._callbacks.append(after)
        server.addAfterRequestHandler(server, after)

    def _wrap(self, handler: Callable[[QHttpServerRequest], QHttpServerResponse]) -> Callable[[QHttpServerRequest], str]:

        def _callback(request: QHttpServerRequest) -> str:
            try:
                response = self._preflight(request)
                if response is None:
                    response = handler(request)
            except Exception as exc:
                response = self._error_response(500, f'internal error: {exc!r}')
            self._pending = response
            self._record(request, int(response.statusCode().value))
            return ''
        return _callback

    def _after_request(self, _request: QHttpServerRequest, response: QHttpServerResponse) -> None:
        pending = self._pending
        self._pending = None
        if pending is not None:
            response.swap(pending)

    def _record(self, request: QHttpServerRequest, status: int) -> None:
        if self._recorder is None:
            return
        peer = f'{request.remoteAddress().toString()}:{request.remotePort()}'
        url = request.url()
        path = url.path()
        if url.hasQuery():
            path = f'{path}?{url.query()}'
        method = (request.method().name or 'UNKNOWN').upper()
        self._recorder(peer, method, path, status, '')

    def _preflight(self, request: QHttpServerRequest) -> QHttpServerResponse | None:
        if request.method() != _Method.Options:
            return None
        headers = self._cors_headers()
        headers['Allow'] = 'GET, POST, PUT, DELETE, OPTIONS'
        return self._build_response('', 204, headers)

    def _v1_root(self, request: QHttpServerRequest) -> QHttpServerResponse:
        err = self._get_only(request)
        if err is not None:
            return err
        ctx = self._ctx()
        ai_raw = np.asarray(ctx.AIO.AI.get_field('raw'), dtype=np.float64)
        ai_phy = np.asarray(ctx.AIO.AI.get_field('phy'), dtype=np.float64)
        ao_raw = np.asarray(ctx.AIO.AO.get_field('raw'), dtype=np.float64)
        params = np.asarray(ctx.AIO.param, dtype=np.float64)
        raw_labels = dss_active_raw_labels(ctx.Control.mode)
        phy_labels = dss_active_physical_labels(ctx.Control.mode)
        par_labels = dss_active_parameter_labels(ctx.Control.mode)
        data: dict[str, object] = {'flag': {'set_board': bool(ctx.Flag.set_board), 'save_data': bool(ctx.Flag.save_data), 'control': bool(ctx.Flag.control)}, 'raw': {f'{ch:02d}': {'value': v, 'label': raw_labels[ch]} for ch, v in enumerate(_finite_list(ai_raw[:DSS_MB_AI_REGISTER_COUNT]))}, 'phy': {f'{ch:02d}': {'value': v, 'label': phy_labels[ch]} for ch, v in enumerate(_finite_list(ai_phy[:DSS_MB_AI_REGISTER_COUNT]))}, 'par': {f'{ch:02d}': {'value': v, 'label': par_labels[ch]} for ch, v in enumerate(_finite_list(params[:DSS_CHDEF_PARAM_MAX]))}, 'out': {f'{ch:02d}': {'value': v, 'label': DSS_VOLTAGE_OUT_LABELS[ch]} for ch, v in enumerate(_finite_list(ao_raw[:DSS_MB_AO_REGISTER_COUNT]))}}
        return self._json_response(data, indent=2)

    def _v1_preview(self, request: QHttpServerRequest) -> QHttpServerResponse:
        err = self._get_only(request)
        if err is not None:
            return err
        buf = get_default_buffer()
        if buf is None or buf.write_count == 0:
            return self._json_response({})
        query = request.query()
        length = self._preview_length(query.queryItemValue('length'))
        n = buf.write_count
        out: dict[str, dict[str, object]] = {}
        if query.hasQueryItem('time'):
            out['time'] = {'label': 'Time[s]', 'data': _finite_list(_decimate(buf._col('elapsed', 0, n), length))}
        if query.hasQueryItem('timestamp'):
            out['timestamp'] = {'label': 'Epoch[s]', 'data': _finite_list(_decimate(buf._col('time', 0, n), length))}
        mode = self._ctx().Control.mode
        raw_labels = dss_active_raw_labels(mode)
        phy_labels = dss_active_physical_labels(mode)
        par_labels = dss_active_parameter_labels(mode)
        for ch in range(DSS_MB_AI_REGISTER_COUNT):
            suffix = f'{ch:02d}'
            if query.hasQueryItem(f'raw_{suffix}'):
                out[f'raw_{suffix}'] = {'label': raw_labels[ch], 'data': _finite_list(_decimate(buf._col('raw', ch, n), length))}
            if query.hasQueryItem(f'phy_{suffix}'):
                out[f'phy_{suffix}'] = {'label': phy_labels[ch], 'data': _finite_list(_decimate(buf._col('phy', ch, n), length))}
        for ch in range(DSS_CHDEF_PARAM_MAX):
            key = f'par_{ch:02d}'
            if query.hasQueryItem(key):
                out[key] = {'label': par_labels[ch], 'data': _finite_list(_decimate(buf._col('par', ch, n), length))}
        for ch in range(DSS_MB_AO_REGISTER_COUNT):
            key = f'out_{ch:02d}'
            if query.hasQueryItem(key):
                out[key] = {'label': DSS_VOLTAGE_OUT_LABELS[ch], 'data': _finite_list(_decimate(buf._col('ao', ch, n), length))}
        return self._json_response(out)

    @staticmethod
    def _preview_length(raw: str) -> int | None:
        if not raw:
            return None
        try:
            value = int(raw)
        except ValueError:
            return None
        return min(DSS_PREVIEW_HTTP_MAX_POINTS, max(1, value))

    def _v1_health(self, request: QHttpServerRequest) -> QHttpServerResponse:
        err = self._get_only(request)
        if err is not None:
            return err
        from datetime import datetime
        return self._json_response({'status': 'OK', 'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')})

    def _v2_health(self, _request: QHttpServerRequest) -> QHttpServerResponse:
        from PySide6 import __version__ as pyside_version
        return self._json_response({'status': 'ok', 'version': DSS_APP_VERSION, 'python': sys.version.split()[0], 'pyside6': pyside_version})

    def _v2_control_type(self, request: QHttpServerRequest) -> QHttpServerResponse:
        ctx = self._ctx()
        if request.method() == _Method.Get:
            return self._json_response({'type': ControlType(ctx.Control.type).name})
        payload = self._read_payload(request)
        if isinstance(payload, QHttpServerResponse):
            return payload
        type_str = payload.get('type')
        if not isinstance(type_str, str):
            return self._error_response(422, "'type' must be a string")
        try:
            new_type = ControlType[type_str]
        except KeyError:
            names = '/'.join((t.name for t in ControlType))
            return self._error_response(422, f'unknown type: {type_str!r} (expected {names})')
        ctx.Control.type = new_type
        return self._json_response({'ok': True, 'type': new_type.name})

    def _v2_control_step(self, request: QHttpServerRequest) -> QHttpServerResponse:
        ctx = self._ctx()
        if request.method() == _Method.Get:
            return self._json_response({'step': int(ctx.Control.current_step)})
        payload = self._read_payload(request)
        if isinstance(payload, QHttpServerResponse):
            return payload
        step = payload.get('step')
        if not isinstance(step, int) or isinstance(step, bool):
            return self._error_response(422, "'step' must be an integer")
        if not 0 <= step < DSS_CHDEF_STEPCTRL_STEP_MAX:
            return self._error_response(422, f'step must be 0..{DSS_CHDEF_STEPCTRL_STEP_MAX - 1}')
        ctx.Control.current_step = step
        return self._json_response({'ok': True, 'step': step})

    def _v2_control_step_all(self, request: QHttpServerRequest) -> QHttpServerResponse:
        ctx = self._ctx()
        steps = ctx.Control.Step
        if request.method() == _Method.Get:
            ctrl_all = np.asarray(steps.get_field('ctrl'), dtype=np.int64).tolist()
            args_all = np.asarray(steps.get_field('args'), dtype=np.float64)
            dump = {f'{i:04d}': {'ctrl': int(ctrl_all[i]), 'args': _finite_list(args_all[i])} for i in range(DSS_CHDEF_STEPCTRL_STEP_MAX)}
            return self._json_response(dump, indent=2)
        payload = self._read_payload(request)
        if isinstance(payload, QHttpServerResponse):
            return payload
        applied = 0
        for key, row in payload.items():
            err = self._apply_step_row(key, row)
            if err is not None:
                return err
            applied += 1
        return self._json_response({'ok': True, 'applied': applied})

    def _apply_step_row(self, key: str, row: object) -> QHttpServerResponse | None:
        try:
            index = int(key)
        except (TypeError, ValueError):
            return self._error_response(422, f'invalid step key: {key!r}')
        if not 0 <= index < DSS_CHDEF_STEPCTRL_STEP_MAX:
            return self._error_response(422, f'step {index} out of range 0..{DSS_CHDEF_STEPCTRL_STEP_MAX - 1}')
        if not isinstance(row, dict):
            return self._error_response(422, f'step {index}: value must be an object')
        if 'ctrl' in row:
            try:
                self._ctx().Control.Step[index].ctrl = int(row['ctrl'])
            except (TypeError, ValueError):
                return self._error_response(422, f"step {index}: 'ctrl' must be an integer")
        if 'args' in row:
            return self._assign_args(index, row['args'])
        return None

    def _v2_control_args(self, request: QHttpServerRequest) -> QHttpServerResponse:
        ctx = self._ctx()
        current = int(ctx.Control.current_step)
        if request.method() == _Method.Get:
            args = np.asarray(ctx.Control.Step[current].args, dtype=np.float64)
            return self._json_response({'args': _finite_list(args)})
        payload = self._read_payload(request)
        if isinstance(payload, QHttpServerResponse):
            return payload
        err = self._assign_args(current, payload.get('args'))
        if err is not None:
            return err
        return self._json_response({'ok': True, 'step': current})

    def _assign_args(self, index: int, raw: object) -> QHttpServerResponse | None:
        try:
            values = np.asarray(raw, dtype=np.float32)
        except (TypeError, ValueError):
            return self._error_response(422, f"step {index}: 'args' must be a float array")
        if values.ndim != 1 or values.size != DSS_CHDEF_STEPCTRL_ARGS_MAX:
            return self._error_response(422, f'step {index}: args must have {DSS_CHDEF_STEPCTRL_ARGS_MAX} floats')
        self._ctx().Control.Step[index].args[:] = values
        return None

    def _v2_control_motor(self, request: QHttpServerRequest) -> QHttpServerResponse:
        payload = self._read_payload(request)
        if isinstance(payload, QHttpServerResponse):
            return payload
        motor = MotorController()
        applied: dict[str, object] = {}
        if 'clutch' in payload:
            clutch = bool(payload['clutch'])
            motor.set_motor_clutch(clutch)
            applied['clutch'] = clutch
        if 'speed' in payload:
            try:
                speed = float(payload['speed'])
            except (TypeError, ValueError):
                return self._error_response(422, "'speed' must be a number")
            if not math.isfinite(speed):
                return self._error_response(422, "'speed' must be finite")
            motor.set_motor_speed(speed)
            applied['speed'] = speed
        direction = payload.get('direction', 'none')
        if direction == 'up':
            motor.set_motor_direction_up()
        elif direction == 'down':
            motor.set_motor_direction_down()
        elif direction != 'none':
            return self._error_response(422, "'direction' must be 'up' / 'down' / 'none'")
        applied['direction'] = direction
        if self._request_analog_output_update is not None:
            self._request_analog_output_update()
        return self._json_response({'ok': True, 'applied': applied})

    def _v2_control_start(self, request: QHttpServerRequest) -> QHttpServerResponse:
        payload = self._read_payload(request)
        if isinstance(payload, QHttpServerResponse):
            return payload
        if payload.get('confirm') is not True:
            return self._error_response(400, 'missing confirm: true')
        if self._request_control_start is not None:
            self._request_control_start()
            return self._json_response({'ok': True})
        return self._json_response({'ok': False, 'note': 'control-start bridge is not available'})

    def _v2_control_stop(self, request: QHttpServerRequest) -> QHttpServerResponse:
        payload = self._read_payload(request)
        if isinstance(payload, QHttpServerResponse):
            return payload
        if self._request_control_stop is not None:
            self._request_control_stop()
            return self._json_response({'ok': True})
        return self._json_response({'ok': False, 'note': 'control-stop bridge is not available'})

    def _get_only(self, request: QHttpServerRequest) -> QHttpServerResponse | None:
        if request.method() == _Method.Get:
            return None
        return self._error_response(405, 'Method Not Allowed', allow='GET')

    def _external_only(self, request: QHttpServerRequest) -> QHttpServerResponse | None:
        if request.method() == _Method.Get:
            return None
        if request.method() not in (_Method.Post, _Method.Put, _Method.Delete):
            return self._error_response(405, 'Method Not Allowed', allow='GET, POST')
        if self._ctx().Control.type != ControlType.EXTERNAL:
            return self._error_response(405, 'External mode is OFF, write APIs are disabled', allow='GET')
        return None

    def _body_size_guard(self, request: QHttpServerRequest) -> QHttpServerResponse | None:
        if request.body().size() > DSS_HTTP_BODY_MAX:
            return self._error_response(413, f'Body too large (max {DSS_HTTP_BODY_MAX} bytes)')
        return None

    def _read_payload(self, request: QHttpServerRequest) -> dict[str, Any] | QHttpServerResponse:
        guard = self._body_size_guard(request) or self._external_only(request)
        if guard is not None:
            return guard
        body = bytes(request.body().data())
        if not body:
            return {}
        try:
            payload = json.loads(body.decode('utf-8'))
        except UnicodeDecodeError:
            return self._error_response(400, 'Invalid JSON: body is not valid UTF-8')
        except json.JSONDecodeError as exc:
            return self._error_response(400, f'Invalid JSON: {exc.msg} at pos {exc.pos}')
        if not isinstance(payload, dict):
            return self._error_response(422, 'JSON body must be an object')
        return payload

    def _cors_headers(self) -> dict[str, str]:
        return {'Access-Control-Allow-Origin': '*', 'Access-Control-Allow-Methods': 'GET, POST, PUT, DELETE, OPTIONS', 'Access-Control-Allow-Headers': 'Content-Type, Authorization'}

    def _json_response(self, data: object, status: int=200, indent: int | None=None) -> QHttpServerResponse:
        if indent is None:
            text = json.dumps(data, separators=(',', ':'), ensure_ascii=False, allow_nan=False)
        else:
            text = json.dumps(data, indent=indent, ensure_ascii=False, allow_nan=False)
        return self._build_response(text, status, self._cors_headers())

    def _error_response(self, status: int, msg: str, allow: str | None=None) -> QHttpServerResponse:
        text = json.dumps({'error': msg}, separators=(',', ':'), ensure_ascii=False)
        headers = self._cors_headers()
        if allow is not None:
            headers['Allow'] = allow
        return self._build_response(text, status, headers)

    def _build_response(self, text: str, status: int, headers: dict[str, str]) -> QHttpServerResponse:
        payload = text.encode('utf-8')
        response = QHttpServerResponse(_JSON_MIME, payload, _StatusCode(status))
        qt_headers = QHttpHeaders()
        qt_headers.append('Content-Type', _JSON_MIME.decode('ascii'))
        for key, value in headers.items():
            qt_headers.append(key, value)
        response.setHeaders(qt_headers)
        return response
__all__ = ['DSS_HTTP_BODY_MAX', 'ControlApiHandler']
