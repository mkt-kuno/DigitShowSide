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
import collections
import contextlib
import os
import sys
import threading
import numpy as np
import numpy.typing as npt
from pymodbus.client import ModbusSerialClient
from PySide6.QtCore import QObject, QTimer, Signal, Slot
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utl_context import DSS_MB_AI_REGISTER_COUNT, DSS_MB_AI_START_ADDRESS, DSS_MB_AO_MAX_MV, DSS_MB_AO_MIN_MV, DSS_MB_AO_REGISTER_COUNT, DSS_MB_AO_START_ADDRESS, DSS_MB_BAUDRATE, DSS_MB_BYTESIZE, DSS_MB_F32_INPUT_PROBE_REGS, DSS_MB_F32_INPUT_REGS_TOTAL, DSS_MB_F32_INPUT_START, DSS_MB_PARITY, DSS_MB_SLAVE_ID, DSS_MB_STOPBITS, DSS_TIM_POLL_MS

def _u16_to_i16(value: int) -> int:
    return value if value < 32768 else value - 65536

def _pad_int_array(arr: np.ndarray, length: int, dtype: npt.DTypeLike) -> np.ndarray:
    n = arr.shape[0]
    if n == length:
        return arr
    out = np.zeros(length, dtype=dtype)
    if n > length:
        return arr[:length].astype(dtype, copy=False)
    out[:n] = arr.astype(dtype, copy=False)
    return out

def _u16_array_to_i16_array(arr: np.ndarray) -> np.ndarray:
    out = arr.astype(np.int32)
    mask = out >= 32768
    out[mask] -= 65536
    return out.astype(np.int16)

def _regs_to_float32_array(regs: np.ndarray) -> np.ndarray:
    u32 = regs.astype(np.uint32, copy=False).reshape(-1, 2)
    word = u32[:, 0] << np.uint32(16) | u32[:, 1]
    return word.view(np.float32).copy()

class ModbusWorker(QObject):
    new_values = Signal(list)
    error = Signal(str)
    status = Signal(str)
    voltage_changed = Signal(int, int)
    write_register_request = Signal(int, int)

    def __init__(self, port: str):
        super().__init__()
        self._port = port
        self._client: ModbusSerialClient | None = None
        self._timer: QTimer | None = None
        self._bus_lock = threading.RLock()
        self._is_polling = False
        self._pending_writes: collections.deque[tuple[int, int]] = collections.deque()
        self._last_voltage_mV: list[int] = [0] * DSS_MB_AO_REGISTER_COUNT
        self._use_float32: bool = False
        self.write_register_request.connect(self._on_write_requested)

    @Slot()
    def start(self) -> None:
        self._client = ModbusSerialClient(port=self._port, baudrate=DSS_MB_BAUDRATE, parity=DSS_MB_PARITY, stopbits=DSS_MB_STOPBITS, bytesize=DSS_MB_BYTESIZE, timeout=0.5)
        if not self._client.connect():
            self.error.emit(f'Modbus connect failed: {self._port}')
            self._client = None
            return
        self.status.emit(f'Modbus RTU 38400 8N1 connected @ {self._port} (slave={DSS_MB_SLAVE_ID})')
        self._zero_all_outputs()
        self._use_float32 = self._probe_float32_capability()
        from utl_context import get_context
        get_context().Modbus.float_input_reg = self._use_float32
        if self._use_float32:
            self.status.emit('InputRegister: f32t mode (16ch float32 @5000)')
        else:
            self.status.emit('InputRegister: i16t mode (16ch int16 @0)')
        self._timer = QTimer()
        self._timer.setInterval(DSS_TIM_POLL_MS)
        self._timer.timeout.connect(self._poll)
        self._timer.start()

    @Slot()
    def stop(self) -> None:
        if self._timer is not None:
            self._timer.stop()
            self._timer.deleteLater()
            self._timer = None
        if self._client is not None:
            self._zero_all_outputs()
            with contextlib.suppress(Exception):
                self._client.close()
            self._client = None
        self.status.emit('Modbus disconnected')

    def get_last_voltage_mV(self) -> list[int]:
        return list(self._last_voltage_mV)

    def is_using_float32(self) -> bool:
        return self._use_float32

    def _zero_all_outputs(self) -> None:
        if self._client is None:
            return
        with self._bus_lock:
            try:
                result = self._client.write_registers(address=DSS_MB_AO_START_ADDRESS, values=[0] * DSS_MB_AO_REGISTER_COUNT, device_id=DSS_MB_SLAVE_ID)
            except Exception as e:
                self.error.emit(f'VOUT zero-write exception: {e}')
                return
            if result.isError():
                self.error.emit(f'VOUT zero-write error: {result}')
                return
        for ch in range(DSS_MB_AO_REGISTER_COUNT):
            self._last_voltage_mV[ch] = 0
            self.voltage_changed.emit(ch, 0)
        self.status.emit('VOUT all-zero written')

    def _probe_float32_capability(self) -> bool:
        if self._client is None:
            return False
        if not self._read_input_ok(DSS_MB_F32_INPUT_START, DSS_MB_F32_INPUT_PROBE_REGS):
            return False
        self.status.emit('Float32 probe: 1ch OK')
        if not self._read_input_ok(DSS_MB_F32_INPUT_START, DSS_MB_F32_INPUT_REGS_TOTAL):
            return False
        self.status.emit('Float32 probe: 16ch OK')
        return True

    def _read_input_ok(self, address: int, count: int) -> bool:
        if self._client is None:
            return False
        with self._bus_lock:
            try:
                result = self._client.read_input_registers(address=address, count=count, device_id=DSS_MB_SLAVE_ID)
            except Exception:
                return False
            if result.isError():
                return False
            regs = getattr(result, 'registers', None)
            if regs is None or len(regs) != count:
                return False
        return True

    @Slot()
    def _poll(self) -> None:
        if self._client is None:
            return
        self._is_polling = True
        try:
            values = self._read_inputs_float32() if self._use_float32 else self._read_inputs_i16()
            self.new_values.emit(values.tolist())
        finally:
            self._is_polling = False
        self._drain_pending_writes()

    def _read_inputs_i16(self) -> np.ndarray:
        if self._client is None:
            return np.zeros(DSS_MB_AI_REGISTER_COUNT, dtype=np.float32)
        with self._bus_lock:
            try:
                result = self._client.read_input_registers(address=DSS_MB_AI_START_ADDRESS, count=DSS_MB_AI_REGISTER_COUNT, device_id=DSS_MB_SLAVE_ID)
            except Exception as e:
                self.error.emit(f'Modbus exception: {e}')
                return np.zeros(DSS_MB_AI_REGISTER_COUNT, dtype=np.float32)
        if result.isError():
            self.error.emit(f'Modbus error: {result}')
            return np.zeros(DSS_MB_AI_REGISTER_COUNT, dtype=np.float32)
        try:
            regs_list = result.registers
        except AttributeError:
            self.error.emit(f'Unexpected response: {result!r}')
            return np.zeros(DSS_MB_AI_REGISTER_COUNT, dtype=np.float32)
        regs = _pad_int_array(np.asarray(regs_list, dtype=np.uint16), DSS_MB_AI_REGISTER_COUNT, np.uint16)
        return _u16_array_to_i16_array(regs).astype(np.float32)

    def _read_inputs_float32(self) -> np.ndarray:
        if self._client is None:
            return np.zeros(DSS_MB_AI_REGISTER_COUNT, dtype=np.float32)
        with self._bus_lock:
            try:
                result = self._client.read_input_registers(address=DSS_MB_F32_INPUT_START, count=DSS_MB_F32_INPUT_REGS_TOTAL, device_id=DSS_MB_SLAVE_ID)
            except Exception as e:
                self.error.emit(f'Modbus exception: {e}')
                return np.zeros(DSS_MB_AI_REGISTER_COUNT, dtype=np.float32)
        if result.isError():
            self.error.emit(f'Modbus error: {result}')
            return np.zeros(DSS_MB_AI_REGISTER_COUNT, dtype=np.float32)
        regs_list = getattr(result, 'registers', None)
        if regs_list is None or len(regs_list) != DSS_MB_F32_INPUT_REGS_TOTAL:
            self.error.emit(f'f32t short read ({(len(regs_list) if regs_list else 0)}/{DSS_MB_F32_INPUT_REGS_TOTAL})')
            return np.zeros(DSS_MB_AI_REGISTER_COUNT, dtype=np.float32)
        return _regs_to_float32_array(np.asarray(regs_list, dtype=np.uint16))[:DSS_MB_AI_REGISTER_COUNT]

    def _drain_pending_writes(self) -> None:
        if self._client is None:
            return
        while self._pending_writes:
            ch, mV = self._pending_writes.popleft()
            if not 0 <= ch < DSS_MB_AO_REGISTER_COUNT:
                continue
            with self._bus_lock:
                if self._client is None:
                    return
                try:
                    result = self._client.write_register(address=DSS_MB_AO_START_ADDRESS + ch, value=mV, device_id=DSS_MB_SLAVE_ID)
                except Exception as e:
                    self.error.emit(f'Modbus write exception (CH{ch:02d}): {e}')
                    continue
                if result.isError():
                    self.error.emit(f'Modbus write error (CH{ch:02d}): {result}')
                    continue
            self._last_voltage_mV[ch] = mV
            self.voltage_changed.emit(ch, mV)
            self.status.emit(f'VOUT CH{ch:02d} = {mV} mV')

    @Slot(int, int)
    def _on_write_requested(self, channel: int, value_mv: int) -> None:
        if self._client is None:
            self.error.emit('Modbus not connected')
            return
        if not 0 <= channel < DSS_MB_AO_REGISTER_COUNT:
            self.error.emit(f'Invalid VOUT channel: {channel}')
            return
        mV = max(DSS_MB_AO_MIN_MV, min(DSS_MB_AO_MAX_MV, int(value_mv)))
        already_pending = any((ch == channel for ch, _ in self._pending_writes))
        if self._last_voltage_mV[channel] == mV and (not already_pending):
            return
        replaced = False
        for i, (ch, _) in enumerate(self._pending_writes):
            if ch == channel:
                self._pending_writes[i] = (channel, mV)
                replaced = True
                break
        if not replaced:
            self._pending_writes.append((channel, mV))
        self._last_voltage_mV[channel] = mV
        self.voltage_changed.emit(channel, mV)
        self._drain_pending_writes()
