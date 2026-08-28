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
import sys

class PreventSleep:

    def __init__(self) -> None:
        self._active: bool = False
        self._supported: bool = self._probe_supported()

    @staticmethod
    def _probe_supported() -> bool:
        if not sys.platform.startswith('win'):
            return False
        try:
            import ctypes
        except ImportError:
            return False
        return True

    def is_supported(self) -> bool:
        return self._supported

    def is_active(self) -> bool:
        return self._active

    def acquire(self) -> bool:
        if not self._supported:
            return False
        if self._active:
            return True
        try:
            import ctypes
            ES_CONTINUOUS = 2147483648
            ES_SYSTEM_REQUIRED = 1
            ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED)
            self._active = True
        except Exception:
            self._active = False
        return self._active

    def release(self) -> bool:
        if not self._supported:
            return False
        if not self._active:
            return True
        try:
            import ctypes
            ES_CONTINUOUS = 2147483648
            ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS)
        except Exception:
            pass
        finally:
            self._active = False
        return True
