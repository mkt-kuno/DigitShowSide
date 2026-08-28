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
from pathlib import Path
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from PySide6.QtCore import QLockFile

def _appdata_dir() -> Path:
    base = os.environ.get('APPDATA') or str(Path.home() / 'AppData' / 'Roaming')
    return Path(base) / 'DigitShowSide'

class SingleInstanceGuard:
    LOCK_FILENAME: str = 'DigitShowSide.lock'

    def __init__(self, appdata_dir: Path | None=None) -> None:
        self._appdata_dir: Path = appdata_dir if appdata_dir is not None else _appdata_dir()
        self._lock_path: Path = self._appdata_dir / self.LOCK_FILENAME
        self._lock: QLockFile | None = None

    def _ensure_lock_object(self) -> QLockFile:
        if self._lock is None:
            from PySide6.QtCore import QLockFile
            self._lock = QLockFile(str(self._lock_path))
            self._lock.setStaleLockTime(0)
        return self._lock

    def try_acquire(self, timeout_ms: int=0) -> bool:
        try:
            self._appdata_dir.mkdir(parents=True, exist_ok=True)
        except OSError:
            return False
        lock = self._ensure_lock_object()
        return bool(lock.tryLock(timeout_ms))

    def is_primary(self) -> bool:
        return self._lock is not None and self._lock.isLocked()

    def release(self) -> None:
        if self._lock is not None:
            self._lock.unlock()
            self._lock = None
