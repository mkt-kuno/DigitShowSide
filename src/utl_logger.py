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
import contextlib
import os
import sys
import threading
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from enum import IntEnum
from pathlib import Path
from typing import ClassVar, Final
from PySide6.QtCore import QFile, QFileInfo, QIODevice, QMutex, QMutexLocker, QObject, QStandardPaths, QStringConverter, Qt, QTextStream, QTimer, Signal, Slot
from utl_context import DSS_APP_NAME, DSS_LOG_FILENAME, DSS_LOG_FLUSH_INTERVAL_MS, DSS_LOG_LATEST_LINES, DSS_LOG_ROTATE_MAX_FILES, DSS_LOG_ROTATE_MAX_SIZE

class LogLevel(IntEnum):
    TRACE = 0
    DEBUG = 1
    INFO = 2
    WARN = 3
    ERR = 4
    CRITICAL = 5

    @property
    def label(self) -> str:
        return _LEVEL_LABELS[self]
_LEVEL_LABELS: Final[dict[LogLevel, str]] = {LogLevel.TRACE: 'TRACE', LogLevel.DEBUG: 'DEBUG', LogLevel.INFO: 'INFO', LogLevel.WARN: 'WARN', LogLevel.ERR: 'ERR', LogLevel.CRITICAL: 'CRIT'}
_STR_LEVEL_MAP: Final[dict[str, LogLevel]] = {'TRACE': LogLevel.TRACE, 'DEBUG': LogLevel.DEBUG, 'INFO': LogLevel.INFO, 'WARN': LogLevel.WARN, 'WARNING': LogLevel.WARN, 'ERR': LogLevel.ERR, 'ERROR': LogLevel.ERR, 'CRIT': LogLevel.CRITICAL, 'CRITICAL': LogLevel.CRITICAL}

def parse_level(name: str) -> LogLevel:
    return _STR_LEVEL_MAP.get(name.upper(), LogLevel.INFO)

@dataclass(slots=True)
class LogRecord:
    ts: datetime
    level: LogLevel
    msg: str
    formatted: str

def _format_line(ts: datetime, level: LogLevel, msg: str) -> str:
    ms = ts.microsecond // 1000
    return f"{ts.strftime('%Y-%m-%d %H:%M:%S')}.{ms:03d} [{level.label}] {msg}"

class _LogRingBuffer:

    def __init__(self, maxlen: int=DSS_LOG_LATEST_LINES) -> None:
        self._buf: deque[LogRecord] = deque(maxlen=maxlen)
        self._mutex = QMutex()

    def append(self, record: LogRecord) -> None:
        with QMutexLocker(self._mutex):
            self._buf.append(record)

    def snapshot(self) -> list[LogRecord]:
        with QMutexLocker(self._mutex):
            return list(self._buf)

class _RotatingFileAppender(QObject):

    def __init__(self, log_dir: Path, base_name: str=DSS_LOG_FILENAME, max_size: int=DSS_LOG_ROTATE_MAX_SIZE, max_files: int=DSS_LOG_ROTATE_MAX_FILES, flush_interval_ms: int=DSS_LOG_FLUSH_INTERVAL_MS, parent: QObject | None=None) -> None:
        super().__init__(parent)
        self._log_dir = Path(log_dir)
        self._base_name = base_name
        self._max_size = max_size
        self._max_files = max_files
        self._mutex = QMutex()
        self._current_bytes = 0
        self._file: QFile | None = None
        self._stream: QTextStream | None = None
        self._flush_timer = QTimer(self)
        self._flush_timer.setInterval(flush_interval_ms)
        self._flush_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._flush_timer.timeout.connect(self._flush)
        self._open()
        self._flush_timer.start()

    def _path_for_index(self, index: int) -> Path:
        if index == 0:
            return self._log_dir / self._base_name
        stem = Path(self._base_name).stem
        suffix = Path(self._base_name).suffix
        return self._log_dir / f'{stem}.{index}{suffix}'

    def _open(self) -> None:
        self._log_dir.mkdir(parents=True, exist_ok=True)
        path = self._path_for_index(0)
        if self._file is not None:
            self._file.close()
            self._file = None
        self._file = QFile(str(path))
        if not self._file.open(QIODevice.OpenModeFlag.WriteOnly | QIODevice.OpenModeFlag.Append):
            sys.stderr.write(f'[utl_logger] failed to open log file: {path}\n')
            self._file = None
            self._stream = None
            self._current_bytes = 0
            return
        self._stream = QTextStream(self._file)
        self._stream.setEncoding(QStringConverter.Encoding.Utf8)
        self._current_bytes = QFileInfo(str(path)).size()

    @Slot()
    def _flush(self) -> None:
        with QMutexLocker(self._mutex):
            if self._stream is not None:
                with contextlib.suppress(RuntimeError):
                    self._stream.flush()
            if self._file is not None:
                with contextlib.suppress(RuntimeError):
                    self._file.flush()

    def write(self, record: LogRecord) -> None:
        line = record.formatted + '\n'
        line_bytes = len(line.encode('utf-8'))
        with QMutexLocker(self._mutex):
            if self._file is None or self._stream is None:
                return
            if self._current_bytes + line_bytes > self._max_size:
                self._rotate_locked()
            if self._file is None or self._stream is None:
                return
            self._stream << line
            self._current_bytes += line_bytes

    def _rotate_locked(self) -> None:
        if self._stream is not None:
            with contextlib.suppress(RuntimeError):
                self._stream.flush()
            self._stream = None
        if self._file is not None:
            with contextlib.suppress(RuntimeError):
                self._file.close()
            self._file = None
        oldest = self._path_for_index(self._max_files)
        if oldest.exists():
            oldest.unlink()
        for i in range(self._max_files - 1, 0, -1):
            src = self._path_for_index(i)
            if src.exists():
                src.rename(self._path_for_index(i + 1))
        current = self._path_for_index(0)
        target = self._path_for_index(1)
        if current.exists():
            current.rename(target)
        self._open()

    def close(self) -> None:
        self._flush_timer.stop()
        with QMutexLocker(self._mutex):
            if self._stream is not None:
                with contextlib.suppress(RuntimeError):
                    self._stream.flush()
            if self._file is not None:
                with contextlib.suppress(RuntimeError):
                    self._file.close()
            self._file = None
            self._stream = None

class AppLogger(QObject):
    _instance: ClassVar[AppLogger | None] = None
    _instance_lock: ClassVar[threading.Lock] = threading.Lock()
    record_logged = Signal(object)

    def __init__(self, parent: QObject | None=None) -> None:
        super().__init__(parent)
        self._ring = _LogRingBuffer()
        self._appender: _RotatingFileAppender | None = None

    @classmethod
    def instance(cls) -> AppLogger:
        existing = cls._instance
        if existing is not None:
            return existing
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
        return cls._instance

    @classmethod
    def shutdown_instance(cls) -> None:
        with cls._instance_lock:
            if cls._instance is not None:
                cls._instance.close()
                cls._instance = None

    @staticmethod
    def default_log_dir() -> Path:
        appdata = os.environ.get('APPDATA')
        if appdata:
            return Path(appdata) / DSS_APP_NAME / 'Log'
        loc = QStandardPaths.writableLocation(QStandardPaths.StandardLocation.AppDataLocation)
        if loc:
            return Path(loc) / 'Log'
        return Path.home() / DSS_APP_NAME / 'Log'

    def install(self, log_dir: Path | None=None) -> None:
        target = Path(log_dir) if log_dir is not None else self.default_log_dir()
        target.mkdir(parents=True, exist_ok=True)
        if self._appender is not None:
            self._appender.close()
        self._appender = _RotatingFileAppender(log_dir=target, parent=self)

    def log_dir(self) -> Path | None:
        if self._appender is None:
            return None
        return self._appender._log_dir

    def log_file_path(self) -> Path | None:
        if self._appender is None:
            return None
        return self._appender._path_for_index(0)

    def log(self, level: LogLevel, msg: str) -> None:
        now = datetime.now()
        record = LogRecord(ts=now, level=level, msg=msg, formatted=_format_line(now, level, msg))
        self._ring.append(record)
        if self._appender is not None:
            self._appender.write(record)
        self.record_logged.emit(record)

    def snapshot(self) -> list[LogRecord]:
        return self._ring.snapshot()

    def trace(self, msg: str) -> None:
        self.log(LogLevel.TRACE, msg)

    def debug(self, msg: str) -> None:
        self.log(LogLevel.DEBUG, msg)

    def info(self, msg: str) -> None:
        self.log(LogLevel.INFO, msg)

    def warn(self, msg: str) -> None:
        self.log(LogLevel.WARN, msg)

    def error(self, msg: str) -> None:
        self.log(LogLevel.ERR, msg)

    def critical(self, msg: str) -> None:
        self.log(LogLevel.CRITICAL, msg)

    def close(self) -> None:
        if self._appender is not None:
            self._appender.close()

def get_logger() -> AppLogger:
    return AppLogger.instance()

def shutdown_logger() -> None:
    AppLogger.shutdown_instance()
__all__ = ['AppLogger', 'LogLevel', 'LogRecord', 'get_logger', 'parse_level', 'shutdown_logger']
