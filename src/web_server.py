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
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from PySide6.QtCore import QObject, Signal
from PySide6.QtHttpServer import QHttpServer
from PySide6.QtNetwork import QHostAddress, QTcpServer
from utl_context import CDSBPyContext, get_context
DSS_HTTP_PORT_DEFAULT: int = 8080
DSS_HTTP_PORT_RETRY_MAX: int = 5
DSS_HTTP_PORT_RETRY_DELAY_SEC: float = 0.1
DSS_HTTP_LOG_MAX_ENTRIES: int = 256

@dataclass(slots=True)
class ClientLogEntry:
    ts: datetime
    peer: str
    method: str
    path: str
    status: int
    msg: str = ''

class ControlApiServer(QObject):
    started = Signal(bool)
    client_accessed = Signal(object)
    control_start_requested = Signal()
    control_stop_requested = Signal()
    analog_output_update_requested = Signal()

    def __init__(self, parent: QObject | None=None) -> None:
        super().__init__(parent)
        self._http: QHttpServer | None = None
        self._tcp: QTcpServer | None = None
        self._handler: object | None = None
        self._port: int = 0
        self._entries: deque[ClientLogEntry] = deque(maxlen=DSS_HTTP_LOG_MAX_ENTRIES)

    def start(self, port: int=DSS_HTTP_PORT_DEFAULT) -> bool:
        for attempt in range(DSS_HTTP_PORT_RETRY_MAX):
            candidate = port + attempt
            if self._bind_one(candidate):
                self._port = candidate
                self.started.emit(True)
                return True
            self._cleanup_http()
            if attempt < DSS_HTTP_PORT_RETRY_MAX - 1:
                time.sleep(DSS_HTTP_PORT_RETRY_DELAY_SEC)
        self._port = 0
        self.started.emit(False)
        return False

    def stop(self) -> None:
        self._cleanup_http()
        self._port = 0

    def restart(self, port: int) -> bool:
        self.stop()
        return self.start(port)

    def is_running(self) -> bool:
        return self._port > 0 and self._http is not None

    def port(self) -> int:
        return self._port

    def log_entries(self) -> list[ClientLogEntry]:
        return list(self._entries)

    def record_client(self, entry: ClientLogEntry) -> None:
        self._entries.append(entry)
        self.client_accessed.emit(entry)

    def _bind_one(self, port: int) -> bool:
        from web_api import ControlApiHandler
        tcp = QTcpServer(self)
        if not tcp.listen(QHostAddress.SpecialAddress.AnyIPv4, port):
            tcp.deleteLater()
            return False
        http = QHttpServer(self)
        handler = ControlApiHandler(self._ctx(), recorder=self._on_access, request_control_start=self._on_control_start_requested, request_control_stop=self._on_control_stop_requested, request_analog_output_update=self._on_analog_output_update_requested)
        handler.setup_router(http)
        if not http.bind(tcp):
            tcp.close()
            tcp.deleteLater()
            http.deleteLater()
            return False
        self._http = http
        self._tcp = tcp
        self._handler = handler
        return True

    def _on_access(self, peer: str, method: str, path: str, status: int, msg: str) -> None:
        self.record_client(ClientLogEntry(ts=datetime.now(), peer=peer, method=method, path=path, status=status, msg=msg))

    def _on_control_start_requested(self) -> None:
        self.control_start_requested.emit()

    def _on_control_stop_requested(self) -> None:
        self.control_stop_requested.emit()

    def _on_analog_output_update_requested(self) -> None:
        self.analog_output_update_requested.emit()

    def _cleanup_http(self) -> None:
        if self._http is not None:
            self._http.deleteLater()
            self._http = None
        if self._tcp is not None:
            self._tcp.close()
            self._tcp.deleteLater()
            self._tcp = None
        self._handler = None

    def _ctx(self) -> CDSBPyContext:
        return get_context()
__all__ = ['DSS_HTTP_LOG_MAX_ENTRIES', 'DSS_HTTP_PORT_DEFAULT', 'DSS_HTTP_PORT_RETRY_DELAY_SEC', 'DSS_HTTP_PORT_RETRY_MAX', 'ClientLogEntry', 'ControlApiServer']
