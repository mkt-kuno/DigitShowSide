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
from PySide6.QtGui import QFont
from PySide6.QtWidgets import QDialog, QFormLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QSpinBox, QVBoxLayout, QWidget
from utl_ip import get_device_ip_addresses
from web_server import DSS_HTTP_PORT_DEFAULT, ControlApiServer

class WebServerDialog(QDialog):

    def __init__(self, server: ControlApiServer, parent: QWidget | None=None) -> None:
        super().__init__(parent)
        self.setModal(True)
        self.setWindowTitle('Web Server Info')
        self.resize(720, 320)
        self._server = server
        self._build_ui()
        self._refresh_status_label()
        self._refresh_ip_labels()
        self._refresh_toggle_btn()
        self._server.started.connect(self._on_started)

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(6)
        font = QFont()
        font.setPointSize(12)
        self.setFont(font)
        status_group = QGroupBox('Status')
        status_layout = QHBoxLayout(status_group)
        self._status_label = QLabel('(initializing)')
        status_layout.addWidget(self._status_label, stretch=1)
        port_form = QFormLayout()
        self._port_spin = QSpinBox()
        self._port_spin.setRange(1024, 65535)
        self._port_spin.setValue(DSS_HTTP_PORT_DEFAULT)
        port_form.addRow('Listen Port', self._port_spin)
        status_layout.addLayout(port_form)
        apply_btn = QPushButton('Apply')
        apply_btn.clicked.connect(self._on_apply_port)
        status_layout.addWidget(apply_btn)
        self._toggle_btn = QPushButton('Start')
        self._toggle_btn.setCheckable(True)
        self._toggle_btn.toggled.connect(self._on_toggle)
        status_layout.addWidget(self._toggle_btn)
        root.addWidget(status_group)
        ip_group = QGroupBox('Listen Targets')
        ip_layout = QVBoxLayout(ip_group)
        self._ip_labels: dict[str, QLabel] = {}
        for tier in ('global', 'tailscale', 'private', 'link_local'):
            lab = QLabel('(none)')
            ip_layout.addWidget(lab)
            self._ip_labels[tier] = lab
        root.addWidget(ip_group)

    def _refresh_status_label(self) -> None:
        if self._server.is_running():
            port = self._server.port()
            self._status_label.setText(f'Running on 0.0.0.0:{port}')
        else:
            self._status_label.setText('Stopped')

    def _refresh_toggle_btn(self) -> None:
        running = self._server.is_running()
        self._toggle_btn.blockSignals(True)
        self._toggle_btn.setChecked(running)
        self._toggle_btn.setText('Stop' if running else 'Start')
        self._toggle_btn.blockSignals(False)

    def _refresh_ip_labels(self) -> None:
        tier1, tier2, tier3, tier4 = get_device_ip_addresses()
        port = self._server.port() if self._server.is_running() else DSS_HTTP_PORT_DEFAULT
        mapping: dict[str, list[str]] = {'global': tier1, 'tailscale': tier2, 'private': tier3, 'link_local': tier4}
        title = {'global': 'Global Address', 'tailscale': 'Tailscale Address', 'private': 'Private Address', 'link_local': 'Link Local Address'}
        for tier, ips in mapping.items():
            if not ips:
                self._ip_labels[tier].setText(f'{title[tier]}: (none)')
                continue
            lines = [f'{title[tier]}:']
            lines.extend((f'  {ip}:{port}  →  http://{ip}:{port}/' for ip in ips))
            self._ip_labels[tier].setText('\n'.join(lines))

    def _on_started(self, ok: bool) -> None:
        self._refresh_toggle_btn()
        self._refresh_status_label()
        self._refresh_ip_labels()

    def _on_toggle(self, checked: bool) -> None:
        if self._server.is_running() == checked:
            return
        if checked:
            self._server.start(port=self._port_spin.value())
        else:
            self._server.stop()
            self._refresh_toggle_btn()
            self._refresh_status_label()
            self._refresh_ip_labels()

    def _on_apply_port(self) -> None:
        new_port = int(self._port_spin.value())
        self._server.restart(new_port)
        self._refresh_status_label()
        self._refresh_ip_labels()
__all__ = ['WebServerDialog']
