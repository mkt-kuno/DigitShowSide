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
import ipaddress
import socket
from typing import cast
_TAILSCALE_CGNAT: ipaddress.IPv4Network = cast('ipaddress.IPv4Network', ipaddress.ip_network('100.64.0.0/10'))
_Tiers = tuple[list[str], list[str], list[str], list[str]]

def _classify(raw: str, tiers: _Tiers) -> None:
    tier1, tier2, tier3, tier4 = tiers
    try:
        ip = ipaddress.ip_address(raw)
    except ValueError:
        return
    if ip.version != 4 or ip.is_loopback:
        return
    if ip in _TAILSCALE_CGNAT:
        tier2.append(raw)
    elif ip.is_link_local:
        tier4.append(raw)
    elif ip.is_private:
        tier3.append(raw)
    else:
        tier1.append(raw)

def _candidate_addresses() -> list[str]:
    try:
        from PySide6.QtNetwork import QAbstractSocket, QNetworkInterface
    except ImportError:
        pass
    else:
        addresses = [addr.toString() for addr in QNetworkInterface.allAddresses() if addr.protocol() == QAbstractSocket.NetworkLayerProtocol.IPv4Protocol]
        if addresses:
            return addresses
    try:
        _, _, candidates = socket.gethostbyname_ex(socket.gethostname())
    except OSError:
        return []
    return list(candidates)

def get_device_ip_addresses() -> _Tiers:
    tiers: _Tiers = ([], [], [], [])
    for raw in _candidate_addresses():
        _classify(raw, tiers)
    return tiers
__all__ = ['get_device_ip_addresses']
