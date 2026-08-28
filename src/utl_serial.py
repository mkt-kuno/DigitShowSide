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

import os
import sys
from serial.tools import list_ports
from serial.tools.list_ports_common import ListPortInfo
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
KNOWN_DEVICES = [('2341', '0069', 95, 'Arduino UNO R4 Minima'), ('2341', '0074', 95, 'Arduino Nano R4'), ('2341', '0243', 95, 'Arduino Nano ESP32'), ('2341', None, 90, 'Arduino'), ('2E8A', '000A', 90, 'Raspberry Pi Pico (CDC)'), ('2E8A', '0005', 88, 'Raspberry Pi Pico (MicroPython)'), ('2E8A', None, 86, 'Raspberry Pi (RP2040)'), ('0483', '5740', 85, 'STM32 CDC ACM'), ('0483', '374B', 82, 'STM32 ST-LINK CDC'), ('1A86', '7523', 80, 'CH340'), ('1A86', '55D3', 80, 'CH340C'), ('1A86', '7522', 80, 'CH341'), ('1A86', None, 78, 'CH340/CH341 (any)'), ('10C4', 'EA60', 80, 'CP2102/CP2104'), ('10C4', None, 78, 'CP210x (any)'), ('0403', '6001', 80, 'FTDI FT232'), ('0403', '6015', 80, 'FTDI FT234X'), ('0403', None, 78, 'FTDI (any)')]

def _score_port(port: ListPortInfo) -> tuple[int, str]:
    if port.vid is None or port.pid is None:
        return (-1, '')
    vid_hex = f'{port.vid:04X}'
    pid_hex = f'{port.pid:04X}'
    best = (-1, '')
    for vid, pid, priority, name in KNOWN_DEVICES:
        if vid.upper() != vid_hex:
            continue
        if (pid is None or pid.upper() == pid_hex) and priority > best[0]:
            best = (priority, name)
    return best

def find_arduino_like_port() -> tuple[int, ListPortInfo, str] | None:
    candidates = []
    for port in list_ports.comports():
        priority, name = _score_port(port)
        if priority >= 0:
            candidates.append((priority, port, name))
    if not candidates:
        return None
    candidates.sort(key=lambda c: c[0], reverse=True)
    return candidates[0]
