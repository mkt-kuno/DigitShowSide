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
import importlib
import importlib.metadata
import os
import sys
from PySide6.QtCore import QProcess, Qt
from PySide6.QtGui import QFontDatabase
from PySide6.QtWidgets import QDialog, QLabel, QVBoxLayout, QWidget
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utl_context import DSS_APP_NAME as APP_NAME, DSS_APP_VERSION as APP_VERSION

def _safe_version(name: str) -> str:
    try:
        return importlib.metadata.version(name)
    except Exception:
        try:
            mod = importlib.import_module(name)
            return str(getattr(mod, '__version__', '?'))
        except Exception:
            return '?'

def _git_info() -> dict[str, str]:
    def _run(args: list[str]) -> str:
        proc = QProcess()
        proc.start('git', args)
        if proc.waitForFinished(1000) and proc.exitCode() == 0:
            return bytes(proc.readAllStandardOutput()).decode('utf-8', 'replace').strip()
        return ''

    remote = _run(['config', '--get', 'remote.origin.url']) or _run(['remote', 'get-url', 'origin'])
    branch = _run(['rev-parse', '--abbrev-ref', 'HEAD'])
    commit = _run(['rev-parse', '--short', 'HEAD'])
    status = _run(['status', '--porcelain'])
    dirty = 'dirty' if status else 'clean'

    return {
        'remote': remote or 'unknown',
        'branch': branch or 'unknown',
        'commit': commit or 'unknown',
        'dirty': dirty,
    }

class VersionDialog(QDialog):

    def __init__(self, parent: QWidget | None=None) -> None:
        super().__init__(parent)
        self.setModal(True)
        self.setWindowTitle(APP_NAME)
        self.resize(640, 480)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(0)
        np_ver = _safe_version('numpy')
        qt_ver = _safe_version('PySide6')
        pymodbus_ver = _safe_version('pymodbus')
        pyserial_ver = _safe_version('pyserial')
        pyqtgraph_ver = _safe_version('pyqtgraph')
        git = _git_info()
        git_branch = git['branch']
        git_commit = git['commit']
        git_dirty = git['dirty']
        git_remote = git['remote']
        text = '\n'.join([
            f'{APP_NAME} Information',
            '',
            f'Version:\tv{APP_VERSION}',
            f'Git Branch:\t{git_branch}',
            f'Git Commit:\t{git_commit} ({git_dirty})',
            f'Git Remote:\t{git_remote}',
            '',
            'Development Team:',
            '\tMakoto KUNO (Orchestrator)',
            '\tKimi K3 (Orchestrator)',
            '\tDeepseek V4 Pro (Lead Programmer)',
            '\tDeepseek V4 Flash(Lead Programmer)',
            '\tMinimax M3 (Lead Programmer)',
            '\tGLM-5.3-Flash (Sub Programmer)',
            '\tQwen3.8 27B (Sub Programmer)',
            '\tQwen3.6 30B-A3B (Sub Programmer)',
            '\tGemini3.1 Pro (UI Designer)',
            '\tGemini3.7 Flash (UI Designer)',
            '',
            'License:',
            f'\t{APP_NAME}\tv{APP_VERSION}\t[GPL-3.0]',
            f'\tnumpy\t\tv{np_ver}\t[BSD-3]',
            f'\tPySide6\t\tv{qt_ver}\t[LGPL]',
            f'\tpymodbus\tv{pymodbus_ver}\t[MIT]',
            f'\tpyserial\tv{pyserial_ver}\t[BSD-3]',
            f'\tpyqtgraph\tv{pyqtgraph_ver}\t[MIT]',
        ])
        mono = QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont)
        mono.setPointSize(11)
        label = QLabel(text)
        label.setFont(mono)
        label.setTextInteractionFlags(Qt.TextInteractionFlag.NoTextInteraction)
        label.setAlignment(Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        label.setWordWrap(False)
        layout.addWidget(label)

