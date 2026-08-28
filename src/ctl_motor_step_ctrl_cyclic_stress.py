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
from typing import ClassVar
import numpy as np
from ctl_motor import CtrlResult
from ctl_motor_step_ctrl_class import ArgSpec, MotorStepControlClass, run_cyclic_4quadrant
from utl_context import CyclicState

class MotorStepControl_CyclicStress(MotorStepControlClass):
    cycle_num: ClassVar[int] = 0
    cycle_state: ClassVar[CyclicState] = CyclicState.INIT
    NAME = 'Cyclic Axial Loading Stress'
    DESCRIPTION = ''
    ARGS = (ArgSpec('load_dir (0:compression / 1:extension)'), ArgSpec('motor_speed', 'rpm'), ArgSpec('q_lower_limit', 'kPa'), ArgSpec('q_upper_limit', 'kPa'), ArgSpec('cycle_number', 'cycles'), ArgSpec('eff_rad_stress (positive value enables radial PID)', 'kPa'))

    @classmethod
    def reset_cycle(cls) -> None:
        cls.cycle_num = 0
        cls.cycle_state = CyclicState.INIT

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        new_num, new_state, result = run_cyclic_4quadrant(self.motor, self.ctx, type(self).cycle_num, type(self).cycle_state, args, dt_sec, lambda ctx: ctx.Current.q)
        type(self).cycle_num = new_num
        type(self).cycle_state = new_state
        return result
