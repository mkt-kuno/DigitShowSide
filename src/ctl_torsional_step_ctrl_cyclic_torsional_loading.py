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
from ctl_motor import CtrlResult, is_zero
from ctl_torsional_step_ctrl_class import ArgSpec, TorsionalStepControlClass

class TorsionalStepControl_CyclicTorsionalLoading(TorsionalStepControlClass):
    cycle_num: ClassVar[int] = 0
    flag_dir: ClassVar[int] = 0
    NAME = 'Cyclic Loading (szq/gzq)'
    DESCRIPTION = ''
    ARGS = (ArgSpec('initial_direction (0:CW / 1:CCW)'), ArgSpec('szq_lower_limit', 'kPa'), ArgSpec('szq_upper_limit', 'kPa'), ArgSpec('gzq1_lower_limit', '-'), ArgSpec('gzq1_upper_limit', '-'), ArgSpec('cycle_number', 'cycles'), ArgSpec('torsional_speed', 'rpm'))

    @classmethod
    def reset_cycle(cls) -> None:
        cls.cycle_num = 0
        cls.flag_dir = 0

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        a0 = float(args[0])
        a1 = float(args[1])
        a2 = float(args[2])
        a3 = float(args[3])
        a4 = float(args[4])
        a5 = float(args[5])
        a6 = float(args[6])
        cur_szq = self.ctx.Current.tau
        cur_gzq = self.ctx.Current.gamma
        cls = type(self)
        if cls.cycle_num == 0:
            cls.flag_dir = 0 if is_zero(a0) else 1
            cls.cycle_num = 1
        self.torsional.set_torsional_clutch(True)
        if cls.cycle_num != 0 and cls.cycle_num <= a5:
            if is_zero(a0):
                if cls.flag_dir == 0:
                    self.torsional.set_torsional_direction_cw()
                    self.torsional.set_torsional_speed(a6)
                    if cur_szq >= a2 or cur_gzq >= a4:
                        cls.flag_dir = 1
                elif cls.flag_dir == 1:
                    self.torsional.set_torsional_direction_ccw()
                    self.torsional.set_torsional_speed(a6)
                    if cur_szq <= a1 or cur_gzq <= a3:
                        cls.flag_dir = 0
                        cls.cycle_num += 1
            elif cls.flag_dir == 1:
                self.torsional.set_torsional_direction_cw()
                self.torsional.set_torsional_speed(a6)
                if cur_szq >= a2 or cur_gzq >= a4:
                    cls.flag_dir = 0
            elif cls.flag_dir == 0:
                self.torsional.set_torsional_direction_ccw()
                self.torsional.set_torsional_speed(a6)
                if cur_szq <= a1 or cur_gzq <= a3:
                    cls.flag_dir = 1
                    cls.cycle_num += 1
        if cls.cycle_num > a5:
            self.torsional.set_torsional_speed(0.0)
            cls.cycle_num = 0
            cls.flag_dir = 0
            return CtrlResult.NEXT_STEP
        return CtrlResult.CONTINUE
TorsionalStepControl_CyclicStress = TorsionalStepControl_CyclicTorsionalLoading
TorsionalStepControl_CyclicStrain = TorsionalStepControl_CyclicTorsionalLoading
