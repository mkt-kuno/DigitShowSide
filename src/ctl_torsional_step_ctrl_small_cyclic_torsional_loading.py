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

class TorsionalStepControl_SmallCyclicTorsionalLoading(TorsionalStepControlClass):
    cycle_num: ClassVar[int] = 0
    flag_phase: ClassVar[int] = 0
    target_gamma: ClassVar[float] = 0.0
    NAME = 'Small Cyclic (gamma)'
    DESCRIPTION = ''
    ARGS = (ArgSpec('reference_direction (0:CW / 1:CCW)'), ArgSpec('delta_gamma', '-'), ArgSpec('cycle_number', 'cycles'), ArgSpec('torsional_speed', 'rpm'))

    @classmethod
    def reset_cycle(cls) -> None:
        cls.cycle_num = 0
        cls.flag_phase = 0
        cls.target_gamma = 0.0

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        a0 = float(args[0])
        delta = float(args[1])
        a2 = float(args[2])
        a3 = float(args[3])
        cur_gamma = self.ctx.Current.gamma
        cls = type(self)
        if cls.cycle_num == 0:
            cls.target_gamma = cur_gamma
            cls.cycle_num = 1
            cls.flag_phase = 0
        self.torsional.set_torsional_clutch(True)
        if cls.cycle_num != 0 and cls.cycle_num <= a2:
            if is_zero(a0):
                if cls.flag_phase == 0:
                    self.torsional.set_torsional_direction_ccw()
                    self.torsional.set_torsional_speed(a3)
                    if cur_gamma <= cls.target_gamma - delta:
                        cls.flag_phase = 1
                elif cls.flag_phase == 1:
                    self.torsional.set_torsional_direction_cw()
                    self.torsional.set_torsional_speed(a3)
                    if cur_gamma >= cls.target_gamma + delta:
                        cls.flag_phase = 2
                elif cls.flag_phase == 2:
                    self.torsional.set_torsional_direction_ccw()
                    self.torsional.set_torsional_speed(a3)
                    if cur_gamma <= cls.target_gamma:
                        cls.flag_phase = 0
                        cls.cycle_num += 1
            elif cls.flag_phase == 0:
                self.torsional.set_torsional_direction_cw()
                self.torsional.set_torsional_speed(a3)
                if cur_gamma >= cls.target_gamma + delta:
                    cls.flag_phase = 1
            elif cls.flag_phase == 1:
                self.torsional.set_torsional_direction_ccw()
                self.torsional.set_torsional_speed(a3)
                if cur_gamma <= cls.target_gamma - delta:
                    cls.flag_phase = 2
            elif cls.flag_phase == 2:
                self.torsional.set_torsional_direction_cw()
                self.torsional.set_torsional_speed(a3)
                if cur_gamma >= cls.target_gamma:
                    cls.flag_phase = 0
                    cls.cycle_num += 1
        if cls.cycle_num > a2:
            self.torsional.set_torsional_speed(0.0)
            cls.cycle_num = 0
            cls.flag_phase = 0
            cls.target_gamma = 0.0
            return CtrlResult.NEXT_STEP
        return CtrlResult.CONTINUE
TorsionalStepControl_SmallCyclic = TorsionalStepControl_SmallCyclicTorsionalLoading
