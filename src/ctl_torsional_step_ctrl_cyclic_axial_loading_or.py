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

class TorsionalStepControl_CyclicAxialLoadingOR(TorsionalStepControlClass):
    NAME = 'Cyclic Axial AND/OR'
    DESCRIPTION = ''
    ARGS = (ArgSpec('first_direction (0:Loading / 1:Unloading)'), ArgSpec('sigma_z_lower', 'kPa'), ArgSpec('sigma_z_upper', 'kPa'), ArgSpec('epsilon_z_lower', '%'), ArgSpec('epsilon_z_upper', '%'), ArgSpec('cycle_number', 'cycles'), ArgSpec('axial_motor_speed', 'rpm'))
    cycle_num: ClassVar[int] = 0
    flag_cyclic: ClassVar[bool] = False

    @classmethod
    def reset_cycle(cls) -> None:
        cls.cycle_num = 0
        cls.flag_cyclic = False

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        direction = float(args[0])
        sz_lower = float(args[1])
        sz_upper = float(args[2])
        ez_lower = float(args[3])
        ez_upper = float(args[4])
        cycles = float(args[5])
        axis_speed = float(args[6])
        cur_sz = self.ctx.Current.e_sa
        cur_ez = self.ctx.Current.ea
        if is_zero(direction):
            if type(self).cycle_num == 0:
                type(self).flag_cyclic = False
                type(self).cycle_num = 1
            self.motor.set_motor_clutch(True)
            self.motor.set_motor_speed(axis_speed)
            if type(self).cycle_num != 0 and type(self).cycle_num <= cycles:
                if not type(self).flag_cyclic:
                    if cur_sz <= sz_upper and cur_ez <= ez_upper:
                        self.motor.set_motor_direction_down()
                    else:
                        type(self).flag_cyclic = True
                elif cur_sz >= sz_lower or cur_ez >= ez_lower:
                    self.motor.set_motor_direction_up()
                else:
                    type(self).flag_cyclic = False
                    type(self).cycle_num += 1
            if type(self).cycle_num > cycles:
                type(self).reset_cycle()
                return CtrlResult.NEXT_STEP
            return CtrlResult.CONTINUE
        if is_zero(direction - 1.0):
            if type(self).cycle_num == 0:
                type(self).flag_cyclic = True
                type(self).cycle_num = 1
            self.motor.set_motor_clutch(True)
            self.motor.set_motor_speed(axis_speed)
            if type(self).cycle_num != 0 and type(self).cycle_num <= cycles:
                if not type(self).flag_cyclic:
                    if cur_sz <= sz_upper or cur_ez <= ez_upper:
                        self.motor.set_motor_direction_down()
                    else:
                        type(self).flag_cyclic = True
                        type(self).cycle_num += 1
                elif cur_sz >= sz_lower and cur_ez >= ez_lower:
                    self.motor.set_motor_direction_up()
                else:
                    type(self).flag_cyclic = False
            if type(self).cycle_num > cycles:
                type(self).reset_cycle()
                return CtrlResult.NEXT_STEP
            return CtrlResult.CONTINUE
        self.motor.set_motor_speed(0.0)
        return CtrlResult.STOP
