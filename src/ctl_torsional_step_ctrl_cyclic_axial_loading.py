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
from ctl_motor import CtrlResult, is_one, is_zero
from ctl_torsional_step_ctrl_class import ArgSpec, TorsionalStepControlClass

class TorsionalStepControl_CyclicAxialLoading(TorsionalStepControlClass):
    cycle_num: ClassVar[int] = 0
    flag_phase: ClassVar[int] = 0
    NAME = 'Cyclic Axial'
    DESCRIPTION = ''
    ARGS = (ArgSpec('load_dir (0:Loading / 1:Unloading)'), ArgSpec('sigma_z_lower', 'kPa'), ArgSpec('sigma_z_upper', 'kPa'), ArgSpec('epsilon_z_lower', '%'), ArgSpec('epsilon_z_upper', '%'), ArgSpec('cycle_number', 'cycles'), ArgSpec('axial_motor_speed', 'rpm'))

    @classmethod
    def reset_cycle(cls) -> None:
        cls.cycle_num = 0
        cls.flag_phase = 0

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        load_dir = float(args[0])
        sz_lo = float(args[1])
        sz_hi = float(args[2])
        ez_lo = float(args[3])
        ez_hi = float(args[4])
        cycles = float(args[5])
        axis_speed = float(args[6])
        cur_sz = self.ctx.Current.e_sa
        cur_ez = self.ctx.Current.ea
        self.motor.set_motor_clutch(True)
        self.motor.set_motor_speed(axis_speed)
        if is_zero(load_dir):
            if type(self).cycle_num == 0:
                type(self).flag_phase = 0
                type(self).cycle_num = 1
            if type(self).cycle_num != 0 and type(self).cycle_num <= cycles:
                if type(self).flag_phase == 0:
                    if cur_sz <= sz_hi and cur_ez <= ez_hi:
                        self.motor.set_motor_direction_down()
                    else:
                        type(self).flag_phase = 1
                elif type(self).flag_phase == 1:
                    if cur_sz >= sz_lo and cur_ez >= ez_lo:
                        self.motor.set_motor_direction_up()
                    else:
                        type(self).flag_phase = 0
                        type(self).cycle_num = type(self).cycle_num + 1
            if type(self).cycle_num > cycles:
                self.motor.set_motor_speed(0.0)
                type(self).cycle_num = 0
                type(self).flag_phase = 0
                return CtrlResult.NEXT_STEP
            return CtrlResult.CONTINUE
        if is_one(load_dir):
            if type(self).cycle_num == 0:
                type(self).flag_phase = 1
                type(self).cycle_num = 1
            if type(self).cycle_num != 0 and type(self).cycle_num <= cycles:
                if type(self).flag_phase == 0:
                    if cur_sz <= sz_hi and cur_ez <= ez_hi:
                        self.motor.set_motor_direction_down()
                    else:
                        type(self).flag_phase = 1
                        type(self).cycle_num = type(self).cycle_num + 1
                elif type(self).flag_phase == 1:
                    if cur_sz >= sz_lo and cur_ez >= ez_lo:
                        self.motor.set_motor_direction_up()
                    else:
                        type(self).flag_phase = 0
            if type(self).cycle_num > cycles:
                self.motor.set_motor_speed(0.0)
                type(self).cycle_num = 0
                type(self).flag_phase = 0
                return CtrlResult.NEXT_STEP
            return CtrlResult.CONTINUE
        self.motor.set_motor_speed(0.0)
        return CtrlResult.STOP
