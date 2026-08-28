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
import numpy as np
from ctl_motor import CtrlResult, is_one, is_zero
from ctl_torsional_step_ctrl_class import ArgSpec, TorsionalStepControlClass

class TorsionalStepControl_MonotonicAxialLoading(TorsionalStepControlClass):
    NAME = 'Monotonic Axial'
    DESCRIPTION = ''
    ARGS = (ArgSpec('load_dir (0:Loading / 1:Unloading)'), ArgSpec('target_sigma_z', 'kPa'), ArgSpec('target_epsilon_z', '%'), ArgSpec('axial_motor_speed', 'rpm'))

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        load_dir = float(args[0])
        target_sz = float(args[1])
        target_ez = float(args[2])
        axis_speed = float(args[3])
        cur_sz = self.ctx.Current.e_sa
        cur_ez = self.ctx.Current.ea
        self.motor.set_motor_clutch(True)
        if is_zero(load_dir):
            if cur_sz <= target_sz and cur_ez < target_ez:
                self.motor.set_motor_direction_down()
                self.motor.set_motor_speed(axis_speed)
                return CtrlResult.CONTINUE
            self.motor.set_motor_speed(0.0)
            return CtrlResult.NEXT_STEP
        if is_one(load_dir):
            if cur_sz >= target_sz and cur_ez > target_ez:
                self.motor.set_motor_direction_up()
                self.motor.set_motor_speed(axis_speed)
                return CtrlResult.CONTINUE
            self.motor.set_motor_speed(0.0)
            return CtrlResult.NEXT_STEP
        self.motor.set_motor_speed(0.0)
        return CtrlResult.STOP
