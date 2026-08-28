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
from ctl_motor import CtrlResult, is_one
from ctl_torsional_step_ctrl_class import ArgSpec, TorsionalStepControlClass

class TorsionalStepControl_Stop(TorsionalStepControlClass):
    NAME = 'Stop'
    DESCRIPTION = 'Motor 版の Stop を両軸 (軸+ねじり) に拡張'
    ARGS = (ArgSpec('motor_off (0:do nothing / 1:disengage both clutches)'), ArgSpec('motor_up (0:do nothing / 1:axis up + torsion CW direction)'), ArgSpec('motor_speed_zero (0:do nothing / 1:set both speeds zero)'), ArgSpec('ep_cell_zero (0:do nothing / 1:set cell pressure zero)'), ArgSpec('ep_axis_zero (0:do nothing / 1:set axis pressure zero)'))

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        if is_one(float(args[0])):
            self.motor.set_motor_clutch(False)
            self.torsional.set_torsional_clutch(False)
        if is_one(float(args[1])):
            self.motor.set_motor_direction_up()
            self.torsional.set_torsional_direction_cw()
        if is_one(float(args[2])):
            self.motor.set_motor_speed(0.0)
            self.torsional.set_torsional_speed(0.0)
        if is_one(float(args[3])):
            self.motor.set_ep_cell_pressure_kpa(0.0)
        if is_one(float(args[4])):
            self.motor.set_ep_axial_pressure_n(0.0)
        return CtrlResult.STOP
