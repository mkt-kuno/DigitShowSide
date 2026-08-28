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
from ctl_motor_step_ctrl_class import ArgSpec, MotorStepControlClass
from utl_context import DSS_AO_CH_EP_AXIS, DSS_AO_CH_EP_CELL

class MotorStepControl_Stop(MotorStepControlClass):
    NAME = 'Stop'
    DESCRIPTION = ''
    ARGS = (ArgSpec('motor_off (0:do nothing / 1:turn off)'), ArgSpec('motor_up (0:do nothing / 1:up direction)'), ArgSpec('motor_speed_zero (0:do nothing / 1:set speed zero)'), ArgSpec('ep_cell_zero (0:do nothing / 1:set cell pressure zero)'), ArgSpec('ep_axis_zero (0:do nothing / 1:set axis pressure zero)'))

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        if is_one(float(args[0])):
            self.motor.set_motor_clutch(False)
        if is_one(float(args[1])):
            self.motor.set_motor_direction_up()
        if is_one(float(args[2])):
            self.motor.set_motor_speed(0.0)
        if is_one(float(args[3])):
            self.ctx.AIO.AO.row(DSS_AO_CH_EP_CELL).raw = 0.0
        if is_one(float(args[4])):
            self.ctx.AIO.AO.row(DSS_AO_CH_EP_AXIS).raw = 0.0
        return CtrlResult.STOP
