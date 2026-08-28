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
from ctl_motor import CtrlResult, is_zero
from ctl_motor_step_ctrl_class import ArgSpec, MotorStepControlClass

class MotorStepControl_Creep(MotorStepControlClass):
    NAME = 'Creep'
    DESCRIPTION = ''
    ARGS = (ArgSpec('q', 'kPa'), ArgSpec('q_error_at_max_motor_speed', 'kPa'), ArgSpec('max_motor_speed', 'rpm'), ArgSpec('duration_time', 'min'), ArgSpec('eff_rad_stress (positive value enables radial PID)', 'kPa'))

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        q_target = float(args[0])
        q_err_max = abs(float(args[1]))
        max_speed = float(args[2])
        duration_min = float(args[3])
        eff_rad = float(args[4])
        elapsed = self.ctx.Control.watch.get_elapsed_sec()
        if elapsed > duration_min * 60.0:
            self.motor.set_motor_speed(0.0)
            return CtrlResult.NEXT_STEP
        if eff_rad > 0.0:
            self.motor._apply_radial_pressure_pid(eff_rad, dt_sec)
        cur_q = self.ctx.Current.q
        diff = abs(cur_q - q_target)
        err_com = float(self.ctx.Control.ErrorStress.com)
        err_ext = float(self.ctx.Control.ErrorStress.ext)
        self.motor.set_motor_clutch(True)
        target_speed = 0.0
        if cur_q < q_target + err_ext:
            self.motor.set_motor_direction_down()
            if cur_q < q_target - q_err_max or is_zero(q_err_max):
                target_speed = max_speed
            else:
                target_speed = max_speed * (diff / q_err_max)
        elif cur_q > q_target + err_com:
            self.motor.set_motor_direction_up()
            if cur_q > q_target + q_err_max or is_zero(q_err_max):
                target_speed = max_speed
            else:
                target_speed = max_speed * (diff / q_err_max)
        self.motor.set_motor_speed(target_speed)
        return CtrlResult.CONTINUE
