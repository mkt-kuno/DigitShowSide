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
from ctl_motor import CtrlResult, StepLoadDir, is_one, is_zero, resolve_load_dir
from ctl_motor_step_ctrl_class import ArgSpec, MotorStepControlClass

class MotorStepControl_Monotonic(MotorStepControlClass):
    NAME = 'Monotonic Axial Loading'
    DESCRIPTION = ''
    ARGS = (ArgSpec('load_dir (0:compression / 1:extension)'), ArgSpec('motor_speed', 'rpm'), ArgSpec('eff_rad_stress (positive value enables radial PID)', 'kPa'), ArgSpec('enable_axial_strain_limiter (0:Disable / 1:Enable)'), ArgSpec('axial_strain_limit', '%'), ArgSpec('enable_q_limiter (0:Disable / 1:Enable)'), ArgSpec('q_limit', 'kPa'))

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        load_dir = resolve_load_dir(float(args[0]))
        speed = float(args[1])
        eff_rad = float(args[2])
        en_eps = float(args[3])
        eps_limit = float(args[4])
        en_q = float(args[5])
        q_limit = float(args[6])
        if is_zero(en_eps) and is_zero(en_q):
            return CtrlResult.STOP
        if eff_rad > 0.0:
            self.motor._apply_radial_pressure_pid(eff_rad, dt_sec)
        self.motor.set_motor_clutch(True)
        if load_dir is StepLoadDir.COMPRESSION:
            if is_one(en_eps) and self.ctx.Current.ea >= eps_limit:
                return CtrlResult.NEXT_STEP
            if is_one(en_q) and self.ctx.Current.q >= q_limit:
                return CtrlResult.NEXT_STEP
            self.motor.set_motor_direction_down()
            self.motor.set_motor_speed(speed)
        elif load_dir is StepLoadDir.EXTENSION:
            if is_one(en_eps) and self.ctx.Current.ea <= eps_limit:
                return CtrlResult.NEXT_STEP
            if is_one(en_q) and self.ctx.Current.q <= q_limit:
                return CtrlResult.NEXT_STEP
            self.motor.set_motor_direction_up()
            self.motor.set_motor_speed(speed)
        else:
            return CtrlResult.STOP
        return CtrlResult.CONTINUE
