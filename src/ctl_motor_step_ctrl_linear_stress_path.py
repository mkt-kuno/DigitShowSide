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
from ctl_motor import DSS_MOTOR_PID_CTRL_COEFF_P, CtrlResult, is_zero
from ctl_motor_step_ctrl_class import ArgSpec, MotorStepControlClass

class MotorStepControl_LinearStressPath(MotorStepControlClass):
    NAME = 'Linear Stress Path Loading'
    DESCRIPTION = ''
    ARGS = (ArgSpec('ini_eff_axial_stress', 'kPa'), ArgSpec('ini_eff_rad_stress', 'kPa'), ArgSpec('end_eff_axial_stress', 'kPa'), ArgSpec('end_eff_rad_stress', 'kPa'), ArgSpec('cell_pressure_rate', 'kPa/min'), ArgSpec('eff_axial_stress_error_at_max_motor_speed', 'kPa'), ArgSpec('max_motor_speed', 'rpm'))

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        ini_sa = float(args[0])
        ini_sr = float(args[1])
        end_sa = float(args[2])
        end_sr = float(args[3])
        cell_rate = float(args[4])
        q_err_max = abs(float(args[5]))
        max_speed = float(args[6])
        err_air = float(self.ctx.Control.ErrorStress.air)
        err_ext = float(self.ctx.Control.ErrorStress.ext)
        err_com = float(self.ctx.Control.ErrorStress.com)
        cur_sa = self.ctx.Current.e_sa
        cur_sr = self.ctx.Current.e_sr
        step_cell_kpa = cell_rate / 60.0 * dt_sec
        if cur_sa > end_sa + err_ext and cur_sa < end_sa + err_com and (abs(cur_sr - end_sr) < err_air):
            self.motor.set_motor_speed(0.0)
            return CtrlResult.NEXT_STEP
        if cur_sr < end_sr + err_air:
            self.motor.increase_ep_cell_pressure_kpa(step_cell_kpa)
        elif cur_sr > end_sr + err_air:
            self.motor.decrease_ep_cell_pressure_kpa(step_cell_kpa)
        else:
            self.motor.add_ep_cell_pressure_kpa_signed(float(-dt_sec * DSS_MOTOR_PID_CTRL_COEFF_P * (cur_sr - end_sr)))
        self.motor.set_motor_clutch(True)
        if is_zero(ini_sr - end_sr):
            cur_target_sa = end_sa
        else:
            cur_target_sa = ini_sa + (end_sa - ini_sa) * (cur_sr - ini_sr) / (end_sr - ini_sr)
        diff = abs(cur_sa - cur_target_sa)
        target_speed = 0.0
        if cur_sa < cur_target_sa + err_ext:
            self.motor.set_motor_direction_down()
            if cur_sa < cur_target_sa - q_err_max or is_zero(q_err_max):
                target_speed = max_speed
            else:
                target_speed = max_speed * (diff / q_err_max)
        elif cur_sa > cur_target_sa + err_com:
            self.motor.set_motor_direction_up()
            if cur_sa > cur_target_sa + q_err_max or is_zero(q_err_max):
                target_speed = max_speed
            else:
                target_speed = max_speed * (diff / q_err_max)
        self.motor.set_motor_speed(target_speed)
        return CtrlResult.CONTINUE
