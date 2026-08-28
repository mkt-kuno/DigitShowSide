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
from ctl_torsional_step_ctrl_class import ArgSpec, TorsionalStepControlClass, approach_by_rate, servo_ep_cell_proportional
from utl_context import DSS_TORS_EP_GAIN_CREEP

class TorsionalStepControl_Creep(TorsionalStepControlClass):
    NAME = 'Creep'
    DESCRIPTION = ''
    ARGS = (ArgSpec('target_sigma_z', 'kPa'), ArgSpec('target_sigma_r', 'kPa'), ArgSpec('target_sigma_zq', 'kPa'), ArgSpec('duration_time', 'min'), ArgSpec('torsional_motor_speed', 'rpm'), ArgSpec('axial_motor_speed', 'rpm'), ArgSpec('cell_pressure_rate', 'kPa/min'))
    step_time_min: ClassVar[float] = 0.0

    @classmethod
    def reset_cycle(cls) -> None:
        cls.step_time_min = 0.0

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        target_sz = float(args[0])
        target_sr = float(args[1])
        target_szq = float(args[2])
        duration_min = float(args[3])
        tors_speed = float(args[4])
        axis_speed = float(args[5])
        cell_rate = float(args[6])
        cur_sz = self.ctx.Current.e_sa
        cur_sr = self.ctx.Current.e_sr
        cur_szq = self.ctx.Current.tau
        err_motor = abs(float(self.ctx.Control.ErrorStress.ext))
        err_air = float(self.ctx.Control.ErrorStress.air)
        type(self).step_time_min += dt_sec / 60.0
        self.torsional.set_torsional_clutch(True)
        if cur_szq > target_szq + err_motor:
            self.torsional.set_torsional_direction_ccw()
            self.torsional.set_torsional_speed(tors_speed)
        elif cur_szq < target_szq - err_motor:
            self.torsional.set_torsional_direction_cw()
            self.torsional.set_torsional_speed(tors_speed)
        else:
            self.torsional.set_torsional_speed(0.0)
        self.motor.set_motor_clutch(True)
        if cur_sz > target_sz + err_motor:
            self.motor.set_motor_direction_up()
            self.motor.set_motor_speed(axis_speed)
        elif cur_sz < target_sz - err_motor:
            self.motor.set_motor_direction_down()
            self.motor.set_motor_speed(axis_speed)
        else:
            self.motor.set_motor_speed(0.0)
        if not is_zero(cell_rate):
            if abs(cur_sr - target_sr) > err_air:
                sign = 1.0 if cur_sr < target_sr else -1.0
                self.motor.add_ep_cell_pressure_kpa_signed(sign * cell_rate * dt_sec / 60.0)
            else:
                target_sr_approach = approach_by_rate(cur_sr, target_sr, cell_rate, dt_sec)
                servo_ep_cell_proportional(self.motor, target_sr_approach, DSS_TORS_EP_GAIN_CREEP, dt_sec)
        if type(self).step_time_min >= duration_min:
            type(self).step_time_min = 0.0
            return CtrlResult.NEXT_STEP
        return CtrlResult.CONTINUE
