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
from utl_context import DSS_TORS_EP_GAIN_CNS_CYCLIC

class TorsionalStepControl_CyclicTorsionalLoadingCNS(TorsionalStepControlClass):
    cycle_num: ClassVar[int] = 0
    flag_dir: ClassVar[int] = 0
    NAME = 'Cyclic Loading w/ Cell Servo'
    DESCRIPTION = ''
    ARGS = (ArgSpec('initial_direction (0:CW / 1:CCW)'), ArgSpec('szq_lower_limit', 'kPa'), ArgSpec('szq_upper_limit', 'kPa'), ArgSpec('gzq1_lower_limit', '-'), ArgSpec('gzq1_upper_limit', '-'), ArgSpec('cycle_number', 'cycles'), ArgSpec('torsional_speed', 'rpm'), ArgSpec('axial_speed', 'rpm'), ArgSpec('cell_pressure_rate', 'kPa/min'), ArgSpec('target_eff_axial_stress', 'kPa'), ArgSpec('target_eff_rad_stress', 'kPa'))

    @classmethod
    def reset_cycle(cls) -> None:
        cls.cycle_num = 0
        cls.flag_dir = 0

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        a0 = float(args[0])
        a1 = float(args[1])
        a2 = float(args[2])
        a3 = float(args[3])
        a4 = float(args[4])
        a5 = float(args[5])
        a6 = float(args[6])
        axis_speed = float(args[7])
        cell_rate = float(args[8])
        target_sz = float(args[9])
        target_sr = float(args[10])
        cur_sz = self.ctx.Current.e_sa
        cur_sr = self.ctx.Current.e_sr
        cur_szq = self.ctx.Current.tau
        cur_gzq = self.ctx.Current.gamma
        err_air = float(self.ctx.Control.ErrorStress.air)
        err_ext = abs(float(self.ctx.Control.ErrorStress.ext))
        cls = type(self)
        if cls.cycle_num == 0:
            cls.flag_dir = 0 if is_zero(a0) else 1
            cls.cycle_num = 1
        self.motor.set_motor_clutch(True)
        if cur_sz > target_sz + err_ext:
            self.motor.set_motor_direction_up()
            self.motor.set_motor_speed(axis_speed)
        elif cur_sz < target_sz - err_ext:
            self.motor.set_motor_direction_down()
            self.motor.set_motor_speed(axis_speed)
        else:
            self.motor.set_motor_speed(0.0)
        if is_zero(cell_rate):
            self.motor.add_ep_cell_pressure_kpa_signed(0.0)
        elif cur_sr < target_sr - err_air:
            self.motor.add_ep_cell_pressure_kpa_signed(+cell_rate * dt_sec / 60.0)
        elif cur_sr > target_sr + err_air:
            self.motor.add_ep_cell_pressure_kpa_signed(-cell_rate * dt_sec / 60.0)
        else:
            target_sr_aim = approach_by_rate(cur_sr, target_sr, cell_rate, dt_sec)
            servo_ep_cell_proportional(self.motor, target_sr_aim, DSS_TORS_EP_GAIN_CNS_CYCLIC, dt_sec)
        self.torsional.set_torsional_clutch(True)
        if cls.cycle_num != 0 and cls.cycle_num <= a5:
            if is_zero(a0):
                if cls.flag_dir == 0:
                    self.torsional.set_torsional_direction_cw()
                    self.torsional.set_torsional_speed(a6)
                    if cur_szq >= a2 or cur_gzq >= a4:
                        cls.flag_dir = 1
                elif cls.flag_dir == 1:
                    self.torsional.set_torsional_direction_ccw()
                    self.torsional.set_torsional_speed(a6)
                    if cur_szq <= a1 or cur_gzq <= a3:
                        cls.flag_dir = 0
                        cls.cycle_num += 1
            elif cls.flag_dir == 1:
                self.torsional.set_torsional_direction_cw()
                self.torsional.set_torsional_speed(a6)
                if cur_szq >= a2 or cur_gzq >= a4:
                    cls.flag_dir = 0
            elif cls.flag_dir == 0:
                self.torsional.set_torsional_direction_ccw()
                self.torsional.set_torsional_speed(a6)
                if cur_szq <= a1 or cur_gzq <= a3:
                    cls.flag_dir = 1
                    cls.cycle_num += 1
        if cls.cycle_num > a5:
            self.torsional.set_torsional_speed(0.0)
            cls.cycle_num = 0
            cls.flag_dir = 0
            return CtrlResult.NEXT_STEP
        return CtrlResult.CONTINUE
