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
from ctl_torsional import _clamp
from ctl_torsional_step_ctrl_class import ArgSpec, TorsionalStepControlClass, approach_by_rate
from utl_context import DSS_TORS_EXIT_BAND_FACTOR

class TorsionalStepControl_FileControlableConsolidation(TorsionalStepControlClass):
    NAME = 'Consolidation'
    DESCRIPTION = ''
    ARGS = (ArgSpec('initial_sigma_z', 'kPa'), ArgSpec('initial_sigma_r', 'kPa'), ArgSpec('initial_sigma_zq', 'kPa'), ArgSpec('target_sigma_z', 'kPa'), ArgSpec('target_sigma_r', 'kPa'), ArgSpec('target_sigma_zq', 'kPa'), ArgSpec('axial_motor_speed', 'rpm'), ArgSpec('torsional_motor_speed', 'rpm'), ArgSpec('cell_pressure_rate', 'kPa/min'))
    step_time_min: ClassVar[float] = 0.0
    target_sr_state: ClassVar[float] = 0.0

    @classmethod
    def reset_cycle(cls) -> None:
        cls.step_time_min = 0.0
        cls.target_sr_state = 0.0

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        ini_sz = float(args[0])
        ini_sr = float(args[1])
        ini_szq = float(args[2])
        end_sz = float(args[3])
        end_sr = float(args[4])
        end_szq = float(args[5])
        axis_speed = float(args[6])
        tors_speed = float(args[7])
        cell_rate = float(args[8])
        cur_sz = self.ctx.Current.e_sa
        cur_sr = self.ctx.Current.e_sr
        cur_szq = self.ctx.Current.tau
        err_motor = abs(float(self.ctx.Control.ErrorStress.ext))
        exit_band = DSS_TORS_EXIT_BAND_FACTOR * err_motor
        type(self).step_time_min += dt_sec / 60.0
        if cell_rate > 0.0:
            type(self).target_sr_state = approach_by_rate(cur_sr, end_sr, cell_rate, dt_sec)
            if cur_sr < end_sr - float(self.ctx.Control.ErrorStress.air):
                self.motor.add_ep_cell_pressure_kpa_signed(+cell_rate * dt_sec / 60.0)
            elif cur_sr > end_sr + float(self.ctx.Control.ErrorStress.air):
                self.motor.add_ep_cell_pressure_kpa_signed(-cell_rate * dt_sec / 60.0)
        target_sr = type(self).target_sr_state
        if is_zero(end_sr - ini_sr):
            type(self).step_time_min = 0.0
            return CtrlResult.NEXT_STEP
        comp_rate = _clamp((target_sr - ini_sr) / (end_sr - ini_sr), 0.0, 1.0)
        target_szq = ini_szq + comp_rate * (end_szq - ini_szq)
        target_sz = ini_sz + comp_rate * (end_sz - ini_sz)
        self.motor.set_motor_clutch(True)
        self.motor.set_motor_speed(axis_speed)
        if cur_sz > target_sz + err_motor:
            self.motor.set_motor_direction_up()
        elif cur_sz < target_sz - err_motor:
            self.motor.set_motor_direction_down()
        else:
            self.motor.set_motor_speed(0.0)
        self.torsional.set_torsional_clutch(True)
        self.torsional.set_torsional_speed(tors_speed)
        if cur_szq > target_szq + err_motor:
            self.torsional.set_torsional_direction_ccw()
        elif cur_szq < target_szq - err_motor:
            self.torsional.set_torsional_direction_cw()
        else:
            self.torsional.set_torsional_speed(0.0)
        if abs(cur_sz - end_sz) <= exit_band and abs(cur_sr - end_sr) <= exit_band and (abs(cur_szq - end_szq) <= exit_band):
            type(self).step_time_min = 0.0
            return CtrlResult.NEXT_STEP
        return CtrlResult.CONTINUE
