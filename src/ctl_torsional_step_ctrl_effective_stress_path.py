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
from ctl_torsional_step_ctrl_class import ArgSpec, TorsionalStepControlClass, approach_by_rate, servo_ep_cell_proportional
from utl_context import DSS_TORS_EP_GAIN_ESP, DSS_TORS_EXIT_BAND_FACTOR

class TorsionalStepControl_EffectiveStressPath(TorsionalStepControlClass):
    NAME = 'Effective Stress Path'
    DESCRIPTION = ''
    ARGS = (ArgSpec('initial_eff_axial_stress', 'kPa'), ArgSpec('initial_eff_rad_stress', 'kPa'), ArgSpec('initial_eff_shear_stress', 'kPa'), ArgSpec('target_eff_axial_stress', 'kPa'), ArgSpec('target_eff_rad_stress', 'kPa'), ArgSpec('target_eff_shear_stress', 'kPa'), ArgSpec('axial_motor_speed', 'rpm'), ArgSpec('torsional_motor_speed', 'rpm'), ArgSpec('cell_pressure_rate', 'kPa/min'))

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
        err_air = float(self.ctx.Control.ErrorStress.air)
        err_ext = abs(float(self.ctx.Control.ErrorStress.ext))
        exit_band = DSS_TORS_EXIT_BAND_FACTOR * err_ext
        if is_zero(cell_rate):
            self.motor.add_ep_cell_pressure_kpa_signed(0.0)
        elif cur_sr < end_sr - err_air:
            self.motor.add_ep_cell_pressure_kpa_signed(+cell_rate * dt_sec / 60.0)
        elif cur_sr > end_sr + err_air:
            self.motor.add_ep_cell_pressure_kpa_signed(-cell_rate * dt_sec / 60.0)
        else:
            target_sr = approach_by_rate(cur_sr, end_sr, cell_rate, dt_sec)
            servo_ep_cell_proportional(self.motor, target_sr, DSS_TORS_EP_GAIN_ESP, dt_sec)
        if end_sr != ini_sr:
            target_sz = (end_sz - ini_sz) / (end_sr - ini_sr) * (cur_sr - ini_sr) + ini_sz
            if end_sz > ini_sz and target_sz > end_sz or (end_sz < ini_sz and target_sz < end_sz):
                target_sz = end_sz
            target_szq = (end_szq - ini_szq) / (end_sr - ini_sr) * (cur_sr - ini_sr) + ini_szq
            if end_szq > ini_szq and target_szq > end_szq or (end_szq < ini_szq and target_szq < end_szq):
                target_szq = end_szq
        elif end_sz != ini_sz and abs(end_sz - ini_sz) >= abs(end_szq - ini_szq):
            target_sz = end_sz
            target_szq = (end_szq - ini_szq) / (end_sz - ini_sz) * (cur_sz - ini_sz) + ini_szq
            if end_szq > ini_szq and target_szq > end_szq or (end_szq < ini_szq and target_szq < end_szq):
                target_szq = end_szq
        elif end_szq != ini_szq:
            target_sz = (end_sz - ini_sz) / (end_szq - ini_szq) * (cur_szq - ini_szq) + ini_sz
            if end_sz > ini_sz and target_sz > end_sz or (end_sz < ini_sz and target_sz < end_sz):
                target_sz = end_sz
            target_szq = end_szq
        else:
            target_sz = end_sz
            target_szq = end_szq
        self.motor.set_motor_clutch(True)
        if cur_sz > target_sz + err_ext:
            self.motor.set_motor_direction_up()
            self.motor.set_motor_speed(axis_speed)
        elif cur_sz < target_sz - err_ext:
            self.motor.set_motor_direction_down()
            self.motor.set_motor_speed(axis_speed)
        else:
            self.motor.set_motor_speed(0.0)
        self.torsional.set_torsional_clutch(True)
        if cur_szq > target_szq + err_ext:
            self.torsional.set_torsional_direction_ccw()
            self.torsional.set_torsional_speed(tors_speed)
        elif cur_szq < target_szq - err_ext:
            self.torsional.set_torsional_direction_cw()
            self.torsional.set_torsional_speed(tors_speed)
        else:
            self.torsional.set_torsional_speed(0.0)
        if abs(cur_sz - end_sz) <= exit_band and abs(cur_sr - end_sr) <= exit_band and (abs(cur_szq - end_szq) <= exit_band):
            self.motor.set_motor_speed(0.0)
            self.torsional.set_torsional_speed(0.0)
            return CtrlResult.NEXT_STEP
        return CtrlResult.CONTINUE
