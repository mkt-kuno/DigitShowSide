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

class TorsionalStepControl_MonotonicTorsionalLoadingCNS(TorsionalStepControlClass):
    NAME = 'Monotonic w/ Cell Servo'
    DESCRIPTION = ''
    ARGS = (ArgSpec('torsional_dir (0:CW / 1:CCW)'), ArgSpec('target_eff_shear_stress', 'kPa'), ArgSpec('target_eff_shear_strain', '-'), ArgSpec('torsional_motor_speed', 'rpm'), ArgSpec('axial_motor_speed', 'rpm'), ArgSpec('cell_pressure_rate', 'kPa/min'), ArgSpec('target_eff_axial_stress', 'kPa'), ArgSpec('target_eff_rad_stress', 'kPa'))

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        tdir = float(args[0])
        target_szq = float(args[1])
        target_gzq = float(args[2])
        tors_speed = float(args[3])
        axis_speed = float(args[4])
        cell_rate = float(args[5])
        target_sz = float(args[6])
        target_sr = float(args[7])
        cur_sz = self.ctx.Current.e_sa
        cur_sr = self.ctx.Current.e_sr
        cur_szq = self.ctx.Current.tau
        cur_gzq = self.ctx.Current.gamma
        err_air = float(self.ctx.Control.ErrorStress.air)
        err_ext = abs(float(self.ctx.Control.ErrorStress.ext))
        self.motor.set_motor_clutch(True)
        if cur_sz > target_sz + err_ext:
            self.motor.set_motor_direction_up()
            self.motor.set_motor_speed(axis_speed)
        elif cur_sz < target_sz - err_ext:
            self.motor.set_motor_direction_down()
            self.motor.set_motor_speed(axis_speed)
        else:
            self.motor.set_motor_speed(0.0)
        if not is_zero(cell_rate):
            if cur_sr < target_sr - err_air:
                self.motor.add_ep_cell_pressure_kpa_signed(+cell_rate * dt_sec / 60.0)
            elif cur_sr > target_sr + err_air:
                self.motor.add_ep_cell_pressure_kpa_signed(-cell_rate * dt_sec / 60.0)
        self.torsional.set_torsional_clutch(True)
        if is_zero(tdir):
            if cur_szq < target_szq and cur_gzq < target_gzq:
                self.torsional.set_torsional_direction_cw()
                self.torsional.set_torsional_speed(tors_speed)
                return CtrlResult.CONTINUE
            self.torsional.set_torsional_speed(0.0)
            return CtrlResult.NEXT_STEP
        if is_one(tdir):
            if cur_szq > target_szq and cur_gzq > target_gzq:
                self.torsional.set_torsional_direction_ccw()
                self.torsional.set_torsional_speed(tors_speed)
                return CtrlResult.CONTINUE
            self.torsional.set_torsional_speed(0.0)
            return CtrlResult.NEXT_STEP
        self.torsional.set_torsional_speed(0.0)
        return CtrlResult.STOP
