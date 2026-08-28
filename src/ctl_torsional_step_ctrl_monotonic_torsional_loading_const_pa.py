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
from ctl_torsional_step_ctrl_class import ArgSpec, TorsionalStepControlClass, servo_ep_cell_proportional
from utl_context import DSS_TORS_EP_GAIN_CONST_P

class TorsionalStepControl_MonotonicTorsionalLoadingConstPA(TorsionalStepControlClass):
    NAME = 'Monotonic w/ Const P and Alpha'
    DESCRIPTION = ''
    ARGS = (ArgSpec('direction (0:CW / 1:CCW)'), ArgSpec('target_sigma_zq', 'kPa'), ArgSpec('target_gamma_zq', '-'), ArgSpec('torsional_motor_speed', 'rpm'), ArgSpec('axial_motor_speed', 'rpm'), ArgSpec('target_p_prime', 'kPa'), ArgSpec('tan(2*alpha)', '-'))

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        direction = float(args[0])
        target_szq = float(args[1])
        target_gzq = float(args[2])
        tors_speed = float(args[3])
        axis_speed = float(args[4])
        target_p = float(args[5])
        tan_alpha = float(args[6])
        if is_zero(tan_alpha):
            self.torsional.set_torsional_speed(0.0)
            self.motor.set_motor_speed(0.0)
            return CtrlResult.STOP
        cur_sz = self.ctx.Current.e_sa
        cur_szq = self.ctx.Current.tau
        cur_gzq = self.ctx.Current.gamma
        err_motor = abs(float(self.ctx.Control.ErrorStress.ext))
        self.torsional.set_torsional_clutch(True)
        if is_zero(direction):
            if cur_szq < target_szq and cur_gzq < target_gzq:
                self.torsional.set_torsional_direction_cw()
                self.torsional.set_torsional_speed(tors_speed)
                target_sr = (3.0 * target_p - 2.0 * cur_szq / tan_alpha) / 3.0
                target_sz = (3.0 * target_p + 4.0 * cur_szq / tan_alpha) / 3.0
                servo_ep_cell_proportional(self.motor, target_sr, DSS_TORS_EP_GAIN_CONST_P, dt_sec)
                self.motor.set_motor_clutch(True)
                if cur_sz > target_sz + err_motor:
                    self.motor.set_motor_direction_up()
                    self.motor.set_motor_speed(axis_speed)
                elif cur_sz < target_sz - err_motor:
                    self.motor.set_motor_direction_down()
                    self.motor.set_motor_speed(axis_speed)
                else:
                    self.motor.set_motor_speed(0.0)
                return CtrlResult.CONTINUE
            self.torsional.set_torsional_speed(0.0)
            return CtrlResult.NEXT_STEP
        if is_zero(direction - 1.0):
            if cur_szq > target_szq and cur_gzq > target_gzq:
                self.torsional.set_torsional_direction_ccw()
                self.torsional.set_torsional_speed(tors_speed)
                target_sr = (3.0 * target_p - 2.0 * cur_szq / tan_alpha) / 3.0
                target_sz = (3.0 * target_p + 4.0 * cur_szq / tan_alpha) / 3.0
                servo_ep_cell_proportional(self.motor, target_sr, DSS_TORS_EP_GAIN_CONST_P, dt_sec)
                self.motor.set_motor_clutch(True)
                if cur_sz > target_sz + err_motor:
                    self.motor.set_motor_direction_up()
                    self.motor.set_motor_speed(axis_speed)
                elif cur_sz < target_sz - err_motor:
                    self.motor.set_motor_direction_down()
                    self.motor.set_motor_speed(axis_speed)
                else:
                    self.motor.set_motor_speed(0.0)
                return CtrlResult.CONTINUE
            self.torsional.set_torsional_speed(0.0)
            return CtrlResult.NEXT_STEP
        self.torsional.set_torsional_speed(0.0)
        self.motor.set_motor_speed(0.0)
        return CtrlResult.STOP
