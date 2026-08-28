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

class TorsionalStepControl_MonotonicAxialLoadingConstP(TorsionalStepControlClass):
    NAME = 'Monotonic Axial w/ Const P'
    DESCRIPTION = ''
    ARGS = (ArgSpec('direction (0:Compression / 1:Extension)'), ArgSpec('target_sigma_z', 'kPa'), ArgSpec('target_epsilon_z', '%'), ArgSpec('axial_motor_speed', 'rpm'), ArgSpec('target_p_prime', 'kPa'), ArgSpec('sigma_zq_hold', 'kPa'), ArgSpec('torsional_motor_speed', 'rpm'))

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        direction = float(args[0])
        target_sz = float(args[1])
        target_ez = float(args[2])
        axis_speed = float(args[3])
        target_p = float(args[4])
        target_szq = float(args[5])
        tors_speed = float(args[6])
        cur_sz = self.ctx.Current.e_sa
        cur_szq = self.ctx.Current.tau
        cur_ez = self.ctx.Current.ea
        err_motor = abs(float(self.ctx.Control.ErrorStress.ext))
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
        if is_zero(direction):
            if cur_sz <= target_sz and cur_ez < target_ez:
                self.motor.set_motor_direction_down()
                self.motor.set_motor_speed(axis_speed)
                target_sr_base = (3.0 * target_p - cur_sz) / 2.0
                servo_ep_cell_proportional(self.motor, target_sr_base, DSS_TORS_EP_GAIN_CONST_P, dt_sec)
                return CtrlResult.CONTINUE
            self.motor.set_motor_speed(0.0)
            return CtrlResult.NEXT_STEP
        if is_zero(direction - 1.0):
            if cur_sz >= target_sz and cur_ez > target_ez:
                self.motor.set_motor_direction_up()
                self.motor.set_motor_speed(axis_speed)
                target_sr_base = (3.0 * target_p - cur_sz) / 2.0
                servo_ep_cell_proportional(self.motor, target_sr_base, DSS_TORS_EP_GAIN_CONST_P, dt_sec)
                return CtrlResult.CONTINUE
            self.motor.set_motor_speed(0.0)
            return CtrlResult.NEXT_STEP
        self.motor.set_motor_speed(0.0)
        self.torsional.set_torsional_speed(0.0)
        return CtrlResult.STOP
