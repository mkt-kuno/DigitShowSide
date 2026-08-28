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

class TorsionalStepControl_MonotonicTorsionalLoading(TorsionalStepControlClass):
    NAME = 'Monotonic'
    DESCRIPTION = ''
    ARGS = (ArgSpec('torsional_dir (0:CW / 1:CCW)'), ArgSpec('target_eff_shear_stress', 'kPa'), ArgSpec('target_eff_shear_strain', '-'), ArgSpec('torsional_motor_speed', 'rpm'))

    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        tdir = float(args[0])
        target_szq = float(args[1])
        target_gzq = float(args[2])
        tors_speed = float(args[3])
        cur_szq = self.ctx.Current.tau
        cur_gzq = self.ctx.Current.gamma
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
TorsionalStepControl_Monotonic = TorsionalStepControl_MonotonicTorsionalLoading
