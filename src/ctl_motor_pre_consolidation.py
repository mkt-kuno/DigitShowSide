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
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ctl_motor import MotorController, is_zero
from utl_context import get_context

class MotorPreConsolidationController:

    def __init__(self, motor: MotorController | None=None) -> None:
        self.ctx = get_context()
        self.motor = motor if motor is not None else MotorController()

    def run(self) -> None:
        ctx = self.ctx
        pc = ctx.Control.PreConsolidation
        target = float(pc.target)
        error = float(pc.error)
        max_speed = float(pc.motor_speed)
        err_com = float(ctx.Control.ErrorStress.com)
        err_ext = float(ctx.Control.ErrorStress.ext)
        cur_q = float(ctx.Current.q)
        self.motor.set_motor_clutch(True)
        if cur_q > target + err_com:
            self.motor.set_motor_direction_up()
            if cur_q > target + error:
                self.motor.set_motor_speed(max_speed)
            else:
                ratio = (cur_q - target) / error if not is_zero(error) else 0.0
                self.motor.set_motor_speed(max_speed * ratio)
        elif cur_q < target + err_ext:
            self.motor.set_motor_direction_down()
            if cur_q < target - error:
                self.motor.set_motor_speed(max_speed)
            else:
                ratio = (target - cur_q) / error if not is_zero(error) else 0.0
                self.motor.set_motor_speed(max_speed * ratio)
        else:
            self.motor.set_motor_speed(0.0)
