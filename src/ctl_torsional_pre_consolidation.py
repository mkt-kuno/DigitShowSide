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
from ctl_torsional_step_ctrl_class import servo_ep_cell_proportional
from utl_context import DSS_TORS_PRECON_EP_GAIN, get_context

class TorsionalPreConsolidationController:

    def __init__(self, motor: MotorController | None=None) -> None:
        self.ctx = get_context()
        self.motor = motor if motor is not None else MotorController()

    def run(self, dt_sec: float) -> None:
        ctx = self.ctx
        pct = ctx.Control.PreConsolidationTorsional
        axis_speed_max = float(pct.axis_speed_max_rpm)
        q_at_max_speed = float(pct.q_at_max_speed_kpa)
        cell_target = float(pct.cell_target_kpa)
        cell_rate = float(pct.cell_rate_kpa_per_min)
        err_air = float(ctx.Control.ErrorStress.air)
        err_motor = abs(float(ctx.Control.ErrorStress.ext))
        cur_q = float(ctx.Current.q)
        cur_sr = float(ctx.Current.e_sr)
        if cell_target > 0.0 and cell_rate > 0.0:
            if cur_sr <= cell_target - err_air:
                delta_kpa = cell_rate * dt_sec / 60.0
                self.motor.increase_ep_cell_pressure_kpa(delta_kpa)
            elif cur_sr >= cell_target + err_air:
                delta_kpa = cell_rate * dt_sec / 60.0
                self.motor.decrease_ep_cell_pressure_kpa(delta_kpa)
            else:
                servo_ep_cell_proportional(self.motor, target_sr_kpa=cell_target, gain=DSS_TORS_PRECON_EP_GAIN, dt_sec=dt_sec)
        self.motor.set_motor_clutch(True)
        if cur_q > err_motor:
            self.motor.set_motor_direction_up()
            if cur_q > q_at_max_speed or is_zero(q_at_max_speed):
                self.motor.set_motor_speed(axis_speed_max)
            else:
                self.motor.set_motor_speed(axis_speed_max * (cur_q / q_at_max_speed))
        elif cur_q < -err_motor:
            self.motor.set_motor_direction_down()
            if cur_q < -q_at_max_speed or is_zero(q_at_max_speed):
                self.motor.set_motor_speed(axis_speed_max)
            else:
                self.motor.set_motor_speed(axis_speed_max * (-cur_q / q_at_max_speed))
        else:
            self.motor.set_motor_speed(0.0)
