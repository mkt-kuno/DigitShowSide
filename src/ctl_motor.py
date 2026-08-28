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
import math
import os
import sys
from enum import IntEnum
from typing import TYPE_CHECKING
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utl_context import DSS_AI_CH_HCDPT, DSS_AI_CH_LCDPT, DSS_AI_CH_LDT1, DSS_AI_CH_LDT2, DSS_AI_CH_V_DISP, DSS_AI_CH_VLC, DSS_AO_CH_EP_AXIS, DSS_AO_CH_EP_CELL, DSS_AO_CH_MOTOR_ONOFF, DSS_AO_CH_MOTOR_SPEED, DSS_AO_CH_MOTOR_UPDOWN, DSS_CHDEF_PARAM_MAX, DSS_CHDEF_STEPCTRL_ARGS_MAX, DSS_PARAM_DTYPE, DSS_STAGE_PRESENT, ControlMode, ControlType, get_context
DSS_MOTOR_PID_CTRL_COEFF_P: float = 0.4
if TYPE_CHECKING:
    from ctl_motor_step_ctrl_cyclic_strain import MotorStepControl_CyclicStrain
    from ctl_motor_step_ctrl_cyclic_stress import MotorStepControl_CyclicStress
DSS_FLT_M_PI: float = 3.141592654
DSS_FLT_EPSILON: float = 1e-07

def is_zero(x: float) -> bool:
    return -DSS_FLT_EPSILON <= x <= DSS_FLT_EPSILON

def is_one(x: float) -> bool:
    return -DSS_FLT_EPSILON <= x - 1.0 <= DSS_FLT_EPSILON

class CtrlResult(IntEnum):
    STOP = 0
    CONTINUE = 1
    NEXT_STEP = 2
STEP_CTRL_NUM_STOP = 0
STEP_CTRL_NUM_MONOTONIC = 1
STEP_CTRL_NUM_CYCLIC_STRESS = 2
STEP_CTRL_NUM_CYCLIC_STRAIN = 3
STEP_CTRL_NUM_CREEP = 4
STEP_CTRL_NUM_LINEAR_STRESS_PATH = 5
STEP_CTRL_NUM_MAX = 6

class StepLoadDir(IntEnum):
    COMPRESSION = 0
    EXTENSION = 1

def resolve_load_dir(raw: float) -> StepLoadDir | None:
    if is_zero(raw):
        return StepLoadDir.COMPRESSION
    if is_one(raw):
        return StepLoadDir.EXTENSION
    return None

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))
from ctl_motor_step_ctrl_class import MotorStepControlClass

class MotorController:

    def __init__(self) -> None:
        self.ctx = get_context()

    def set_motor_clutch(self, on: bool) -> None:
        ao = self.ctx.AIO.AO
        mv = self.ctx.Control.MotorVoltage
        ao.row(DSS_AO_CH_MOTOR_ONOFF).raw = float(mv.on if on else mv.off)

    def set_motor_direction_up(self) -> None:
        ao = self.ctx.AIO.AO
        mv = self.ctx.Control.MotorVoltage
        ao.row(DSS_AO_CH_MOTOR_UPDOWN).raw = float(mv.up)

    def set_motor_direction_down(self) -> None:
        ao = self.ctx.AIO.AO
        mv = self.ctx.Control.MotorVoltage
        ao.row(DSS_AO_CH_MOTOR_UPDOWN).raw = float(mv.down)

    def set_motor_speed(self, motor_speed: float) -> None:
        ao = self.ctx.AIO.AO
        cal = ao.row(DSS_AO_CH_MOTOR_SPEED).Cal
        ao.row(DSS_AO_CH_MOTOR_SPEED).raw = float(motor_speed) * float(cal.a) + float(cal.b)

    def set_pressure_cell(self, raw_value: float) -> None:
        self.ctx.AIO.AO.row(DSS_AO_CH_EP_CELL).raw = float(raw_value)

    def set_ep_cell_pressure_kpa(self, kpa: float) -> None:
        ao = self.ctx.AIO.AO
        cal = ao.row(DSS_AO_CH_EP_CELL).Cal
        ao.row(DSS_AO_CH_EP_CELL).raw = float(kpa) * float(cal.a) + float(cal.b)

    def increase_ep_cell_pressure_kpa(self, delta_kpa: float) -> None:
        ao = self.ctx.AIO.AO
        cal = ao.row(DSS_AO_CH_EP_CELL).Cal
        ao.row(DSS_AO_CH_EP_CELL).raw += abs(float(delta_kpa)) * float(cal.a)

    def decrease_ep_cell_pressure_kpa(self, delta_kpa: float) -> None:
        ao = self.ctx.AIO.AO
        cal = ao.row(DSS_AO_CH_EP_CELL).Cal
        ao.row(DSS_AO_CH_EP_CELL).raw -= abs(float(delta_kpa)) * float(cal.a)

    def set_pressure_axis(self, raw_value: float) -> None:
        self.ctx.AIO.AO.row(DSS_AO_CH_EP_AXIS).raw = float(raw_value)

    def set_ep_axial_pressure_n(self, n: float) -> None:
        ao = self.ctx.AIO.AO
        cal = ao.row(DSS_AO_CH_EP_AXIS).Cal
        ao.row(DSS_AO_CH_EP_AXIS).raw = float(n) * float(cal.a) + float(cal.b)

    def increase_ep_axial_pressure_n(self, delta_n: float) -> None:
        ao = self.ctx.AIO.AO
        cal = ao.row(DSS_AO_CH_EP_AXIS).Cal
        ao.row(DSS_AO_CH_EP_AXIS).raw += abs(float(delta_n)) * float(cal.a)

    def decrease_ep_axial_pressure_n(self, delta_n: float) -> None:
        ao = self.ctx.AIO.AO
        cal = ao.row(DSS_AO_CH_EP_AXIS).Cal
        ao.row(DSS_AO_CH_EP_AXIS).raw -= abs(float(delta_n)) * float(cal.a)

    def add_ep_cell_pressure_kpa_signed(self, delta_kpa: float) -> None:
        cal = self.ctx.AIO.AO.row(DSS_AO_CH_EP_CELL).Cal
        self.ctx.AIO.AO.row(DSS_AO_CH_EP_CELL).raw += float(delta_kpa) * float(cal.a)

    def _apply_radial_pressure_pid(self, e_sr_target: float, dt_sec: float) -> None:
        cur = self.ctx.Current
        err = float(cur.e_sr) - e_sr_target
        if abs(err) <= float(self.ctx.Control.ErrorStress.air):
            return
        cal = self.ctx.AIO.AO.row(DSS_AO_CH_EP_CELL).Cal
        delta_raw = float(dt_sec * DSS_MOTOR_PID_CTRL_COEFF_P * float(cal.a) * err)
        self.ctx.AIO.AO.row(DSS_AO_CH_EP_CELL).raw -= delta_raw

    def start(self) -> None:
        self.ctx.Control.mode = ControlMode.MOTOR
        from ctl_motor_step_ctrl_cyclic_strain import MotorStepControl_CyclicStrain
        from ctl_motor_step_ctrl_cyclic_stress import MotorStepControl_CyclicStress
        MotorStepControl_CyclicStress.reset_cycle()
        MotorStepControl_CyclicStrain.reset_cycle()
        self.ctx.Control.watch.reset()
        self.ctx.Control.watch.start()

    def stop(self) -> None:
        self.set_motor_speed(0.0)
        self.ctx.Control.watch.stop()

    def step(self, dt_sec: float) -> CtrlResult:
        ctx = self.ctx
        self.ctx.Control.watch.update_elapsed_and_interval()
        if not self.ctx.Flag.control and ctx.Control.type != ControlType.EXTERNAL:
            return CtrlResult.STOP
        if ctx.Control.mode != ControlMode.MOTOR:
            return CtrlResult.STOP
        ctype = ctx.Control.type
        if ctype == ControlType.NONE:
            return CtrlResult.STOP
        if ctype == ControlType.EXTERNAL:
            return CtrlResult.CONTINUE
        if ctype == ControlType.PRECONSOLIDATION:
            from ctl_motor_pre_consolidation import MotorPreConsolidationController
            MotorPreConsolidationController().run()
            return CtrlResult.CONTINUE
        if ctype == ControlType.STEP:
            return StepController().step(dt_sec)
        return CtrlResult.STOP

    def _active_cyclic_class(self) -> type[MotorStepControl_CyclicStress] | type[MotorStepControl_CyclicStrain]:
        from ctl_motor_step_ctrl_cyclic_strain import MotorStepControl_CyclicStrain
        from ctl_motor_step_ctrl_cyclic_stress import MotorStepControl_CyclicStress
        cs = self.ctx.Control.current_step
        ctrl = int(self.ctx.Control.Step[cs].ctrl)
        if ctrl == STEP_CTRL_NUM_CYCLIC_STRAIN:
            return MotorStepControl_CyclicStrain
        return MotorStepControl_CyclicStress

    def calculate_param(self) -> None:
        ctx = self.ctx
        ai = ctx.AIO.AI
        present = ctx.SpecimenData.Stage.row(DSS_STAGE_PRESENT)
        vlc_phy = float(ai[DSS_AI_CH_VLC].phy)
        lvdt_phy = float(ai[DSS_AI_CH_V_DISP].phy)
        lcdpt_phy = float(ai[DSS_AI_CH_LCDPT].phy)
        hcdpt_phy = float(ai[DSS_AI_CH_HCDPT].phy)
        ldt1_phy = float(ai[DSS_AI_CH_LDT1].phy)
        ldt2_phy = float(ai[DSS_AI_CH_LDT2].phy)
        p_h0 = float(present.height)
        p_v0 = float(present.volume)
        p_ldt1 = float(present.ldt_1)
        p_ldt2 = float(present.ldt_2)
        if p_h0 <= 0.0:
            p_h0 = 1e-09
        if p_v0 <= 0.0:
            p_v0 = 1e-09
        if p_ldt1 <= 0.0:
            p_ldt1 = 1e-09
        if p_ldt2 <= 0.0:
            p_ldt2 = 1e-09
        height = p_h0 - lvdt_phy
        volume = p_v0 - lcdpt_phy
        area = volume / height if height > 0.0 else 0.0
        diameter = math.sqrt(max(0.0, 4.0 * area / DSS_FLT_M_PI))
        ea = (p_h0 - height) / p_h0 * 100.0
        ev = (p_v0 - volume) / p_v0 * 100.0
        er_denom = 1.0 - ea * 0.01
        if abs(er_denom) < DSS_FLT_EPSILON:
            er_ratio = 0.0
        else:
            er_ratio = max(0.0, (1.0 - ev * 0.01) / er_denom)
        er = (1.0 - math.sqrt(er_ratio)) * 100.0
        q = vlc_phy / area * 1000.0 if area > 0.0 else 0.0
        e_sr = hcdpt_phy
        e_sa = e_sr + q
        e_p = (e_sa + 2.0 * e_sr) / 3.0
        cur = ctx.Current
        cur.Specimen.height = height
        cur.Specimen.volume = volume
        cur.Specimen.area = area
        cur.Specimen.diameter = diameter
        cur.ea = ea
        cur.ev = ev
        cur.er = er
        cur.q = q
        cur.e_sr = e_sr
        cur.e_sa = e_sa
        cur.e_p = e_p
        cur.p = e_p
        ldt1_now = p_ldt1 - ldt1_phy
        ldt2_now = p_ldt2 - ldt2_phy
        local_avg = 0.5 * (ldt1_phy / p_ldt1 + ldt2_phy / p_ldt2) * 100.0
        local_ldt1 = ldt1_phy / p_ldt1 * 100.0
        local_ldt2 = ldt2_phy / p_ldt2 * 100.0
        param = np.zeros(DSS_CHDEF_PARAM_MAX, dtype=DSS_PARAM_DTYPE)
        param[0] = q
        param[1] = e_p
        param[2] = e_sa
        param[3] = e_sr
        param[4] = ea
        param[5] = er
        param[6] = ev
        param[7] = ldt1_now
        param[8] = ldt2_now
        param[9] = local_avg
        param[10] = local_ldt1
        param[11] = local_ldt2
        param[16] = diameter
        param[17] = height
        param[18] = area
        param[19] = volume
        param[20] = float(present.diameter)
        param[21] = p_h0
        param[22] = float(present.area)
        param[23] = p_v0
        param[24] = float(int(ctx.Control.type))
        param[25] = float(int(ctx.Control.current_step))
        param[26] = float(int(ctx.Control.Step[ctx.Control.current_step].ctrl))
        param[27] = float(ctx.Control.watch.get_elapsed_sec())
        param[28] = float(self._active_cyclic_class().cycle_num)
        ctx.AIO.param[:] = param

    def info_string(self) -> str:
        ctx = self.ctx
        ctype = ctx.Control.type
        if ctype == ControlType.NONE:
            return 'Mode: None'
        if ctype == ControlType.PRECONSOLIDATION:
            pc = ctx.Control.PreConsolidation
            return f'Mode: PreConsolidation\nTime Elapsed: {ctx.Control.watch.get_elapsed_sec():6.01f} [sec]\nTarget={float(pc.target):.01f}[kPa], Error={float(pc.error):.01f}[kPa], MaxSpeed={float(pc.motor_speed):.01f}[rpm]\n'
        if ctype == ControlType.STEP:
            cs = ctx.Control.current_step
            cur_ctrl = int(ctx.Control.Step[cs].ctrl)
            cyc_cls = self._active_cyclic_class()
            cyc_num = cyc_cls.cycle_num
            cyc_state = cyc_cls.cycle_state
            cur_sub_cls = MotorStepControlClass.get_class_by_ctrl_num(cur_ctrl)
            cur_name = cur_sub_cls.NAME if cur_sub_cls else 'Undefined'
            head = f'Mode: StepControl\nTime Elapsed: {ctx.Control.watch.get_elapsed_sec():8.01f} [sec]\nCyclic: Num={int(cyc_num)}, State={int(cyc_state)}\nCurrent: Step={cs}, Control={cur_name}\n'
            args_cur = ctx.Control.Step[cs].args
            max_args = min(8, DSS_CHDEF_STEPCTRL_ARGS_MAX)
            arg_str = ', '.join((f'{float(args_cur[i]):.01f}' for i in range(max_args)))
            head += f'Args: {arg_str}\n'
            if cs + 1 < len(ctx.Control.Step):
                nxt_ctrl = int(ctx.Control.Step[cs + 1].ctrl)
                nxt_sub_cls = MotorStepControlClass.get_class_by_ctrl_num(nxt_ctrl)
                nxt_name = nxt_sub_cls.NAME if nxt_sub_cls else 'Undefined'
                args_nxt = ctx.Control.Step[cs + 1].args
                arg_str_n = ', '.join((f'{float(args_nxt[i]):.01f}' for i in range(max_args)))
                head += f'Next: Step={cs + 1}, Control={nxt_name}\nArgs: {arg_str_n}\n'
            return head
        return ''

class StepController:

    def __init__(self) -> None:
        self.ctx = get_context()
        self._sub_cache: dict[int, MotorStepControlClass] = {}

    def _get_sub(self, ctrl_num: int) -> MotorStepControlClass | None:
        sub = self._sub_cache.get(ctrl_num)
        if sub is not None:
            return sub
        sub_cls = MotorStepControlClass.get_class_by_ctrl_num(ctrl_num)
        if sub_cls is None:
            return None
        sub = sub_cls()
        self._sub_cache[ctrl_num] = sub
        return sub

    def advance_to_next_step(self) -> bool:
        from ctl_motor_step_ctrl_cyclic_strain import MotorStepControl_CyclicStrain
        from ctl_motor_step_ctrl_cyclic_stress import MotorStepControl_CyclicStress
        if self.ctx.Control.current_step + 1 >= len(self.ctx.Control.Step):
            MotorStepControl_CyclicStress.reset_cycle()
            MotorStepControl_CyclicStrain.reset_cycle()
            return False
        self.ctx.Control.current_step += 1
        MotorStepControl_CyclicStress.reset_cycle()
        MotorStepControl_CyclicStrain.reset_cycle()
        self.ctx.Control.watch.reset()
        self.ctx.Control.watch.start()
        return True

    def step(self, dt_sec: float) -> CtrlResult:
        ctx = self.ctx
        cs = ctx.Control.current_step
        if cs < 0 or cs >= len(ctx.Control.Step):
            return CtrlResult.STOP
        row = ctx.Control.Step[cs]
        ctrl = int(row.ctrl)
        sub = self._get_sub(ctrl)
        if sub is None:
            return CtrlResult.STOP
        result = sub.process(row.args, dt_sec)
        if result == CtrlResult.NEXT_STEP and (not self.advance_to_next_step()):
            return CtrlResult.STOP
        return result
