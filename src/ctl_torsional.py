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
from enum import IntEnum
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ctl_motor import CtrlResult
from utl_context import DSS_AI_CAL_A_TOR_DISP_RAD_PER_V2, DSS_AI_CAL_A_TORQUE_LC_NCM_PER_V2, DSS_AI_CAL_B_TOR_DISP_RAD_PER_V, DSS_AI_CAL_B_TORQUE_LC_NCM_PER_V, DSS_AI_CAL_C_TOR_DISP_RAD, DSS_AI_CAL_C_TORQUE_LC_NCM, DSS_AI_CH_CG1, DSS_AI_CH_CG2, DSS_AI_CH_CG3, DSS_AI_CH_HCDPT, DSS_AI_CH_LCDPT, DSS_AI_CH_LDT1, DSS_AI_CH_LDT2, DSS_AI_CH_TOR_DISP, DSS_AI_CH_TORQUE_LC, DSS_AI_CH_V_DISP, DSS_AI_CH_VLC, DSS_AO_CH_TORSIONAL_CWCCW, DSS_AO_CH_TORSIONAL_ONOFF, DSS_AO_CH_TORSIONAL_SPEED, DSS_CHDEF_PARAM_MAX, DSS_CHDEF_STEPCTRL_ARGS_MAX, DSS_PARAM_DTYPE, DSS_STAGE_PRESENT, DSS_TORS_NCM_TO_NM, DSS_TORS_SZQ_GEOM_FACTOR, DSS_TORS_TORQUE_M_DIVISOR, ControlMode, ControlType, get_context
STEP_CTRL_NUM_STOP_TORSIONAL = 0
STEP_CTRL_NUM_EFFECTIVE_STRESS_PATH_TORSIONAL = 1
STEP_CTRL_NUM_MONOTONIC_TORSIONAL_LOADING_TORSIONAL = 2
STEP_CTRL_NUM_MONOTONIC_TORSIONAL_LOADING_CNS_TORSIONAL = 3
STEP_CTRL_NUM_CYCLIC_TORSIONAL_LOADING_TORSIONAL = 4
STEP_CTRL_NUM_CYCLIC_TORSIONAL_LOADING_CNS_TORSIONAL = 5
STEP_CTRL_NUM_SMALL_CYCLIC_TORSIONAL_LOADING_TORSIONAL = 6
STEP_CTRL_NUM_SMALL_CYCLIC_TORSIONAL_LOADING_CNS_TORSIONAL = 7
STEP_CTRL_NUM_MONOTONIC_AXIAL_LOADING_TORSIONAL = 8
STEP_CTRL_NUM_CYCLIC_AXIAL_LOADING_TORSIONAL = 9
STEP_CTRL_NUM_SMALL_CYCLIC_AXIAL_LOADING_TORSIONAL = 10
STEP_CTRL_NUM_CREEP_TORSIONAL = 11
STEP_CTRL_NUM_MONOTONIC_AXIAL_LOADING_CONST_P_TORSIONAL = 12
STEP_CTRL_NUM_MONOTONIC_TORSIONAL_LOADING_CONST_PA_TORSIONAL = 13
STEP_CTRL_NUM_CYCLIC_AXIAL_LOADING_OR_TORSIONAL = 14
STEP_CTRL_NUM_FILE_CONTROLABLE_CONSOLIDATION_TORSIONAL = 15
STEP_CTRL_NUM_MAX_TORSIONAL = 16

class StepTorsionalDir(IntEnum):
    CW = 0
    CCW = 1

def resolve_torsional_dir(raw: float) -> StepTorsionalDir | None:
    from ctl_motor import is_one, is_zero
    if is_zero(raw):
        return StepTorsionalDir.CW
    if is_one(raw):
        return StepTorsionalDir.CCW
    return None

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))
from ctl_torsional_step_ctrl_class import TorsionalStepControlClass

class TorsionalController:

    def __init__(self) -> None:
        self.ctx = get_context()

    def set_torsional_clutch(self, on: bool) -> None:
        ao = self.ctx.AIO.AO
        tv = self.ctx.Control.TorsionalVoltage
        ao.row(DSS_AO_CH_TORSIONAL_ONOFF).raw = float(tv.on if on else tv.off)

    def set_torsional_direction_cw(self) -> None:
        ao = self.ctx.AIO.AO
        tv = self.ctx.Control.TorsionalVoltage
        ao.row(DSS_AO_CH_TORSIONAL_CWCCW).raw = float(tv.cw)

    def set_torsional_direction_ccw(self) -> None:
        ao = self.ctx.AIO.AO
        tv = self.ctx.Control.TorsionalVoltage
        ao.row(DSS_AO_CH_TORSIONAL_CWCCW).raw = float(tv.ccw)

    def set_torsional_speed(self, rpm: float) -> None:
        ao = self.ctx.AIO.AO
        cal = ao.row(DSS_AO_CH_TORSIONAL_SPEED).Cal
        ao.row(DSS_AO_CH_TORSIONAL_SPEED).raw = float(rpm) * float(cal.a) + float(cal.b)

    def set_pressure_cell(self, raw_value: float) -> None:
        from ctl_motor import MotorController
        MotorController().set_pressure_cell(raw_value)

    def set_ep_cell_pressure_kpa(self, kpa: float) -> None:
        from ctl_motor import MotorController
        MotorController().set_ep_cell_pressure_kpa(kpa)

    def increase_ep_cell_pressure_kpa(self, delta_kpa: float) -> None:
        from ctl_motor import MotorController
        MotorController().increase_ep_cell_pressure_kpa(delta_kpa)

    def decrease_ep_cell_pressure_kpa(self, delta_kpa: float) -> None:
        from ctl_motor import MotorController
        MotorController().decrease_ep_cell_pressure_kpa(delta_kpa)

    def set_pressure_axis(self, raw_value: float) -> None:
        from ctl_motor import MotorController
        MotorController().set_pressure_axis(raw_value)

    def set_ep_axial_pressure_n(self, n: float) -> None:
        from ctl_motor import MotorController
        MotorController().set_ep_axial_pressure_n(n)

    def increase_ep_axial_pressure_n(self, delta_n: float) -> None:
        from ctl_motor import MotorController
        MotorController().increase_ep_axial_pressure_n(delta_n)

    def decrease_ep_axial_pressure_n(self, delta_n: float) -> None:
        from ctl_motor import MotorController
        MotorController().decrease_ep_axial_pressure_n(delta_n)

    def _apply_radial_pressure_pid(self, e_sr_target: float, dt_sec: float) -> None:
        from ctl_motor import MotorController
        MotorController()._apply_radial_pressure_pid(e_sr_target, dt_sec)

    def start(self) -> None:
        self.ctx.Control.mode = ControlMode.TORSIONAL
        from ctl_torsional_step_ctrl_class import reset_all_cycle_states
        reset_all_cycle_states()
        self.ctx.Control.watch.reset()
        self.ctx.Control.watch.start()

    def stop(self) -> None:
        self.set_torsional_speed(0.0)
        self.set_torsional_clutch(False)
        from ctl_motor import MotorController
        motor = MotorController()
        motor.set_motor_speed(0.0)
        motor.set_motor_clutch(False)
        self.ctx.Control.watch.stop()

    def step(self, dt_sec: float) -> CtrlResult:
        ctx = self.ctx
        self.ctx.Control.watch.update_elapsed_and_interval()
        if not self.ctx.Flag.control and ctx.Control.type != ControlType.EXTERNAL:
            return CtrlResult.STOP
        if ctx.Control.mode != ControlMode.TORSIONAL:
            return CtrlResult.STOP
        ctype = ctx.Control.type
        if ctype == ControlType.NONE:
            return CtrlResult.STOP
        if ctype == ControlType.EXTERNAL:
            return CtrlResult.CONTINUE
        if ctype == ControlType.PRECONSOLIDATION:
            from ctl_torsional_pre_consolidation import TorsionalPreConsolidationController
            TorsionalPreConsolidationController().run(dt_sec)
            return CtrlResult.CONTINUE
        if ctype == ControlType.STEP:
            return TorsionalStepController().step(dt_sec)
        return CtrlResult.STOP

    def calculate_param(self) -> None:
        ctx = self.ctx
        cur = ctx.Current
        ai = ctx.AIO.AI
        spec = ctx.SpecimenTorsional
        stage0 = spec.Stage.row(DSS_STAGE_PRESENT)
        v_load = float(ai[DSS_AI_CH_TORQUE_LC].phy)
        torque_ncm_raw = DSS_AI_CAL_A_TORQUE_LC_NCM_PER_V2 * v_load * v_load + DSS_AI_CAL_B_TORQUE_LC_NCM_PER_V * v_load + DSS_AI_CAL_C_TORQUE_LC_NCM
        v_disp = float(ai[DSS_AI_CH_TOR_DISP].phy)
        rotation1 = DSS_AI_CAL_A_TOR_DISP_RAD_PER_V2 * v_disp * v_disp + DSS_AI_CAL_B_TOR_DISP_RAD_PER_V * v_disp + DSS_AI_CAL_C_TOR_DISP_RAD
        rotation2 = rotation1
        d_in0 = float(stage0.diameter_in)
        d_out0 = float(stage0.diameter_out)
        h0 = float(stage0.height)
        v0 = float(stage0.volume)
        ext_mm = float(ai[DSS_AI_CH_V_DISP].phy)
        bw2_mm3 = float(ai[DSS_AI_CH_LCDPT].phy)
        height = h0 - ext_mm
        volume = v0 - bw2_mm3
        area = volume / height if height > 0.0 else 0.0
        shrink = 1.0
        if v0 > 0.0 and h0 > 0.0 and (1.0 - ext_mm / h0 > 0.0):
            shrink = float(np.sqrt((1.0 - bw2_mm3 / v0) / (1.0 - ext_mm / h0)))
        diameter_in = d_in0 * shrink
        diameter_out = d_out0 * shrink
        t_mm = float(spec.membrane_thickness)
        diameter_in_m = diameter_in - t_mm / 2.0
        diameter_out_m = diameter_out + t_mm / 2.0
        ez = ext_mm / h0 if h0 > 0.0 else 0.0
        er = 0.0
        if diameter_out > diameter_in > 0.0:
            er = -(diameter_out - d_out0 - (diameter_in - d_in0)) / (diameter_out - diameter_in)
        gzq1 = 0.0
        gzq2 = 0.0
        if height > 0.0 and diameter_out > diameter_in > 0.0:
            gzq1 = rotation1 * (diameter_out ** 3 - diameter_in ** 3) / 3.0 / height / (diameter_out ** 2 - diameter_in ** 2)
            gzq2 = rotation2 * (diameter_out ** 3 - diameter_in ** 3) / 3.0 / height / (diameter_out ** 2 - diameter_in ** 2)
        ev = bw2_mm3 / v0 if v0 > 0.0 else 0.0
        r_h_in = float(spec.r_height_in_m)
        r_h_out = float(spec.r_height_out_m)
        gzq_in_m = diameter_in_m / 2.0 * rotation1 / r_h_in if r_h_in > 0.0 else 0.0
        gzq_out_m = diameter_out_m / 2.0 * rotation1 / r_h_out if r_h_out > 0.0 else 0.0
        torque_m_nm = -1.0 / 6.0 * float(np.pi) * float(spec.membrane_modulus) * t_mm * (diameter_in ** 2 * gzq_in_m + diameter_out ** 2 * gzq_out_m) / DSS_TORS_TORQUE_M_DIVISOR
        force_m = 0.0
        pressure_in_m = 0.0
        pressure_out_m = 0.0
        load_n = float(ai[DSS_AI_CH_VLC].phy) + force_m + float(spec.cap_weight)
        torque_ncm = torque_ncm_raw + torque_m_nm * DSS_TORS_NCM_TO_NM
        ai.row(DSS_AI_CH_VLC).phy = load_n
        ai.row(DSS_AI_CH_TORQUE_LC).phy = torque_ncm
        ai.row(DSS_AI_CH_TOR_DISP).phy = np.float32(rotation1)
        cell_out = float(ai[DSS_AI_CH_HCDPT].phy) + pressure_out_m
        cell_in = float(ai[DSS_AI_CH_HCDPT].phy) + pressure_in_m
        sz = 0.0
        sr = 0.0
        sq = 0.0
        szq = 0.0
        if area > 0.0:
            sz = (load_n + float(np.pi) / 4.0 * (cell_out * diameter_out ** 2 - cell_in * diameter_in ** 2) / 1000.0) / area * 1000.0
        if diameter_out + diameter_in > 0.0:
            sr = (cell_out * diameter_out + cell_in * diameter_in) / (diameter_out + diameter_in)
        if diameter_out > diameter_in > 0.0:
            sq = (cell_out * diameter_out - cell_in * diameter_in) / (diameter_out - diameter_in)
            geom = 3.0 / 2.0 / (diameter_out ** 3 - diameter_in ** 3) + 1.0 / ((diameter_out ** 2 + diameter_in ** 2) * (diameter_out - diameter_in))
            szq = 4.0 * (torque_ncm / 100.0) / float(np.pi) * geom * DSS_TORS_SZQ_GEOM_FACTOR
        p = (sz + sr + sq) / 3.0
        q = sz - sr
        root = float(np.sqrt((sz - sq) ** 2 / 4.0 + szq ** 2))
        sigma1 = (sz + sq) / 2.0 + root
        sigma2 = sr
        sigma3 = (sz + sq) / 2.0 - root
        cur.tau = float(np.float32(szq))
        cur.gamma = float(np.float32(gzq1))
        cur.torque = float(np.float32(torque_ncm))
        cur.rotation = float(np.float32(rotation1))
        cur.e_sa = float(np.float32(sz))
        cur.e_sr = float(np.float32(sr))
        cur.q = float(np.float32(q))
        cur.p = float(np.float32(p))
        cur.e_p = cur.p
        cur.ea = float(np.float32(ez * 100.0))
        cur.er = float(np.float32(er * 100.0))
        cur.ev = float(np.float32(ev * 100.0))
        param = np.zeros(DSS_CHDEF_PARAM_MAX, dtype=DSS_PARAM_DTYPE)
        param[0] = sz
        param[1] = sr
        param[2] = sq
        param[3] = szq
        param[4] = ev * 100.0
        param[5] = ez * 100.0
        param[6] = float(ai[DSS_AI_CH_LDT1].phy)
        param[7] = float(ai[DSS_AI_CH_LDT2].phy)
        param[8] = float(ai[DSS_AI_CH_CG1].phy)
        param[9] = float(ai[DSS_AI_CH_CG2].phy)
        param[10] = float(ai[DSS_AI_CH_CG3].phy)
        param[11] = p
        param[12] = q
        param[13] = sigma1
        param[14] = sigma2
        param[15] = sigma3
        param[16] = gzq1 * 100.0
        param[17] = gzq2 * 100.0
        param[18] = cell_in
        param[19] = cell_out
        param[20] = diameter_in
        param[21] = diameter_out
        param[22] = height
        param[23] = volume
        param[24] = float(int(ctx.Control.type))
        param[25] = float(int(ctx.Control.current_step))
        param[26] = float(int(ctx.Control.Step[ctx.Control.current_step].ctrl))
        param[27] = float(ctx.Control.watch.get_elapsed_sec())
        from ctl_torsional_step_ctrl_cyclic_torsional_loading import TorsionalStepControl_CyclicTorsionalLoading
        param[28] = float(TorsionalStepControl_CyclicTorsionalLoading.cycle_num)
        ctx.AIO.param[:] = param

    def info_string(self) -> str:
        from ctl_torsional_step_ctrl_cyclic_torsional_loading import TorsionalStepControl_CyclicTorsionalLoading
        ctx = self.ctx
        ctype = ctx.Control.type
        if ctype == ControlType.NONE:
            return 'Mode: None'
        if ctype == ControlType.PRECONSOLIDATION:
            pct = ctx.Control.PreConsolidationTorsional
            return f'Mode: PreConsolidation\nTime Elapsed: {ctx.Control.watch.get_elapsed_sec():6.01f} [sec]\nAxisSpeedMax={float(pct.axis_speed_max_rpm):.0f}[rpm], q@MaxSpeed={float(pct.q_at_max_speed_kpa):.02f}[kPa], CellTarget={float(pct.cell_target_kpa):.01f}[kPa], CellRate={float(pct.cell_rate_kpa_per_min):.01f}[kPa/min]\n'
        if ctype == ControlType.STEP:
            cs = ctx.Control.current_step
            cur_ctrl = int(ctx.Control.Step[cs].ctrl)
            cyc_num = TorsionalStepControl_CyclicTorsionalLoading.cycle_num
            cyc_state = 0 if cyc_num == 0 else 1 if TorsionalStepControl_CyclicTorsionalLoading.flag_dir == 0 else 2
            cur_sub_cls = TorsionalStepControlClass.get_class_by_ctrl_num(cur_ctrl)
            cur_name = cur_sub_cls.NAME if cur_sub_cls else 'Undefined'
            head = f'Mode: StepControl (Torsional)\nTime Elapsed: {ctx.Control.watch.get_elapsed_sec():8.01f} [sec]\nCyclic: Num={int(cyc_num)}, State={int(cyc_state)}\nCurrent: Step={cs}, Control={cur_name}\n'
            args_cur = ctx.Control.Step[cs].args
            max_args = min(8, DSS_CHDEF_STEPCTRL_ARGS_MAX)
            arg_str = ', '.join((f'{float(args_cur[i]):.01f}' for i in range(max_args)))
            head += f'Args: {arg_str}\n'
            if cs + 1 < len(ctx.Control.Step):
                nxt_ctrl = int(ctx.Control.Step[cs + 1].ctrl)
                nxt_sub_cls = TorsionalStepControlClass.get_class_by_ctrl_num(nxt_ctrl)
                nxt_name = nxt_sub_cls.NAME if nxt_sub_cls else 'Undefined'
                args_nxt = ctx.Control.Step[cs + 1].args
                arg_str_n = ', '.join((f'{float(args_nxt[i]):.01f}' for i in range(max_args)))
                head += f'Next: Step={cs + 1}, Control={nxt_name}\nArgs: {arg_str_n}\n'
            return head
        return ''

class TorsionalStepController:

    def __init__(self) -> None:
        self.ctx = get_context()
        self._sub_cache: dict[int, TorsionalStepControlClass] = {}

    def _get_sub(self, ctrl_num: int) -> TorsionalStepControlClass | None:
        sub = self._sub_cache.get(ctrl_num)
        if sub is not None:
            return sub
        sub_cls = TorsionalStepControlClass.get_class_by_ctrl_num(ctrl_num)
        if sub_cls is None:
            return None
        sub = sub_cls()
        self._sub_cache[ctrl_num] = sub
        return sub

    def advance_to_next_step(self) -> bool:
        from ctl_torsional_step_ctrl_class import reset_all_cycle_states
        if self.ctx.Control.current_step + 1 >= len(self.ctx.Control.Step):
            reset_all_cycle_states()
            return False
        self.ctx.Control.current_step += 1
        reset_all_cycle_states()
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
