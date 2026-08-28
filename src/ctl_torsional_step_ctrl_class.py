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
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, ClassVar
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ctl_motor_step_ctrl_class import ArgSpec
from utl_context import get_context
__all__ = ['ArgSpec', 'TorsionalStepControlClass']
if TYPE_CHECKING:
    from ctl_motor import CtrlResult

class TorsionalStepControlClass(ABC):
    CTRL_NUM: ClassVar[int] = -1
    NAME: ClassVar[str] = ''
    DESCRIPTION: ClassVar[str] = ''
    ARGS: ClassVar[tuple[ArgSpec, ...]] = ()
    MAX_ARGS: ClassVar[int] = 0
    _REGISTRY: ClassVar[dict[int, type[TorsionalStepControlClass]]] = {}

    def __init_subclass__(cls, **kwargs: object) -> None:
        super().__init_subclass__(**kwargs)
        cls.CTRL_NUM = len(cls._REGISTRY)
        cls.MAX_ARGS = len(cls.ARGS)
        from utl_context import DSS_CHDEF_STEPCTRL_ARGS_MAX
        if cls.MAX_ARGS < 0 or cls.MAX_ARGS > DSS_CHDEF_STEPCTRL_ARGS_MAX:
            raise ValueError(f'{cls.__name__}.MAX_ARGS={cls.MAX_ARGS} out of range [0, {DSS_CHDEF_STEPCTRL_ARGS_MAX}]')
        for i, spec in enumerate(cls.ARGS):
            if not isinstance(spec, ArgSpec):
                raise TypeError(f'{cls.__name__}.ARGS[{i}] must be ArgSpec, got {type(spec)}')
        cls._REGISTRY[cls.CTRL_NUM] = cls

    def __init__(self) -> None:
        self.ctx = get_context()
        from ctl_motor import MotorController
        from ctl_torsional import TorsionalController
        self.motor = MotorController()
        self.torsional = TorsionalController()

    @abstractmethod
    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        ...

    @classmethod
    def get_by_ctrl_num(cls, ctrl_num: int) -> TorsionalStepControlClass | None:
        sub_cls = cls._REGISTRY.get(ctrl_num)
        return sub_cls() if sub_cls is not None else None

    @classmethod
    def get_class_by_ctrl_num(cls, ctrl_num: int) -> type[TorsionalStepControlClass] | None:
        return cls._REGISTRY.get(ctrl_num)

    @classmethod
    def all_subclasses(cls) -> list[type[TorsionalStepControlClass]]:
        return [cls._REGISTRY[k] for k in sorted(cls._REGISTRY)]

    @classmethod
    def format_args_info(cls) -> str:
        lines: list[str] = []
        for sub_cls in cls.all_subclasses():
            head = f'  {sub_cls.CTRL_NUM}: {sub_cls.NAME}'
            if sub_cls.DESCRIPTION:
                head += f' ({sub_cls.DESCRIPTION})'
            lines.append(head)
            for i, spec in enumerate(sub_cls.ARGS):
                unit = spec.unit or '-'
                lines.append(f'    [{i}] {spec.description} [{unit}]')
            lines.append('')
        if lines and lines[-1] == '':
            lines.pop()
        return '\n'.join(lines)

def approach_by_rate(cur_val: float, target: float, rate_per_min: float, dt_sec: float) -> float:
    delta = rate_per_min * dt_sec / 60.0
    if cur_val < target:
        return min(cur_val + delta, target)
    if cur_val > target:
        return max(cur_val - delta, target)
    return cur_val

def servo_ep_cell_proportional(motor: object, target_sr_kpa: float, gain: float, dt_sec: float, tick_sec: float=0.5) -> None:
    cur = get_context().Current.e_sr
    if tick_sec <= 0:
        tick_sec = 0.5
    delta_kpa = gain * (target_sr_kpa - cur) * (dt_sec / tick_sec)
    motor.add_ep_cell_pressure_kpa_signed(float(delta_kpa))
from ctl_torsional_step_ctrl_stop import TorsionalStepControl_Stop
from ctl_torsional_step_ctrl_effective_stress_path import TorsionalStepControl_EffectiveStressPath
from ctl_torsional_step_ctrl_monotonic import TorsionalStepControl_MonotonicTorsionalLoading
from ctl_torsional_step_ctrl_monotonic_torsional_loading import TorsionalStepControl_MonotonicTorsionalLoadingCNS
from ctl_torsional_step_ctrl_cyclic_torsional_loading import TorsionalStepControl_CyclicTorsionalLoading
from ctl_torsional_step_ctrl_cyclic_torsional_loading_cns import TorsionalStepControl_CyclicTorsionalLoadingCNS
from ctl_torsional_step_ctrl_small_cyclic_torsional_loading import TorsionalStepControl_SmallCyclicTorsionalLoading
from ctl_torsional_step_ctrl_small_cyclic_torsional_loading_cns import TorsionalStepControl_SmallCyclicTorsionalLoadingCNS
from ctl_torsional_step_ctrl_monotonic_axial_loading import TorsionalStepControl_MonotonicAxialLoading
from ctl_torsional_step_ctrl_cyclic_axial_loading import TorsionalStepControl_CyclicAxialLoading
from ctl_torsional_step_ctrl_small_cyclic_axial_loading import TorsionalStepControl_SmallCyclicAxialLoading
from ctl_torsional_step_ctrl_creep import TorsionalStepControl_Creep
from ctl_torsional_step_ctrl_monotonic_axial_loading_const_p import TorsionalStepControl_MonotonicAxialLoadingConstP
from ctl_torsional_step_ctrl_monotonic_torsional_loading_const_pa import TorsionalStepControl_MonotonicTorsionalLoadingConstPA
from ctl_torsional_step_ctrl_cyclic_axial_loading_or import TorsionalStepControl_CyclicAxialLoadingOR
from ctl_torsional_step_ctrl_file_controlable_consolidation import TorsionalStepControl_FileControlableConsolidation

def reset_all_cycle_states() -> None:
    TorsionalStepControl_CyclicTorsionalLoading.reset_cycle()
    TorsionalStepControl_CyclicTorsionalLoadingCNS.reset_cycle()
    TorsionalStepControl_SmallCyclicTorsionalLoading.reset_cycle()
    TorsionalStepControl_SmallCyclicTorsionalLoadingCNS.reset_cycle()
    TorsionalStepControl_CyclicAxialLoading.reset_cycle()
    TorsionalStepControl_SmallCyclicAxialLoading.reset_cycle()
    TorsionalStepControl_Creep.reset_cycle()
    TorsionalStepControl_CyclicAxialLoadingOR.reset_cycle()
    TorsionalStepControl_FileControlableConsolidation.reset_cycle()
