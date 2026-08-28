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
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, ClassVar
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from utl_context import DSS_CHDEF_STEPCTRL_ARGS_MAX, CDSBPyContext, CyclicState, get_context
if TYPE_CHECKING:
    from ctl_motor import CtrlResult

@dataclass(frozen=True, slots=True)
class ArgSpec:
    description: str
    unit: str = '-'

class MotorStepControlClass(ABC):
    CTRL_NUM: ClassVar[int] = -1
    NAME: ClassVar[str] = ''
    DESCRIPTION: ClassVar[str] = ''
    ARGS: ClassVar[tuple[ArgSpec, ...]] = ()
    MAX_ARGS: ClassVar[int] = 0
    _REGISTRY: ClassVar[dict[int, type[MotorStepControlClass]]] = {}

    def __init_subclass__(cls, **kwargs: object) -> None:
        super().__init_subclass__(**kwargs)
        cls.CTRL_NUM = len(cls._REGISTRY)
        cls.MAX_ARGS = len(cls.ARGS)
        if cls.MAX_ARGS < 0 or cls.MAX_ARGS > DSS_CHDEF_STEPCTRL_ARGS_MAX:
            raise ValueError(f'{cls.__name__}.MAX_ARGS={cls.MAX_ARGS} out of range [0, {DSS_CHDEF_STEPCTRL_ARGS_MAX}]')
        for i, spec in enumerate(cls.ARGS):
            if not isinstance(spec, ArgSpec):
                raise TypeError(f'{cls.__name__}.ARGS[{i}] must be ArgSpec, got {type(spec)}')
        cls._REGISTRY[cls.CTRL_NUM] = cls

    def __init__(self) -> None:
        self.ctx = get_context()
        from ctl_motor import MotorController
        self.motor = MotorController()

    @abstractmethod
    def process(self, args: np.ndarray, dt_sec: float) -> CtrlResult:
        ...

    @classmethod
    def get_by_ctrl_num(cls, ctrl_num: int) -> MotorStepControlClass | None:
        sub_cls = cls._REGISTRY.get(ctrl_num)
        return sub_cls() if sub_cls is not None else None

    @classmethod
    def get_class_by_ctrl_num(cls, ctrl_num: int) -> type[MotorStepControlClass] | None:
        return cls._REGISTRY.get(ctrl_num)

    @classmethod
    def all_subclasses(cls) -> list[type[MotorStepControlClass]]:
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

def run_cyclic_4quadrant(motor: object, ctx: CDSBPyContext, cycle_num: int, cycle_state: CyclicState, args: np.ndarray, dt_sec: float, axis_value_getter: Callable[[CDSBPyContext], float]) -> tuple[int, CyclicState, CtrlResult]:
    from ctl_motor import CtrlResult, StepLoadDir, resolve_load_dir
    load_dir = resolve_load_dir(float(args[0]))
    speed = float(args[1])
    low = float(args[2])
    up = float(args[3])
    cycles = float(args[4])
    eff_rad = float(args[5])
    if low > up:
        return (cycle_num, cycle_state, CtrlResult.STOP)
    if eff_rad > 0.0:
        motor._apply_radial_pressure_pid(eff_rad, dt_sec)
    motor.set_motor_clutch(True)
    if cycle_num == 0:
        return (1, CyclicState.FIRST_QUARTER, CtrlResult.CONTINUE)
    if cycle_num > cycles:
        motor.set_motor_speed(0.0)
        return (0, CyclicState.INIT, CtrlResult.NEXT_STEP)
    motor.set_motor_speed(speed)
    value = axis_value_getter(ctx)
    center = 0.5 * (low + up)
    new_num = cycle_num
    new_state = cycle_state
    if load_dir is StepLoadDir.COMPRESSION:
        if cycle_state == CyclicState.FIRST_QUARTER:
            if value <= up:
                motor.set_motor_direction_down()
            else:
                motor.set_motor_direction_up()
                new_state = CyclicState.SECOND_QUARTER
        elif cycle_state == CyclicState.SECOND_QUARTER:
            motor.set_motor_direction_up()
            if value < center:
                new_state = CyclicState.THIRD_QUARTER
        elif cycle_state == CyclicState.THIRD_QUARTER:
            if value >= low:
                motor.set_motor_direction_up()
            else:
                motor.set_motor_direction_down()
                new_state = CyclicState.FOURTH_QUARTER
        elif cycle_state == CyclicState.FOURTH_QUARTER:
            motor.set_motor_direction_down()
            if value > center:
                motor.set_motor_direction_down()
                new_state = CyclicState.FIRST_QUARTER
                new_num = cycle_num + 1
    elif load_dir is StepLoadDir.EXTENSION:
        if cycle_state == CyclicState.FIRST_QUARTER:
            if value >= low:
                motor.set_motor_direction_up()
            else:
                motor.set_motor_direction_down()
                new_state = CyclicState.SECOND_QUARTER
        elif cycle_state == CyclicState.SECOND_QUARTER:
            motor.set_motor_direction_down()
            if value > center:
                new_state = CyclicState.THIRD_QUARTER
        elif cycle_state == CyclicState.THIRD_QUARTER:
            if value <= up:
                motor.set_motor_direction_down()
            else:
                motor.set_motor_direction_up()
                new_state = CyclicState.FOURTH_QUARTER
        elif cycle_state == CyclicState.FOURTH_QUARTER:
            motor.set_motor_direction_up()
            if value < center:
                new_state = CyclicState.FIRST_QUARTER
                new_num = cycle_num + 1
    else:
        return (cycle_num, cycle_state, CtrlResult.STOP)
    return (new_num, new_state, CtrlResult.CONTINUE)
from ctl_motor_step_ctrl_stop import MotorStepControl_Stop
from ctl_motor_step_ctrl_monotonic import MotorStepControl_Monotonic
from ctl_motor_step_ctrl_cyclic_stress import MotorStepControl_CyclicStress
from ctl_motor_step_ctrl_cyclic_strain import MotorStepControl_CyclicStrain
from ctl_motor_step_ctrl_creep import MotorStepControl_Creep
from ctl_motor_step_ctrl_linear_stress_path import MotorStepControl_LinearStressPath
