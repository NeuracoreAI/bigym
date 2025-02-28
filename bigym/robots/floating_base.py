"""Robot floating base."""
from typing import Optional

import numpy as np
from dm_control import mjcf
from mojo import Mojo
from mojo.elements import Body
from mojo.elements.consts import JointType
from pyquaternion import Quaternion

from bigym.action_modes import PelvisDof
from bigym.const import TOLERANCE_LINEAR, TOLERANCE_ANGULAR
from bigym.robots.animated_legs import AnimatedLegs
from bigym.robots.config import FloatingBaseConfig
from bigym.utils.physics_utils import is_target_reached


DOFS_ORDER = {PelvisDof.X: 1, PelvisDof.Y: 2, PelvisDof.Z: 3, PelvisDof.RZ: 4}


class RobotFloatingBase:
    """Floating base of the robot to simplify control."""

    DELTA_RANGE_POS: tuple[float, float] = (-0.01, 0.01)
    DELTA_RANGE_ROT: tuple[float, float] = (-0.05, 0.05)

    def __init__(
        self,
        config: FloatingBaseConfig,
        pelvis: Body,
        floating_dofs: list[PelvisDof],
        model: mjcf.RootElement,
        mojo: Mojo,
    ):
        """Init."""
        self._config = config
        self._pelvis = pelvis
        self._mojo = mojo
        self._offset_position = np.array(self._config.offset_position)
        self._position_actuators: list[Optional[mjcf.Element]] = [None, None, None]
        self._rotation_actuators: list[Optional[mjcf.Element]] = [None, None, None]

        floating_dofs = sorted(floating_dofs, key=lambda d: DOFS_ORDER[d])
        for i, floating_dof in enumerate(floating_dofs):
            if floating_dof not in self._config.dofs:
                raise ValueError(
                    f"Floating DOF {floating_dof} is not supported by this robot."
                )
            dof = self._config.dofs[floating_dof]
            joint = self._pelvis.mjcf.add(
                "joint",
                type=dof.joint_type.value,
                name=floating_dof.value,
                axis=dof.axis,
            )
            if dof.joint_range:
                joint.limited = True
                joint.range = dof.joint_range
            else:
                joint.limited = False

            actuator = model.actuator.insert(
                "position",
                position=i,
                name=floating_dof.value,
                joint=joint,
                kp=dof.stiffness,
            )
            if dof.action_range:
                actuator.ctrllimited = True
                actuator.ctrlrange = dof.action_range
            else:
                actuator.ctrllimited = False

            self._add_actuator(
                positional=dof.joint_type == JointType.SLIDE,
                axis=np.array(dof.axis),
                actuator=actuator,
            )

        self._animated_legs: Optional[AnimatedLegs] = None
        if self._config.animated_legs_class:
            self._animated_legs = self._config.animated_legs_class(
                self._mojo, self._pelvis
            )

        self._accumulated_actions: np.ndarray = np.zeros(len(self.all_actuators))
        self._last_action: np.ndarray = np.zeros(len(self.all_actuators))

    def reset(self, position: np.ndarray, quaternion: np.ndarray):
        """Set position and orientation of the floating base."""
        self._accumulated_actions *= 0
        self._last_action *= 0

        self._set_position(position)
        self._set_quaternion(quaternion)
        if self._animated_legs:
            self._animated_legs.step(self._pelvis_z, False)

    def get_action_bounds(self) -> list[tuple[float, float]]:
        """Get action bounds of all actuators."""
        bounds = []
        for actuator in self._position_actuators:
            if actuator:
                bounds.append(self._config.delta_range_position)
        for actuator in self._rotation_actuators:
            if actuator:
                bounds.append(self._config.delta_range_rotation)
        return bounds

    def set_control(self, control: np.ndarray):
        """Set control of all actuators."""
        self._accumulated_actions += self._last_action
        self._last_action = control.copy()

        def set_actuator_control(actuator_mjcf: mjcf.Element, ctrl):
            bound_actuator = self._mojo.physics.bind(actuator_mjcf)
            bound_actuator.ctrl += ctrl
            if actuator_mjcf.ctrlrange is not None:
                clipped = np.clip(bound_actuator.ctrl, *bound_actuator.ctrlrange)
                bound_actuator.ctrl = clipped

        index = 0
        for actuator in self._position_actuators:
            if not actuator:
                continue
            set_actuator_control(actuator, control[index])
            index += 1
        for actuator in self._rotation_actuators:
            if not actuator:
                continue
            set_actuator_control(actuator, control[index])
            index += 1
        if self._animated_legs:
            self._animated_legs.step(self._pelvis_z)

    @property
    def is_target_reached(self) -> bool:
        """Check if the target state of all actuators is reached."""
        for actuator in self._position_actuators:
            if actuator and not is_target_reached(
                actuator, self._mojo.physics, TOLERANCE_LINEAR
            ):
                return False
        for actuator in self._rotation_actuators:
            if actuator and not is_target_reached(
                actuator, self._mojo.physics, TOLERANCE_ANGULAR
            ):
                return False
        return True

    @property
    def dof_amount(self) -> int:
        """Get number of actuated DOF."""
        return len(self.all_actuators)

    @property
    def qpos(self) -> np.ndarray:
        """Get positions of actuated joints."""
        qpos = []
        for actuator in self._position_actuators:
            if actuator:
                qpos.append(self._mojo.physics.bind(actuator.joint).qpos.item())
        for actuator in self._rotation_actuators:
            if actuator:
                qpos.append(self._mojo.physics.bind(actuator.joint).qpos.item())
        return np.array(qpos, np.float32)

    @property
    def qvel(self) -> np.ndarray:
        """Get velocities of actuated joints."""
        qpos = []
        for actuator in self._position_actuators:
            if actuator:
                qpos.append(self._mojo.physics.bind(actuator.joint).qvel.item())
        for actuator in self._rotation_actuators:
            if actuator:
                qpos.append(self._mojo.physics.bind(actuator.joint).qvel.item())
        return np.array(qpos, np.float32)

    @property
    def get_accumulated_actions(self) -> np.ndarray:
        """Get accumulated actions since last reset."""
        return np.array(self._accumulated_actions, np.float32)

    @property
    def all_actuators(self) -> list[mjcf.Element]:
        """Get all actuators."""
        return [a for a in self._position_actuators if a] + [
            a for a in self._rotation_actuators if a
        ]

    @property
    def position_actuators(self) -> list[Optional[mjcf.Element]]:
        """Get all position actuators."""
        return self._position_actuators

    @property
    def rotation_actuators(self) -> list[Optional[mjcf.Element]]:
        """Get all rotation actuators."""
        return self._rotation_actuators

    @property
    def _pelvis_z(self) -> float:
        if self._position_actuators[2]:
            joint = self._mojo.physics.bind(self._position_actuators[2].joint)
            return float(joint.qpos)
        else:
            pelvis = self._mojo.physics.bind(self._pelvis.mjcf)
            return float(pelvis.pos[2])

    def _set_position(self, position: np.ndarray):
        position = position + self._offset_position
        self._set_value(True, position)

    def _set_quaternion(self, quaternion: np.ndarray):
        rotation = np.flip(np.array(Quaternion(quaternion).yaw_pitch_roll))
        self._set_value(False, rotation)

    def _set_value(self, position: bool, values: np.ndarray):
        actuators = self._position_actuators if position else self._rotation_actuators
        assert len(values) == len(actuators)
        for i, value, actuator in zip(range(len(values)), values, actuators):
            if actuator:
                bound_joint = self._mojo.physics.bind(actuator.joint)
                bound_joint.qpos = value
                bound_joint.qvel *= 0
                bound_joint.qacc *= 0
                self._mojo.physics.bind(actuator).ctrl = value
            else:
                pelvis = self._mojo.physics.bind(self._pelvis.mjcf)
                if position:
                    pelvis.pos[i] = value
                else:
                    # ToDo: implement rotation setting
                    pass

    def _add_actuator(self, positional: bool, axis: np.ndarray, actuator: mjcf.Element):
        """Add floating base actuator."""
        actuator_index = np.argmax(axis)
        actuators = self._position_actuators if positional else self._rotation_actuators
        actuators[actuator_index] = actuator
