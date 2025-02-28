"""Action modes for H1."""
from __future__ import annotations

from enum import Enum
from typing import TYPE_CHECKING

import warnings
from abc import abstractmethod, ABC
from typing import Optional

import numpy as np
from gymnasium import spaces
from mojo import Mojo

from bigym.const import TOLERANCE_ANGULAR
from bigym.utils.physics_utils import (
    is_target_reached,
)

if TYPE_CHECKING:
    from bigym.robots.robot import Robot


class TargetStateNotReachedWarning(Warning):
    """Warning raised when the target state is not reached within the maximum steps."""

    pass


class PelvisDof(Enum):
    """Set of floating base DOFs."""

    X = "pelvis_x"
    Y = "pelvis_y"
    Z = "pelvis_z"
    RZ = "pelvis_rz"


DEFAULT_DOFS = [PelvisDof.X, PelvisDof.Y, PelvisDof.RZ]


class ActionMode(ABC):
    """Base action mode class used for controlling H1."""

    def __init__(
        self,
        floating_base: bool = True,
        floating_dofs: Optional[list[PelvisDof]] = None,
    ):
        """Init.

        :param floating_base: If True, then legs are frozen, and the robot base
            controlled by positional actuators.
            If False, then user has full control of legs (i.e. for whole-body control).
        :param floating_dofs: Set of floating DOFs. By default, it is: [X, Y, RZ].
        """
        self._floating_base = floating_base
        self._floating_dofs = DEFAULT_DOFS if floating_dofs is None else floating_dofs

        # Will be assigned later
        self._mojo: Optional[Mojo] = None
        self._robot: Optional[Robot] = None

    def bind_robot(self, robot: Robot, mojo: Mojo):
        """Bind action mode to robot."""
        self._robot = robot
        self._mojo = mojo

    @property
    def floating_base(self) -> bool:
        """Is floating base enabled."""
        return self._floating_base

    @property
    def floating_dofs(self) -> list[PelvisDof]:
        """Set of floating DOFs."""
        return self._floating_dofs

    @abstractmethod
    def action_space(
        self, action_scale: float, seed: Optional[int] = None
    ) -> spaces.Box:
        """The action space for this action mode."""
        pass

    @abstractmethod
    def step(self, action: np.ndarray):
        """Apply the control command and step the physics.

        Note: This function has the responsibility of calling `mujoco.mj_step`.

        :param action: The entire action passed to the action mode.
        """
        pass

    @abstractmethod
    def reset(self, reset_state: np.ndarray):
        """Reset state of the robot accordingly to the action mode.

        :param reset_state: Target reset state of robot actuators.
        """
        pass


class TorqueActionMode(ActionMode):
    """Control all joints through torque control.

    Enables the user to control joints using torque values.

    Notes:
        - Grippers are controlled in positional mode.
        - Joints of the 'floating_base' are always controlled in delta position mode.
    """

    def action_space(
        self, action_scale: float, seed: Optional[int] = None
    ) -> spaces.Box:
        """See base."""
        bounds = []
        if self.floating_base:
            action_bounds = self._robot.floating_base.get_action_bounds()
            action_bounds = [np.array(b) * action_scale for b in action_bounds]
            bounds.extend(action_bounds)
        for actuator in self._robot.limb_actuators:
            action_bounds = np.array(actuator.ctrlrange)
            bounds.append(action_bounds)
        for _, gripper in self._robot.grippers.items():
            bounds.append(gripper.range)
        bounds = np.array(bounds).copy().astype(np.float32)
        low, high = bounds.T
        return spaces.Box(
            low=low,
            high=high,
            dtype=np.float32,
            seed=seed,
        )

    def step(self, action: np.ndarray):
        """See base."""
        if self.floating_base:
            base_action = action[: self._robot.floating_base.dof_amount]
            action = action[self._robot.floating_base.dof_amount :]
            self._robot.floating_base.set_control(base_action)
        for i, actuator in enumerate(self._robot.limb_actuators):
            self._mojo.physics.bind(actuator).ctrl = action[i]
        gripper_actions = action[-len(self._robot.grippers) :]
        for side, action in zip(self._robot.grippers, gripper_actions):
            self._robot.grippers[side].set_control(action)
        self._mojo.step()

    def reset(self, reset_state: np.ndarray):
        """See base."""
        if len(reset_state) != len(self._robot.limb_actuators):
            raise ValueError(
                f"Mismatch between reset_state length "
                f"({len(reset_state)}) "
                f"and number of actuators ({len(self._robot.limb_actuators)}). "
                f"Ensure reset_state matches the actuators count in the model."
            )
        for value, actuator in zip(reset_state, self._robot.limb_actuators):
            if actuator.joint:
                joint = self._mojo.physics.bind(actuator.joint)
                joint.qpos = value
                joint.qvel *= 0
                joint.qacc *= 0
            elif actuator.tendon:
                warnings.warn(
                    f"Tendon actuators are not fully supported "
                    f"for {self.__class__.__name__} action mode."
                )


class JointPositionActionMode(ActionMode):
    """Control all joints through joint position.

    Allows to control joint positions, supporting both absolute and delta positions.
    For absolute control, set 'absolute' to True. If the floating base is enabled,
    only delta position control is applied to it.

    Notes:
        - `block_until_reached` does not guarantee reaching the target position because
          the target position could be unreachable due to collisions.
        - Joints of the `floating_base` are always controlled in delta position mode.
    """

    MAX_STEPS = 200

    def __init__(
        self,
        absolute: bool = False,
        block_until_reached: bool = False,
        floating_base: bool = True,
        floating_dofs: list[PelvisDof] = None,
    ):
        """See base.

        :param absolute: Use absolute or delta joint positions.
        :param block_until_reached: Continue stepping until the target
            position is reached or the step threshold is exceeded.
        """
        super().__init__(
            floating_base=floating_base,
            floating_dofs=floating_dofs,
        )
        self.absolute = absolute
        self.block_until_reached = block_until_reached

    def action_space(
        self, action_scale: float, seed: Optional[int] = None
    ) -> spaces.Box:
        """See base."""
        bounds = []
        if self.floating_base:
            action_bounds = self._robot.floating_base.get_action_bounds()
            action_bounds = [np.array(b) * action_scale for b in action_bounds]
            bounds.extend(action_bounds)
        for actuator in self._robot.limb_actuators:
            action_bounds = np.array(
                self._robot.get_limb_control_range(actuator, self.absolute)
            )
            action_bounds *= 1 if self.absolute else action_scale
            bounds.append(action_bounds)
        for _, gripper in self._robot.grippers.items():
            bounds.append(gripper.range)
        bounds = np.array(bounds).copy().astype(np.float32)
        low, high = bounds.T
        return spaces.Box(
            low=low,
            high=high,
            dtype=np.float32,
            seed=seed,
        )

    def step(self, action: np.ndarray):
        """See base."""
        if self.floating_base:
            base_action = action[: self._robot.floating_base.dof_amount]
            action = action[self._robot.floating_base.dof_amount :]
            self._robot.floating_base.set_control(base_action)
        for i, actuator in enumerate(self._robot.limb_actuators):
            actuator = self._mojo.physics.bind(actuator)
            actuator.ctrl = action[i] if self.absolute else actuator.ctrl + action[i]
        gripper_actions = action[-len(self._robot.grippers) :]
        for side, action in zip(self._robot.grippers, gripper_actions):
            self._robot.grippers[side].set_control(action)

        if self.block_until_reached:
            self._step_until_reached()
        else:
            self._mojo.step()

    def reset(self, reset_state: np.ndarray):
        """See base."""
        if len(reset_state) != len(self._robot.limb_actuators):
            raise ValueError(
                f"Mismatch between reset_state length "
                f"({len(reset_state)}) "
                f"and number of actuators ({len(self._robot.limb_actuators)}). "
                f"Ensure reset_state matches the actuators count in the model."
            )
        for value, actuator in zip(reset_state, self._robot.limb_actuators):
            if actuator.joint:
                bound_joint = self._mojo.physics.bind(actuator.joint)
                bound_joint.qpos = value
                bound_joint.qvel *= 0
                bound_joint.qacc *= 0
            elif actuator.tendon:
                if actuator.tendon.joint is None or len(actuator.tendon.joint) == 0:
                    raise RuntimeError(
                        "Currently only fixed tendons with joints are supported."
                    )
                joint_value = value / len(actuator.tendon.joint)
                for tendon_joint in actuator.tendon.joint:
                    value_coefficient = tendon_joint.coef
                    bound_joint = self._mojo.physics.bind(tendon_joint.joint)
                    bound_joint.qpos = joint_value / value_coefficient
                    bound_joint.qvel *= 0
                    bound_joint.qacc *= 0
            bound_actuator = self._mojo.physics.bind(actuator)
            bound_actuator.ctrl = value

    def _step_until_reached(self):
        """Step physics until the target position is reached."""
        steps_counter = 0
        while True:
            self._mojo.step()
            steps_counter += 1
            if self._is_target_state_reached() or steps_counter >= self.MAX_STEPS:
                if steps_counter >= self.MAX_STEPS:
                    warnings.warn(
                        f"Failed to reach target state in " f"{self.MAX_STEPS} steps!",
                        TargetStateNotReachedWarning,
                    )
                break

    def _is_target_state_reached(self):
        if self.floating_base:
            if not self._robot.floating_base.is_target_reached:
                return False
        for actuator in self._robot.limb_actuators:
            if not is_target_reached(actuator, self._mojo.physics, TOLERANCE_ANGULAR):
                return False
        return True
