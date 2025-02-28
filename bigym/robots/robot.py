"""BiGym Robot."""
from abc import ABC, abstractmethod
from enum import Enum
from typing import Optional, Union, Iterable

import numpy as np
from dm_control import mjcf
from mojo import Mojo
from mojo.elements import Body, MujocoElement, Geom, Camera, Joint, Site
from mujoco_utils import mjcf_utils

from bigym.action_modes import (
    ActionMode,
    TorqueActionMode,
    JointPositionActionMode,
)
from bigym.robots.config import ArmConfig, RobotConfig
from bigym.const import (
    HandSide,
    WORLD_MODEL,
)
from bigym.envs.props.prop import Prop
from bigym.robots.floating_base import RobotFloatingBase
from bigym.robots.animated_legs import H1AnimatedLegs
from bigym.robots.gripper import Gripper
from bigym.utils.physics_utils import (
    get_critical_damping_from_stiffness,
    get_actuator_qpos,
    get_actuator_qvel,
)


class ActuatorType(Enum):
    """Supported body actuator types."""

    POSITION = "position"
    MOTOR = "motor"
    GENERAL = "general"


class Robot(ABC):
    """Robot."""

    def __init__(
        self,
        action_mode: ActionMode,
        mojo: Optional[Mojo] = None,
    ):
        """Init."""
        self._action_mode = action_mode
        self._mojo = mojo or Mojo(WORLD_MODEL)
        self._body = self._mojo.load_model(
            str(self.config.model), on_loaded=self._on_loaded
        )
        self._grippers = self._get_grippers()
        self._joints = self._get_joints()

        if not self._action_mode.floating_base:
            self._body.set_kinematic(True)

        # Bind robot to action mode
        self._action_mode.bind_robot(self, self._mojo)

    @property
    @abstractmethod
    def config(self) -> RobotConfig:
        """Get robot config."""
        pass

    @property
    def action_mode(self) -> ActionMode:
        """Get action mode."""
        return self._action_mode

    @property
    def pelvis(self) -> Body:
        """Get pelvis."""
        return self._pelvis

    @property
    def limb_actuators(self) -> list[mjcf.Element]:
        """Get all limb actuators."""
        return self._limb_actuators

    @property
    def grippers(self) -> dict[HandSide, Gripper]:
        """Get robot grippers."""
        return self._grippers

    @property
    def floating_base(self) -> Optional[RobotFloatingBase]:
        """Get floating base."""
        return self._floating_base

    @property
    def cameras(self) -> list[Camera]:
        """Get robot cameras."""
        return self._cameras

    @property
    def qpos(self) -> np.ndarray:
        """Get positions of all joints."""
        return np.array(
            [joint.get_joint_position() for joint in self._joints], np.float32
        )

    @property
    def qpos_grippers(self) -> np.ndarray:
        """Get current state of gripper actuators."""
        qpos = []
        for _, gripper in self.grippers.items():
            qpos.append(gripper.qpos)
        return np.array(qpos, np.float32)

    @property
    def qpos_actuated(self) -> np.ndarray:
        """Get positions of actuated joints."""
        qpos = []
        if self.floating_base:
            qpos.extend(self._floating_base.qpos)
        for actuator in self._limb_actuators:
            qpos.append(get_actuator_qpos(actuator, self._mojo.physics))
        qpos.extend(self.qpos_grippers)
        return np.array(qpos, np.float32)

    @property
    def qvel(self) -> np.ndarray:
        """Get velocities of all joints."""
        return np.array(
            [joint.get_joint_velocity() for joint in self._joints], np.float32
        )

    @property
    def qvel_actuated(self) -> np.ndarray:
        """Get velocities of actuated joints."""
        qvel = []
        if self.floating_base:
            qvel.extend(self._floating_base.qvel)
        for actuator in self._limb_actuators:
            qvel.append(get_actuator_qvel(actuator, self._mojo.physics))
        for _, gripper in self._grippers.items():
            qvel.append(gripper.qvel)
        return np.array(qvel, np.float32)

    def get_hand_pos(self, side: HandSide) -> np.ndarray:
        """Get position of robot hand site."""
        if side not in self.config.arms.keys():
            return np.zeros(3)
        return self._grippers[side].wrist_position

    def is_gripper_holding_object(
        self, other: Union[Geom, Iterable[Geom], Prop], side: HandSide
    ) -> bool:
        """Check if the gripper is holding an object."""
        if side not in self.config.arms.keys():
            return False
        return self._grippers[side].is_holding_object(other)

    def reset(self, position: np.ndarray, orientation: np.ndarray):
        """Reset robot."""
        self._set_pose(position, orientation)
        reset_state = (
            self.config.floating_base.reset_state
            if self._action_mode.floating_base
            else self.config.full_body.reset_state
        )
        if reset_state is not None:
            self._action_mode.reset(reset_state)

    def _set_pose(self, position: np.ndarray, orientation: np.ndarray):
        """Instantly set pose of the robot pelvis."""
        if self._action_mode.floating_base:
            self._floating_base.reset(position, orientation)
        else:
            self._pelvis.set_position(position + self.config.full_body.offset_position)
            self._pelvis.set_quaternion(orientation)

    def get_limb_control_range(
        self, actuator: mjcf.Element, absolute: bool
    ) -> np.ndarray:
        """Get control ange of the limb actuator."""
        if not absolute:
            return np.array(self.config.delta_range)
        else:
            return self._mojo.physics.bind(actuator).ctrlrange

    def _get_grippers(self) -> dict[HandSide, Gripper]:
        grippers: dict[HandSide, Gripper] = {}
        for side, arm_config in self.config.arms.items():
            grippers[side] = Gripper(
                side,
                self._wrist_sites[side],
                arm_config,
                self.config.gripper,
                self._mojo,
            )
        return grippers

    def _get_joints(self) -> list[Joint]:
        return [
            j for j in self._body.joints if j.mjcf.name != H1AnimatedLegs.TORSO_JOINT
        ]

    def _on_loaded(self, model: mjcf.RootElement):
        mojo_model = MujocoElement(self._mojo, model)

        # Remove all redundant elements from the model
        for namespace in self.config.namespaces_to_remove:
            elements = model.find_all(namespace)
            for element in elements:
                element.remove()

        # Configure cameras
        self._cameras: list[Camera] = [
            Camera.get(self._mojo, camera, mojo_model) for camera in self.config.cameras
        ]

        # Configure wrist joints
        self._wrist_sites: dict[HandSide, Site] = {}
        for side, arm_config in self.config.arms.items():
            if arm_config.wrist_dof:
                self._add_wrist(model, side, arm_config)
            self._wrist_sites[side]: Site = Site.get(
                self._mojo, arm_config.site, mojo_model
            )

        # Configure pelvis
        self._pelvis: Body = Body.get(self._mojo, self.config.pelvis_body, mojo_model)

        # Always remove free joints
        if self._pelvis.is_kinematic():
            self._pelvis.set_kinematic(False)

        # Reset initial position of pelvis
        self._pelvis.mjcf.pos = np.zeros(3)
        self._pelvis.mjcf.euler = np.zeros(3)

        # List of new actuators with damping to be tuned
        new_actuators: list[mjcf.Element] = []

        # Setup floating base
        self._floating_base: Optional[RobotFloatingBase] = None
        if self._action_mode.floating_base:
            self._floating_base = RobotFloatingBase(
                self.config.floating_base,
                self._pelvis,
                self._action_mode.floating_dofs,
                model,
                self._mojo,
            )
            new_actuators.extend(self._floating_base.all_actuators)

        # Assign limb actuators
        self._limb_actuators: list[mjcf.Element] = []
        all_actuators: list[mjcf.Element] = mjcf_utils.safe_find_all(model, "actuator")
        for actuator in all_actuators:
            actuator_name = actuator.name or actuator.joint.name
            if actuator_name not in self.config.actuators.keys():
                continue
            # Remove actuators not used in floating mode
            if self._floating_base and not self.config.actuators[actuator_name]:
                if actuator.joint:
                    actuator.joint.remove()
                if actuator.tendon:
                    actuator.tendon.remove()
                actuator.remove()
                continue
            if isinstance(self._action_mode, TorqueActionMode):
                if (
                    actuator.tag == ActuatorType.MOTOR.value
                    or actuator.tag == ActuatorType.GENERAL.value
                ):
                    self._limb_actuators.append(actuator)
                else:
                    raise ValueError(
                        f"Actuator {actuator_name} with tag: {actuator.tag} "
                        f"can't be used for {self._action_mode} action mode."
                    )
            elif isinstance(self._action_mode, JointPositionActionMode):
                if (
                    actuator.tag == ActuatorType.POSITION.value
                    or actuator.tag == ActuatorType.GENERAL.value
                ):
                    self._limb_actuators.append(actuator)
                else:
                    actuator_name = actuator.name
                    actuator_joint = actuator.joint
                    actuator_tendon = actuator.tendon
                    actuator.remove()

                    actuator = model.actuator.add(
                        "position",
                        name=actuator_name,
                        joint=actuator_joint,
                        tendon=actuator_tendon,
                        kp=self.config.position_kp,
                        ctrlrange=actuator_joint.range if actuator_joint else None,
                    )
                    self._limb_actuators.append(actuator)
                    new_actuators.append(actuator)

        # Sort limb actuators according to the joints tree
        joints = mjcf_utils.safe_find_all(model, "joint")
        self._limb_actuators.sort(key=lambda a: joints.index(a.joint) if a.joint else 0)

        # Temporary instance of physics to simplify model editing
        physics_tmp = mjcf.Physics.from_mjcf_model(model)

        # Fix joint damping
        for actuator in new_actuators:
            damping = get_critical_damping_from_stiffness(
                actuator.kp, actuator.joint.full_identifier, physics_tmp
            )
            actuator.joint.damping = damping

    @staticmethod
    def _add_wrist(model: mjcf.RootElement, side: HandSide, arm_config: ArmConfig):
        join_name = f"{side.name.lower()}_wrist"
        site_name = arm_config.site

        site = mjcf_utils.safe_find(model, "site", site_name)
        site_pos = np.array(site.pos)
        site_parent = site.parent
        site.remove()

        wrist = site_parent.add("body", name=f"{join_name}_link", pos=site_pos)
        wrist.add(
            "inertial", pos="0 0 0", mass="1e-15", diaginertia="1e-15 1e-15 1e-15"
        )
        wrist.add("site", name=site_name)
        joint = wrist.add(
            "joint",
            type=arm_config.wrist_dof.joint_type.value,
            name=join_name,
            axis=arm_config.wrist_dof.axis,
            range=arm_config.wrist_dof.joint_range,
        )
        model.actuator.add(
            "motor",
            name=join_name,
            joint=joint,
            ctrlrange=arm_config.wrist_dof.action_range,
        )
