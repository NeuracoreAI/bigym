"""Robot Gripper."""
from typing import Union, Iterable, Optional

import numpy as np
from dm_control import mjcf
from mojo import Mojo
from mojo.elements import Geom, Site, MujocoElement, Body
from mujoco_utils import mjcf_utils

from bigym.const import HandSide
from bigym.envs.props.prop import Prop
from bigym.robots.config import GripperConfig, ArmConfig
from bigym.utils.physics_utils import get_colliders, has_collided_collections


class Gripper:
    """Robot Gripper."""

    _NORMAL_RANGE = (0, 1)
    _ROUND_DECIMALS = 1

    def __init__(
        self,
        side: HandSide,
        wrist_site: Site,
        arm_config: ArmConfig,
        gripper_config: GripperConfig,
        mojo: Mojo,
    ):
        """Init."""
        self._side = side
        self._wrist_site = wrist_site
        self._config = gripper_config
        self._mojo = mojo

        self._pad_bodies: list[Body] = []
        self._pad_geoms: list[Geom] = []
        self._actuators: list[mjcf.Element] = []
        self._actuated_joints: list[mjcf.Element] = []
        self._pinch_site: Optional[Site] = None
        self._target_ctrl: Optional[float] = None

        if self._config.model:
            self._body: Body = self._mojo.load_model(
                str(gripper_config.model), self._wrist_site, on_loaded=self._on_loaded
            )
            self._body.mjcf.pos = arm_config.offset_position
            self._body.mjcf.euler = arm_config.offset_euler
            self._mojo.mark_dirty()
        elif self._config.body:
            self._body: Body = Body.get(
                self._mojo, self._config.body, mojo.root_element
            )
            self._process_gripper(mojo.root_element.mjcf, self._body)
        self._pad_geoms = self._get_pad_geoms()

    @property
    def body(self) -> Body:
        """Get gripper body."""
        return self._body

    @property
    def wrist_site(self) -> Site:
        """Get wrist site."""
        return self._wrist_site

    @property
    def pinch_position(self) -> np.ndarray:
        """Get position of the pinch site."""
        return self._mojo.physics.bind(self._pinch_site.mjcf).xpos.copy()

    @property
    def wrist_position(self) -> np.ndarray:
        """Get position of the wrist site."""
        return self._mojo.physics.bind(self._wrist_site.mjcf).xpos.copy()

    @property
    def range(self) -> np.ndarray:
        """Get gripper control range."""
        return self._config.range

    @property
    def actuators(self) -> list[mjcf.Element]:
        """Get list of gripper actuators."""
        return self._actuators

    @property
    def qpos(self) -> float:
        """Get average qpos of actuated joints."""
        positions = []
        for joint in self._actuated_joints:
            joint = self._mojo.physics.bind(joint)
            positions.append(
                np.interp(joint.qpos.item(), joint.range, self._config.range)
            )
        return np.round(np.average(positions), decimals=self._ROUND_DECIMALS)

    @property
    def qvel(self) -> float:
        """Get current velocity of gripper actuators."""
        velocities = []
        for joint in self._actuated_joints:
            joint = self._mojo.physics.bind(joint)
            velocities.append(joint.qvel.item())
        return np.round(np.average(velocities), decimals=self._ROUND_DECIMALS)

    def is_holding_object(self, other: Union[Geom, Iterable[Geom], Prop]) -> bool:
        """Check if gripper is holding object."""
        return has_collided_collections(
            self._mojo.physics, self._pad_geoms, get_colliders(other)
        )

    def set_control(self, ctrl: float):
        """Set state of the gripper."""
        ctrl = np.interp(ctrl, self._config.range, self._NORMAL_RANGE)
        if self._config.discrete:
            ctrl = np.round(ctrl)
        ctrl = np.interp(ctrl, self._NORMAL_RANGE, self._config.range)
        self._target_ctrl = float(ctrl)
        for actuator in self._actuators:
            actuator = self._mojo.physics.bind(actuator)
            actuator_ctrl = np.interp(ctrl, self._config.range, actuator.ctrlrange)
            actuator.ctrl = actuator_ctrl

    def is_target_reached(self, tolerance: float) -> bool:
        """Check if the gripper is sufficiently close to the last control target."""
        if self._target_ctrl is None:
            return True
        return abs(self.qpos - self._target_ctrl) <= tolerance

    def _on_loaded(self, model: mjcf.RootElement):
        model.model += f"_{self._side.value.lower()}"
        self._process_gripper(model, MujocoElement(self._mojo, model))

    def _process_gripper(
        self, model: mjcf.RootElement, element: Union[MujocoElement | Body]
    ):
        if self._config.pinch_site:
            self._pinch_site: Site = Site.get(
                self._mojo, self._config.pinch_site, element
            )
        else:
            self._pinch_site: Site = Site.create(
                self._mojo, parent=element, size=np.repeat(0.01, 3), group=5
            )

        # Cache gripper pads
        if self._config.pad_bodies:
            for pad_name in self._config.pad_bodies:
                self._pad_bodies.append(Body.get(self._mojo, pad_name, element))

        # Configure actuators and joints
        all_actuators: list[mjcf.Element] = mjcf_utils.safe_find_all(model, "actuator")
        for actuator in all_actuators:
            actuator_name = actuator.name or actuator.joint.name
            if actuator_name not in self._config.actuators:
                continue
            # Caching actuators
            self._actuators.append(actuator)
            # Caching actuated gripper joints
            if actuator.tendon:
                for tendon_joint in actuator.tendon.joint:
                    self._actuated_joints.append(tendon_joint.joint)
            elif actuator.joint:
                self._actuated_joints.append(actuator.joint)

    def _get_pad_geoms(self) -> list[Geom]:
        geoms = []
        for body in self._pad_bodies:
            geoms.extend([g for g in body.geoms if g.is_collidable()])
        return geoms
