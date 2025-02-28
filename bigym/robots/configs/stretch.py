"""Hello Robotics Stretch Robot."""
import numpy as np
from mojo.elements.consts import JointType

from bigym.action_modes import PelvisDof
from bigym.const import ASSETS_PATH, HandSide
from bigym.robots.config import (
    ArmConfig,
    FloatingBaseConfig,
    GripperConfig,
    RobotConfig,
    FullBodyConfig,
)
from bigym.robots.robot import Robot
from bigym.utils.dof import Dof

STRETCH_ACTUATORS = {
    "forward": False,
    "turn": False,
    "lift": True,
    "arm_extend": True,
    "wrist_yaw": True,
    "grip": True,
    "head_pan": True,
    "head_tilt": True,
}
STRETCH_HAND = ArmConfig(
    site="wrist",
    links=[
        "link_lift",
        "link_arm_l4",
        "link_arm_l3",
        "link_arm_l2",
        "link_arm_l1",
        "link_arm_l0",
        "link_wrist_yaw",
    ],
)
STIFFNESS = 1e4
STRETCH_FULL_BODY = FullBodyConfig(
    offset_position=np.zeros(3),
    reset_state=None,
)
STRETCH_FLOATING_BASE = FloatingBaseConfig(
    dofs={
        PelvisDof.X: Dof(
            joint_type=JointType.SLIDE,
            axis=(1, 0, 0),
            stiffness=STIFFNESS,
        ),
        PelvisDof.Y: Dof(
            joint_type=JointType.SLIDE,
            axis=(0, 1, 0),
            stiffness=STIFFNESS,
        ),
        PelvisDof.RZ: Dof(
            joint_type=JointType.HINGE,
            axis=(0, 0, 1),
            stiffness=STIFFNESS,
        ),
    },
    delta_range_position=(-0.01, 0.01),
    delta_range_rotation=(-0.05, 0.05),
    reset_state=np.array([0, 0, 0, 0, 0, 0]),
)
STRETCH_GRIPPER = GripperConfig(
    body="stretch/link_gripper_slider",
    actuators=["grip"],
    pad_bodies=["rubber_tip_left", "rubber_tip_right"],
    range=np.array([0, 1]),
)
STRETCH_ROBOT = RobotConfig(
    model=ASSETS_PATH / "hello_robot_stretch/stretch.xml",
    delta_range=(-0.1, 0.1),
    position_kp=300,
    pelvis_body="base_link",
    full_body=STRETCH_FULL_BODY,
    floating_base=STRETCH_FLOATING_BASE,
    gripper=STRETCH_GRIPPER,
    arms={HandSide.RIGHT: STRETCH_HAND},
    actuators=STRETCH_ACTUATORS,
    namespaces_to_remove=["light"],
)


class StretchRobot(Robot):
    """Hello Robotics Stretch Robot."""

    @property
    def config(self) -> RobotConfig:
        """Get robot config."""
        return STRETCH_ROBOT
