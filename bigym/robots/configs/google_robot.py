"""Google Robot."""
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

GOOGLE_ROBOT_ACTUATORS = {
    "joint_torso": True,
    "joint_shoulder": True,
    "joint_bicep": True,
    "joint_elbow": True,
    "joint_forearm": True,
    "joint_wrist": True,
    "joint_gripper": True,
}
GOOGLE_ROBOT_HAND = ArmConfig(
    site="wrist",
    links=[
        "link_torso",
        "link_shoulder",
        "link_bicep",
        "link_elbow",
        "link_forearm",
        "link_wrist",
        "link_gripper",
    ],
)
STIFFNESS = 1e4
GOOGLE_ROBOT_FULL_BODY = FullBodyConfig(
    offset_position=np.zeros(3),
    reset_state=None,
)
GOOGLE_ROBOT_FLOATING_BASE = FloatingBaseConfig(
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
    offset_position=np.array([0, 0, 0.065]),
)
GOOGLE_ROBOT_GRIPPER = GripperConfig(
    body="robot/link_gripper",
    pinch_site="gripper",
    actuators=["joint_finger_right", "joint_finger_left"],
    pad_bodies=["link_finger_tip_left", "link_finger_tip_right"],
    range=np.array([0, 1]),
)
GOOGLE_ROBOT = RobotConfig(
    model=ASSETS_PATH / "google_robot/robot.xml",
    delta_range=(-0.1, 0.1),
    position_kp=300,
    pelvis_body="base_link",
    full_body=GOOGLE_ROBOT_FULL_BODY,
    floating_base=GOOGLE_ROBOT_FLOATING_BASE,
    gripper=GOOGLE_ROBOT_GRIPPER,
    arms={HandSide.RIGHT: GOOGLE_ROBOT_HAND},
    actuators=GOOGLE_ROBOT_ACTUATORS,
    namespaces_to_remove=["light"],
)


class GoogleRobot(Robot):
    """Google Robot."""

    @property
    def config(self) -> RobotConfig:
        """Get robot config."""
        return GOOGLE_ROBOT
