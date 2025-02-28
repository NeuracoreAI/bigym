"""H1 Robot."""
import numpy as np
from mojo.elements.consts import JointType

from bigym.action_modes import PelvisDof
from bigym.const import ASSETS_PATH, HandSide
from bigym.robots.animated_legs import H1AnimatedLegs

from bigym.robots.config import (
    ArmConfig,
    FloatingBaseConfig,
    RobotConfig,
    FullBodyConfig,
)
from bigym.robots.configs.robotiq import ROBOTIQ_2F85, ROBOTIQ_2F85_FINE_MANIPULATION
from bigym.robots.robot import Robot
from bigym.utils.dof import Dof

H1_WRIST_DOF = Dof(
    joint_type=JointType.HINGE,
    axis=(1, 0, 0),
    joint_range=(-1.5708, 1.5708),
    action_range=(-18, 18),
)
H1_LEFT_ARM = ArmConfig(
    site="left_end_effector",
    links=[
        "left_shoulder_pitch_link",
        "left_shoulder_roll_link",
        "left_shoulder_yaw_link",
        "left_elbow_link",
    ],
    wrist_dof=H1_WRIST_DOF,
    offset_euler=np.array([np.pi / 2, np.pi / 2, 0]),
)
H1_RIGHT_ARM = ArmConfig(
    site="right_end_effector",
    links=[
        "right_shoulder_pitch_link",
        "right_shoulder_roll_link",
        "right_shoulder_yaw_link",
        "right_elbow_link",
    ],
    wrist_dof=H1_WRIST_DOF,
    offset_euler=np.array([np.pi / 2, np.pi / 2, 0]),
)
H1_ACTUATORS = {
    "left_hip_yaw": False,
    "left_hip_roll": False,
    "left_hip_pitch": False,
    "left_knee": False,
    "left_ankle": False,
    "right_hip_yaw": False,
    "right_hip_roll": False,
    "right_hip_pitch": False,
    "right_knee": False,
    "right_ankle": False,
    "torso": False,
    "left_shoulder_pitch": True,
    "left_shoulder_roll": True,
    "left_shoulder_yaw": True,
    "left_elbow": True,
    "left_wrist": True,
    "right_shoulder_pitch": True,
    "right_shoulder_roll": True,
    "right_shoulder_yaw": True,
    "right_elbow": True,
    "right_wrist": True,
}
STIFFNESS_XY = 1e4
STIFFNESS_Z = 1e6
RANGE_DOF_Z = (0.4, 1.0)
H1_FULL_BODY = FullBodyConfig(
    offset_position=np.array([0, 0, 0.98]),
    reset_state=np.array(
        [
            0,  # left_hip_yaw
            0,
            -0.4,
            0.8,
            -0.4,
            0,  # right_hip_yaw
            0,
            -0.4,
            0.8,
            -0.4,
            0,  # torso
            0,  # left_shoulder_pitch
            0,
            0,
            0,
            0,
            0,  # right_shoulder_pitch
            0,
            0,
            0,
            0,
        ]
    ),
)
H1_FLOATING_BASE = FloatingBaseConfig(
    dofs={
        PelvisDof.X: Dof(
            joint_type=JointType.SLIDE,
            axis=(1, 0, 0),
            stiffness=STIFFNESS_XY,
        ),
        PelvisDof.Y: Dof(
            joint_type=JointType.SLIDE,
            axis=(0, 1, 0),
            stiffness=STIFFNESS_XY,
        ),
        PelvisDof.Z: Dof(
            joint_type=JointType.SLIDE,
            axis=(0, 0, 1),
            joint_range=RANGE_DOF_Z,
            action_range=RANGE_DOF_Z,
            stiffness=STIFFNESS_Z,
        ),
        PelvisDof.RZ: Dof(
            joint_type=JointType.HINGE,
            axis=(0, 0, 1),
            stiffness=STIFFNESS_XY,
        ),
    },
    delta_range_position=(-0.01, 0.01),
    delta_range_rotation=(-0.05, 0.05),
    offset_position=np.array([0, 0, 1]),
    reset_state=np.array(
        [
            0,  # left_shoulder_pitch
            0,
            0,
            0,
            0,
            0,  # right_shoulder_pitch
            0,
            0,
            0,
            0,
        ]
    ),
    animated_legs_class=H1AnimatedLegs,
)
H1_CONFIG = RobotConfig(
    model=ASSETS_PATH / "h1/h1.xml",
    delta_range=(-0.1, 0.1),
    position_kp=300,
    pelvis_body="pelvis",
    full_body=H1_FULL_BODY,
    floating_base=H1_FLOATING_BASE,
    gripper=ROBOTIQ_2F85,
    arms={HandSide.LEFT: H1_LEFT_ARM, HandSide.RIGHT: H1_RIGHT_ARM},
    actuators=H1_ACTUATORS,
    cameras=["head", "left_wrist", "right_wrist"],
    namespaces_to_remove=["light"],
)
H1_FINE_MANIPULATION_CONFIG = RobotConfig(
    model=ASSETS_PATH / "h1/h1.xml",
    delta_range=(-0.1, 0.1),
    position_kp=300,
    pelvis_body="pelvis",
    full_body=H1_FULL_BODY,
    floating_base=H1_FLOATING_BASE,
    gripper=ROBOTIQ_2F85_FINE_MANIPULATION,
    arms={HandSide.LEFT: H1_LEFT_ARM, HandSide.RIGHT: H1_RIGHT_ARM},
    actuators=H1_ACTUATORS,
    cameras=["head", "left_wrist", "right_wrist"],
    namespaces_to_remove=["light"],
)


class H1(Robot):
    """H1 Robot."""

    @property
    def config(self) -> RobotConfig:
        """Get robot config."""
        return H1_CONFIG


class H1FineManipulation(Robot):
    """H1 Robot with Robotiq gripper for fine manipulations."""

    @property
    def config(self) -> RobotConfig:
        """Get robot config."""
        return H1_FINE_MANIPULATION_CONFIG
