"""Robot configs."""

from pathlib import Path
from typing import Optional, Type

import numpy as np

from bigym.action_modes import PelvisDof
from bigym.const import HandSide
from bigym.robots.animated_legs import AnimatedLegs

from bigym.utils.dof import Dof
from dataclasses import dataclass, field


@dataclass
class GripperConfig:
    """Configuration for a gripper embedded into robot model.

    Attributes:
        actuators: A list of gripper's actuators names.
        range: Range of gripper control.
        body: Name of the gripper body in the case of embedded gripper.
        model: Path to the gripper XML model in the case of standalone gripper.
        pad_bodies: A list root pads bodies.
        discrete: Round control signal to min or max range value.
    """

    actuators: list[str]
    range: np.ndarray
    body: Optional[str] = None
    model: Optional[Path] = None
    pad_bodies: list[str] = field(default_factory=list)
    pinch_site: Optional[str] = None
    discrete: bool = True

    def __post_init__(self):
        """Validation."""
        if not self.body and not self.model:
            raise ValueError("Either 'body' or 'model' must be specified.")


@dataclass
class ArmConfig:
    """Configuration for a robot arm.

    Attributes:
        site: The site on the robot where the gripper could be attached.
        links: A list of body links of the hand.
        wrist_dof: Optional wrist DOF which could be added to the arm.
        offset_position: Gripper mounting positional offset.
        offset_euler: Gripper mounting euler offset.
    """

    site: str
    links: list[str]
    wrist_dof: Optional[Dof] = None
    offset_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    offset_euler: np.ndarray = field(default_factory=lambda: np.zeros(3))


@dataclass
class FloatingBaseConfig:
    """Configuration for a floating base."""

    dofs: dict[PelvisDof, Dof]
    delta_range_position: tuple[float, float]
    delta_range_rotation: tuple[float, float]
    offset_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    reset_state: Optional[np.ndarray] = None
    animated_legs_class: Optional[Type[AnimatedLegs]] = None


@dataclass
class FullBodyConfig:
    """Configuration for full-body mode."""

    offset_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    reset_state: Optional[np.ndarray] = None


@dataclass
class RobotConfig:
    """Configuration for a robot.

    Attributes:
        model: The path to the robot XML model.
        delta_range: Action range for delta position action mode.
        position_kp: Stiffness of actuators for absolute position action mode.
        pelvis_body: Name of the pelvis body element.
        full_body: Config for full-body mode.
        floating_base: Configuration for the floating base.
        gripper: Configuration for the robot's gripper.
        arms: Configuration for the robot's hands.
        actuators: Dictionary containing all actuators
            and indicating if it is used in floating action mode.
        cameras: List of available cameras.
        namespaces_to_remove: A list of namespaces to remove from the XML model.
    """

    model: Path
    delta_range: tuple[float, float]
    position_kp: float
    pelvis_body: str
    full_body: FullBodyConfig
    floating_base: FloatingBaseConfig
    gripper: GripperConfig
    arms: dict[HandSide, ArmConfig]
    actuators: dict[str, bool]
    cameras: list[str] = field(default_factory=list)
    namespaces_to_remove: list[str] = field(default_factory=list)
