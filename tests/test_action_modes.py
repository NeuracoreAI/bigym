import numpy as np
import pytest
from numpy.testing import assert_allclose

from bigym.action_modes import (
    JointPositionActionMode,
    TorqueActionMode,
    ActionMode,
    PelvisDof,
)
from bigym.const import TOLERANCE_ANGULAR
from bigym.robots.floating_base import RobotFloatingBase
from bigym.envs.reach_target import ReachTarget

from bigym.bigym_env import BiGymEnv


def test_join_position_absolute_block_until_reached():
    env: BiGymEnv = ReachTarget(
        action_mode=JointPositionActionMode(
            floating_base=True, absolute=True, block_until_reached=True
        )
    )
    env.reset()
    for _ in range(100):
        ctrl = np.zeros_like(env.action_space.sample())
        # Control floating base in delta-position action mode
        ctrl[0:2] = np.random.uniform(
            *env.robot.config.floating_base.delta_range_position
        )
        ctrl[2] = np.random.uniform(
            *env.robot.config.floating_base.delta_range_rotation
        )
        # Controlling other joints in absolute mode
        ctrl[3:8] = np.radians(np.random.uniform([0, 0, 0, 0, 0], [30, 30, 30, 30, 30]))
        ctrl[8:13] = np.radians(
            np.random.uniform([0, 0, 0, 0, 0], [30, -30, -30, 30, 30])
        )
        env.step(ctrl)
        # Validate floating base pose
        assert env.robot.floating_base.is_target_reached
        # Validate other joints
        qpos = env.robot.qpos_actuated
        assert_allclose(qpos[3:], ctrl[3:], atol=TOLERANCE_ANGULAR)


def test_floating_base():
    env: BiGymEnv = ReachTarget(
        action_mode=JointPositionActionMode(
            floating_base=True,
            floating_dofs=[PelvisDof.X, PelvisDof.Y, PelvisDof.Z, PelvisDof.RZ],
            block_until_reached=True,
        )
    )
    env.reset()
    for _ in range(100):
        ctrl = np.zeros_like(env.action_space.sample())
        ctrl[0:3] = np.random.uniform(*RobotFloatingBase.DELTA_RANGE_POS)
        ctrl[4] = np.random.uniform(*RobotFloatingBase.DELTA_RANGE_ROT)
        env.step(ctrl)
        assert env.robot.floating_base.is_target_reached


@pytest.mark.parametrize(
    "action_mode",
    [
        TorqueActionMode(floating_base=True),
        JointPositionActionMode(floating_base=True, absolute=False),
        JointPositionActionMode(floating_base=True, absolute=True),
    ],
)
class TestActionModeStability:
    ENV_STEPS_COUNT = 100
    UNSTABLE_ACTION_MULTIPLIER = 50

    def run_simulation(self, env: BiGymEnv, action: np.ndarray):
        env.reset()
        for _ in range(self.ENV_STEPS_COUNT):
            env.step(action)

    def test_action_space_is_stable(self, action_mode: ActionMode):
        env: BiGymEnv = ReachTarget(action_mode=action_mode)
        low = env.action_space.low
        high = env.action_space.high
        self.run_simulation(env, low)
        self.run_simulation(env, high)

    def test_action_space_raises_when_beyond_bounds(self, action_mode: ActionMode):
        env: BiGymEnv = ReachTarget(action_mode=action_mode)
        low = env.action_space.low
        high = env.action_space.high
        with pytest.raises(ValueError):
            self.run_simulation(env, low * self.UNSTABLE_ACTION_MULTIPLIER)
        with pytest.raises(ValueError):
            self.run_simulation(env, high * self.UNSTABLE_ACTION_MULTIPLIER)
