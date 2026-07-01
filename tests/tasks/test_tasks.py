"""Test tasks."""
import numpy as np
import pytest

from bigym.action_modes import JointPositionActionMode, ActionMode
from bigym.bigym_env import BiGymEnv, MAX_DISTANCE_FROM_ORIGIN
from tools.shared.utils import ENVIRONMENTS


@pytest.mark.parametrize(
    "env_class",
    ENVIRONMENTS.values(),
)
@pytest.mark.parametrize(
    "action_mode_class",
    [JointPositionActionMode],
)
@pytest.mark.slow
class TestEnvs:
    def test_terminates_when_robot_out_of_bounds(
        self, env_class: type[BiGymEnv], action_mode_class: type[ActionMode]
    ):
        action_mode = action_mode_class(floating_base=True)
        env = env_class(action_mode=action_mode)
        obs, _ = env.reset()
        env.robot.pelvis.set_position(np.array([MAX_DISTANCE_FROM_ORIGIN] * 3))
        obs, rew, term, trunc, info = env.step(env.action_space.sample())
        assert term
        env.close()
