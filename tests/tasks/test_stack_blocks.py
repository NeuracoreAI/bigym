"""Test StackBlocks task environment."""
import pytest

import numpy as np
from bigym.action_modes import JointPositionActionMode, ActionMode
from bigym.envs.manipulation import StackBlocks


@pytest.mark.parametrize(
    "action_mode_class",
    [JointPositionActionMode],
)
class TestStackBlocks:
    def test_terminates_when_any_block_is_on_the_ground(
        self, action_mode_class: type[ActionMode]
    ):
        action_mode = action_mode_class(floating_base=True)
        env = StackBlocks(action_mode=action_mode)
        # Check term is True when any block is on the ground
        for block in env.blocks:
            obs, _ = env.reset()
            block.body.set_position(np.zeros(3))
            obs, rew, term, trunc, info = env.step(env.action_space.sample())
            assert term
        # Check term is False when block is not on the ground
        for block in env.blocks:
            obs, _ = env.reset()
            block.body.set_position(np.ones(3))
            obs, rew, term, trunc, info = env.step(env.action_space.sample())
            assert not term
        env.close()
