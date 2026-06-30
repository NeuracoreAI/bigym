"""Test MovePlate task environment."""

import pytest

import numpy as np
from bigym.action_modes import JointPositionActionMode, ActionMode
from bigym.envs.move_plates import MovePlate


@pytest.mark.parametrize(
    "action_mode_class",
    [JointPositionActionMode],
)
class TestMovePlate:
    def test_terminates_when_plate_is_on_the_ground(
        self, action_mode_class: type[ActionMode]
    ):
        action_mode = action_mode_class(floating_base=True)
        env = MovePlate(action_mode=action_mode)
        # Check term is True when plate is on the ground
        env.reset()
        env.plates[0].body.set_position(np.zeros(3), True)
        obs, rew, term, trunc, info = env.step(env.action_space.sample())
        assert term
        # Check term is False when plate is not on the ground
        env.reset()
        env.plates[0].body.set_position(np.ones(3), True)
        obs, rew, term, trunc, info = env.step(env.action_space.sample())
        assert not term
        env.close()
