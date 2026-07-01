"""Test envs."""
from gymnasium import spaces
import pytest
from numpy.testing import assert_allclose

from bigym.action_modes import TorqueActionMode, JointPositionActionMode, ActionMode
from bigym.bigym_env import BiGymEnv
from bigym.envs.move_plates import MovePlate
from bigym.envs.reach_target import ReachTarget
from bigym.envs.manipulation import StackBlocks
from bigym.utils.observation_config import CameraConfig, ObservationConfig

from vr.viewer.vr_viewer import VRViewer
from vr.viewer.control_profiles.h1_floating import H1Floating


@pytest.mark.parametrize(
    "env_class",
    [ReachTarget, MovePlate, StackBlocks],
)
@pytest.mark.parametrize(
    "action_mode_class",
    [TorqueActionMode, JointPositionActionMode],
)
class TestEnvs:
    def _assert_observations(self, observation_space: spaces.Dict, observation: dict):
        assert sorted(observation_space.keys()) == sorted(observation.keys())
        for k in observation.keys():
            assert observation[k].shape == observation_space[k].shape
            assert observation[k].dtype == observation_space[k].dtype

    def _assert_step_data(self, observation_space, obs, rew, term, trunc, info):
        self._assert_observations(observation_space, obs)
        assert isinstance(rew, float)
        assert isinstance(term, bool)
        assert isinstance(trunc, bool)
        assert isinstance(info, dict)

    @staticmethod
    def _are_observations_equal(obs1, obs2):
        for key, val in obs1.items():
            assert_allclose(val, obs2[key], atol=1e-6)

    def test_can_step_with_floating_base(
        self, env_class: type[BiGymEnv], action_mode_class: type[ActionMode]
    ):
        action_mode = action_mode_class(floating_base=True)
        env = env_class(action_mode=action_mode)
        assert env.observation_space is not None
        assert env.action_space is not None
        obs, _ = env.reset()
        self._assert_observations(env.observation_space, obs)
        for i in range(10):
            obs, rew, term, trunc, info = env.step(env.action_space.sample())
            self._assert_step_data(env.observation_space, obs, rew, term, trunc, info)
        env.close()

    def test_can_step_without_floating_base(
        self, env_class: type[BiGymEnv], action_mode_class: type[ActionMode]
    ):
        env = env_class(action_mode=action_mode_class(floating_base=False))
        assert env.observation_space is not None
        assert env.action_space is not None
        obs, _ = env.reset()

        self._assert_observations(env.observation_space, obs)
        for i in range(10):
            obs, rew, term, trunc, info = env.step(env.action_space.sample())
            self._assert_step_data(env.observation_space, obs, rew, term, trunc, info)
        env.close()

    def test_can_step_and_render_human_mode(
        self, env_class: type[BiGymEnv], action_mode_class: type[ActionMode]
    ):
        env = env_class(
            action_mode=action_mode_class(floating_base=False), render_mode="human"
        )
        env.render()  # Just check if this runs without an error
        env.close()

    def test_can_step_and_render_rgb_array_mode(
        self, env_class: type[BiGymEnv], action_mode_class: type[ActionMode]
    ):
        env = env_class(
            action_mode=action_mode_class(floating_base=False), render_mode="rgb_array"
        )
        img = env.render()
        assert img.shape[-1] == 3  # RGB image
        assert img.ndim == 3  # 3 dimensions (h, w, c)
        env.close()

    def test_can_get_pixel_observations(
        self, env_class: type[BiGymEnv], action_mode_class: type[ActionMode]
    ):
        env = env_class(
            action_mode=action_mode_class(floating_base=True),
            observation_config=ObservationConfig(
                cameras=[
                    CameraConfig(name=name, resolution=(32, 32))
                    for name in ["head", "right_wrist", "left_wrist"]
                ],
            ),
        )
        assert "rgb_head" in dict(env.observation_space)
        assert "rgb_right_wrist" in dict(env.observation_space)
        assert "rgb_left_wrist" in dict(env.observation_space)
        obs, _ = env.reset()
        self._assert_observations(env.observation_space, obs)
        assert obs["rgb_head"].shape == (3, 32, 32)
        assert obs["rgb_right_wrist"].shape == (3, 32, 32)
        assert obs["rgb_left_wrist"].shape == (3, 32, 32)
        obs, rew, term, trunc, info = env.step(env.action_space.sample())
        self._assert_step_data(env.observation_space, obs, rew, term, trunc, info)
        assert obs["rgb_head"].shape == (3, 32, 32)
        assert obs["rgb_right_wrist"].shape == (3, 32, 32)
        assert obs["rgb_left_wrist"].shape == (3, 32, 32)
        env.close()

    def test_seed_generation(
        self, env_class: type[BiGymEnv], action_mode_class: type[ActionMode]
    ):
        env = env_class(action_mode=action_mode_class(floating_base=True))
        assert env.seed is None
        env.reset()
        seed = env.seed
        assert isinstance(seed, int)
        obs1, _, _, _, _ = env.step(env.action_space.sample())
        env.reset()
        assert env.seed != seed
        assert isinstance(seed, int)
        obs2, _, _, _, _ = env.step(env.action_space.sample())
        env.close()
        with pytest.raises(AssertionError):
            self._are_observations_equal(obs1, obs2)

    def test_reset_with_seed(
        self, env_class: type[BiGymEnv], action_mode_class: type[ActionMode]
    ):
        env = env_class(action_mode=action_mode_class(floating_base=True))
        reset_obs1, _ = env.reset(seed=42)
        seed = env.seed
        assert seed == 42
        act1 = env.action_space.sample()
        obs1, _, _, _, _ = env.step(act1)
        reset_obs2, _ = env.reset(seed=42)
        assert seed == 42
        act2 = env.action_space.sample()
        obs2, _, _, _, _ = env.step(act2)
        env.close()
        assert_allclose(act1, act2)
        self._are_observations_equal(reset_obs1, reset_obs2)
        self._are_observations_equal(obs1, obs2)

    def test_correct_task_name_in_vr_env(
        self,
        env_class: type[BiGymEnv],
        action_mode_class: type[ActionMode],
    ):
        viewer = VRViewer(
            env_cls=env_class,
            action_mode=action_mode_class(floating_base=True),
            control_profile_cls=H1Floating,
        )
        assert viewer._env.task_name == env_class.__name__
