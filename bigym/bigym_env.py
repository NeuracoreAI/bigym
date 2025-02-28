"""Core BiGym env functionality."""
from __future__ import annotations

import logging
from pathlib import Path
from typing import Any, Optional, Type

import mujoco
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from mojo import Mojo
from mojo.elements import Geom, Camera

from bigym.action_modes import ActionMode
from bigym.const import WORLD_MODEL
from bigym.envs.props.preset import Preset
from bigym.robots.configs.h1 import H1
from bigym.robots.robot import Robot
from bigym.bigym_renderer import BiGymRenderer
from bigym.utils.callables_cache import CallablesCache
from bigym.utils.env_health import EnvHealth
from bigym.utils.observation_config import ObservationConfig

CONTROL_FREQUENCY_MAX = 500
CONTROL_FREQUENCY_MIN = 20

PHYSICS_DT = 0.002

MAX_DISTANCE_FROM_ORIGIN = 10
SPARSE_REWARD_FACTOR = 1


class BiGymEnv(gym.Env):
    """Core BiGym environment which loads in common robot across all tasks."""

    metadata = {
        "render_modes": ["human", "rgb_array", "depth_array"],
        "render_fps": 1 / PHYSICS_DT,
    }

    _ENV_CAMERAS = ["external"]

    _MODEL_PATH: Path = WORLD_MODEL
    _PRESET_PATH: Optional[Path] = None
    _FLOOR = "floor"

    DEFAULT_ROBOT = H1

    RESET_ROBOT_POS = np.array([0, 0, 0])
    RESET_ROBOT_QUAT = np.array([1, 0, 0, 0])

    def __init__(
        self,
        action_mode: ActionMode,
        observation_config: ObservationConfig = ObservationConfig(),
        render_mode: Optional[str] = None,
        start_seed: Optional[int] = None,
        control_frequency: int = CONTROL_FREQUENCY_MAX,
        robot_cls: Optional[Type[Robot]] = None,
    ):
        """Init.

        :param action_mode: The action mode of the robot. Use this to configure how
            you plan to control the robot. E.g. joint position, delta ee pose, ect.
        :param observation_config: Observations configuration. Use this to configure
            collected data.
        :param render_mode: The render mode for mujoco. Options are
            "human", "rgb_array" or "depth_array". If None, the default render mode
            will be used.
        :param start_seed: The seed to start the environment with. If None, a random
            seed will be used.
        :param control_frequency: Control loop frequency, 500 Hz by default.
        :param robot_cls: Environment robot class override.
        """
        # Tracks physics simulation stability
        self._env_health = EnvHealth()
        # Caches results valid for one environment step
        self._step_cache = CallablesCache()

        self._observation_config = observation_config
        self.action_mode = action_mode

        if start_seed is None:
            start_seed = np.random.randint(2**32)
        if not isinstance(start_seed, int):
            raise ValueError("Expected start_seed to be an integer.")
        self._next_seed = start_seed
        self._current_seed = None

        assert CONTROL_FREQUENCY_MIN <= control_frequency <= CONTROL_FREQUENCY_MAX, (
            f"Control frequency must be in "
            f"{CONTROL_FREQUENCY_MIN}-{CONTROL_FREQUENCY_MAX} range."
        )
        self._control_frequency = control_frequency
        self._sub_steps_count = int(
            np.round(CONTROL_FREQUENCY_MAX / self._control_frequency)
        )

        self._mojo = Mojo(str(self._MODEL_PATH), timestep=PHYSICS_DT)
        self._robot = (robot_cls or self.DEFAULT_ROBOT)(self.action_mode, self._mojo)
        self._preset = Preset(self._mojo, self._PRESET_PATH)
        self._initialize_env()
        self._floor = Geom.get(self._mojo, self._FLOOR)

        self.action_space: spaces.Box = self.action_mode.action_space(
            action_scale=self._sub_steps_count, seed=self._next_seed
        )
        self._action: np.ndarray = np.zeros_like(self.action_space.low)

        self.observation_space: spaces.Space = self.get_observation_space()

        assert self.metadata["render_modes"] == [
            "human",
            "rgb_array",
            "depth_array",
        ], self.metadata["render_modes"]

        self.render_mode = render_mode

        # Validate cameras configuration
        available_cameras = set(self._ENV_CAMERAS + self._robot.config.cameras)
        for camera_config in self._observation_config.cameras:
            assert camera_config.name in available_cameras

        # Mapping original camera names to full identifiers
        self._cameras_map = self._initialize_cameras()

        self.mujoco_renderer: Optional[BiGymRenderer] = None
        self.obs_renderers: Optional[dict[tuple[int, int], mujoco.Renderer]] = {}
        self._initialize_renderers()

    @property
    def task_name(self) -> str:
        """Returns the class name of the environment."""
        return self.__class__.__name__

    @property
    def _use_pixels(self):
        """Returns True if the environment uses pixels."""
        return len(self.observation_config.cameras) > 0

    @property
    def seed(self) -> Optional[int]:
        """Initial seed of the environment."""
        return self._current_seed

    @property
    def success(self) -> bool:
        """Check if current step is successful."""
        return bool(self._step_cache.get(self._success))

    @property
    def fail(self) -> bool:
        """Check if current step is successful."""
        return bool(self._step_cache.get(self._fail))

    @property
    def reward(self) -> float:
        """Get current step reward."""
        return float(self._step_cache.get(self._reward))

    @property
    def terminate(self) -> bool:
        """Get current step termination condition."""
        return bool(self.success or self.fail)

    @property
    def truncate(self) -> bool:
        """Get current step truncation condition."""
        return bool(not self.is_healthy)

    @property
    def is_healthy(self) -> bool:
        """Checks if the simulation is currently healthy."""
        return bool(self._env_health.is_healthy)

    @property
    def observation_config(self) -> ObservationConfig:
        """Get the observation configuration."""
        return self._observation_config

    @property
    def control_frequency(self):
        """Control frequency of the environment."""
        return self._control_frequency

    @property
    def robot(self) -> Robot:
        """Get robot."""
        return self._robot

    @property
    def floor(self) -> Geom:
        """Get environment floor."""
        return self._floor

    @property
    def mojo(self) -> Mojo:
        """Get Mojo."""
        return self._mojo

    @property
    def action(self) -> np.ndarray:
        """Get last executed action."""
        return self._action.copy()

    def _initialize_renderers(self):
        self._close_renderers()
        self.mujoco_renderer: BiGymRenderer = BiGymRenderer(self._mojo)
        for camera_config in self._observation_config.cameras:
            resolution = camera_config.resolution
            if resolution in self.obs_renderers:
                continue
            self.obs_renderers[resolution] = mujoco.Renderer(
                self._mojo.model, resolution[0], resolution[1]
            )

    def _initialize_cameras(self) -> dict[str, tuple[int, Camera]]:
        cameras_map: dict[str, tuple[int, Camera]] = {}
        for camera_name in self._ENV_CAMERAS:
            camera: Camera = Camera.get(
                self._mojo, camera_name, self._mojo.root_element
            )
            cameras_map[camera_name] = (camera.id, camera)
        for robot_camera in self._robot.cameras:
            cameras_map[robot_camera.mjcf.name] = (
                robot_camera.id,
                robot_camera,
            )

        for camera_config in self._observation_config.cameras:
            _, camera = cameras_map[camera_config.name]
            if camera_config.pos is not None:
                camera.set_position(np.array(camera_config.pos))
            if camera_config.quat is not None:
                camera.set_quaternion(np.array(camera_config.quat))
        return cameras_map

    def _close_renderers(self):
        if self.mujoco_renderer is not None:
            self.mujoco_renderer.close()
        for renderer in self.obs_renderers.values():
            renderer.close()
        self.mujoco_renderer = None
        self.obs_renderers.clear()

    def _initialize_env(self):
        """Can be overwritten to add task specific items to scene."""
        pass

    def get_observation_space(self) -> spaces.Space:
        """Get observation space."""
        obs_dict = {}
        if self._observation_config.proprioception:
            obs_dict = {
                "proprioception": spaces.Box(
                    low=-np.inf,
                    high=np.inf,
                    shape=(len(self._robot.qpos) + len(self._robot.qvel),),
                    dtype=np.float32,
                ),
                "proprioception_grippers": spaces.Box(
                    low=0,
                    high=1,
                    shape=(len(self.robot.qpos_grippers),),
                    dtype=np.float32,
                ),
            }
            if self.robot.floating_base:
                obs_dict.update(
                    {
                        "proprioception_floating_base": spaces.Box(
                            low=-np.inf,
                            high=np.inf,
                            shape=(len(self.robot.floating_base.qpos),),
                            dtype=np.float32,
                        ),
                        "proprioception_floating_base_actions": spaces.Box(
                            low=-np.inf,
                            high=np.inf,
                            shape=(
                                len(self.robot.floating_base.get_accumulated_actions),
                            ),
                            dtype=np.float32,
                        ),
                    }
                )
        if self._use_pixels:
            for camera in self.observation_config.cameras:
                if camera.rgb:
                    obs_dict[f"rgb_{camera.name}"] = spaces.Box(
                        low=0, high=255, shape=(3, *camera.resolution), dtype=np.uint8
                    )
                if camera.depth:
                    obs_dict[f"depth_{camera.name}"] = spaces.Box(
                        # todo: check if this is the correct range
                        low=0,
                        high=1,
                        shape=camera.resolution,
                        dtype=np.float32,
                    )
        if self._observation_config.privileged_information:
            obs_dict.update(self._get_task_privileged_obs_space())
        return spaces.Dict(obs_dict)

    def _get_task_privileged_obs_space(self) -> dict[str, Any]:
        """Get the task privileged observation space."""
        return {}

    def get_observation(self) -> dict[str, np.ndarray]:
        """Get the observation."""
        obs = {}
        if self._observation_config.proprioception:
            obs |= self._get_proprioception_obs()
        if self._use_pixels:
            obs |= self._get_visual_obs()
        if self._observation_config.privileged_information:
            obs |= self._get_task_privileged_obs()
        return obs

    def _get_task_info(self) -> dict[str, Any]:
        """Get the task info dict."""
        return {}

    def get_info(self) -> dict[str, Any]:
        """Get info dict."""
        info = self._get_task_info()
        info.update({"task_success": float(self.success)})
        return info

    def _get_proprioception_obs(self) -> dict[str, Any]:
        obs = {
            "proprioception": np.concatenate(
                [self._robot.qpos, self._robot.qvel]
            ).astype(np.float32),
            "proprioception_grippers": np.array(self.robot.qpos_grippers).astype(
                np.float32
            ),
        }
        if self.robot.floating_base:
            obs["proprioception_floating_base"] = np.array(
                self.robot.floating_base.qpos
            ).astype(np.float32)
            obs["proprioception_floating_base_actions"] = np.array(
                self.robot.floating_base.get_accumulated_actions
            ).astype(np.float32)
        return obs

    def _get_visual_obs(self) -> dict[str, Any]:
        """Get the visual observation."""
        obs = {}
        for camera_config in self._observation_config.cameras:
            obs_renderer = self.obs_renderers[camera_config.resolution]
            obs_renderer.update_scene(
                self._mojo.data, self._cameras_map[camera_config.name][0]
            )
            if camera_config.rgb:
                rgb = obs_renderer.render()
                obs[f"rgb_{camera_config.name}"] = np.moveaxis(rgb, -1, 0)
            if camera_config.depth:
                obs_renderer.enable_depth_rendering()
                obs[f"depth_{camera_config.name}"] = obs_renderer.render()
                obs_renderer.disable_depth_rendering()
        return obs

    def _get_task_privileged_obs(self) -> dict[str, Any]:
        """Get the task privileged observation."""
        return {}

    def _update_seed(self, override_seed=None):
        """Update the seed for the environment.

        Args:
            override_seed: If not None, the next seed will be set to this value.
        """
        if override_seed is not None:
            if not isinstance(override_seed, int):
                logging.warning(
                    "Expected override_seed to be an integer. Casting to int."
                )
                override_seed = int(override_seed)
            self._next_seed = override_seed
            self.action_space = self.action_mode.action_space(
                action_scale=self._sub_steps_count, seed=override_seed
            )
        self._current_seed = self._next_seed
        assert self._current_seed is not None
        self._next_seed = np.random.randint(2**32)
        np.random.seed(self._current_seed)

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        """Reset the environment.

        Args:
           seed: If not None, the environment will be reset with this seed.
           options: Additional information to specify how the environment is reset
            (optional, depending on the specific environment).
        """
        self._env_health.reset()
        self._update_seed(override_seed=seed)
        self._mojo.physics.reset()
        self._action = np.zeros_like(self._action)
        self._robot.reset(self.RESET_ROBOT_POS, self.RESET_ROBOT_QUAT)
        self._on_reset()
        return self.get_observation(), self.get_info()

    def _on_reset(self):
        """Custom environment reset behaviour."""
        pass

    def _on_step(self):
        """Custom environment behaviour after stepping."""
        pass

    def render(self):
        """Renders a frame of the simulation."""
        return self.mujoco_renderer.render(self.render_mode)

    def step(
        self, action: np.ndarray, fast: bool = False
    ) -> tuple[Any, float, bool, bool, dict]:
        """Step the environment.

        Args:
            action: Action to take.
            fast: If True, perform the environment step without processing observations
                and return default values. Useful when performance is crucial,
                but observations are not required, e.g., demo collection in VR.

        Returns:
            tuple: (observation, reward, terminated, truncated, info).
        """
        self._step_cache.clean()
        self._step_mujoco_simulation(action)
        self._on_step()
        self._action = action
        if fast:
            return {}, 0, False, False, {}
        else:
            return (
                self.get_observation(),
                self.reward,
                self.terminate,
                self.truncate,
                self.get_info(),
            )

    def _step_mujoco_simulation(self, action):
        """Step the mujoco simulation."""
        if action.shape != self.action_space.shape:
            raise ValueError(
                f"Action shape mismatch: "
                f"expected {self.action_space.shape}, but got {action.shape}."
            )
        if np.any(action < self.action_space.low) or np.any(
            action > self.action_space.high
        ):
            clipped_action = np.clip(
                action, self.action_space.low, self.action_space.high
            )
            raise ValueError(
                f"Action {action} is out of the action space bounds. "
                f"Overhead: {action - clipped_action}"
            )
        with self._env_health.track():
            for i in range(self._sub_steps_count):
                if i == 0:
                    self.action_mode.step(action)
                else:
                    self._mojo.step()
                mujoco.mj_rnePostConstraint(self._mojo.model, self._mojo.data)

    def _success(self) -> bool:
        """Check if the episode is successful."""
        return False

    def _fail(self) -> bool:
        """Check if the episode is failed."""
        return (
            np.linalg.norm(self._robot.pelvis.get_position()) > MAX_DISTANCE_FROM_ORIGIN
        )

    def _reward(self) -> float:
        """Get current episode reward."""
        return float(self.success) * SPARSE_REWARD_FACTOR

    def close(self):
        """Close environment."""
        self._close_renderers()
