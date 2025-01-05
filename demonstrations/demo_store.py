"""Script for uploading the collected demos."""
import logging
import os
import warnings
import zipfile
from typing import Optional

import numpy as np
import tempfile
from pathlib import Path
from copy import deepcopy

import wget
from tqdm import tqdm

from bigym.bigym_env import CONTROL_FREQUENCY_MAX
from bigym.const import CACHE_PATH, DEMO_RELEASES, DEMO_VERSION
from demonstrations.const import SAFETENSORS_SUFFIX
from demonstrations.utils import Metadata, ObservationMode
from demonstrations.demo import Demo, LightweightDemo
from demonstrations.demo_converter import DemoConverter


class DemoNotFoundError(Exception):
    """Exception raised when a demo is not found."""

    def __init__(self, metadata: Metadata):
        """Init.

        Args:
            metadata(Metadata): The metadata for the demo.
        """
        self.metadata = metadata
        super().__init__(f"Demo not found for {metadata}.")


class TooManyDemosRequestedError(Exception):
    """Exception raised when too many demos are requested."""

    def __init__(self, requested: int, found: int):
        """Init.

        Args:
            requested(int): The number of demos requested.
            found(int): The number of demos found.
        """
        self.requested = requested
        self.found = found
        super().__init__(
            f"Requested {requested} demos, but only {found} found. "
            "Try requesting a smaller number."
        )


class DemoStore:
    """Class to help storing and retrieving demos from a database."""

    _DEMOS = "demonstrations"
    _LOCK = ".lock"

    def __init__(self, cache_root: Optional[Path] = None):
        """Init."""
        self._cache_root = cache_root or CACHE_PATH
        self._cache_path: Path = self._cache_root / self._DEMOS / DEMO_VERSION
        self._cache_path.mkdir(parents=True, exist_ok=True)

    def cache_demo(self, demo: Demo, frequency: Optional[int] = None):
        """Add a demo to the local cache.

        :param demo: The demo to upload.
        :param frequency: Control frequency of the demo.
        """
        if demo.metadata.observation_mode != ObservationMode.Lightweight:
            if frequency is None and not self.light_demo_exists(
                demo.metadata, frequency
            ):
                self.cache_demo(LightweightDemo.from_demo(demo), frequency)
        if self.demo_exists(demo.metadata, frequency):
            return
        with tempfile.TemporaryDirectory() as temp_dir:
            file_path = demo.save(Path(temp_dir) / demo.metadata.filename)
            self._cache_demo_file(file_path, frequency)

    def _cache_demo_file(self, demo_path: Path, frequency: Optional[int] = None):
        metadata = Metadata.from_safetensors(demo_path)
        new_demo_path = self._create_path(metadata, frequency)
        new_demo_path.parent.mkdir(parents=True, exist_ok=True)
        new_demo_path.write_bytes(demo_path.read_bytes())

    def get_demos(
        self,
        metadata: Metadata,
        amount: int = -1,
        frequency: int = CONTROL_FREQUENCY_MAX,
        shuffle: bool = True,
    ) -> list[Demo]:
        """Download the demos matching the metadata.

        :param metadata: The metadata to match the demos.
        :param amount: The amount of demos to get.
            If < 0, all demonstrations are returned.
        :param frequency: Demo control frequency.
        :param shuffle: Shuffle demonstrations order randomly.

        :return: The demos matching the metadata.
        """
        demos = []
        if amount == 0:
            return demos

        light_metadata = deepcopy(metadata)
        light_metadata.observation_mode = ObservationMode.Lightweight
        light_demos_dir = self._create_path(light_metadata).parent
        light_demos_count = self._get_demos_count(light_demos_dir)

        demos_dir = self._create_path(metadata, frequency).parent
        demos_count = self._get_demos_count(demos_dir)

        # Check if demos with requested metadata are cached locally
        if demos_count == light_demos_count:
            demos = self._get_demos(demos_dir, amount, shuffle)
            # Return locally cached demos
            if demos:
                return demos

        # Attempt to get demos at original frequency
        max_freq_demos_dir = self._create_path(metadata).parent
        demos = self._get_demos(max_freq_demos_dir, -1)

        # Attempt to get raw, lightweight demos
        if not demos and metadata.observation_mode != ObservationMode.Lightweight:
            demos = self._get_demos(light_demos_dir, -1)

        # Raising exception if there are no demos at this stage
        if not demos:
            raise DemoNotFoundError(metadata)

        # Recreate and cache demos
        with tqdm(
            total=len(demos),
            desc="Caching Demos",
            unit="demo",
            leave=True,
        ) as pbar:
            robot = metadata.get_robot()
            env = metadata.get_env(frequency)
            for demo in demos:
                if self.demo_exists(metadata, frequency, demo.uuid):
                    pbar.update()
                    continue
                demo = DemoConverter.decimate(
                    demo,
                    frequency,
                    CONTROL_FREQUENCY_MAX,
                    robot=robot,
                )
                if metadata.observation_mode != ObservationMode.Lightweight:
                    demo = DemoConverter.create_demo_in_new_env(demo, env)
                self.cache_demo(demo, frequency)
                pbar.update()
        return self._get_demos(demos_dir, amount, shuffle)

    def _get_demos(
        self, demos_dir: Path, amount: int, shuffle: bool = True
    ) -> list[Demo]:
        self.pull_demos()
        if not demos_dir.exists():
            return []
        files = list(demos_dir.glob(f"*{SAFETENSORS_SUFFIX}"))
        if amount > len(files):
            raise TooManyDemosRequestedError(amount, len(files))
        elif amount > 0:
            files = files[:amount]
        if shuffle:
            np.random.shuffle(files)
        return [Demo.from_safetensors(file) for file in files]

    def _get_demos_count(self, demos_dir: Path) -> int:
        self.pull_demos()
        if not demos_dir.exists():
            return 0
        return len(list(demos_dir.glob(f"*{SAFETENSORS_SUFFIX}")))

    def pull_demos(self):
        """Pull demos from repository."""
        if self.cached:
            logging.info(f"Demos are cached already: {self._cache_path}")
            return
        url = f"{DEMO_RELEASES}/v{DEMO_VERSION}/{self._DEMOS}.zip"
        logging.warning(f"Cached demos not found. Downloading from: {url}")
        try:
            local_filename = wget.download(url)
            logging.info(f"Demos downloaded successfully and saved as {local_filename}")
        except Exception as e:
            logging.error(f"An error occurred while downloading demos: {e}")
            return
        if not zipfile.is_zipfile(local_filename):
            raise RuntimeError(f"Invalid demos file: {local_filename}")
        with zipfile.ZipFile(local_filename, "r") as zip_ref:
            zip_ref.extractall(self._cache_path)
        os.remove(local_filename)
        self.cached = True

    @property
    def cached(self):
        """Return True if demos are cached locally, else False."""
        lock_file = self._cache_path / self._LOCK
        return lock_file.exists()

    @cached.setter
    def cached(self, value: bool):
        """Set local cache status."""
        lock_file = self._cache_path / self._LOCK
        if value:
            lock_file.touch(exist_ok=True)
        elif lock_file.exists():
            os.remove(lock_file)

    def list_demo_paths(self, metadata: Metadata) -> list[Path]:
        """List the demos matching the metadata."""
        demo_dir = self._create_path(metadata).parent
        if not demo_dir.exists():
            warnings.warn(f"No demos found for {metadata}.")
            return []
        return [p for p in demo_dir.iterdir()]

    def _create_path(self, metadata: Metadata, frequency: Optional[int] = None) -> Path:
        path = self._cache_path
        if metadata.env_cls.DEFAULT_ROBOT != metadata.robot_cls:
            path /= metadata.environment_data.robot_name
        path = (
            path
            / metadata.env_name
            / metadata.environment_data.action_mode_description
            / metadata.observation_mode.value
        )
        if metadata.observation_mode == ObservationMode.Pixel:
            path /= metadata.environment_data.camera_description
        if frequency:
            path /= f"{frequency}hz"
        return path / metadata.filename

    def light_demo_exists(
        self,
        metadata: Metadata,
        frequency: Optional[int] = None,
        uuid_override: Optional[str] = None,
    ) -> bool:
        """Check if a lightweight demo exists.

        :param metadata: The metadata for the demo.
        :param frequency: Control frequency.
        :param uuid_override: Demo uuid.

        :return: True if the lightweight demo exists, False otherwise.
        """
        light_metadata = deepcopy(metadata)
        light_metadata.observation_mode = ObservationMode.Lightweight
        return self.demo_exists(light_metadata, frequency, uuid_override)

    def demo_exists(
        self,
        metadata: Metadata,
        frequency: Optional[int] = None,
        uuid_override: Optional[str] = None,
    ) -> bool:
        """Check if a demo exists.

        :param metadata: The metadata for the demo.
        :param frequency: Control frequency.
        :param uuid_override: Demo uuid.

        :return: True if the demo exists, False otherwise.
        """
        if uuid_override:
            metadata = deepcopy(metadata)
            metadata.uuid = uuid_override
        return self._create_path(metadata, frequency).exists()
