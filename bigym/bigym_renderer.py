"""Fix to be able to render to on- and off- screen at the same time."""
import time
import glfw
import mujoco

from gymnasium.envs.mujoco.mujoco_rendering import (
    MujocoRenderer,
    WindowViewer,
    OffScreenViewer,
    BaseRender,
)
from mojo import Mojo


class BiGymRenderer(MujocoRenderer):
    """Custom mujoco_rendering.MujocoRenderer with fixes.

    Notes:
        - Allows to render in human mode along with visual observations.
    """

    def __init__(self, mojo: Mojo):
        """Init."""
        super().__init__(
            mojo.model,
            mojo.data,
            width=mojo.model.vis.global_.offwidth,
            height=mojo.model.vis.global_.offheight,
        )

    def _get_viewer(self, render_mode: str) -> BaseRender:
        """See base."""
        self.viewer = self._viewers.get(render_mode)
        if self.viewer is None:
            if render_mode == "human":
                self.viewer = BiGymWindowViewer(
                    self.model,
                    self.data,
                    width=self.width,
                    height=self.height,
                )

            elif render_mode in {"rgb_array", "depth_array"}:
                self.viewer = OffScreenViewer(
                    self.model,
                    self.data,
                    width=self.width,
                    height=self.height,
                )
            else:
                raise AttributeError(
                    f"Unexpected mode: {render_mode}, "
                    f"expected modes: human, rgb_array, or depth_array"
                )
            self._set_cam_config()
            self._viewers[render_mode] = self.viewer

        self.viewer.make_context_current()

        return self.viewer

    def get_viewer(self, render_mode: str) -> BaseRender:
        """Get viewer for specified render mode."""
        return self._get_viewer(render_mode)


class BiGymWindowViewer(WindowViewer):
    """Custom mujoco_rendering.WindowViewer with fixes.

    Notes:
        - Fixes GUI overlap when viewer is paused.
    """

    def render(self):
        """See base."""

        def update():
            # fill overlay items
            self._create_overlay()

            render_start = time.time()
            if self.window is None:
                return
            elif glfw.window_should_close(self.window):
                glfw.destroy_window(self.window)
                glfw.terminate()
            self.viewport.width, self.viewport.height = glfw.get_framebuffer_size(
                self.window
            )
            # update scene
            mujoco.mjv_updateScene(
                self.model,
                self.data,
                self.vopt,
                mujoco.MjvPerturb(),
                self.cam,
                mujoco.mjtCatBit.mjCAT_ALL.value,
                self.scn,
            )

            # marker items
            for marker in self._markers:
                self._add_marker_to_scene(marker)

            # render
            mujoco.mjr_render(self.viewport, self.scn, self.con)

            # overlay items
            if not self._hide_menu:
                for gridpos, [t1, t2] in self._overlays.items():
                    mujoco.mjr_overlay(
                        mujoco.mjtFontScale.mjFONTSCALE_150,
                        gridpos,
                        self.viewport,
                        t1,
                        t2,
                        self.con,
                    )

            glfw.swap_buffers(self.window)
            glfw.poll_events()
            self._time_per_render = 0.9 * self._time_per_render + 0.1 * (
                time.time() - render_start
            )

            # clear overlay
            self._overlays.clear()
            # clear markers
            self._markers.clear()

        if self._paused:
            while self._paused:
                update()
                if self._advance_by_one_step:
                    self._advance_by_one_step = False
                    break
        else:
            self._loop_count += self.model.opt.timestep / (
                self._time_per_render * self._run_speed
            )
            if self._render_every_frame:
                self._loop_count = 1
            while self._loop_count > 0:
                update()
                self._loop_count -= 1
