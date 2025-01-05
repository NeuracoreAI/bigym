"""Validation of deterministic demo replay."""
from bigym.action_modes import JointPositionActionMode, PelvisDof
from build.lib.bigym.envs.manipulation import FlipCutlery
from demonstrations.demo_player import DemoPlayer
from demonstrations.demo_store import DemoStore
from demonstrations.utils import Metadata

control_frequency = 50
env = FlipCutlery(
    action_mode=JointPositionActionMode(
        floating_base=True,
        absolute=True,
        floating_dofs=[PelvisDof.X, PelvisDof.Y, PelvisDof.Z, PelvisDof.RZ],
    ),
    control_frequency=control_frequency,
    render_mode="human",
)
metadata = Metadata.from_env(env)

demo_store = DemoStore()
demos = demo_store.get_demos(
    metadata, amount=-1, frequency=control_frequency, shuffle=False
)

uuids = [
    "ff0bb5b5b80948cfaa9e4a4a3fb42b18",
    "1d738d73cead4e9f8849fd6a90bb8511",
    "f539e0f5787b4a74949e72a8a206e6bc",
    "b6b8921f56364bfebf87bab3e89512b9",
]
target_uuid = uuids[-1]
demo = next((d for d in demos if d.uuid == target_uuid), None)
player = DemoPlayer()
timesteps = player._get_timesteps_for_replay(demo, env, control_frequency)
env.reset(seed=demo.seed)
for step in timesteps:
    action = step.executed_action
    env.step(action, fast=True)
    if env.render_mode:
        env.render()
    obs = env.get_observation()["proprioception"]

env.close()
