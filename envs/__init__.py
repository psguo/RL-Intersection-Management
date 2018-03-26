import os, sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
from gym.envs.registration import register

from envs.traffic_env import TrafficEnv
from envs.traffic_env_simple import TrafficEnvSimple
from envs.traffic_env_cross2 import TrafficEnvCross2
from envs.traffic_env_multicar import TrafficEnvMulticar


register(
    id='Traffic-Simple-gui-v0',
    entry_point='envs:TrafficEnvSimple',
    tags={'wrapper_config.TimeLimit.max_episode_steps': 1000},
    kwargs={"mode": "gui"},
    nondeterministic=True
)

register(
    id='Traffic-Simple-cli-v0',
    entry_point='envs:TrafficEnvSimple',
    tags={'wrapper_config.TimeLimit.max_episode_steps': 1000},
    kwargs={"mode": "cli"},
    nondeterministic=True
)

register(
    id='Traffic-Multicar-gui-v0',
    entry_point='envs:TrafficEnvMulticar',
    tags={'wrapper_config.TimeLimit.max_episode_steps': 1000},
    kwargs={"mode": "gui"},
    nondeterministic=True
)

register(
    id='Traffic-Multicar-cli-v0',
    entry_point='envs:TrafficEnvMulticar',
    tags={'wrapper_config.TimeLimit.max_episode_steps': 1000},
    kwargs={"mode": "cli"},
    nondeterministic=True
)

register(
    id='Traffic-Cross2-gui-v0',
    entry_point='envs:TrafficEnvCross2',
    tags={'wrapper_config.TimeLimit.max_episode_steps': 1000},
    kwargs={"mode": "gui"},
    nondeterministic=True
)

register(
    id='Traffic-Cross2-cli-v0',
    entry_point='envs:TrafficEnvCross2',
    tags={'wrapper_config.TimeLimit.max_episode_steps': 1000},
    kwargs={"mode": "cli"},
    nondeterministic=True
)
