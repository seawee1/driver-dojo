import gym
import sys
import numpy as np
import queue as Queue
from tkinter import Button, Frame, Tk
import threading
from pyglet.window import key
import matplotlib.pyplot as plt
from omegaconf import OmegaConf
from subprocess import Popen

import driver_dojo
from driver_dojo.core.config import Config

KEY_MAP_SEM = {
    key.W: 1,
    key.A: 2,
    key.S: 0,
    key.D: 3,
    key.F: 4,
    key.Q: 5,
    key.E: 6
}

KEY_MAP_DISC = {
    key.W: 2,
    key.A: 1,
    key.S: 3,
    key.D: 0
}

KEY_MAP_CONT = {
    key.W: [0.0, 1.0],
    key.A: [1.0, 0.0],
    key.S: [0.0, -1.0],
    key.D: [-1.0, 0.0]
}

if __name__ == "__main__":

    config_cli = OmegaConf.from_cli()
    if "env" in config_cli:
        env_name = config_cli.env
        del config_cli.env
    else:
        # Override this to change the environment
        env_name = "DriverDojo/Cont-KS-Intersection-v0"

    # Default configuration for manual_control.py
    config = dict(
        simulation=dict(
            dt=0.1,
            max_time_steps=100000,
            stand_still_timeout_after=100000,
            interactive=True,
            carla_co_simulation=False,
        ),
        debug=dict(
            debug=False,
            step_period=int(round(1./0.1)),
            verbose=True,
        ),
        observations=dict(
            carla_observers=[],
            relative_to_ego=True,
            feature_scaling=None
        ),
        rendering=dict(
            sumo_gui=True,
            carla_gui=False,
            human_mode=True,
        ),
        scenario=dict(
            traffic_init=False,
            traffic_spawn=False,
        )
    )
    if "Sem" in env_name:
        config['simulation']['keyboard_to_action'] = KEY_MAP_SEM
        config['actions'] = dict(
            space='Semantic',
            extended_road_options=True,
            hie_accel=True,
            disc_hie_cross_prod=False,
        )
        print("A: Switch-left | D: Switch-right | W: Accelerate | S: Break | Q: Follow-left at junction | R: Follow-right at junction")
    elif "Disc" in env_name:
        config['simulation']['keyboard_to_action'] = KEY_MAP_DISC
        config['vehicle']=dict(
            caster_effect=True,
        )
        config['actions'] = dict(
            space='Discretized',
            disc_dimensions=[2,2],
            disc_hie_cross_prod=False,
        )
        print("A: Left-steer | D: Right-steer | W: Accelerate | S: Break")
    else:
        config['simulation']['keyboard_to_action'] = KEY_MAP_CONT
        print("A: Left-steer | D: Right-steer | W: Accelerate | S: Break")

    # Overwrite based config with CLI arguments
    config = OmegaConf.merge(config, config_cli)
    env = gym.make(env_name, config=config)
    obs = env.reset()
    noop_action = env.action_space.n - 1 if 'Disc' in env_name or 'Sem' in env_name else [0.0, 0.0]
    #try:
    while True:
        im = env.render(mode='rgb_array')
        #print(im.shape)
        obs, reward, done, info = env.step(noop_action)
        #print(info)
        #print(reward)

        if done:
            obs = env.reset(mode='human')
    # except:
    #     env.close()
    # finally:
    #     env.close()
