import gym
import time
import numpy as np
from tqdm import tqdm
from omegaconf import OmegaConf

import driver_dojo

if __name__ == "__main__":
    conf = OmegaConf.from_cli()
    if "env_name" in conf:
        env_name = conf.env
        del conf.env
    else:
        # Override this to change the environment
        env_name = "DriverDojo/Sem-TPS-Intersection-v0"

    print(f"Testing performance under environment {env_name}...")
    print("Measuring environment creation time...")
    init_times = []
    for i in tqdm(range(10)):
        a = time.time()
        env = gym.make(env_name)
        b = time.time()
        init_times.append(b - a)
        env.close()
    print(f"    Mean init time of {np.mean(init_times)}s.\n")

    print("Measuring environment reset time...")
    env = gym.make(env_name)
    reset_times = []
    for i in tqdm(range(10)):
        a = time.time()
        obs = env.reset()
        b = time.time()
        reset_times.append(b - a)
    print(f"    Mean reset time of {np.mean(reset_times)}s.\n")

    print("Measuring environment stepping time...")
    step_times = []
    num_steps = 300
    for i in tqdm(range(10)):
        obs = env.reset()
        a = time.time()
        for t in range(num_steps):
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
        b = time.time()
        step_times.append(b - a)
    env.close()
    print(
        f"    Mean time of {np.mean(step_times)}s for {num_steps} environment steps\n\
    -> ~{int(num_steps/np.mean(step_times))} steps/sec for single-threaded environment."
    )
    print(
        "Note that parallel environments for RL train will result in drastic computation speed-up!"
    )

    print("Done!")
