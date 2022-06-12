import gym
from tianshou.env import ShmemVectorEnv
from tianshou.data import Collector, VectorReplayBuffer, Batch

eps_rew = 0.0
arrived_at_goal = False
collision = False
off_route = False
time_step = 0
first_run = True
results = []
resets = 0


def preprocess_fn(**kwargs):
    # modify info before adding into the buffer, and recorded into tfb
    # if obs && env_id exist -> reset
    # if obs_next/rew/done/info/env_id exist -> normal step
    global eps_rew, first_run, arrived_at_goal, collision, off_route, time_step, resets, results
    reset = "obs" in kwargs

    if reset:
        if not first_run:
            print(f"Reset {resets}")
            results.append(
                f"{eps_rew},{collision},{off_route},{arrived_at_goal},{time_step}\n"
            )
            resets += 1
        eps_rew = 0.0
        arrived_at_goal = False
        collision = False
        off_route = False
        first_run = False
        time_step = 0
    else:
        info = kwargs["info"]["done_checks"]
        eps_rew += kwargs["rew"][0]
        collision = info["collision"][0]
        off_route = info["off_route"][0]
        arrived_at_goal = info["arrived_at_goal"][0]
        time_step = info["time_step"][0]

    return Batch()


import torch


def evaluate_agent(env, policy, model_path, on_test_set, n_episodes=100):
    global eps_rew, first_run, arrived_at_goal, collision, off_route, time_step, resets, results
    checkpoint = torch.load(model_path, map_location="cpu")
    policy.load_state_dict(checkpoint)
    policy.eval()
    collector = Collector(policy, env, preprocess_fn=preprocess_fn)
    result = collector.collect(n_episode=n_episodes)
    rews, lens = result["rews"], result["lens"]
    print(f"Final reward: {rews.mean()}, length: {lens.mean()}")
    file_name = "test_results" if on_test_set else "train_results"
    with open(file_name, "a") as f:
        f.writelines(results)

    eps_rew = 0.0
    results = []
    arrived_at_goal = False
    collision = False
    off_route = False
    time_step = 0
    first_run = True
    resets = 0


def make_envs(
    env_name,
    train_config,
    test_config,
    training_num,
    test_num,
    evaluate=False,
    on_test_set=False,
):
    if on_test_set:
        env = gym.make(env_name, config=test_config)
    else:
        env = gym.make(env_name, config=train_config)
    if evaluate:
        return env, None, None

    train_envs = ShmemVectorEnv(
        [lambda: gym.make(env_name, config=train_config) for _ in range(training_num)]
    )
    test_envs = ShmemVectorEnv(
        # Let's not get crazy. We limit the test environments to a maximum of 8.
        [
            lambda: gym.make(env_name, config=test_config)
            for _ in range(min(test_num, 8))
        ]
    )
    return env, train_envs, test_envs
