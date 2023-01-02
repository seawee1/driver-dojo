import ray
from ray import tune
from ray.rllib.algorithms.ppo import PPOConfig
from ray.tune.registry import register_env
from ray.air import RunConfig
from driver_dojo.core.config import Config
from driver_dojo.core.env import DriverDojoEnv
from driver_dojo.core.types import *
from training_scripts.callbacks import CustomCallback

if __name__ == '__main__':
    def env_creator(x):
        config = Config()
        config.actions.space = ActionSpace.Discretized
        config.vehicle.dynamics_model = DynamicsModel.KS
        config.simulation.max_episode_steps = 300
        config.scenario.traffic_init = True
        config.scenario.traffic_init_spread = 30.0
        config.scenario.traffic_spawn = True
        config.scenario.traffic_spawn_period = 1.0
        config.scenario.behavior_dist = False
        config.scenario.ego_init = True
        config.vehicle.v_max = 13.34
        config.scenario.name = 'Intersection'
        config.scenario.kwargs['crossing_style'] = 'Minor'
        config.scenario.tasks = ['L']
        config.scenario.num_maps = 1
        config.scenario.num_traffic = 1
        config.scenario.num_tasks = 1
        env = DriverDojoEnv(_config=config)
        return env


    register_env('custom_env', env_creator)

    ray.init()
    config = PPOConfig()
    config = config.environment(
        env='custom_env'
    )
    config = config.framework(
        framework='torch'
    )
    config = config.rollouts(
        num_rollout_workers=15,
    )
    # config = config.rollouts(
    #     num_rollout_workers=15,
    # )
    config = config.callbacks(CustomCallback)

    tuner = tune.Tuner(
        "PPO",
        run_config=RunConfig(
            stop=dict(
                timesteps_total=10000000,
            ),
        ),
        param_space=config.to_dict()
    )
    results = tuner.fit()
