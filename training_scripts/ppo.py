import ray
from ray import tune, air
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
        config.simulation.dt = 0.2
        config.simulation.max_time_steps = 500
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
        config.scenario.num_maps = 10
        config.scenario.num_traffic = 1
        config.scenario.num_tasks = 1
        #config.observations.feature_scaling='Standardize'
        env = DriverDojoEnv(_config=config)
        from gym.wrappers import FrameStack, NormalizeObservation
        #env = NormalizeObservation(env, 5)
        return env


    register_env('custom_env', env_creator)

    ray.init()
    config = PPOConfig()
    config = config.training(
        gamma=0.99,
        lambda_=0.95,
        train_batch_size=8192,
        sgd_minibatch_size=256,
        lr=0.00005,
        clip_param=0.2,
        num_sgd_iter=20,
        kl_coeff=0.2,
    )
    config = config.environment(
        env='custom_env',
        disable_env_checking=True
    )
    config = config.framework(
        framework='torch',
    )
    config = config.rollouts(
        num_rollout_workers=23,
        num_envs_per_worker=1,
        rollout_fragment_length='auto',
        horizon=500,
    )
    config = config.resources(
        num_gpus=1,
    )
    config = config.callbacks(CustomCallback)
    config.model['framestack'] = True
    config.model['fcnet_hiddens'] = [512, 512]

    tuner = tune.Tuner(
        "PPO",
        run_config=RunConfig(
            stop=dict(
                timesteps_total=10000000,
            ),
            verbose=1,
            checkpoint_config=air.CheckpointConfig(
                checkpoint_frequency=1, checkpoint_at_end=True,
            ),
        ),
        param_space=config.to_dict()
    )
    results = tuner.fit()
