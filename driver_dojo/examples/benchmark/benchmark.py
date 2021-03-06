import hydra
import os
import logging
from omegaconf import OmegaConf, DictConfig
from driver_dojo.examples.benchmark.dqn import train_dqn
from driver_dojo.examples.benchmark.ppo import train_ppo
from driver_dojo.examples.benchmark.fqf import train_fqf


@hydra.main(version_base=None, config_path="experiments", config_name="config")
def benchmark(config: DictConfig) -> None:
    logging.info(os.getcwd())
    logging.info(OmegaConf.to_yaml(config))

    # Extract basic content
    env_name = config.env_name
    algo_config = config.algo.params
    algo_name = config.algo.name

    model_path = None
    if "model_path" in config:
        model_path = config.model_path

    # Setup external environment configurations
    obs_config = None
    if "obs" in config:
        obs_config = config.obs

    env_config_base = dict(observations={}, actions={}, vehicle={}, simulation={},)
    from copy import deepcopy

    train_config = deepcopy(env_config_base)
    if "env_train" in config:
        train_config = OmegaConf.merge(env_config_base, config.env_train)
    if obs_config:
        train_config = OmegaConf.merge(train_config, obs_config)

    test_config = deepcopy(env_config_base)
    if "env_test" in config:
        test_config = OmegaConf.merge(env_config_base, config.env_test)
    if obs_config:
        test_config = OmegaConf.merge(test_config, obs_config)

    # Logpath
    log_path = os.path.join(os.getcwd(), "output")

    # Find correct train function
    train_fn = None
    if algo_name == "dqn" or algo_name == "DQN":
        train_fn = train_dqn
    elif algo_name == "ppo" or algo_name == "PPO":
        train_fn = train_ppo
    elif algo_name == "fqf" or algo_name == "FQF":
        train_fn = train_fqf
    else:
        # It is straightforward to integrate more algorithms.
        # Adapt the implementations from: https://github.com/thu-ml/tianshou/tree/master/test
        raise NotImplementedError(
            "This algorithm isn't yet implemented for the benchmark!"
        )

    # Start training
    train_fn(
        env_name,
        algo_config,
        train_config,
        test_config,
        log_path,
        model_path=model_path,
        on_test_set=False,
    )

    # if model_path is not None:
    #     train_fn(
    #         env_name,
    #         algo_config,
    #         train_config,
    #         test_config,
    #         log_path,
    #         model_path=model_path,
    #         on_test_set=True,
    #     )


if __name__ == "__main__":
    benchmark()
