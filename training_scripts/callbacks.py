from ray.rllib.algorithms.callbacks import DefaultCallbacks

class CustomCallback(DefaultCallbacks):
    def on_episode_end(
            self,
            *,
            worker,
            base_env,
            policies,
            episode,
            env_index = None,
            **kwargs,
    ) -> None:
        info_dict = episode.last_info_for()
        for k, v in info_dict.items():
            episode.custom_metrics[k] = v
