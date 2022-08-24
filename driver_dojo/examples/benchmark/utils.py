import gym
from tianshou.env import ShmemVectorEnv
from tianshou.data import Collector, VectorReplayBuffer, Batch
from collections import deque
import numpy as np


class StatsLogger:
    def __init__(
            self,
            is_train,
            logger,
            buffer_size=100,
            log_interval=5000,
            num_episodes=None,
            empty_after_log=True,
            log_path=None
    ):
        self._prefix = 'train' if is_train else 'test'
        self._logger = logger
        self._log_interval = log_interval
        self._num_episodes = num_episodes
        assert self._log_interval is None or self._num_episodes is None
        self._empty_after_log = empty_after_log
        self._buffer_size = buffer_size
        self._step = 0
        self._step_last_log = 0
        self._log_path = log_path

        self._stats = dict(
            reached_goal=deque(maxlen=buffer_size),
            collision=deque(maxlen=buffer_size),
            off_route=deque(maxlen=buffer_size),
            timeout=deque(maxlen=buffer_size),
            timeout_standing_still=deque(maxlen=buffer_size),
        )

    def preprocess_fn(self, **kwargs):
        # modify info before adding into the buffer, and recorded into tfb
        # if obs && env_id exist -> reset
        # if obs_next/rew/done/info/env_id exist -> normal step
        # n_envs = len(kwargs['env_id'])
        # if self._rews is None:
        #     self._rews = np.zeros(n_envs)
        #     self._lens = np.zeros(n_envs)
        #     # self._rews = {env_id: 0.0 for env_id in kwargs['env_id']}
        #     # self._lens = {env_id: 0.0 for env_id in kwargs['env_id']}
        #
        # # Log reward and eps len
        # if 'obs_next' in kwargs:
        #     self._rews[kwargs['env_id']] += kwargs['rew']
        #     self._lens[kwargs['env_id']] += 1.0
        #     for i in np.argwhere(kwargs['done']):
        #         self._stats['rews'].append(self._rews[i])
        #         self._rews[i] = 0.0
        #         self._stats['lens'].append(self._lens[i])
        #         self._lens[i] = 0.0
        if 'obs_next' in kwargs:
            self._step += len(kwargs['obs_next'])
            print(self._prefix, kwargs['info']['maps_seed'], kwargs['info']['traffic_seed'])

        # Log other stuff
        if 'info' in kwargs:
            if not np.any(kwargs['done']):
                return Batch()

            done_kwargs = kwargs['info'][kwargs['done']]
            for k, v in self._stats.items():
                num_hits = np.sum(done_kwargs[k])
                v.extend([1.0] * num_hits)
                for i, j in self._stats.items():
                    if k == i: continue
                    j.extend([0.0] * num_hits)

        if (self._log_interval and self._step - self._step_last_log >= self._log_interval) or len(self._stats['reached_goal']) == self._num_episodes:
            self._step_last_log = self._step
            summary = {
                "timeout": np.mean(self._stats['timeout']),
                "timeout_std": np.std(self._stats['timeout']),
                "timeout_standing_still": np.mean(self._stats['timeout_standing_still']),
                "timeout_standing_still_std": np.std(self._stats['timeout_standing_still']),
                "goal": np.mean(self._stats['reached_goal']),
                "goal_std": np.std(self._stats['reached_goal']),
                "collision": np.mean(self._stats['collision']),
                "collision_std": np.std(self._stats['collision']),
                "off_route": np.mean(self._stats['off_route']),
                "off_route_std": np.std(self._stats['off_route']),
            }

            if self._log_path:
                # import yaml
                # summary_add = {
                #     "num_samples": len(self._stats['rews']),
                # #     "reward": np.mean(self._stats['rews']),
                # #     "reward_std": np.std(self._stats['rews']),
                # #     "len": np.mean(self._stats['lens']),
                # #     "len_std": np.std(self._stats['lens'])
                # }
                # summary.update(summary_add)
                self.log_to_file(summary)
            else:
                self.log_to_tensorboard(summary)

            if self._empty_after_log:
                for k, v in self._stats.items():
                    self._stats[k] = deque(maxlen=self._buffer_size)
        return Batch()

    def log_to_tensorboard(self, summary):
        log_data = {}
        for k, v in summary.items():
            log_data[f"{self._prefix}/{k}"] = v
        self._logger.write(f"{self._prefix}/env_step", self._step if not hasattr(self, '_step_overwrite') else self._step_overwrite, log_data)

    def log_to_file(self, summary):
        import yaml, os
        for k, v in summary.items():
            summary[k] = [float(v)]

        if os.path.isfile(self._log_path):
            with open(self._log_path, 'r') as f:
                summary_old = yaml.safe_load(f)

                for k, v in summary_old.items():
                    if not isinstance(v, list): v = list(v)

                    if k in summary:
                        summary[k] = v + summary[k]
                    else:
                        summary[k] = v

        with open(self._log_path, 'w') as f:
            yaml.dump(summary, f, default_flow_style=False)


def make_envs(
        env_name,
        train_config,
        test_config,
        training_num,
        test_num,
):
    train_envs, test_envs = None, None
    env = gym.make(env_name, config=train_config)

    from tianshou.env.venvs import SubprocVectorEnv
    if training_num:
        train_envs = SubprocVectorEnv(
            [lambda: gym.make(env_name, config=train_config) for _ in range(training_num)]
        )
    if test_num:
        test_envs = SubprocVectorEnv(
            [
                lambda: gym.make(env_name, config=test_config)
                for _ in range(min(test_num, 8))
            ]
        )
    return env, train_envs, test_envs


def make_collectors(**kwargs):
    import os
    train_collector, train_stats = None, None
    if kwargs['train_envs']:
        train_stats = StatsLogger(True, kwargs['logger'])
        train_collector = Collector(
            kwargs['policy'], kwargs['train_envs'], VectorReplayBuffer(kwargs['args'].buffer_size, len(kwargs['train_envs'])), preprocess_fn=train_stats.preprocess_fn
        )
    test_collector, test_stats = None, None
    if kwargs['test_envs']:
        test_stats = StatsLogger(False, kwargs['logger'], log_interval=None, num_episodes=kwargs['args'].test_num, buffer_size=kwargs['args'].test_num,
                                 log_path=os.path.join(kwargs['log_path'], kwargs['eval_file']) if kwargs['eval'] else None)
        test_collector = Collector(kwargs['policy'], kwargs['test_envs'], preprocess_fn=test_stats.preprocess_fn)

    return train_collector, test_collector, train_stats, test_stats
