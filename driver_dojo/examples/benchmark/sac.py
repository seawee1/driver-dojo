import os
import pprint

import gym
import numpy as np
import torch
from torch.utils.tensorboard import SummaryWriter

from tianshou.data import Collector, VectorReplayBuffer
from tianshou.policy import DiscreteSACPolicy, SACPolicy
from tianshou.trainer import offpolicy_trainer
from tianshou.utils import TensorboardLogger
from tianshou.utils.net.common import Net

from driver_dojo.examples.benchmark.utils import make_envs, evaluate_agent
from tianshou.data import Collector, VectorReplayBuffer, Batch


def train_sac(
        env_name,
        algo_config,
        train_config,
        test_config,
        log_path,
        model_path=None,
        on_test_set=False,
):
    evaluate = model_path is not None
    args = algo_config
    # Create probe, train and test environments
    training_num, test_num = algo_config.training_num, algo_config.test_num
    env, train_envs, test_envs = make_envs(
        env_name,
        train_config,
        test_config,
        training_num,
        test_num,
        evaluate=evaluate,
        on_test_set=on_test_set,
    )

    state_shape = env.observation_space.shape or env.observation_space.n
    action_shape = env.action_space.shape or env.action_space.n
    discrete_actions = isinstance(env.action_space, gym.spaces.Discrete)
    if discrete_actions:
        from tianshou.utils.net.discrete import Actor, Critic
    else:
        from tianshou.utils.net.continuous import ActorProb, Critic

    # Seed
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    # model
    net = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)  # .to(args.device)
    if discrete_actions:
        actor = Actor(net, action_shape, softmax_output=False, device=args.device).to(args.device)
        net_c1 = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
        critic1 = Critic(net_c1, last_size=action_shape, device=args.device).to(args.device)
        net_c2 = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
        critic2 = Critic(net_c2, last_size=action_shape, device=args.device).to(args.device)
    else:
        actor = ActorProb(net, action_shape, max_action=1, device=args.device, unbounded=False).to(args.device)
        net_c1 = Net(state_shape, action_shape, hidden_sizes=args.hidden_sizes, concat=True, device=args.device)
        critic1 = Critic(net_c1, device=args.device).to(args.device)
        net_c2 = Net(state_shape, action_shape, hidden_sizes=args.hidden_sizes, concat=True, device=args.device)
        critic2 = Critic(net_c2, device=args.device).to(args.device)
    actor_optim = torch.optim.Adam(actor.parameters(), lr=args.lr)
    critic1_optim = torch.optim.Adam(critic1.parameters(), lr=args.lr)
    critic2_optim = torch.optim.Adam(critic2.parameters(), lr=args.lr)

    # better not to use auto alpha in CartPole
    if args.auto_alpha:
        target_entropy = 0.98 * np.log(np.prod(action_shape))
        log_alpha = torch.zeros(1, requires_grad=True, device=args.device)
        alpha_optim = torch.optim.Adam([log_alpha], lr=args.alpha_lr)
        args.alpha = (target_entropy, log_alpha, alpha_optim)

    if discrete_actions:
        policy = DiscreteSACPolicy(
            actor,
            actor_optim,
            critic1,
            critic1_optim,
            critic2,
            critic2_optim,
            args.tau,
            args.gamma,
            args.alpha,
            reward_normalization=args.reward_normalization
        )
    else:
        from tianshou.exploration import GaussianNoise
        policy = SACPolicy(
            actor,
            actor_optim,
            critic1,
            critic1_optim,
            critic2,
            critic2_optim,
            tau=args.tau,
            gamma=args.gamma,
            alpha=args.alpha,
            reward_normalization=args.reward_normalization,
            exploration_noise=GaussianNoise(sigma=args.exploration_noise),
            action_space=env.action_space
        )

    from collections import deque
    class StatsLogger:
        def __init__(self, is_train, logger, buffer_size=100, log_interval=5000, num_episodes=None, empty_after_log=True):
            self._prefix = 'train' if is_train else 'test'
            self._logger = logger
            self._log_interval = log_interval
            self._num_episodes = num_episodes
            assert self._log_interval is None or self._num_episodes is None
            self._empty_after_log = empty_after_log
            self._buffer_size = buffer_size
            self._step = 0
            self._step_last_log = 0

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
            self._step += len(kwargs['env_id'])

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
                log_data = {
                    f"{self._prefix}/timeout": np.mean(self._stats['timeout']),
                    f"{self._prefix}/timeout_standing_still": np.mean(self._stats['timeout_standing_still']),
                    f"{self._prefix}/goal": np.mean(self._stats['reached_goal']),
                    f"{self._prefix}/collision": np.mean(self._stats['collision']),
                    f"{self._prefix}/off_route": np.mean(self._stats['off_route']),
                }
                self._logger.write(f"{self._prefix}/env_step", self._step if not hasattr(self, '_step_overwrite') else self._step_overwrite, log_data)

                if self._empty_after_log:
                    for k, v in self._stats.items():
                        self._stats[k] = deque(maxlen=self._buffer_size)
            return Batch()

    writer = SummaryWriter(log_path)
    logger = TensorboardLogger(writer)

    train_stats = StatsLogger(True, logger)
    # collector
    train_collector = Collector(
        policy, train_envs, VectorReplayBuffer(args.buffer_size, len(train_envs)), preprocess_fn=train_stats.preprocess_fn
    )
    test_collector = None
    if test_envs:
        test_stats = StatsLogger(False, logger, log_interval=None, num_episodes=args.test_num, buffer_size=args.test_num)
        test_collector = Collector(policy, test_envs, preprocess_fn=test_stats.preprocess_fn)

    def save_best_fn(policy):
        torch.save(policy.state_dict(), os.path.join(log_path, 'policy.pth'))

    def test_fn(epoch, env_step):
        test_stats._step_overwrite = env_step

    # trainer
    result = offpolicy_trainer(
        policy,
        train_collector,
        test_collector,
        args.epoch,
        args.step_per_epoch,
        args.step_per_collect,
        args.test_num if args.test_num else 1,
        args.batch_size,
        save_best_fn=save_best_fn,
        test_fn=test_fn,
        logger=logger,
        update_per_step=args.update_per_step,
        test_in_train=(test_envs is not None)
    )
    pprint.pprint(result)
