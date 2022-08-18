import os
import pprint

import gym
import numpy as np
import torch
from torch.utils.tensorboard import SummaryWriter

from tianshou.data import Collector, PrioritizedVectorReplayBuffer, VectorReplayBuffer
from tianshou.policy import DQNPolicy
from tianshou.trainer import offpolicy_trainer
from tianshou.utils import TensorboardLogger
from tianshou.utils.net.common import Net

from driver_dojo.examples.benchmark.utils import make_envs, make_collectors


def train_dqn(
    env_name,
    algo_config,
    train_config,
    test_config,
    log_path,
    eval=False,
    eval_file=None,
):
    args = algo_config
    # Create probe, train and test environments

    training_num, test_num = algo_config.training_num, algo_config.test_num
    env, train_envs, test_envs = make_envs(
        env_name,
        train_config,
        test_config,
        training_num,
        test_num,
    )

    # Probe the environment and close
    state_shape = env.observation_space.shape or env.observation_space.n
    action_shape = env.action_space.shape or env.action_space.n

    # Seed
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    # Dueling DQN
    dueling = None
    if algo_config.dueling_q_hidden_sizes:
        Q_param = {"hidden_sizes": algo_config.dueling_q_hidden_sizes}
        V_param = {"hidden_sizes": algo_config.dueling_v_hidden_sizes}
        dueling = (Q_param, V_param)

    # model
    net = Net(
        state_shape,
        action_shape,
        hidden_sizes=args.hidden_sizes,
        device=args.device,
        dueling_param=dueling,
        softmax=False,
    ).to(args.device)
    optim = torch.optim.Adam(net.parameters(), lr=args.lr)
    policy = DQNPolicy(
        net,
        optim,
        args.gamma,
        args.n_step,
        is_double=True,  # args.is_double,
        target_update_freq=args.target_update_freq,
    )

    # Prioritized Replay
    if args.prioritized_replay:
        buf = PrioritizedVectorReplayBuffer(
            args.buffer_size,
            buffer_num=training_num if training_num else 1,
            alpha=args.alpha,
            beta=args.beta,
        )
    else:
        buf = VectorReplayBuffer(args.buffer_size, buffer_num=training_num if training_num else 1)

    # Tensorboard
    writer = SummaryWriter(log_path)
    logger = TensorboardLogger(writer)

    train_collector, test_collector, train_stats, test_stats = make_collectors(**locals())

    if eval:
        policy.load_state_dict(torch.load(os.path.join(log_path, 'policy.pth')), strict=False)
        test_collector.collect(n_episode=test_num)
        env.close()
        test_envs.close()
        train_envs.close() if train_envs else None
        return

    num_steps = algo_config.step_per_epoch * algo_config.epoch
    num_steps_decay = num_steps * algo_config.eps_fraction

    def train_fn(epoch, env_step):
        frac = min(env_step / num_steps_decay, 1.0)
        eps = algo_config.eps_train - frac * (
            algo_config.eps_train - algo_config.eps_test
        )
        policy.set_eps(eps)
        policy.train() if not policy.training else None

    def test_fn(epoch, env_step):
        test_stats._step_overwrite = env_step
        policy.set_eps(algo_config.eps_test)
        policy.eval()

    def save_best_fn(policy):
        torch.save(policy.state_dict(), os.path.join(log_path, "policy.pth"))

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
        update_per_step=args.update_per_step,
        train_fn=train_fn,
        test_fn=test_fn,
        save_best_fn=save_best_fn,
        logger=logger,
        test_in_train=(test_envs is not None),
    )

    pprint.pprint(result)
