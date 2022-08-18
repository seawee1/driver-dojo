import argparse
import os
import pprint

import gym
import numpy as np
import torch
from torch.utils.tensorboard import SummaryWriter

from tianshou.data import Collector, PrioritizedVectorReplayBuffer, VectorReplayBuffer
from tianshou.policy import FQFPolicy
from tianshou.trainer import offpolicy_trainer
from tianshou.utils import TensorboardLogger
from tianshou.utils.net.common import Net
from tianshou.utils.net.discrete import FractionProposalNetwork, FullQuantileFunction

from driver_dojo.examples.benchmark.utils import make_envs, StatsLogger


def train_fqf(
    env_name,
    algo_config,
    train_config,
    test_config,
    log_path,
    eval=False,
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

    state_shape = env.observation_space.shape or env.observation_space.n
    action_shape = env.action_space.shape or env.action_space.n

    # seed
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    # model
    feature_net = Net(
        state_shape,
        args.hidden_sizes[-1],
        hidden_sizes=args.hidden_sizes[:-1],
        device=args.device,
        softmax=False,
    )
    net = FullQuantileFunction(
        feature_net,
        action_shape,
        args.hidden_sizes,
        num_cosines=args.num_cosines,
        device=args.device,
    )
    optim = torch.optim.Adam(net.parameters(), lr=args.lr)
    fraction_net = FractionProposalNetwork(args.num_fractions, net.input_dim)
    fraction_optim = torch.optim.RMSprop(fraction_net.parameters(), lr=args.fraction_lr)
    policy = FQFPolicy(
        net,
        optim,
        fraction_net,
        fraction_optim,
        args.gamma,
        args.num_fractions,
        args.ent_coef,
        args.n_step,
        target_update_freq=args.target_update_freq,
    ).to(args.device)

    # buffer
    if args.prioritized_replay:
        buf = PrioritizedVectorReplayBuffer(
            args.buffer_size,
            buffer_num=len(train_envs),
            alpha=args.alpha,
            beta=args.beta,
        )
    else:
        buf = VectorReplayBuffer(args.buffer_size, buffer_num=len(train_envs))

    # collector
    train_collector = Collector(policy, train_envs, buf, exploration_noise=True)
    test_collector = Collector(policy, test_envs, exploration_noise=True)
    train_collector.collect(n_step=args.batch_size * args.training_num)

    # log
    writer = SummaryWriter(log_path)
    logger = TensorboardLogger(writer)

    def save_best_fn(policy):
        torch.save(policy.state_dict(), os.path.join(log_path, "policy.pth"))

    def save_checkpoint_fn(epoch, env_step, gradient_step):
        # see also: https://pytorch.org/tutorials/beginner/saving_loading_models.html
        torch.save(
            {"model": policy.state_dict(), "optim": optim.state_dict(),},
            os.path.join(log_path, "checkpoint.pth"),
        )

    num_steps = algo_config.step_per_epoch * algo_config.epoch
    num_steps_decay = num_steps * algo_config.eps_fraction

    def train_fn(epoch, env_step):
        # linear decay
        frac = min(env_step / num_steps_decay, 1.0)
        eps = algo_config.eps_train - frac * (
            algo_config.eps_train - algo_config.eps_test
        )
        policy.set_eps(eps)

    def test_fn(epoch, env_step):
        policy.set_eps(algo_config.eps_test)

    # trainer
    result = offpolicy_trainer(
        policy,
        train_collector,
        test_collector,
        args.epoch,
        args.step_per_epoch,
        args.step_per_collect,
        args.test_num,
        args.batch_size,
        train_fn=train_fn,
        test_fn=test_fn,
        save_best_fn=save_best_fn,
        logger=logger,
        update_per_step=args.update_per_step,
        test_in_train=True,
        save_checkpoint_fn=save_checkpoint_fn,
    )

    pprint.pprint(result)
    # Let's watch its performance!
    env = gym.make(args.task)
    policy.eval()
    policy.set_eps(args.eps_test)
    collector = Collector(policy, env)
    result = collector.collect(n_episode=1, render=args.render)
    rews, lens = result["rews"], result["lens"]
    print(f"Final reward: {rews.mean()}, length: {lens.mean()}")
