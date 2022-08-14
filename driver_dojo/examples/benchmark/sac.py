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
    net = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
    if discrete_actions:
        actor = Actor(net, action_shape, softmax_output=False, device=args.device).to(args.device)
    else:
        actor = ActorProb(net, action_shape, max_action=1, device=args.device, unbounded=False)
    actor_optim = torch.optim.Adam(actor.parameters(), lr=args.lr)

    if discrete_actions:
        net_c1 = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
        critic1 = Critic(net_c1, last_size=action_shape, device=args.device).to(args.device)
    else:
        net_c1 = Net(state_shape, action_shape, hidden_sizes=args.hidden_sizes, concat=True, device=args.device)
        critic1 = Critic(net_c1, device=args.device).to(args.device)
    critic1_optim = torch.optim.Adam(critic1.parameters(), lr=args.lr)

    if discrete_actions:
        net_c2 = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
        critic2 = Critic(net_c2, last_size=action_shape, device=args.device).to(args.device)
    else:
        net_c2 = Net(state_shape, action_shape, hidden_sizes=args.hidden_sizes, concat=True, device=args.device)
        critic2 = Critic(net_c2, device=args.device).to(args.device)
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
    # collector
    train_collector = Collector(
        policy, train_envs, VectorReplayBuffer(args.buffer_size, len(train_envs))
    )
    test_collector = None
    if test_envs:
        test_collector = Collector(policy, test_envs)

    writer = SummaryWriter(log_path)
    logger = TensorboardLogger(writer)

    def save_best_fn(policy):
        torch.save(policy.state_dict(), os.path.join(log_path, 'policy.pth'))

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
        logger=logger,
        update_per_step=args.update_per_step,
        test_in_train=(test_envs is not None)
    )
    pprint.pprint(result)
