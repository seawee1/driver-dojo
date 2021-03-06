import argparse
import os
import pprint

import gym
import numpy as np
import torch
from torch.optim.lr_scheduler import LambdaLR
from torch.utils.tensorboard import SummaryWriter

from tianshou.data import Collector, VectorReplayBuffer
from tianshou.policy import PPOPolicy
from tianshou.trainer import onpolicy_trainer
from tianshou.utils import TensorboardLogger
from tianshou.utils.net.common import ActorCritic, DataParallelNet, Net

from driver_dojo.examples.benchmark.utils import make_envs, evaluate_agent
from driver_dojo.examples.benchmark.atari_network import DQN


def train_ppo(
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

    # Probe the environment and close
    state_shape = env.observation_space.shape or env.observation_space.n
    action_shape = env.action_space.shape or env.action_space.n

    # Seed
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    # Create model
    visual = False
    if "visual" in args:
        visual = args.visual
    print(visual)
    if visual and isinstance(env.action_space, gym.spaces.Discrete):
        from tianshou.utils.net.discrete import Actor, Critic

        # IMPORTANT: Set cwh to True inside env
        net = DQN(
            state_shape[0],
            state_shape[1],
            state_shape[2],
            action_shape,
            device=args.device,
            features_only=True,
            output_dim=args.hidden_sizes,
        )
        actor = Actor(net, action_shape, device=args.device).to(args.device)
        critic = Critic(net, device=args.device,).to(args.device)
        actor_critic = ActorCritic(actor, critic)
    else:
        if isinstance(env.action_space, gym.spaces.Box):
            from tianshou.utils.net.continuous import ActorProb, Critic

            net_a = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
            net_b = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
            actor = ActorProb(
                net_a, action_shape, max_action=args.max_action, device=args.device
            ).to(args.device)
            critic = Critic(net_b, device=args.device,).to(args.device)
            actor_critic = ActorCritic(actor, critic)

        elif isinstance(env.action_space, gym.spaces.Discrete):
            from tianshou.utils.net.discrete import Actor, Critic

            net_a = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
            net_b = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
            if args.device == "cuda" and torch.cuda.is_available():
                actor = DataParallelNet(
                    Actor(net_a, action_shape, device=None).to(args.device)
                )
                critic = DataParallelNet(Critic(net_b, device=None).to(args.device))
            else:
                actor = Actor(net_a, action_shape, device=args.device).to(args.device)
                critic = Critic(net_b, device=args.device).to(args.device)
            actor_critic = ActorCritic(actor, critic)

    # orthogonal initialization
    # for m in actor_critic.modules():
    #     if isinstance(m, torch.nn.Linear):
    #         torch.nn.init.orthogonal_(m.weight)
    #         torch.nn.init.zeros_(m.bias)

    # Optimizer
    optim = torch.optim.Adam(actor_critic.parameters(), lr=args.lr)

    # learning rate schedule
    lr_scheduler = None
    if algo_config.lr_decay:
        # decay learning rate to 0 linearly
        max_update_num = (
            np.ceil(algo_config.step_per_epoch / algo_config.step_per_collect)
            * algo_config.epoch
        )

        lr_scheduler = LambdaLR(
            optim, lr_lambda=lambda epoch: 1 - epoch / max_update_num
        )

    if isinstance(env.action_space, gym.spaces.Box):

        def dist(*logits):
            return torch.distributions.Independent(
                torch.distributions.Normal(*logits), 1
            )

    elif isinstance(env.action_space, gym.spaces.Discrete):
        dist = torch.distributions.Categorical

    policy = PPOPolicy(
        actor,
        critic,
        optim,
        dist,
        discount_factor=args.gamma,
        eps_clip=args.eps_clip,
        dual_clip=args.dual_clip,
        value_clip=args.value_clip,
        max_grad_norm=args.max_grad_norm,
        vf_coef=args.vf_coef,
        ent_coef=args.ent_coef,
        gae_lambda=args.gae_lambda,
        reward_normalization=args.reward_normalization,
        action_space=env.action_space,
        action_scaling=algo_config.action_scaling,
        deterministic_eval=True,
        advantage_normalization=args.advantage_normalization,
        recompute_advantage=args.recompute_advantage,
    )

    if evaluate:
        evaluate_agent(env, policy, model_path, on_test_set)
        env.close()
        return

    # Collectors
    train_collector = Collector(
        policy, train_envs, VectorReplayBuffer(args.buffer_size, len(train_envs))
    )
    test_collector = Collector(policy, test_envs)

    # Log
    writer = SummaryWriter(log_path)
    logger = TensorboardLogger(writer)

    def save_best_fn(policy):
        torch.save(policy.state_dict(), os.path.join(log_path, "policy.pth"))

    def save_checkpoint_fn(epoch, env_step, gradient_step):
        ckpt_path = os.path.join(log_path, "checkpoint.pth")
        torch.save({"model": policy.state_dict()}, ckpt_path)
        return ckpt_path

    # We can close the probe env now

    env.close()
    # trainer
    result = onpolicy_trainer(
        policy,
        train_collector,
        test_collector,
        args.epoch,
        args.step_per_epoch,
        args.repeat_per_collect,
        args.test_num,
        args.batch_size,
        step_per_collect=args.step_per_collect,
        save_best_fn=save_best_fn,
        logger=logger,
        test_in_train=True,
        save_checkpoint_fn=save_checkpoint_fn,
    )

    pprint.pprint(result)
