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

from driver_dojo.examples.benchmark.utils import make_envs, StatsLogger, make_collectors
from driver_dojo.examples.benchmark.network import DQN, DictNet


def train_ppo(
    env_name,
    algo_config,
    train_config,
    test_config,
    log_path,
    eval=False,
    eval_file=None,
    eval_checkpoint=False,
):
    args = algo_config
    training_num, test_num = algo_config.training_num, algo_config.test_num
    env, train_envs, test_envs = make_envs(
        env_name,
        train_config,
        test_config,
        training_num,
        test_num,
    )


    # Seed
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    action_shape = env.action_space.shape or env.action_space.n

    if isinstance(env.observation_space, gym.spaces.Dict):
        shape_visual = env.observation_space['image'].shape
        num_feat = 128
        shape_vector = env.observation_space['vector'].shape
        shape_concat = (shape_vector[0] + num_feat)

        net_visual = DQN(
            shape_visual[0],
            shape_visual[1],
            shape_visual[2],
            action_shape,
            device=args.device,
            features_only=True,
            output_dim=num_feat
        )
        net_a = DictNet(
            net_visual,
            num_feat,
            shape_vector,
            args.device
        )
        net_b = DictNet(
            net_visual,
            num_feat,
            shape_vector,
            args.device
        )
    else:
        # Probe the environment and close
        state_shape = env.observation_space.shape or env.observation_space.n
        net_a = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
        net_b = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)

    if isinstance(env.action_space, gym.spaces.Box):
        from tianshou.utils.net.continuous import ActorProb, Critic

        #net_a = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
        #net_b = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
        actor = ActorProb(
            net_a, action_shape, max_action=args.max_action, device=args.device
        ).to(args.device)
        critic = Critic(net_b, device=args.device).to(args.device)
        actor_critic = ActorCritic(actor, critic)

    else:
        from tianshou.utils.net.discrete import Actor, Critic

        #net_a = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
        #net_b = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
        # if args.device == "cuda" and torch.cuda.is_available():
        #     actor = DataParallelNet(
        #         Actor(net_a, action_shape, device=None).to(args.device)
        #     )
        #     critic = DataParallelNet(Critic(net_b, device=None).to(args.device))
        # else:
        actor = Actor(net_a, action_shape, device=args.device).to(args.device)
        critic = Critic(net_b, device=args.device).to(args.device)
        actor_critic = ActorCritic(actor, critic)

    # Create model
    # if isinstance(env.action_space, gym.spaces.Box):
    #     from tianshou.utils.net.continuous import ActorProb, Critic
    #
    #     net_a = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
    #     net_b = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
    #     actor = ActorProb(
    #         net_a, action_shape, max_action=args.max_action, device=args.device
    #     ).to(args.device)
    #     critic = Critic(net_b, device=args.device,).to(args.device)
    #     actor_critic = ActorCritic(actor, critic)
    #
    # else:
    #     from tianshou.utils.net.discrete import Actor, Critic
    #
    #     net_a = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
    #     net_b = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
    #     if args.device == "cuda" and torch.cuda.is_available():
    #         actor = DataParallelNet(
    #             Actor(net_a, action_shape, device=None).to(args.device)
    #         )
    #         critic = DataParallelNet(Critic(net_b, device=None).to(args.device))
    #     else:
    #         actor = Actor(net_a, action_shape, device=args.device).to(args.device)
    #         critic = Critic(net_b, device=args.device).to(args.device)
    #     actor_critic = ActorCritic(actor, critic)

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
        deterministic_eval=False,
        advantage_normalization=args.advantage_normalization,
        recompute_advantage=args.recompute_advantage,
    )

    # Log
    writer = SummaryWriter(log_path)
    logger = TensorboardLogger(writer)

    train_collector, test_collector, train_stats, test_stats = make_collectors(**locals())

    if eval:
        if eval_checkpoint:
            policy.load_state_dict(torch.load(os.path.join(log_path, 'checkpoint.pth'))['model'], strict=True)
        else:
            policy.load_state_dict(torch.load(os.path.join(log_path, 'policy.pth')), strict=True)

        result = test_collector.collect(n_episode=test_num)
        summary = {
            "n/ep": result["n/ep"],
            "rew": result['rew'],
            "len": result['len'],
            "rew_std": result['rew_std'],
            "len_std": result['len_std'],
        }
        test_stats.log_to_file(summary)
        env.close()
        test_envs.close()
        train_envs.close() if train_envs else None
        return

    def train_fn(epoch, env_step):
        policy.train() if not policy.training else None

    def test_fn(epoch, env_step):
        test_stats._step_overwrite = env_step
        policy.eval()

    def save_best_fn(policy):
        torch.save(policy.state_dict(), os.path.join(log_path, "policy.pth"))

    def save_checkpoint_fn(epoch, env_step, gradient_step):
        # see also: https://pytorch.org/tutorials/beginner/saving_loading_models.html
        ckpt_path = os.path.join(log_path, "checkpoint.pth")
        # Example: saving by epoch num
        # ckpt_path = os.path.join(log_path, f"checkpoint_{epoch}.pth")
        torch.save(
            {
                "model": policy.state_dict(),
                "optim": optim.state_dict(),
            }, ckpt_path
        )
        return ckpt_path

    env.close()
    result = onpolicy_trainer(
        policy,
        train_collector,
        test_collector,
        args.epoch,
        args.step_per_epoch,
        args.repeat_per_collect,
        args.test_num if args.test_num else 1,
        args.batch_size,
        step_per_collect=args.step_per_collect,
        save_best_fn=save_best_fn,
        train_fn=train_fn,
        test_fn=test_fn,
        logger=logger,
        test_in_train=(test_collector is not None),
        save_checkpoint_fn=save_checkpoint_fn,
    )

    pprint.pprint(result)
