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

from driver_dojo.examples.benchmark.utils import make_envs, StatsLogger, make_collectors
from tianshou.data import Collector, VectorReplayBuffer, Batch


def train_sac(
        env_name,
        algo_config,
        train_config,
        test_config,
        log_path,
        eval=False,
        eval_file='results.yaml',
        eval_checkpoint=False,
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
    discrete_actions = isinstance(env.action_space, gym.spaces.Discrete)

    # Seed
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    # model
    net = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)  # .to(args.device)
    if discrete_actions:
        from tianshou.utils.net.discrete import Actor, Critic
        actor = Actor(net, action_shape, softmax_output=False, device=args.device).to(args.device)
        net_c1 = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
        critic1 = Critic(net_c1, last_size=action_shape, device=args.device).to(args.device)
        net_c2 = Net(state_shape, hidden_sizes=args.hidden_sizes, device=args.device)
        critic2 = Critic(net_c2, last_size=action_shape, device=args.device).to(args.device)
    else:
        from tianshou.utils.net.continuous import ActorProb, Critic
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
            reward_normalization=args.reward_normalization,
            deterministic_eval=False,
        )
    else:
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
            action_space=env.action_space,
            deterministic_eval=False,
        )

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
        policy.eval()
        test_stats._step_overwrite = env_step

    def save_best_fn(policy):
        torch.save(policy.state_dict(), os.path.join(log_path, 'policy.pth'))

    def save_checkpoint_fn(epoch, env_step, gradient_step):
        # see also: https://pytorch.org/tutorials/beginner/saving_loading_models.html
        ckpt_path = os.path.join(log_path, "checkpoint.pth")
        # Example: saving by epoch num
        # ckpt_path = os.path.join(log_path, f"checkpoint_{epoch}.pth")
        torch.save(
            {
                "model": policy.state_dict(),
                "optim_actor": actor_optim.state_dict(),
                "optim_critic1": critic1_optim.state_dict(),
                "optim_critic2": critic2_optim.state_dict(),
            }, ckpt_path
        )
        return ckpt_path

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
        train_fn=train_fn,
        test_fn=test_fn,
        logger=logger,
        update_per_step=args.update_per_step,
        test_in_train=(test_envs is not None),
        save_checkpoint_fn=save_checkpoint_fn
    )
    pprint.pprint(result)
