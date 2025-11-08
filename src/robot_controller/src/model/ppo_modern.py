"""
Modern PPO implementation with advanced techniques:
- Value clipping (PPO paper)
- Huber loss option
- Better gradient handling
- Improved logging

Based on:
- OpenAI Spinning Up
- Stable-Baselines3
- CleanRL
"""
import torch
import logging
import os
import numpy as np
import socket
from torch.utils.data import TensorDataset, DataLoader, RandomSampler
import torch.nn.functional as F
from typing import Optional, Dict
from torch.optim import Adam

from .net import ActorCriticNetwork

hostname = socket.gethostname()
if not os.path.exists('./log/' + hostname):
    os.makedirs('./log/' + hostname)
ppo_file = './log/' + hostname + '/ppo_modern.log'

logger_ppo = logging.getLogger('loggerppo_modern')
logger_ppo.setLevel(logging.INFO)
ppo_file_handler = logging.FileHandler(ppo_file, mode='a')
ppo_file_handler.setLevel(logging.INFO)
logger_ppo.addHandler(ppo_file_handler)


def explained_variance(y_pred: torch.Tensor, y_true: torch.Tensor) -> float:
    """
    Explained variance between predictions and targets.
    1.0 is perfect, 0.0 is no better than mean.
    """
    var_y = torch.var(y_true)
    if var_y.item() == 0.0:
        return 0.0
    return (1.0 - torch.var(y_true - y_pred) / var_y).item()


def ppo_update_stage1_modern(
        policy: ActorCriticNetwork,
        optimizer,
        batch_size: int,
        memory,
        epoch: int,
        coeff_entropy: float = 0.02,
        clip_value: float = 0.2,
        num_step: int = 2048,
        num_env: int = 12,
        frames: int = 1,
        obs_size: int = 24,
        act_size: int = 4,
        value_loss_coeff: float = 1.0,
        value_clip: bool = True,
        use_huber_loss: bool = False,
        huber_delta: float = 10.0,
        max_grad_norm: float = 0.5,
        target_kl: float = 0.01,
    ) -> Optional[Dict[str, float]]:
    """
    Modern PPO update for stage 1 with advanced techniques

    Args:
        policy: Actor-critic network
        optimizer: Optimizer (can be list of optimizers)
        batch_size: Mini-batch size
        memory: Tuple of (obs, goals, speeds, actions, logprobs, targets, values, rewards, advantages)
        epoch: Number of epochs to train
        coeff_entropy: Entropy coefficient
        clip_value: PPO clip epsilon
        num_step: Number of steps in rollout
        num_env: Number of parallel environments
        frames: Number of observation frames
        obs_size: Observation size
        act_size: Action size
        value_loss_coeff: Value loss coefficient
        value_clip: Enable value clipping (from PPO paper)
        use_huber_loss: Use Huber loss instead of MSE for value function
        huber_delta: Delta parameter for Huber loss
        max_grad_norm: Maximum gradient norm for clipping
        target_kl: Target KL divergence for early stopping
    """
    # Unpack rollout
    obss, goals, speeds, actions, logprobs, targets, values, rewards, gaes = memory

    # Flatten (num_env × num_step) → (N, …)
    N = num_step * num_env
    device = torch.device("cuda")

    obss = torch.as_tensor(obss.reshape(N, frames, obs_size), device=device, dtype=torch.float32)
    goals = torch.as_tensor(goals.reshape(N, 2), device=device, dtype=torch.float32)
    speeds = torch.as_tensor(speeds.reshape(N, 2), device=device, dtype=torch.float32)
    actions = torch.as_tensor(actions.reshape(N, act_size), device=device, dtype=torch.float32)
    logp_old = torch.as_tensor(logprobs.reshape(N, 1), device=device, dtype=torch.float32)
    targets = torch.as_tensor(targets.reshape(N, 1), device=device, dtype=torch.float32)
    gaes = torch.as_tensor(gaes.reshape(N, 1), device=device, dtype=torch.float32)
    values_old = torch.as_tensor(values.reshape(N, 1), device=device, dtype=torch.float32)

    # Advantage normalization
    gaes = (gaes - gaes.mean()) / (gaes.std() + 1e-8)

    # DataLoader for mini-batching
    dataset = TensorDataset(obss, goals, speeds, actions, logp_old, targets, gaes, values_old)
    loader = DataLoader(dataset,
                        batch_size=batch_size,
                        sampler=RandomSampler(dataset),
                        drop_last=False)

    # Separate optimizers
    if isinstance(optimizer, list):
        critic_opt = optimizer[0]
        actor_opt = optimizer[1]
    else:
        # If unified optimizer, create separate ones
        critic_opt = Adam(policy.enc_critic_params(), lr=3e-4)
        actor_opt = Adam(policy.actor_params(), lr=3e-4)

    # Training metrics
    epoch_p_loss = 0.0
    epoch_v_loss = 0.0
    epoch_ent = 0.0
    total_pi_iters = 0
    total_v_iters = 0
    clip_frac = 0.0
    kl_div = 0.0

    # Policy training with KL early stopping
    for ep in range(epoch):
        for obs_b, goal_b, speed_b, act_b, logp_old_b, tgt_b, gae_b, values_old_b in loader:
            actor_opt.zero_grad()

            value_b, logp_b, entropy_b = policy.evaluate_actions(obs_b, goal_b, speed_b, act_b)

            # Policy loss calculation
            ratio = torch.exp(logp_b - logp_old_b)
            surr1 = ratio * gae_b
            surr2 = torch.clamp(ratio, 1 - clip_value, 1 + clip_value) * gae_b
            policy_loss = -torch.min(surr1, surr2).mean()

            clipped = (ratio - 1.0).abs() > clip_value
            clip_frac = clipped.float().mean().item()

            # Add entropy regularization to policy loss
            policy_loss = policy_loss - coeff_entropy * entropy_b.mean()

            # Calculate KL divergence for monitoring
            kl_div = (logp_old_b - logp_b).mean().item()

            policy_loss.backward()
            # Gradient clipping for actor
            torch.nn.utils.clip_grad_norm_(policy.actor_params(), max_grad_norm)

            actor_opt.step()
            epoch_p_loss += policy_loss.item()
            epoch_ent += entropy_b.mean().item()
            total_pi_iters += 1

            # Early stop actor updates when KL exceeds target
            if kl_div > target_kl:
                logger_ppo.info(f"Early stopping policy updates at epoch {ep} due to KL={kl_div:.4f} > {target_kl}")
                break

        # Break outer loop too if KL exceeded
        if kl_div > target_kl:
            break

    # Value function training
    last_v_pred = None
    for ep in range(epoch):
        for obs_b, goal_b, speed_b, act_b, logp_old_b, tgt_b, gae_b, values_old_b in loader:
            critic_opt.zero_grad()
            value_b, logp_b, entropy_b = policy.evaluate_actions(obs_b, goal_b, speed_b, act_b)

            # Calculate value loss with optional clipping and Huber loss
            if use_huber_loss:
                # Huber loss (smooth L1 loss)
                if value_clip:
                    # Clipped value prediction
                    value_clipped = values_old_b + torch.clamp(
                        value_b - values_old_b,
                        -clip_value, clip_value
                    )
                    # Huber loss for both clipped and unclipped
                    value_loss_unclipped = F.smooth_l1_loss(value_b, tgt_b, beta=huber_delta, reduction='none')
                    value_loss_clipped = F.smooth_l1_loss(value_clipped, tgt_b, beta=huber_delta, reduction='none')
                    # Take maximum (more conservative)
                    value_loss = torch.max(value_loss_unclipped, value_loss_clipped).mean()
                else:
                    # Simple Huber loss
                    value_loss = F.smooth_l1_loss(value_b, tgt_b, beta=huber_delta)
            else:
                # MSE loss (default)
                if value_clip:
                    # Value clipping from PPO paper
                    value_clipped = values_old_b + torch.clamp(
                        value_b - values_old_b,
                        -clip_value, clip_value
                    )
                    # Take maximum of clipped and unclipped loss
                    value_loss_unclipped = F.mse_loss(value_b, tgt_b, reduction='none')
                    value_loss_clipped = F.mse_loss(value_clipped, tgt_b, reduction='none')
                    value_loss = torch.max(value_loss_unclipped, value_loss_clipped).mean()
                else:
                    # Simple MSE loss
                    value_loss = F.mse_loss(value_b, tgt_b)

            # Scale critic update
            (value_loss * value_loss_coeff).backward()
            # Gradient clipping for critic
            torch.nn.utils.clip_grad_norm_(policy.enc_critic_params(), max_norm=1.0)

            critic_opt.step()
            epoch_v_loss += value_loss.item()
            total_v_iters += 1
            last_v_pred = value_b.detach()

    # Extra critic pass for better value fitting
    for obs_b, goal_b, speed_b, act_b, logp_old_b, tgt_b, gae_b, values_old_b in loader:
        critic_opt.zero_grad()
        value_b, _, _ = policy.evaluate_actions(obs_b, goal_b, speed_b, act_b)

        # Use same loss calculation as main training
        if use_huber_loss:
            value_loss = F.smooth_l1_loss(value_b, tgt_b, beta=huber_delta)
        else:
            value_loss = F.mse_loss(value_b, tgt_b)

        (value_loss * value_loss_coeff).backward()
        torch.nn.utils.clip_grad_norm_(policy.enc_critic_params(), max_grad_norm * 2.0)

        critic_opt.step()
        epoch_v_loss += value_loss.item()
        total_v_iters += 1
        last_v_pred = value_b.detach()

    # Calculate average losses
    n_pi_batches = max(total_pi_iters, 1)
    n_v_batches = max(total_v_iters, 1)

    # Explained variance (diagnostic)
    ev = 0.0
    try:
        if last_v_pred is not None:
            ev = explained_variance(last_v_pred, tgt_b.detach())
    except Exception:
        ev = 0.0

    logger_ppo.info(
        f"[STAGE1] Policy: {total_pi_iters} iters | "
        f"P_loss={epoch_p_loss / n_pi_batches:.4f} | "
        f"Value: {total_v_iters} iters | "
        f"V_loss={epoch_v_loss / n_v_batches:.4f} | "
        f"Entropy={epoch_ent / n_pi_batches:.4f} | "
        f"EV={ev:.3f} | KL={kl_div:.4f} | clip_frac={clip_frac:.3f}"
    )

    return {
        'policy_loss': epoch_p_loss / n_pi_batches,
        'value_loss': epoch_v_loss / n_v_batches,
        'entropy': epoch_ent / n_pi_batches,
        'clip_fraction': clip_frac,
        'kl_divergence': kl_div,
        'policy_iterations': total_pi_iters,
        'value_iterations': total_v_iters,
        'explained_variance': ev
    }


def ppo_update_stage2_modern(
        policy,
        optimizer,
        batch_size,
        memory,
        filter_index,
        epoch,
        coeff_entropy=0.02,
        clip_value=0.2,
        num_step=2048,
        num_env=12,
        frames=1,
        obs_size=24,
        act_size=4,
        target_kl=0.02,
        value_loss_coeff=2.0,
        value_clip=True,
        use_huber_loss=False,
        huber_delta=10.0,
        max_grad_norm=0.5
    ):
    """
    Modern PPO update for stage 2 with advanced techniques
    """
    obss, goals, speeds, actions, logprobs, targets, values, rewards, advs = memory

    # Normalize advantages
    advs = (advs - advs.mean()) / (advs.std() + 1e-8)

    # Reshape tensors
    obss = obss.reshape((num_step * num_env, frames, obs_size))
    goals = goals.reshape((num_step * num_env, 2))
    speeds = speeds.reshape((num_step * num_env, 2))
    actions = actions.reshape(num_step * num_env, act_size)
    logprobs = logprobs.reshape(num_step * num_env, 1)
    advs = advs.reshape(num_step * num_env, 1)
    targets = targets.reshape(num_step * num_env, 1)
    values = values.reshape(num_step * num_env, 1)

    # Apply filter for stage2 (remove invalid transitions)
    obss = np.delete(obss, filter_index, 0)
    goals = np.delete(goals, filter_index, 0)
    speeds = np.delete(speeds, filter_index, 0)
    actions = np.delete(actions, filter_index, 0)
    logprobs = np.delete(logprobs, filter_index, 0)
    advs = np.delete(advs, filter_index, 0)
    targets = np.delete(targets, filter_index, 0)
    values = np.delete(values, filter_index, 0)

    # Convert to tensors
    obs_tensor = torch.FloatTensor(obss).cuda()
    goal_tensor = torch.FloatTensor(goals).cuda()
    speed_tensor = torch.FloatTensor(speeds).cuda()
    act_tensor = torch.FloatTensor(actions).cuda()
    logp_old_tensor = torch.FloatTensor(logprobs).cuda()
    adv_tensor = torch.FloatTensor(advs).cuda()
    tgt_tensor = torch.FloatTensor(targets).cuda()
    values_old_tensor = torch.FloatTensor(values).cuda()

    # Create data loader
    dataset = torch.utils.data.TensorDataset(
        obs_tensor, goal_tensor, speed_tensor, act_tensor,
        logp_old_tensor, tgt_tensor, adv_tensor, values_old_tensor
    )
    loader = torch.utils.data.DataLoader(dataset, batch_size=batch_size, shuffle=True, drop_last=True)

    # Separate optimizers
    if isinstance(optimizer, list) and len(optimizer) == 2:
        critic_opt, actor_opt = optimizer
    else:
        # Fallback to single optimizer
        critic_opt = actor_opt = optimizer

    # Training metrics
    epoch_p_loss = 0.0
    epoch_v_loss = 0.0
    epoch_ent = 0.0
    total_pi_iters = 0
    total_v_iters = 0
    clip_frac = 0.0
    kl_div = 0.0

    # Policy training with KL early stopping
    for ep in range(epoch):
        for obs_b, goal_b, speed_b, act_b, logp_old_b, tgt_b, gae_b, values_old_b in loader:
            actor_opt.zero_grad()

            value_b, logp_b, entropy_b = policy.evaluate_actions(obs_b, goal_b, speed_b, act_b)

            # Policy loss calculation
            ratio = torch.exp(logp_b - logp_old_b)
            surr1 = ratio * gae_b
            surr2 = torch.clamp(ratio, 1 - clip_value, 1 + clip_value) * gae_b
            policy_loss = -torch.min(surr1, surr2).mean()

            clipped = (ratio - 1.0).abs() > clip_value
            clip_frac = clipped.float().mean().item()

            # Add entropy regularization to policy loss
            policy_loss = policy_loss - coeff_entropy * entropy_b.mean()

            # Calculate KL divergence for monitoring
            kl_div = (logp_old_b - logp_b).mean().item()

            policy_loss.backward()
            torch.nn.utils.clip_grad_norm_(policy.actor_params(), max_grad_norm)

            actor_opt.step()
            epoch_p_loss += policy_loss.item()
            epoch_ent += entropy_b.mean().item()
            total_pi_iters += 1

            # Early stop actor updates when KL exceeds target
            if kl_div > target_kl:
                logger_ppo.info(f"Early stopping policy updates at epoch {ep} due to KL={kl_div:.4f} > {target_kl}")
                break

        if kl_div > target_kl:
            break

    # Value function training
    last_v_pred = None
    for ep in range(epoch):
        for obs_b, goal_b, speed_b, act_b, logp_old_b, tgt_b, gae_b, values_old_b in loader:
            critic_opt.zero_grad()
            value_b, logp_b, entropy_b = policy.evaluate_actions(obs_b, goal_b, speed_b, act_b)

            # Calculate value loss with optional clipping and Huber loss
            if use_huber_loss:
                if value_clip:
                    value_clipped = values_old_b + torch.clamp(
                        value_b - values_old_b,
                        -clip_value, clip_value
                    )
                    value_loss_unclipped = F.smooth_l1_loss(value_b, tgt_b, beta=huber_delta, reduction='none')
                    value_loss_clipped = F.smooth_l1_loss(value_clipped, tgt_b, beta=huber_delta, reduction='none')
                    value_loss = torch.max(value_loss_unclipped, value_loss_clipped).mean()
                else:
                    value_loss = F.smooth_l1_loss(value_b, tgt_b, beta=huber_delta)
            else:
                if value_clip:
                    value_clipped = values_old_b + torch.clamp(
                        value_b - values_old_b,
                        -clip_value, clip_value
                    )
                    value_loss_unclipped = F.mse_loss(value_b, tgt_b, reduction='none')
                    value_loss_clipped = F.mse_loss(value_clipped, tgt_b, reduction='none')
                    value_loss = torch.max(value_loss_unclipped, value_loss_clipped).mean()
                else:
                    value_loss = F.mse_loss(value_b, tgt_b)

            (value_loss * value_loss_coeff).backward()
            torch.nn.utils.clip_grad_norm_(policy.enc_critic_params(), max_norm=1.0)

            critic_opt.step()
            epoch_v_loss += value_loss.item()
            total_v_iters += 1
            last_v_pred = value_b.detach()

    # Extra critic pass
    for obs_b, goal_b, speed_b, act_b, logp_old_b, tgt_b, gae_b, values_old_b in loader:
        critic_opt.zero_grad()
        value_b, _, _ = policy.evaluate_actions(obs_b, goal_b, speed_b, act_b)

        if use_huber_loss:
            value_loss = F.smooth_l1_loss(value_b, tgt_b, beta=huber_delta)
        else:
            value_loss = F.mse_loss(value_b, tgt_b)

        (value_loss * value_loss_coeff).backward()
        torch.nn.utils.clip_grad_norm_(policy.enc_critic_params(), max_grad_norm * 2.0)

        critic_opt.step()
        epoch_v_loss += value_loss.item()
        total_v_iters += 1
        last_v_pred = value_b.detach()

    # Calculate averages
    avg_p_loss = epoch_p_loss / max(total_pi_iters, 1)
    avg_v_loss = epoch_v_loss / max(total_v_iters, 1)
    avg_entropy = epoch_ent / max(total_pi_iters, 1)

    # Explained variance
    ev = 0.0
    try:
        if last_v_pred is not None:
            ev = explained_variance(last_v_pred, tgt_b.detach())
    except Exception:
        ev = 0.0

    logger_ppo.info(
        f"[STAGE2] Filter {len(filter_index)} transitions | "
        f"P_loss={avg_p_loss:.4f} | V_loss={avg_v_loss:.4f} | "
        f"Entropy={avg_entropy:.4f} | EV={ev:.3f} | KL={kl_div:.4f} | clip_frac={clip_frac:.3f}"
    )

    return {
        'policy_loss': avg_p_loss,
        'value_loss': avg_v_loss,
        'entropy': avg_entropy,
        'clip_fraction': clip_frac,
        'kl_divergence': kl_div,
        'explained_variance': ev
    }
