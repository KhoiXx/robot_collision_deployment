"""
Normalization utilities for PPO training
Including observation normalization and reward normalization
Based on OpenAI Baselines and Stable-Baselines3
"""
import numpy as np
import torch


class RunningMeanStd:
    """
    Tracks the mean, std and count of values using Welford's algorithm.
    https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Parallel_algorithm
    """
    def __init__(self, epsilon=1e-4, shape=()):
        """
        Args:
            epsilon: Small value to avoid division by zero
            shape: Shape of the data to normalize
        """
        self.mean = np.zeros(shape, dtype=np.float64)
        self.var = np.ones(shape, dtype=np.float64)
        self.count = epsilon

    def update(self, x):
        """Update statistics with a batch of observations"""
        batch_mean = np.mean(x, axis=0)
        batch_var = np.var(x, axis=0)
        batch_count = x.shape[0]
        self.update_from_moments(batch_mean, batch_var, batch_count)

    def update_from_moments(self, batch_mean, batch_var, batch_count):
        """Update from batch mean and variance"""
        delta = batch_mean - self.mean
        tot_count = self.count + batch_count

        new_mean = self.mean + delta * batch_count / tot_count
        m_a = self.var * self.count
        m_b = batch_var * batch_count
        M2 = m_a + m_b + np.square(delta) * self.count * batch_count / tot_count
        new_var = M2 / tot_count
        new_count = tot_count

        self.mean = new_mean
        self.var = new_var
        self.count = new_count


class ObservationNormalizer:
    """
    Normalize observations using running mean and standard deviation.
    """
    def __init__(self, shape, epsilon=1e-8, clip_range=10.0):
        """
        Args:
            shape: Shape of observation
            epsilon: Small value to avoid division by zero
            clip_range: Clip normalized observations to [-clip_range, clip_range]
        """
        self.rms = RunningMeanStd(shape=shape)
        self.epsilon = epsilon
        self.clip_range = clip_range
        self.training = True

    def normalize(self, obs):
        """
        Normalize observation

        Args:
            obs: Observation array (can be batched)
        Returns:
            Normalized observation
        """
        if self.training:
            self.rms.update(obs)

        normalized = (obs - self.rms.mean) / np.sqrt(self.rms.var + self.epsilon)

        if self.clip_range is not None:
            normalized = np.clip(normalized, -self.clip_range, self.clip_range)

        return normalized

    def denormalize(self, normalized_obs):
        """Denormalize observation back to original scale"""
        return normalized_obs * np.sqrt(self.rms.var + self.epsilon) + self.rms.mean

    def set_training(self, mode=True):
        """Set training mode (update statistics or not)"""
        self.training = mode


class RewardNormalizer:
    """
    Normalize rewards using running return statistics.
    This normalizes based on the discounted return rather than immediate reward.
    """
    def __init__(self, gamma=0.99, epsilon=1e-8, clip_range=10.0):
        """
        Args:
            gamma: Discount factor
            epsilon: Small value to avoid division by zero
            clip_range: Clip normalized rewards to [-clip_range, clip_range]
        """
        self.rms = RunningMeanStd(shape=())
        self.gamma = gamma
        self.epsilon = epsilon
        self.clip_range = clip_range
        self.returns = None
        self.training = True

    def normalize(self, reward, done=False):
        """
        Normalize reward using running return statistics

        Args:
            reward: Single reward value or array
            done: Whether episode is done (resets return)
        Returns:
            Normalized reward
        """
        # Update return
        if self.returns is None:
            self.returns = 0.0

        self.returns = reward + self.gamma * self.returns * (1 - done)

        if self.training:
            self.rms.update(np.array([self.returns]))

        # Normalize reward using return statistics
        normalized = reward / np.sqrt(self.rms.var + self.epsilon)

        if self.clip_range is not None:
            normalized = np.clip(normalized, -self.clip_range, self.clip_range)

        # Reset return on episode end
        if done:
            self.returns = 0.0

        return normalized

    def normalize_batch(self, rewards, dones):
        """
        Normalize a batch of rewards

        Args:
            rewards: Array of rewards [batch_size] or [timesteps, batch_size]
            dones: Array of done flags
        Returns:
            Normalized rewards
        """
        normalized_rewards = np.zeros_like(rewards)

        # Handle both 1D and 2D arrays
        if len(rewards.shape) == 1:
            for i in range(len(rewards)):
                normalized_rewards[i] = self.normalize(rewards[i], dones[i])
        else:
            # For 2D arrays [timesteps, batch_size]
            for env_idx in range(rewards.shape[1]):
                for t in range(rewards.shape[0]):
                    normalized_rewards[t, env_idx] = self.normalize(
                        rewards[t, env_idx],
                        dones[t, env_idx]
                    )

        return normalized_rewards

    def set_training(self, mode=True):
        """Set training mode (update statistics or not)"""
        self.training = mode


class LearningRateScheduler:
    """
    Learning rate scheduler for PPO training.
    Supports linear decay, cosine annealing, and step decay.
    """
    def __init__(self, optimizer, initial_lr, schedule_type='linear',
                 total_steps=None, min_lr=1e-6, decay_rate=0.1, decay_steps=None):
        """
        Args:
            optimizer: PyTorch optimizer or list of optimizers
            initial_lr: Initial learning rate or list of initial learning rates
            schedule_type: 'linear', 'cosine', or 'step'
            total_steps: Total training steps (required for linear and cosine)
            min_lr: Minimum learning rate
            decay_rate: Decay rate for step decay
            decay_steps: Steps between decays for step decay
        """
        if isinstance(optimizer, list):
            self.optimizers = optimizer
            self.initial_lrs = initial_lr if isinstance(initial_lr, list) else [initial_lr] * len(optimizer)
        else:
            self.optimizers = [optimizer]
            self.initial_lrs = [initial_lr]

        self.schedule_type = schedule_type
        self.total_steps = total_steps
        self.min_lr = min_lr
        self.decay_rate = decay_rate
        self.decay_steps = decay_steps
        self.current_step = 0

    def step(self):
        """Update learning rate"""
        self.current_step += 1

        for opt_idx, optimizer in enumerate(self.optimizers):
            initial_lr = self.initial_lrs[opt_idx]

            if self.schedule_type == 'linear':
                # Linear decay from initial_lr to min_lr
                progress = min(1.0, self.current_step / self.total_steps)
                new_lr = initial_lr - (initial_lr - self.min_lr) * progress

            elif self.schedule_type == 'cosine':
                # Cosine annealing
                progress = min(1.0, self.current_step / self.total_steps)
                new_lr = self.min_lr + (initial_lr - self.min_lr) * \
                         0.5 * (1 + np.cos(np.pi * progress))

            elif self.schedule_type == 'step':
                # Step decay
                num_decays = self.current_step // self.decay_steps
                new_lr = max(self.min_lr, initial_lr * (self.decay_rate ** num_decays))

            else:
                new_lr = initial_lr

            # Update optimizer learning rate
            for param_group in optimizer.param_groups:
                param_group['lr'] = new_lr

    def get_lr(self):
        """Get current learning rates"""
        return [param_group['lr'] for optimizer in self.optimizers
                for param_group in optimizer.param_groups]
