"""
Experience replay buffer for online learning
Collects (s, a, r, s', done) transitions and computes GAE for PPO updates
"""
import numpy as np
from typing import Dict, Tuple, Optional


class ExperienceBuffer:
    """
    Circular buffer for storing robot experiences
    Supports GAE computation for PPO training
    """

    def __init__(self, capacity: int = 2048, num_envs: int = 1, obs_dim: int = 454,
                 gamma: float = 0.99, lam: float = 0.95):
        """
        Args:
            capacity: Maximum number of transitions to store
            num_envs: Number of parallel environments (1 for single robot)
            obs_dim: Observation dimension (454 for upsampled laser)
            gamma: Discount factor for rewards
            lam: Lambda for GAE computation
        """
        self.capacity = capacity
        self.num_envs = num_envs
        self.obs_dim = obs_dim
        self.gamma = gamma
        self.lam = lam

        # Initialize storage arrays
        self.reset()

    def reset(self):
        """Clear all stored experiences"""
        self.observations = []      # List of [3, 454] laser stacks
        self.goals = []              # List of [2] goal positions
        self.speeds = []             # List of [2] velocities
        self.actions = []            # List of [2] actions
        self.rewards = []            # List of scalars
        self.dones = []              # List of bools
        self.logprobs = []           # List of scalars (log probabilities)
        self.values = []             # List of scalars (value estimates)
        self.ptr = 0                 # Current position in buffer

    def add(self, obs: np.ndarray, goal: np.ndarray, speed: np.ndarray,
            action: np.ndarray, reward: float, done: bool,
            logprob: float, value: float):
        """
        Add a single transition to the buffer

        Args:
            obs: Laser observation stack [3, 454]
            goal: Local goal [2]
            speed: Current velocity [2]
            action: Executed action [2]
            reward: Received reward (scalar)
            done: Episode termination flag
            logprob: Log probability of action
            value: Value estimate from critic
        """
        if len(self.observations) < self.capacity:
            # Buffer not full yet, append
            self.observations.append(obs)
            self.goals.append(goal)
            self.speeds.append(speed)
            self.actions.append(action)
            self.rewards.append(reward)
            self.dones.append(done)
            self.logprobs.append(logprob)
            self.values.append(value)
        else:
            # Buffer full, overwrite oldest experience (circular)
            idx = self.ptr % self.capacity
            self.observations[idx] = obs
            self.goals[idx] = goal
            self.speeds[idx] = speed
            self.actions[idx] = action
            self.rewards[idx] = reward
            self.dones[idx] = done
            self.logprobs[idx] = logprob
            self.values[idx] = value

        self.ptr += 1

    def __len__(self):
        """Return current size of buffer"""
        return min(len(self.observations), self.capacity)

    def is_ready(self, min_samples: int = 128) -> bool:
        """Check if buffer has enough samples for training"""
        return len(self) >= min_samples

    def get_all(self) -> Dict[str, np.ndarray]:
        """
        Get all stored experiences as numpy arrays

        Returns:
            Dictionary with keys: observations, goals, speeds, actions,
            rewards, dones, logprobs, values
        """
        return {
            'observations': np.array(self.observations),  # [N, 3, 454]
            'goals': np.array(self.goals),                 # [N, 2]
            'speeds': np.array(self.speeds),               # [N, 2]
            'actions': np.array(self.actions),             # [N, 2]
            'rewards': np.array(self.rewards),             # [N]
            'dones': np.array(self.dones),                 # [N]
            'logprobs': np.array(self.logprobs),           # [N]
            'values': np.array(self.values)                # [N]
        }

    def compute_gae(self, last_value: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute Generalized Advantage Estimation (GAE)

        Args:
            last_value: Value estimate of the last state (for bootstrapping)

        Returns:
            advantages: GAE advantages [N]
            returns: TD(lambda) returns [N]
        """
        N = len(self)
        if N == 0:
            return np.array([]), np.array([])

        rewards = np.array(self.rewards)
        values = np.array(self.values)
        dones = np.array(self.dones)

        # Initialize arrays
        advantages = np.zeros(N, dtype=np.float32)
        returns = np.zeros(N, dtype=np.float32)

        # Bootstrap from last value
        next_value = last_value
        gae = 0.0

        # Compute GAE backwards through trajectory
        for t in reversed(range(N)):
            if t == N - 1:
                # Last step
                next_non_terminal = 1.0 - float(dones[t])
                next_value = last_value
            else:
                next_non_terminal = 1.0 - float(dones[t])
                next_value = values[t + 1]

            # TD error: δ_t = r_t + γ * V(s_{t+1}) - V(s_t)
            delta = rewards[t] + self.gamma * next_value * next_non_terminal - values[t]

            # GAE: A_t = δ_t + γλ * A_{t+1}
            gae = delta + self.gamma * self.lam * next_non_terminal * gae
            advantages[t] = gae

            # Returns: R_t = A_t + V(s_t)
            returns[t] = gae + values[t]

        return advantages, returns

    def get_training_batch(self, last_value: float = 0.0) -> Tuple[Dict[str, np.ndarray], np.ndarray, np.ndarray]:
        """
        Prepare a complete training batch with GAE

        Args:
            last_value: Value estimate for bootstrapping

        Returns:
            batch: Dictionary of experiences
            advantages: GAE advantages
            returns: TD(lambda) returns
        """
        batch = self.get_all()
        advantages, returns = self.compute_gae(last_value)

        # Normalize advantages (important for PPO stability)
        if len(advantages) > 0:
            advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        return batch, advantages, returns

    def get_last_obs(self) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """
        Get the last stored observation for bootstrapping

        Returns:
            (obs, goal, speed) or None if buffer is empty
        """
        if len(self) == 0:
            return None
        return self.observations[-1], self.goals[-1], self.speeds[-1]


class RollingBuffer:
    """
    Alternative buffer that maintains a fixed-size sliding window
    Useful for continuous online learning without clearing buffer
    """

    def __init__(self, window_size: int = 128):
        self.window_size = window_size
        self.buffer = []

    def add(self, transition: Dict):
        """Add transition and maintain window size"""
        self.buffer.append(transition)
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)  # Remove oldest

    def sample(self, batch_size: int) -> Dict[str, np.ndarray]:
        """Sample random batch from buffer"""
        if len(self.buffer) < batch_size:
            batch_size = len(self.buffer)

        indices = np.random.choice(len(self.buffer), batch_size, replace=False)

        # Collect samples
        batch = {
            'observations': [],
            'goals': [],
            'speeds': [],
            'actions': [],
            'rewards': [],
            'dones': [],
            'logprobs': [],
            'values': []
        }

        for idx in indices:
            trans = self.buffer[idx]
            for key in batch.keys():
                batch[key].append(trans[key])

        # Convert to numpy
        for key in batch.keys():
            batch[key] = np.array(batch[key])

        return batch

    def __len__(self):
        return len(self.buffer)
