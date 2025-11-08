import math
import numpy as np
import torch
from torch.distributions import Normal
import torch.nn as nn
from torch.nn import init
from torch.nn import functional as F

from model.utils import log_normal_density

NUM_OBS = 454

def orthogonal_init(layer, gain=np.sqrt(2)):
    """Orthogonal initialization from OpenAI Baselines"""
    if isinstance(layer, (nn.Linear, nn.Conv1d)):
        nn.init.orthogonal_(layer.weight, gain=gain)
        if layer.bias is not None:
            nn.init.constant_(layer.bias, 0)
    return layer

class ActorCriticNetwork(nn.Module):
    def __init__(self, frames, action_space, hidden_size=128, use_layer_norm=False):
        super(ActorCriticNetwork, self).__init__()
        # Improved initialization for better exploration
        self.logstd = nn.Parameter(torch.full((action_space,), -0.3))
        self.frames = frames
        self.action_space = action_space
        self.use_layer_norm = use_layer_norm

        # Calculate conv output size
        def conv1d_out_len(L, kernel, stride, padding, dilation=1):
            return math.floor((L + 2*padding - dilation*(kernel-1) - 1) / stride) + 1

        L1 = conv1d_out_len(NUM_OBS, 5, 2, 1)  # padding=1
        L2 = conv1d_out_len(L1, 3, 2, 1)      # padding=1
        conv_output_size = L2 * 32  # 32 channels

        # Actor CNN layers with orthogonal initialization
        self.act_fea_cv1 = orthogonal_init(nn.Conv1d(in_channels=frames, out_channels=32, kernel_size=5, stride=2, padding=1))
        self.act_fea_cv2 = orthogonal_init(nn.Conv1d(in_channels=32, out_channels=32, kernel_size=3, stride=2, padding=1))
        self.act_fc1 = orthogonal_init(nn.Linear(conv_output_size, 256))
        self.act_fc2 = orthogonal_init(nn.Linear(256+2+2, 128))  # +goal(2) +speed(2)

        # Layer normalization for actor (optional)
        if use_layer_norm:
            self.act_ln1 = nn.LayerNorm(256)
            self.act_ln2 = nn.LayerNorm(128)

        # Separate actor heads for linear and angular velocity
        self.actor_linear = nn.Linear(128, 1)   # Linear velocity [0,1] with sigmoid
        self.actor_angular = nn.Linear(128, 1)  # Angular velocity [-1,1] with tanh

        # Initialize output layers with smaller weights for stable learning (gain=0.01)
        orthogonal_init(self.actor_linear, gain=0.01)
        orthogonal_init(self.actor_angular, gain=0.01)

        # Critic CNN layers (separate from actor for better learning)
        self.crt_fea_cv1 = orthogonal_init(nn.Conv1d(in_channels=frames, out_channels=32, kernel_size=5, stride=2, padding=1))
        self.crt_fea_cv2 = orthogonal_init(nn.Conv1d(in_channels=32, out_channels=32, kernel_size=3, stride=2, padding=1))
        self.crt_fc1 = orthogonal_init(nn.Linear(conv_output_size, 256))
        self.crt_fc2 = orthogonal_init(nn.Linear(256+2+2, 128))
        self.critic = nn.Linear(128, 1)

        # Layer normalization for critic (optional)
        if use_layer_norm:
            self.crt_ln1 = nn.LayerNorm(256)
            self.crt_ln2 = nn.LayerNorm(128)

        # Initialize critic output with gain=1.0
        orthogonal_init(self.critic, gain=1.0)

    def forward(self, obs, goal, speed):
        """
        Returns value estimation, action, log_action_prob, mean
        """
        # Actor forward pass
        a = F.relu(self.act_fea_cv1(obs))
        a = F.relu(self.act_fea_cv2(a))
        a = a.view(a.shape[0], -1)  # Flatten
        a = F.relu(self.act_fc1(a))
        if self.use_layer_norm:
            a = self.act_ln1(a)

        # Concatenate with goal and speed
        a = torch.cat((a, goal, speed), dim=-1)
        a = F.relu(self.act_fc2(a))
        if self.use_layer_norm:
            a = self.act_ln2(a)

        # Separate outputs for linear and angular velocity
        mean_linear = torch.sigmoid(self.actor_linear(a))    # [0,1] for linear velocity
        mean_angular = torch.tanh(self.actor_angular(a))     # [-1,1] for angular velocity
        mean = torch.cat((mean_linear, mean_angular), dim=-1)

        # Sample action with exploration noise
        logstd = self.logstd.expand_as(mean)
        std = torch.exp(logstd)
        action_dist = Normal(mean, std)
        action = action_dist.sample()
        logprob = action_dist.log_prob(action).sum(dim=-1, keepdim=True)

        # Critic forward pass
        v = F.relu(self.crt_fea_cv1(obs))
        v = F.relu(self.crt_fea_cv2(v))
        v = v.view(v.shape[0], -1)  # Flatten
        v = F.relu(self.crt_fc1(v))
        if self.use_layer_norm:
            v = self.crt_ln1(v)
        v = torch.cat((v, goal, speed), dim=-1)
        v = F.relu(self.crt_fc2(v))
        if self.use_layer_norm:
            v = self.crt_ln2(v)
        v = self.critic(v)

        return action, v, logprob, mean

    def policy(self, obs, goal, speed):
        """Get policy mean without sampling"""
        a = F.relu(self.act_fea_cv1(obs))
        a = F.relu(self.act_fea_cv2(a))
        a = a.view(a.shape[0], -1)
        a = F.relu(self.act_fc1(a))
        if self.use_layer_norm:
            a = self.act_ln1(a)
        a = torch.cat((a, goal, speed), dim=-1)
        a = F.relu(self.act_fc2(a))
        if self.use_layer_norm:
            a = self.act_ln2(a)

        mean_linear = torch.sigmoid(self.actor_linear(a))
        mean_angular = torch.tanh(self.actor_angular(a))
        mean = torch.cat((mean_linear, mean_angular), dim=-1)
        return mean

    def value(self, obs, goal, speed):
        """Get value estimation"""
        v = F.relu(self.crt_fea_cv1(obs))
        v = F.relu(self.crt_fea_cv2(v))
        v = v.view(v.shape[0], -1)
        v = F.relu(self.crt_fc1(v))
        if self.use_layer_norm:
            v = self.crt_ln1(v)
        v = torch.cat((v, goal, speed), dim=-1)
        v = F.relu(self.crt_fc2(v))
        if self.use_layer_norm:
            v = self.crt_ln2(v)
        v = self.critic(v)
        return v

    def evaluate_actions(self, obs, goal, speed, action):
        """Evaluate actions for PPO update"""
        # Get mean and value
        mean = self.policy(obs, goal, speed)
        v = self.value(obs, goal, speed)
        
        # Calculate log probability and entropy
        logstd = self.logstd.expand_as(mean)
        std = torch.exp(logstd)
        action_dist = Normal(mean, std)
        logprob = action_dist.log_prob(action).sum(dim=-1, keepdim=True)
        entropy = action_dist.entropy().sum(-1).mean()
        
        return v, logprob, entropy

    def shared_layers_params(self) -> list:
        """No shared layers in this architecture"""
        return []
    
    def enc_critic_params(self) -> list:
        """All critic parameters"""
        critic_params = list(self.crt_fea_cv1.parameters()) + \
                       list(self.crt_fea_cv2.parameters()) + \
                       list(self.crt_fc1.parameters()) + \
                       list(self.crt_fc2.parameters()) + \
                       list(self.critic.parameters())
        return critic_params
    
    def actor_params(self) -> list:
        """All actor parameters"""
        actor_params = list(self.act_fea_cv1.parameters()) + \
                      list(self.act_fea_cv2.parameters()) + \
                      list(self.act_fc1.parameters()) + \
                      list(self.act_fc2.parameters()) + \
                      list(self.actor_linear.parameters()) + \
                      list(self.actor_angular.parameters())
        return actor_params + [self.logstd]