import math
import numpy as np
import torch
from torch.distributions import Normal
import torch.nn as nn
from torch.nn import init
from torch.nn import functional as F

from model.utils import log_normal_density

NUM_OBS = 454

class Flatten(nn.Module):
    def forward(self, input):
        return input.view(input.shape[0], 1,  -1)


class ActorCriticNetwork(nn.Module):
    def __init__(self, frames, action_space, hidden_size=128):
        super(ActorCriticNetwork, self).__init__()
        # Improved initialization for better exploration
        self.logstd = nn.Parameter(torch.full((action_space,), -0.3))
        self.frames = frames
        self.action_space = action_space

        # Calculate conv output size
        def conv1d_out_len(L, kernel, stride, padding, dilation=1):
            return math.floor((L + 2*padding - dilation*(kernel-1) - 1) / stride) + 1

        L1 = conv1d_out_len(NUM_OBS, 5, 2, 1)  # padding=1
        L2 = conv1d_out_len(L1, 3, 2, 1)      # padding=1
        conv_output_size = L2 * 32  # 32 channels

        # Actor CNN layers
        self.act_fea_cv1 = nn.Conv1d(in_channels=frames, out_channels=32, kernel_size=5, stride=2, padding=1)
        self.act_fea_cv2 = nn.Conv1d(in_channels=32, out_channels=32, kernel_size=3, stride=2, padding=1)
        self.act_fc1 = nn.Linear(conv_output_size, 256)
        self.act_fc2 = nn.Linear(256+2+2, 128)  # +goal(2) +speed(2)

        # Separate actor heads for linear and angular velocity
        self.actor_linear = nn.Linear(128, 1)   # Linear velocity [0,1] with sigmoid
        self.actor_angular = nn.Linear(128, 1)  # Angular velocity [-1,1] with tanh

        # Initialize actor weights for stable learning
        with torch.no_grad():
            self.actor_linear.weight.mul_(0.1)
            self.actor_angular.weight.mul_(0.1)

        # Critic CNN layers (separate from actor for better learning)
        self.crt_fea_cv1 = nn.Conv1d(in_channels=frames, out_channels=32, kernel_size=5, stride=2, padding=1)
        self.crt_fea_cv2 = nn.Conv1d(in_channels=32, out_channels=32, kernel_size=3, stride=2, padding=1)
        self.crt_fc1 = nn.Linear(conv_output_size, 256)
        self.crt_fc2 = nn.Linear(256+2+2, 128)
        self.critic = nn.Linear(128, 1)

        # Initialize critic weights
        with torch.no_grad():
            self.critic.weight.mul_(0.1)

    def forward(self, obs, goal, speed):
        """
        Returns action, value, log_action_prob, mean (compatible with deployment)
        """
        # Actor forward pass
        a = F.relu(self.act_fea_cv1(obs))
        a = F.relu(self.act_fea_cv2(a))
        a = a.view(a.shape[0], -1)  # Flatten
        a = F.relu(self.act_fc1(a))

        # Concatenate with goal and speed
        a = torch.cat((a, goal, speed), dim=-1)
        a = F.relu(self.act_fc2(a))

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
        v = torch.cat((v, goal, speed), dim=-1)
        v = F.relu(self.crt_fc2(v))
        v = self.critic(v)

        return v, action, logprob, mean

    def policy(self, obs, goal, speed):
        """Get policy mean without sampling (for inference)"""
        a = F.relu(self.act_fea_cv1(obs))
        a = F.relu(self.act_fea_cv2(a))
        a = a.view(a.shape[0], -1)
        a = F.relu(self.act_fc1(a))
        a = torch.cat((a, goal, speed), dim=-1)
        a = F.relu(self.act_fc2(a))

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
        v = torch.cat((v, goal, speed), dim=-1)
        v = F.relu(self.crt_fc2(v))
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


# Keep CNNPolicy for backward compatibility
class CNNPolicy(nn.Module):
    def __init__(self, frames, action_space):
        super(CNNPolicy, self).__init__()
        self.logstd = nn.Parameter(torch.zeros(action_space))

        self.act_fea_cv1 = nn.Conv1d(in_channels=frames, out_channels=32, kernel_size=5, stride=2, padding=1)
        self.act_fea_cv2 = nn.Conv1d(in_channels=32, out_channels=32, kernel_size=3, stride=2, padding=1)
        self.lstm = nn.LSTM(input_size=32, hidden_size=32, num_layers=1, batch_first=True)

        self.act_fc1 = nn.Linear(56*32, 256)
        self.act_fc2 =  nn.Linear(256+2+2, 128)
        self.actor1 = nn.Linear(128, 1)
        self.actor2 = nn.Linear(128, 1)


        self.crt_fea_cv1 = nn.Conv1d(in_channels=frames, out_channels=32, kernel_size=5, stride=2, padding=1)
        self.crt_fea_cv2 = nn.Conv1d(in_channels=32, out_channels=32, kernel_size=3, stride=2, padding=1)
        self.lstm_critic = nn.LSTM(input_size=32, hidden_size=32, num_layers=1, batch_first=True)

        self.crt_fc1 = nn.Linear(56*32, 256)
        self.crt_fc2 = nn.Linear(256+2+2, 128)
        self.critic = nn.Linear(128, 1)

    def reset_hidden_state(self):
        self.hidden = None
        self.hidden_critic = None

    def forward(self, x, goal, speed):
        """
            returns value estimation, action, log_action_prob
        """
        # action
        a = F.relu(self.act_fea_cv1(x))
        a = F.relu(self.act_fea_cv2(a))
        a = a.permute(0, 2, 1)
        if self.hidden is None or self.hidden[0].size(1) != a.shape[0]:
            self.hidden = (torch.zeros(1, a.shape[0], self.lstm.hidden_size).to(x.device),
                                        torch.zeros(1, a.shape[0], self.lstm.hidden_size).to(x.device))
        else:
            self.hidden = (self.hidden[0].detach(), self.hidden[1].detach())
        a, self.hidden = self.lstm(a, self.hidden)

        a = a.contiguous().view(a.shape[0], -1)
        a = F.relu(self.act_fc1(a))

        a = torch.cat((a, goal, speed), dim=-1)
        a = F.relu(self.act_fc2(a))
        mean1 = F.sigmoid(self.actor1(a))
        mean2 = F.tanh(self.actor2(a))
        mean = torch.cat((mean1, mean2), dim=-1)

        logstd = self.logstd.expand_as(mean)
        std = torch.exp(logstd)
        action = torch.normal(mean, std)

        # action prob on log scale
        logprob = log_normal_density(action, mean, std=std, log_std=logstd)

        # value
        v = F.relu(self.crt_fea_cv1(x))
        v = F.relu(self.crt_fea_cv2(v))
        v = v.permute(0, 2, 1)

        if self.hidden_critic is None or self.hidden_critic[0].size(1) != v.shape[0]:
            self.hidden_critic = (torch.zeros(1, v.shape[0], self.lstm_critic.hidden_size).to(x.device),
                                        torch.zeros(1, v.shape[0], self.lstm_critic.hidden_size).to(x.device))
        else:
            self.hidden_critic = (self.hidden_critic[0].detach(), self.hidden_critic[1].detach())
        v, self.hidden_critic = self.lstm_critic(v, self.hidden_critic)

        v = v.contiguous().view(v.shape[0], -1)
        v = F.relu(self.crt_fc1(v))
        v = torch.cat((v, goal, speed), dim=-1)
        v = F.relu(self.crt_fc2(v))
        v = self.critic(v)


        return v, action, logprob, mean

    def evaluate_actions(self, x, goal, speed, action):
        v, _, _, mean = self.forward(x, goal, speed)
        logstd = self.logstd.expand_as(mean)
        std = torch.exp(logstd)
        # evaluate
        logprob = log_normal_density(action, mean, log_std=logstd, std=std)
        dist_entropy = 0.5 + 0.5 * math.log(2 * math.pi) + logstd
        dist_entropy = dist_entropy.sum(-1).mean()
        return v, logprob, dist_entropy
