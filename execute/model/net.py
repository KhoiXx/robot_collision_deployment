import math
import numpy as np
import torch
import torch.nn as nn
from torch.nn import init
from torch.nn import functional as F

from model.utils import log_normal_density

class Flatten(nn.Module):
    def forward(self, input):

        return input.view(input.shape[0], 1,  -1)


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
