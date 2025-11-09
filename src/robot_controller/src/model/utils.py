from custom_msgs.msg import RobotState
from collections import deque

import numpy as np
import bisect
import torch
from torch.autograd import Variable
from functools import reduce
from serial import Serial

def log_normal_density(x, mean, log_std, std):
    """returns guassian density given x on log scale"""

    variance = std.pow(2)
    log_density = -(x - mean).pow(2) / (2 * variance) - 0.5 *\
        np.log(2 * np.pi) - log_std    # num_env * frames * act_size
    log_density = log_density.sum(dim=-1, keepdim=True) # num_env * frames * 1
    return log_density


def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def convert_statemsg2state(state_msg: RobotState):
    obs_stack = deque([
        np.array(state_msg.obs_prev),
        np.array(state_msg.obs_cur),
        np.array(state_msg.obs_next)
    ])
    goal = np.array(state_msg.goal)
    speed = np.array(state_msg.speed)
    return [obs_stack, goal, speed]

