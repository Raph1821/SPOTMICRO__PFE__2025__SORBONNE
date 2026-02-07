"""
Neural Networks for SAC and TD3 Algorithms
============================================

Actor and Critic networks with:
- Configurable hidden layers
- Layer normalization support
- Target network soft updates
- Action scaling for continuous control
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np


class Actor(nn.Module):
    """Actor network for policy (SAC and TD3)"""
    
    def __init__(self, state_dim, action_dim, hidden_dims=[256, 256], 
                 max_action=1.0, layer_norm=False):
        """
        Args:
            state_dim: Observation space dimension
            action_dim: Action space dimension
            hidden_dims: List of hidden layer sizes
            max_action: Maximum action value for scaling
            layer_norm: Use layer normalization
        """
        super(Actor, self).__init__()
        
        self.max_action = max_action
        
        # Build network layers
        layers = []
        prev_dim = state_dim
        
        for hidden_dim in hidden_dims:
            layers.append(nn.Linear(prev_dim, hidden_dim))
            if layer_norm:
                layers.append(nn.LayerNorm(hidden_dim))
            layers.append(nn.ReLU())
            prev_dim = hidden_dim
        
        self.feature_layers = nn.Sequential(*layers)
        self.output_layer = nn.Linear(prev_dim, action_dim)
        
    def forward(self, state):
        """Forward pass through network"""
        x = self.feature_layers(state)
        action = self.max_action * torch.tanh(self.output_layer(x))
        return action


class Critic(nn.Module):
    """Critic network for Q-value estimation (TD3)"""
    
    def __init__(self, state_dim, action_dim, hidden_dims=[256, 256], layer_norm=False):
        """
        Args:
            state_dim: Observation space dimension
            action_dim: Action space dimension
            hidden_dims: List of hidden layer sizes
            layer_norm: Use layer normalization
        """
        super(Critic, self).__init__()
        
        # Build Q1 network
        layers_q1 = []
        prev_dim = state_dim + action_dim
        
        for hidden_dim in hidden_dims:
            layers_q1.append(nn.Linear(prev_dim, hidden_dim))
            if layer_norm:
                layers_q1.append(nn.LayerNorm(hidden_dim))
            layers_q1.append(nn.ReLU())
            prev_dim = hidden_dim
        
        self.q1_layers = nn.Sequential(*layers_q1)
        self.q1_output = nn.Linear(prev_dim, 1)
        
        # Build Q2 network (twin critic)
        layers_q2 = []
        prev_dim = state_dim + action_dim
        
        for hidden_dim in hidden_dims:
            layers_q2.append(nn.Linear(prev_dim, hidden_dim))
            if layer_norm:
                layers_q2.append(nn.LayerNorm(hidden_dim))
            layers_q2.append(nn.ReLU())
            prev_dim = hidden_dim
        
        self.q2_layers = nn.Sequential(*layers_q2)
        self.q2_output = nn.Linear(prev_dim, 1)
        
    def forward(self, state, action):
        """Forward pass through both Q networks"""
        sa = torch.cat([state, action], dim=1)
        
        q1 = self.q1_layers(sa)
        q1 = self.q1_output(q1)
        
        q2 = self.q2_layers(sa)
        q2 = self.q2_output(q2)
        
        return q1, q2
    
    def Q1(self, state, action):
        """Get Q1 value only (for policy optimization)"""
        sa = torch.cat([state, action], dim=1)
        q1 = self.q1_layers(sa)
        q1 = self.q1_output(q1)
        return q1


class GaussianActor(nn.Module):
    """Stochastic actor with Gaussian policy (SAC)"""
    
    def __init__(self, state_dim, action_dim, hidden_dims=[256, 256], 
                 max_action=1.0, layer_norm=False, log_std_min=-20, log_std_max=2):
        """
        Args:
            state_dim: Observation space dimension
            action_dim: Action space dimension
            hidden_dims: List of hidden layer sizes
            max_action: Maximum action value for scaling
            layer_norm: Use layer normalization
            log_std_min: Minimum log std for numerical stability
            log_std_max: Maximum log std to prevent too much exploration
        """
        super(GaussianActor, self).__init__()
        
        self.max_action = max_action
        self.log_std_min = log_std_min
        self.log_std_max = log_std_max
        
        # Build shared feature layers
        layers = []
        prev_dim = state_dim
        
        for hidden_dim in hidden_dims:
            layers.append(nn.Linear(prev_dim, hidden_dim))
            if layer_norm:
                layers.append(nn.LayerNorm(hidden_dim))
            layers.append(nn.ReLU())
            prev_dim = hidden_dim
        
        self.feature_layers = nn.Sequential(*layers)
        
        # Separate heads for mean and log_std
        self.mean_layer = nn.Linear(prev_dim, action_dim)
        self.log_std_layer = nn.Linear(prev_dim, action_dim)
        
    def forward(self, state):
        """Get action distribution parameters"""
        x = self.feature_layers(state)
        mean = self.mean_layer(x)
        log_std = self.log_std_layer(x)
        log_std = torch.clamp(log_std, self.log_std_min, self.log_std_max)
        return mean, log_std
    
    def sample(self, state, deterministic=False):
        """Sample action from policy"""
        mean, log_std = self.forward(state)
        
        if deterministic:
            action = torch.tanh(mean)
        else:
            std = log_std.exp()
            normal = torch.distributions.Normal(mean, std)
            x_t = normal.rsample()  # Reparameterization trick
            action = torch.tanh(x_t)
            
            # Compute log probability with correction for tanh squashing
            log_prob = normal.log_prob(x_t)
            log_prob -= torch.log(1 - action.pow(2) + 1e-6)
            log_prob = log_prob.sum(1, keepdim=True)
        
        action = action * self.max_action
        
        if deterministic:
            return action, None
        else:
            return action, log_prob
    
    def get_log_prob(self, state, action):
        """Compute log probability of given action"""
        mean, log_std = self.forward(state)
        std = log_std.exp()
        
        # Inverse tanh to get pre-squashing action
        action_normalized = action / self.max_action
        x_t = torch.atanh(torch.clamp(action_normalized, -0.999, 0.999))
        
        normal = torch.distributions.Normal(mean, std)
        log_prob = normal.log_prob(x_t)
        log_prob -= torch.log(1 - action_normalized.pow(2) + 1e-6)
        log_prob = log_prob.sum(1, keepdim=True)
        
        return log_prob


class SoftCritic(nn.Module):
    """Soft Critic network for SAC (state-action value)"""
    
    def __init__(self, state_dim, action_dim, hidden_dims=[256, 256], layer_norm=False):
        """
        Args:
            state_dim: Observation space dimension
            action_dim: Action space dimension
            hidden_dims: List of hidden layer sizes
            layer_norm: Use layer normalization
        """
        super(SoftCritic, self).__init__()
        
        # Q1 network
        layers_q1 = []
        prev_dim = state_dim + action_dim
        
        for hidden_dim in hidden_dims:
            layers_q1.append(nn.Linear(prev_dim, hidden_dim))
            if layer_norm:
                layers_q1.append(nn.LayerNorm(hidden_dim))
            layers_q1.append(nn.ReLU())
            prev_dim = hidden_dim
        
        self.q1_layers = nn.Sequential(*layers_q1)
        self.q1_output = nn.Linear(prev_dim, 1)
        
        # Q2 network
        layers_q2 = []
        prev_dim = state_dim + action_dim
        
        for hidden_dim in hidden_dims:
            layers_q2.append(nn.Linear(prev_dim, hidden_dim))
            if layer_norm:
                layers_q2.append(nn.LayerNorm(hidden_dim))
            layers_q2.append(nn.ReLU())
            prev_dim = hidden_dim
        
        self.q2_layers = nn.Sequential(*layers_q2)
        self.q2_output = nn.Linear(prev_dim, 1)
        
    def forward(self, state, action):
        """Forward pass through both Q networks"""
        sa = torch.cat([state, action], dim=1)
        
        q1 = self.q1_layers(sa)
        q1 = self.q1_output(q1)
        
        q2 = self.q2_layers(sa)
        q2 = self.q2_output(q2)
        
        return q1, q2
    
    def Q1(self, state, action):
        """Get Q1 value only"""
        sa = torch.cat([state, action], dim=1)
        q1 = self.q1_layers(sa)
        q1 = self.q1_output(q1)
        return q1


def soft_update(target, source, tau):
    """Soft update target network parameters
    
    θ_target = τ*θ_source + (1-τ)*θ_target
    """
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(target_param.data * (1.0 - tau) + param.data * tau)


def hard_update(target, source):
    """Hard update target network (copy all parameters)"""
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(param.data)
