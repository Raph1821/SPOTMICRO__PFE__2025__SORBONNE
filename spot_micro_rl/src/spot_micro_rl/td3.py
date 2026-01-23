"""
Twin Delayed Deep Deterministic Policy Gradient (TD3)
======================================================

Off-policy actor-critic algorithm with:
- Twin Q-networks to reduce overestimation bias
- Delayed policy updates
- Target policy smoothing
- Deterministic policy

Reference: Fujimoto et al., "Addressing Function Approximation Error in Actor-Critic Methods"
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import os
import pickle

from .networks import Actor, Critic, soft_update, hard_update
from .replay_buffer import ReplayBuffer


class TD3Agent:
    """Twin Delayed DDPG Agent"""
    
    def __init__(
        self,
        state_dim,
        action_dim,
        max_action=1.0,
        hidden_dims=[256, 256],
        lr_actor=3e-4,
        lr_critic=3e-4,
        gamma=0.99,
        tau=0.005,
        policy_noise=0.2,
        noise_clip=0.5,
        policy_freq=2,
        buffer_size=1000000,
        device='cpu',
        layer_norm=False
    ):
        """
        Args:
            state_dim: Observation space dimension
            action_dim: Action space dimension
            max_action: Maximum action magnitude
            hidden_dims: Hidden layer sizes for networks
            lr_actor: Actor learning rate
            lr_critic: Critic learning rate
            gamma: Discount factor
            tau: Target network soft update rate
            policy_noise: Std of Gaussian noise for target policy smoothing
            noise_clip: Range to clip target policy noise
            policy_freq: Frequency of delayed policy updates
            buffer_size: Replay buffer capacity
            device: PyTorch device
            layer_norm: Use layer normalization
        """
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.max_action = max_action
        self.gamma = gamma
        self.tau = tau
        self.policy_noise = policy_noise
        self.noise_clip = noise_clip
        self.policy_freq = policy_freq
        self.device = device
        
        # Networks
        self.actor = Actor(state_dim, action_dim, hidden_dims, max_action, layer_norm).to(device)
        self.actor_target = Actor(state_dim, action_dim, hidden_dims, max_action, layer_norm).to(device)
        hard_update(self.actor_target, self.actor)
        
        self.critic = Critic(state_dim, action_dim, hidden_dims, layer_norm).to(device)
        self.critic_target = Critic(state_dim, action_dim, hidden_dims, layer_norm).to(device)
        hard_update(self.critic_target, self.critic)
        
        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr_actor)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=lr_critic)
        
        # Replay buffer
        self.replay_buffer = ReplayBuffer(state_dim, action_dim, buffer_size, device)
        
        # Training stats
        self.total_it = 0
        self.critic_loss_history = []
        self.actor_loss_history = []
        
    def select_action(self, state, noise=0.0):
        """Select action with optional exploration noise"""
        state = torch.FloatTensor(state.reshape(1, -1)).to(self.device)
        
        with torch.no_grad():
            action = self.actor(state).cpu().numpy().flatten()
        
        # Add exploration noise
        if noise > 0:
            action += np.random.normal(0, noise * self.max_action, size=self.action_dim)
            action = np.clip(action, -self.max_action, self.max_action)
        
        return action
    
    def train(self, batch_size=256):
        """Update networks with a batch from replay buffer"""
        if len(self.replay_buffer) < batch_size:
            return None, None
        
        self.total_it += 1
        
        # Sample batch
        state, action, reward, next_state, done = self.replay_buffer.sample(batch_size)
        
        # ========== Critic Update ==========
        with torch.no_grad():
            # Target policy smoothing: add clipped noise to target actions
            noise = (torch.randn_like(action) * self.policy_noise).clamp(
                -self.noise_clip, self.noise_clip
            )
            next_action = (self.actor_target(next_state) + noise).clamp(
                -self.max_action, self.max_action
            )
            
            # Compute target Q value (minimum of two Q-networks)
            target_Q1, target_Q2 = self.critic_target(next_state, next_action)
            target_Q = torch.min(target_Q1, target_Q2)
            target_Q = reward + (1 - done) * self.gamma * target_Q
        
        # Get current Q estimates
        current_Q1, current_Q2 = self.critic(state, action)
        
        # Compute critic loss (MSE)
        critic_loss = nn.MSELoss()(current_Q1, target_Q) + nn.MSELoss()(current_Q2, target_Q)
        
        # Optimize critic
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()
        
        self.critic_loss_history.append(critic_loss.item())
        
        # ========== Delayed Actor Update ==========
        actor_loss = None
        if self.total_it % self.policy_freq == 0:
            # Compute actor loss (maximize Q1)
            actor_loss = -self.critic.Q1(state, self.actor(state)).mean()
            
            # Optimize actor
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()
            
            self.actor_loss_history.append(actor_loss.item())
            
            # Soft update target networks
            soft_update(self.actor_target, self.actor, self.tau)
            soft_update(self.critic_target, self.critic, self.tau)
        
        return critic_loss.item(), actor_loss.item() if actor_loss is not None else None
    
    def save(self, filename):
        """Save agent networks and replay buffer"""
        checkpoint = {
            'actor': self.actor.state_dict(),
            'actor_target': self.actor_target.state_dict(),
            'critic': self.critic.state_dict(),
            'critic_target': self.critic_target.state_dict(),
            'actor_optimizer': self.actor_optimizer.state_dict(),
            'critic_optimizer': self.critic_optimizer.state_dict(),
            'total_it': self.total_it,
            'config': {
                'state_dim': self.state_dim,
                'action_dim': self.action_dim,
                'max_action': self.max_action,
                'gamma': self.gamma,
                'tau': self.tau,
                'policy_noise': self.policy_noise,
                'noise_clip': self.noise_clip,
                'policy_freq': self.policy_freq
            }
        }
        
        torch.save(checkpoint, filename)
        print(f"✅ TD3 agent saved to: {filename}")
        
        # Save replay buffer separately
        buffer_file = filename.replace('.pth', '_buffer.npz')
        self.replay_buffer.save(buffer_file)
        
    def load(self, filename, load_buffer=False):
        """Load agent networks and optionally replay buffer"""
        checkpoint = torch.load(filename, map_location=self.device)
        
        self.actor.load_state_dict(checkpoint['actor'])
        self.actor_target.load_state_dict(checkpoint['actor_target'])
        self.critic.load_state_dict(checkpoint['critic'])
        self.critic_target.load_state_dict(checkpoint['critic_target'])
        self.actor_optimizer.load_state_dict(checkpoint['actor_optimizer'])
        self.critic_optimizer.load_state_dict(checkpoint['critic_optimizer'])
        self.total_it = checkpoint['total_it']
        
        print(f"✅ TD3 agent loaded from: {filename}")
        print(f"   Total iterations: {self.total_it}")
        
        # Load replay buffer if requested
        if load_buffer:
            buffer_file = filename.replace('.pth', '_buffer.npz')
            if os.path.exists(buffer_file):
                self.replay_buffer.load(buffer_file)
                print(f"✅ Replay buffer loaded ({len(self.replay_buffer)} transitions)")
        
    def get_stats(self):
        """Get training statistics"""
        stats = {}
        
        if self.critic_loss_history:
            stats['critic_loss_mean'] = np.mean(self.critic_loss_history[-100:])
            stats['critic_loss_std'] = np.std(self.critic_loss_history[-100:])
        
        if self.actor_loss_history:
            stats['actor_loss_mean'] = np.mean(self.actor_loss_history[-100:])
            stats['actor_loss_std'] = np.std(self.actor_loss_history[-100:])
        
        stats['buffer_size'] = len(self.replay_buffer)
        stats['total_iterations'] = self.total_it
        
        return stats


class TD3Policy:
    """Wrapper for TD3 policy (for compatibility with ARS interface)"""
    
    def __init__(self, agent):
        """
        Args:
            agent: TD3Agent instance
        """
        self.agent = agent
        self.state_dim = agent.state_dim
        self.action_dim = agent.action_dim
        
    def evaluate(self, state, delta=None, direction=None):
        """Evaluate policy (ignores delta/direction - TD3 is deterministic)"""
        return self.agent.select_action(state, noise=0.0)
    
    def sample_deltas(self, num_deltas):
        """Not used in TD3 (for ARS compatibility only)"""
        return []
    
    def update(self, rollouts, sigma_r):
        """Not used in TD3 (for ARS compatibility only)"""
        pass
    
    def save(self, filename):
        """Save policy"""
        self.agent.save(filename)
        
    def load(self, filename):
        """Load policy"""
        self.agent.load(filename)
