"""
Soft Actor-Critic (SAC)
========================

Maximum entropy off-policy actor-critic algorithm with:
- Stochastic Gaussian policy
- Automatic temperature tuning
- Twin Q-networks
- Entropy-regularized objective

Reference: Haarnoja et al., "Soft Actor-Critic Algorithms and Applications"
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import os

from .networks import GaussianActor, SoftCritic, soft_update, hard_update
from .replay_buffer import ReplayBuffer


class SACAgent:
    """Soft Actor-Critic Agent with automatic entropy tuning"""
    
    def __init__(
        self,
        state_dim,
        action_dim,
        max_action=1.0,
        hidden_dims=[256, 256],
        lr_actor=3e-4,
        lr_critic=3e-4,
        lr_alpha=3e-4,
        gamma=0.99,
        tau=0.005,
        alpha=0.2,
        auto_entropy_tuning=True,
        buffer_size=1000000,
        device='cpu',
        layer_norm=False
    ):
        """
        Args:
            state_dim: Observation space dimension
            action_dim: Action space dimension
            max_action: Maximum action magnitude
            hidden_dims: Hidden layer sizes
            lr_actor: Actor learning rate
            lr_critic: Critic learning rate
            lr_alpha: Temperature parameter learning rate
            gamma: Discount factor
            tau: Target network soft update rate
            alpha: Initial temperature parameter
            auto_entropy_tuning: Automatically tune temperature
            buffer_size: Replay buffer capacity
            device: PyTorch device
            layer_norm: Use layer normalization
        """
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.max_action = max_action
        self.gamma = gamma
        self.tau = tau
        self.device = device
        self.auto_entropy_tuning = auto_entropy_tuning
        
        # Networks
        self.actor = GaussianActor(
            state_dim, action_dim, hidden_dims, max_action, layer_norm
        ).to(device)
        
        self.critic = SoftCritic(state_dim, action_dim, hidden_dims, layer_norm).to(device)
        self.critic_target = SoftCritic(state_dim, action_dim, hidden_dims, layer_norm).to(device)
        hard_update(self.critic_target, self.critic)
        
        # Optimizers
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr_actor)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=lr_critic)
        
        # Temperature parameter (entropy coefficient)
        if auto_entropy_tuning:
            # Target entropy = -dim(A)
            self.target_entropy = -action_dim
            self.log_alpha = torch.zeros(1, requires_grad=True, device=device)
            self.alpha_optimizer = optim.Adam([self.log_alpha], lr=lr_alpha)
            self.alpha = self.log_alpha.exp()
        else:
            self.alpha = torch.tensor(alpha).to(device)
        
        # Replay buffer
        self.replay_buffer = ReplayBuffer(state_dim, action_dim, buffer_size, device)
        
        # Training stats
        self.total_it = 0
        self.critic_loss_history = []
        self.actor_loss_history = []
        self.alpha_loss_history = []
        self.alpha_history = []
        
    def select_action(self, state, deterministic=False):
        """Select action from policy"""
        state = torch.FloatTensor(state.reshape(1, -1)).to(self.device)
        
        with torch.no_grad():
            if deterministic:
                action, _ = self.actor.sample(state, deterministic=True)
            else:
                action, _ = self.actor.sample(state, deterministic=False)
        
        return action.cpu().numpy().flatten()
    
    def train(self, batch_size=256):
        """Update networks with a batch from replay buffer"""
        if len(self.replay_buffer) < batch_size:
            return None, None, None
        
        self.total_it += 1
        
        # Sample batch
        state, action, reward, next_state, done = self.replay_buffer.sample(batch_size)
        
        # ========== Critic Update ==========
        with torch.no_grad():
            # Sample actions from current policy for next states
            next_action, next_log_prob = self.actor.sample(next_state)
            
            # Compute target Q values (minimum of two Q-networks)
            target_Q1, target_Q2 = self.critic_target(next_state, next_action)
            target_Q = torch.min(target_Q1, target_Q2)
            
            # Add entropy term
            target_Q = target_Q - self.alpha * next_log_prob
            
            # Compute target value
            target_Q = reward + (1 - done) * self.gamma * target_Q
        
        # Get current Q estimates
        current_Q1, current_Q2 = self.critic(state, action)
        
        # Compute critic loss
        critic_loss = nn.MSELoss()(current_Q1, target_Q) + nn.MSELoss()(current_Q2, target_Q)
        
        # Optimize critic
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()
        
        self.critic_loss_history.append(critic_loss.item())
        
        # ========== Actor Update ==========
        # Sample new actions from current policy
        new_action, log_prob = self.actor.sample(state)
        
        # Compute Q values for new actions
        Q1_new, Q2_new = self.critic(state, new_action)
        Q_new = torch.min(Q1_new, Q2_new)
        
        # Actor loss: maximize Q - alpha * log_prob
        actor_loss = (self.alpha * log_prob - Q_new).mean()
        
        # Optimize actor
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()
        
        self.actor_loss_history.append(actor_loss.item())
        
        # ========== Temperature Update ==========
        alpha_loss = None
        if self.auto_entropy_tuning:
            # Alpha loss: match current entropy to target
            alpha_loss = -(self.log_alpha * (log_prob + self.target_entropy).detach()).mean()
            
            # Optimize alpha
            self.alpha_optimizer.zero_grad()
            alpha_loss.backward()
            self.alpha_optimizer.step()
            
            self.alpha = self.log_alpha.exp()
            self.alpha_loss_history.append(alpha_loss.item())
            self.alpha_history.append(self.alpha.item())
        
        # ========== Soft Update Target Networks ==========
        soft_update(self.critic_target, self.critic, self.tau)
        
        return (
            critic_loss.item(),
            actor_loss.item(),
            alpha_loss.item() if alpha_loss is not None else None
        )
    
    def save(self, filename):
        """Save agent networks and replay buffer"""
        checkpoint = {
            'actor': self.actor.state_dict(),
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
                'auto_entropy_tuning': self.auto_entropy_tuning
            }
        }
        
        if self.auto_entropy_tuning:
            checkpoint['log_alpha'] = self.log_alpha
            checkpoint['alpha_optimizer'] = self.alpha_optimizer.state_dict()
        else:
            checkpoint['alpha'] = self.alpha
        
        torch.save(checkpoint, filename)
        print(f"✅ SAC agent saved to: {filename}")
        
        # Save replay buffer separately
        buffer_file = filename.replace('.pth', '_buffer.npz')
        self.replay_buffer.save(buffer_file)
        
    def load(self, filename, load_buffer=False):
        """Load agent networks and optionally replay buffer"""
        checkpoint = torch.load(filename, map_location=self.device)
        
        self.actor.load_state_dict(checkpoint['actor'])
        self.critic.load_state_dict(checkpoint['critic'])
        self.critic_target.load_state_dict(checkpoint['critic_target'])
        self.actor_optimizer.load_state_dict(checkpoint['actor_optimizer'])
        self.critic_optimizer.load_state_dict(checkpoint['critic_optimizer'])
        self.total_it = checkpoint['total_it']
        
        if self.auto_entropy_tuning:
            self.log_alpha = checkpoint['log_alpha']
            self.alpha_optimizer.load_state_dict(checkpoint['alpha_optimizer'])
            self.alpha = self.log_alpha.exp()
        else:
            self.alpha = checkpoint['alpha']
        
        print(f"✅ SAC agent loaded from: {filename}")
        print(f"   Total iterations: {self.total_it}")
        print(f"   Current alpha: {self.alpha.item():.4f}")
        
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
        
        if self.alpha_history:
            stats['alpha'] = self.alpha.item()
            stats['alpha_mean'] = np.mean(self.alpha_history[-100:])
        
        stats['buffer_size'] = len(self.replay_buffer)
        stats['total_iterations'] = self.total_it
        
        return stats


class SACPolicy:
    """Wrapper for SAC policy (for compatibility with ARS interface)"""
    
    def __init__(self, agent):
        """
        Args:
            agent: SACAgent instance
        """
        self.agent = agent
        self.state_dim = agent.state_dim
        self.action_dim = agent.action_dim
        
    def evaluate(self, state, delta=None, direction=None):
        """Evaluate policy (deterministic mode)"""
        return self.agent.select_action(state, deterministic=True)
    
    def sample_deltas(self, num_deltas):
        """Not used in SAC (for ARS compatibility only)"""
        return []
    
    def update(self, rollouts, sigma_r):
        """Not used in SAC (for ARS compatibility only)"""
        pass
    
    def save(self, filename):
        """Save policy"""
        self.agent.save(filename)
        
    def load(self, filename):
        """Load policy"""
        self.agent.load(filename)
