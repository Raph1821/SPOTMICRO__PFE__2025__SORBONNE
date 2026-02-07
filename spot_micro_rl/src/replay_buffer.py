"""
Replay Buffer for Off-Policy RL Algorithms
===========================================

Experience replay buffer for SAC and TD3:
- Efficient random sampling
- Configurable buffer size
- Batched transitions
"""

import numpy as np
import torch


class ReplayBuffer:
    """Fixed-size buffer to store experience tuples (s, a, r, s', done)"""
    
    def __init__(self, state_dim, action_dim, max_size=1000000, device='cpu'):
        """
        Args:
            state_dim: Dimension of state space
            action_dim: Dimension of action space
            max_size: Maximum buffer size
            device: PyTorch device for tensors
        """
        self.max_size = max_size
        self.ptr = 0
        self.size = 0
        self.device = device
        
        # Preallocate memory
        self.states = np.zeros((max_size, state_dim), dtype=np.float32)
        self.actions = np.zeros((max_size, action_dim), dtype=np.float32)
        self.rewards = np.zeros((max_size, 1), dtype=np.float32)
        self.next_states = np.zeros((max_size, state_dim), dtype=np.float32)
        self.dones = np.zeros((max_size, 1), dtype=np.float32)
        
    def add(self, state, action, reward, next_state, done):
        """Add a new experience to buffer"""
        self.states[self.ptr] = state
        self.actions[self.ptr] = action
        self.rewards[self.ptr] = reward
        self.next_states[self.ptr] = next_state
        self.dones[self.ptr] = done
        
        self.ptr = (self.ptr + 1) % self.max_size
        self.size = min(self.size + 1, self.max_size)
        
    def sample(self, batch_size):
        """Sample a batch of experiences"""
        indices = np.random.randint(0, self.size, size=batch_size)
        
        batch = (
            torch.FloatTensor(self.states[indices]).to(self.device),
            torch.FloatTensor(self.actions[indices]).to(self.device),
            torch.FloatTensor(self.rewards[indices]).to(self.device),
            torch.FloatTensor(self.next_states[indices]).to(self.device),
            torch.FloatTensor(self.dones[indices]).to(self.device)
        )
        
        return batch
    
    def __len__(self):
        """Return current size of buffer"""
        return self.size
    
    def save(self, filename):
        """Save buffer to disk"""
        np.savez(
            filename,
            states=self.states[:self.size],
            actions=self.actions[:self.size],
            rewards=self.rewards[:self.size],
            next_states=self.next_states[:self.size],
            dones=self.dones[:self.size],
            ptr=self.ptr,
            size=self.size
        )
        
    def load(self, filename):
        """Load buffer from disk"""
        data = np.load(filename)
        
        self.size = int(data['size'])
        self.ptr = int(data['ptr'])
        
        self.states[:self.size] = data['states']
        self.actions[:self.size] = data['actions']
        self.rewards[:self.size] = data['rewards']
        self.next_states[:self.size] = data['next_states']
        self.dones[:self.size] = data['dones']


class PrioritizedReplayBuffer(ReplayBuffer):
    """Prioritized Experience Replay Buffer
    
    Samples transitions with priority based on TD error.
    More important transitions are sampled more frequently.
    """
    
    def __init__(self, state_dim, action_dim, max_size=1000000, 
                 alpha=0.6, beta=0.4, device='cpu'):
        """
        Args:
            state_dim: Dimension of state space
            action_dim: Dimension of action space
            max_size: Maximum buffer size
            alpha: Priority exponent (0 = uniform, 1 = full prioritization)
            beta: Importance sampling correction (increases to 1)
            device: PyTorch device
        """
        super().__init__(state_dim, action_dim, max_size, device)
        
        self.alpha = alpha
        self.beta = beta
        self.beta_increment = 0.001  # Anneal beta to 1
        
        # Priority tree (simplified with array)
        self.priorities = np.zeros(max_size, dtype=np.float32)
        self.max_priority = 1.0
        
    def add(self, state, action, reward, next_state, done):
        """Add experience with maximum priority"""
        super().add(state, action, reward, next_state, done)
        
        # New transitions get max priority
        self.priorities[self.ptr - 1] = self.max_priority
        
    def sample(self, batch_size):
        """Sample batch according to priorities"""
        # Compute sampling probabilities
        if self.size == self.max_size:
            priorities = self.priorities
        else:
            priorities = self.priorities[:self.size]
            
        probs = priorities ** self.alpha
        probs /= probs.sum()
        
        # Sample indices
        indices = np.random.choice(self.size, batch_size, p=probs, replace=False)
        
        # Compute importance sampling weights
        weights = (self.size * probs[indices]) ** (-self.beta)
        weights /= weights.max()
        weights = torch.FloatTensor(weights).to(self.device).reshape(-1, 1)
        
        # Anneal beta
        self.beta = min(1.0, self.beta + self.beta_increment)
        
        # Get batch
        batch = (
            torch.FloatTensor(self.states[indices]).to(self.device),
            torch.FloatTensor(self.actions[indices]).to(self.device),
            torch.FloatTensor(self.rewards[indices]).to(self.device),
            torch.FloatTensor(self.next_states[indices]).to(self.device),
            torch.FloatTensor(self.dones[indices]).to(self.device),
            weights,
            indices
        )
        
        return batch
    
    def update_priorities(self, indices, td_errors):
        """Update priorities based on TD errors"""
        priorities = np.abs(td_errors) + 1e-6  # Small constant for stability
        self.priorities[indices] = priorities
        self.max_priority = max(self.max_priority, priorities.max())
