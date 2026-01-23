#!/usr/bin/env python3
"""
ARS (Augmented Random Search) Algorithm Implementation
Adapted for Spot Micro RL_app integration
"""

import os
import pickle
import numpy as np
from scipy.signal import butter, filtfilt
import copy

np.random.seed(0)

# Messages for Pipes (multiprocessing)
_RESET = 1
_CLOSE = 2
_EXPLORE = 3

# Filter actions
alpha = 0.7

# For auto yaw control
P_yaw = 5.0


def butter_lowpass_filter(data, cutoff, fs, order=2):
    """Apply butterworth low-pass filter to data"""
    nyq = 0.5 * fs  # Nyquist Frequency
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y


def ParallelWorker(childPipe, env, nb_states):
    """
    Function to deploy multiple ARS agents in parallel
    Used with multiprocessing
    """
    normalizer = Normalizer(nb_states)
    max_action = float(env.action_space.high[0])
    _ = env.reset()
    
    while True:
        try:
            # Only block for short times to have keyboard exceptions be raised
            if not childPipe.poll(0.001):
                continue
            message, payload = childPipe.recv()
        except (EOFError, KeyboardInterrupt):
            break
            
        if message == _RESET:
            _ = env.reset()
            childPipe.send(["reset ok"])
            continue
            
        if message == _EXPLORE:
            # Payloads: [1]: policy, [2]: direction, [3]: delta
            #           [4]: desired_velocity, [5]: desired_rate
            policy = payload[1]
            direction = payload[2]
            delta = payload[3]
            desired_velocity = payload[4]
            desired_rate = payload[5]
            
            state = env.reset()
            sum_rewards = 0.0
            timesteps = 0
            done = False
            
            while not done and timesteps < policy.episode_steps:
                normalizer.observe(state)
                # Normalize State
                state = normalizer.normalize(state)
                action = policy.evaluate(state, delta, direction)
                # Clip action between +-1
                action = np.clip(action, -max_action, max_action)
                
                state, reward, done, _ = env.step(action)
                reward = max(min(reward, 1), -1)  # Clip reward
                sum_rewards += reward
                timesteps += 1
                
            childPipe.send([sum_rewards])
            continue
            
        if message == _CLOSE:
            childPipe.send(["close ok"])
            break
            
    childPipe.close()


class Policy:
    """
    Linear policy: state --> action
    action = theta . state
    """
    
    def __init__(
            self,
            state_dim,
            action_dim,
            learning_rate=0.02,
            num_deltas=16,
            num_best_deltas=16,
            episode_steps=2000,
            expl_noise=0.01,
            seed=0):
        
        # Tunable Hyperparameters
        self.learning_rate = learning_rate
        self.num_deltas = num_deltas
        self.num_best_deltas = num_best_deltas
        assert self.num_best_deltas <= self.num_deltas
        self.episode_steps = episode_steps
        self.expl_noise = expl_noise
        self.seed = seed
        np.random.seed(seed)
        self.state_dim = state_dim
        self.action_dim = action_dim
        
        # Policy matrix (perception matrix)
        self.theta = np.zeros((action_dim, state_dim))
    
    def evaluate(self, state, delta=None, direction=None):
        """
        Evaluate policy to get action
        
        Args:
            state: Current state
            delta: Perturbation (for exploration)
            direction: '+' or '-' (for exploration)
        
        Returns:
            action: Computed action
        """
        if direction is None:
            # Deployment mode: direct policy
            return self.theta.dot(state)
        elif direction == "+":
            # Positive perturbation
            return (self.theta + self.expl_noise * delta).dot(state)
        elif direction == "-":
            # Negative perturbation
            return (self.theta - self.expl_noise * delta).dot(state)
    
    def sample_deltas(self):
        """
        Generate random perturbation matrices
        
        Returns:
            deltas: List of num_deltas random matrices
        """
        deltas = []
        for _ in range(self.num_deltas):
            deltas.append(
                np.random.randn(self.theta.shape[0], self.theta.shape[1])
            )
        return deltas
    
    def update(self, rollouts, std_dev_rewards):
        """
        Update policy weights based on rollouts
        
        Args:
            rollouts: List of (r_pos, r_neg, delta) tuples
            std_dev_rewards: Standard deviation of rewards
        """
        step = np.zeros(self.theta.shape)
        for r_pos, r_neg, delta in rollouts:
            step += (r_pos - r_neg) * delta
        self.theta += self.learning_rate / (self.num_best_deltas * std_dev_rewards) * step


class Normalizer:
    """
    State normalizer - ensures equal weight for each state component
    Computes running average and variance
    """
    
    def __init__(self, state_dim):
        """Initialize state space (all zeros)"""
        self.n = 0
        self.mean = np.zeros(state_dim)
        self.mean_diff = np.zeros(state_dim)
        self.var = np.zeros(state_dim)
        self.std = np.ones(state_dim)
    
    def observe(self, x):
        """
        Update running statistics
        
        Args:
            x: New state observation
        """
        self.n += 1.0
        last_mean = self.mean.copy()
        
        # Running average
        self.mean += (x - self.mean) / self.n
        
        # Used to compute variance
        self.mean_diff += (x - last_mean) * (x - self.mean)
        
        # Variance (clipped to avoid division by zero)
        self.var = (self.mean_diff / self.n).clip(min=1e-2)
        self.std = np.sqrt(self.var)
    
    def normalize(self, states):
        """
        Normalize state: (state - mean) / std
        
        Args:
            states: State to normalize
        
        Returns:
            Normalized state
        """
        state_mean = self.mean
        state_std = self.std
        return (states - state_mean) / state_std


class ARSAgent:
    """
    ARS Agent - manages training and deployment
    """
    
    def __init__(self,
                 normalizer,
                 policy,
                 env,
                 desired_velocity=0.5,
                 desired_rate=0.0):
        
        self.normalizer = normalizer
        self.policy = policy
        self.state_dim = self.policy.state_dim
        self.action_dim = self.policy.action_dim
        self.env = env
        self.max_action = float(self.env.action_space.high[0])
        
        self.desired_velocity = desired_velocity
        self.desired_rate = desired_rate
    
    def deploy(self, direction=None, delta=None):
        """
        Deploy policy for one episode
        
        Args:
            direction: '+' or '-' (for exploration) or None (deployment)
            delta: Perturbation (for exploration)
        
        Returns:
            sum_rewards: Total reward for episode
        """
        state = self.env.reset()
        sum_rewards = 0.0
        timesteps = 0
        done = False
        
        while not done and timesteps < self.policy.episode_steps:
            self.normalizer.observe(state)
            # Normalize State
            state = self.normalizer.normalize(state)
            action = self.policy.evaluate(state, delta, direction)
            # Clip action
            action = np.clip(action, -self.max_action, self.max_action)
            
            state, reward, done, _ = self.env.step(action)
            # Clip reward
            reward = np.clip(reward, -self.max_action, self.max_action)
            sum_rewards += reward
            timesteps += 1
            
        return sum_rewards
    
    def train(self):
        """
        Single-threaded training (not recommended for production)
        
        Returns:
            eval_reward: Reward from current policy
        """
        print("-------------------------------")
        print("Sampling Deltas")
        deltas = self.policy.sample_deltas()
        
        positive_rewards = [0] * self.policy.num_deltas
        negative_rewards = [0] * self.policy.num_deltas
        
        print("Deploying Rollouts")
        for i in range(self.policy.num_deltas):
            print(f"Rollout #{i + 1}")
            positive_rewards[i] = self.deploy(direction="+", delta=deltas[i])
            negative_rewards[i] = self.deploy(direction="-", delta=deltas[i])
        
        # Calculate std dev
        std_dev_rewards = np.array(positive_rewards + negative_rewards).std()
        
        # Sort rollouts
        unsorted_rollouts = [
            (positive_rewards[i], negative_rewards[i], deltas[i])
            for i in range(self.policy.num_deltas)
        ]
        sorted_rollouts = sorted(
            unsorted_rollouts,
            key=lambda x: max(x[0], x[1]),
            reverse=True
        )
        
        # Take best rollouts
        rollouts = sorted_rollouts[:self.policy.num_best_deltas]
        
        # Update Policy
        self.policy.update(rollouts, std_dev_rewards)
        
        # Evaluate current policy
        eval_reward = self.deploy()
        return eval_reward
    
    def train_parallel(self, parentPipes):
        """
        Parallel training using multiprocessing
        
        Args:
            parentPipes: List of parent pipes for workers
        
        Returns:
            (eval_reward, timesteps): Reward and timesteps from current policy
        """
        # Sample perturbations
        deltas = self.policy.sample_deltas()
        positive_rewards = [0] * self.policy.num_deltas
        negative_rewards = [0] * self.policy.num_deltas
        
        if not parentPipes:
            raise ValueError("Use 'train' method if not using multiprocessing!")
        
        # Positive perturbations
        for i in range(self.policy.num_deltas):
            parentPipe = parentPipes[i]
            parentPipe.send([
                _EXPLORE,
                [
                    self.normalizer, self.policy, "+", deltas[i],
                    self.desired_velocity, self.desired_rate
                ]
            ])
        
        for i in range(self.policy.num_deltas):
            positive_rewards[i] = parentPipes[i].recv()[0]
        
        # Negative perturbations
        for i in range(self.policy.num_deltas):
            parentPipe = parentPipes[i]
            parentPipe.send([
                _EXPLORE,
                [
                    self.normalizer, self.policy, "-", deltas[i],
                    self.desired_velocity, self.desired_rate
                ]
            ])
        
        for i in range(self.policy.num_deltas):
            negative_rewards[i] = parentPipes[i].recv()[0]
        
        # Calculate std dev
        std_dev_rewards = np.array(positive_rewards + negative_rewards).std()
        
        # Sort rollouts by max(r_pos, r_neg)
        scores = {
            k: max(r_pos, r_neg)
            for k, (r_pos, r_neg) in enumerate(zip(positive_rewards, negative_rewards))
        }
        indices = sorted(scores.keys(), key=lambda x: scores[x], reverse=True)
        indices = indices[:self.policy.num_best_deltas]
        
        rollouts = [
            (positive_rewards[k], negative_rewards[k], deltas[k])
            for k in indices
        ]
        
        # Update Policy
        self.policy.update(rollouts, std_dev_rewards)
        
        # Evaluate current policy
        state = self.env.reset()
        sum_rewards = 0.0
        timesteps = 0
        done = False
        
        while not done and timesteps < self.policy.episode_steps:
            self.normalizer.observe(state)
            state = self.normalizer.normalize(state)
            action = self.policy.evaluate(state)
            action = np.clip(action, -self.max_action, self.max_action)
            state, reward, done, _ = self.env.step(action)
            sum_rewards += reward
            timesteps += 1
        
        return sum_rewards, timesteps
    
    def save(self, filename):
        """
        Save policy and normalizer (numpy format)
        
        Args:
            filename: Base filename (without extension)
        """
        np.save(filename + '_policy', self.policy.theta)
        np.save(filename + '_mean', self.normalizer.mean)
        np.save(filename + '_std', self.normalizer.std)
        print(f"Saved model to {filename}_*")
    
    def load(self, filename):
        """
        Load policy and normalizer (numpy format)
        
        Args:
            filename: Base filename (without extension)
        """
        self.policy.theta = np.load(filename + '_policy.npy')
        self.normalizer.mean = np.load(filename + '_mean.npy')
        self.normalizer.std = np.load(filename + '_std.npy')
        print(f"Loaded model from {filename}_*")
    
    def save_policy(self, filename):
        """
        Save complete policy to pickle file (compatible with src_RL format)
        
        Args:
            filename: Filename (with or without extension)
        """
        # Remove extension if present
        if filename.endswith('.pkl'):
            filename = filename[:-4]
        
        policy_data = {
            'state_dim': self.state_dim,
            'action_dim': self.action_dim,
            'learning_rate': self.policy.learning_rate,
            'num_deltas': self.policy.num_deltas,
            'num_best_deltas': self.policy.num_best_deltas,
            'episode_steps': self.policy.episode_steps,
            'expl_noise': self.policy.expl_noise,
            'seed': self.policy.seed,
            'theta': self.policy.theta,
            'normalizer_mean': self.normalizer.mean,
            'normalizer_std': self.normalizer.std,
            'normalizer_n': self.normalizer.n,
            'normalizer_var': self.normalizer.var
        }
        
        with open(filename, 'wb') as f:
            pickle.dump(policy_data, f)
        
        print(f"✅ Saved policy to: {filename}")
    
    def load_policy(self, filename):
        """
        Load complete policy from pickle file (compatible with src_RL format)
        
        Args:
            filename: Filename (with or without extension)
        """
        # Add extension if not present
        if not filename.endswith('.pkl') and not os.path.exists(filename):
            filename = filename + '.pkl'
        
        with open(filename, 'rb') as f:
            policy_data = pickle.load(f)
        
        # Restore policy parameters
        self.policy.theta = policy_data['theta']
        self.policy.learning_rate = policy_data.get('learning_rate', 0.02)
        self.policy.num_deltas = policy_data.get('num_deltas', 16)
        self.policy.num_best_deltas = policy_data.get('num_best_deltas', 16)
        self.policy.episode_steps = policy_data.get('episode_steps', 2000)
        self.policy.expl_noise = policy_data.get('expl_noise', 0.01)
        self.policy.seed = policy_data.get('seed', 0)
        
        # Restore normalizer
        self.normalizer.mean = policy_data['normalizer_mean']
        self.normalizer.std = policy_data['normalizer_std']
        self.normalizer.n = policy_data.get('normalizer_n', 0)
        self.normalizer.var = policy_data.get('normalizer_var', np.zeros_like(self.normalizer.mean))
        
        print(f"✅ Loaded policy from: {filename}")
        print(f"   State dim: {policy_data['state_dim']}")
        print(f"   Action dim: {policy_data['action_dim']}")
        print(f"   Episode steps: {self.policy.episode_steps}")
