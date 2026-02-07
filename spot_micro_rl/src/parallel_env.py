#!/usr/bin/env python3
"""
Parallel Environment Wrapper for Deep RL
Manages multiple PyBullet environments in separate processes for efficient data collection
"""

import numpy as np
import multiprocessing as mp
from multiprocessing import Process, Pipe
from typing import List, Tuple, Callable


def worker(remote, parent_remote, env_fn):
    """
    Worker process function
    Runs one environment and communicates via pipe
    
    Args:
        remote: Communication pipe (worker side)
        parent_remote: Communication pipe (parent side)
        env_fn: Function to create environment
    """
    parent_remote.close()
    env = env_fn()
    
    try:
        while True:
            cmd, data = remote.recv()
            
            if cmd == 'step':
                obs, reward, done, info = env.step(data)
                if done:
                    # Auto-reset on episode end
                    obs = env.reset()
                remote.send((obs, reward, done, info))
                
            elif cmd == 'reset':
                obs = env.reset()
                remote.send(obs)
                
            elif cmd == 'close':
                env.close()
                remote.close()
                break
                
            elif cmd == 'get_spaces':
                remote.send((env.observation_space, env.action_space))
                
            else:
                raise NotImplementedError(f"Unknown command: {cmd}")
                
    except KeyboardInterrupt:
        print(f"Worker interrupted")
    finally:
        env.close()


class ParallelEnv:
    """
    Parallel environment wrapper using multiprocessing
    
    Manages N environments in separate processes for parallel data collection
    Useful for Deep RL algorithms (SAC, TD3) to speed up rollouts
    
    Usage:
        def make_env():
            return SpotMicroEnv(render=False, terrain_randomization=True)
        
        vec_env = ParallelEnv(make_env, num_workers=4)
        obs = vec_env.reset()
        
        for _ in range(1000):
            actions = agent.select_actions(obs)  # Shape (4, action_dim)
            obs, rewards, dones, infos = vec_env.step(actions)
    """
    
    def __init__(self, env_fn: Callable, num_workers: int = 4):
        """
        Args:
            env_fn: Function that creates and returns an environment
            num_workers: Number of parallel environments
        """
        self.num_workers = num_workers
        self.env_fn = env_fn
        
        # Create pipes for communication
        self.remotes, self.work_remotes = zip(*[Pipe() for _ in range(num_workers)])
        
        # Create worker processes
        self.processes = []
        for work_remote, remote in zip(self.work_remotes, self.remotes):
            args = (work_remote, remote, env_fn)
            process = Process(target=worker, args=args, daemon=True)
            process.start()
            self.processes.append(process)
            work_remote.close()
        
        # Get observation/action spaces from first env
        self.remotes[0].send(('get_spaces', None))
        self.observation_space, self.action_space = self.remotes[0].recv()
        
        print(f"✅ ParallelEnv initialized with {num_workers} workers")
    
    def reset(self) -> np.ndarray:
        """
        Reset all environments
        
        Returns:
            Observations from all environments, shape (num_workers, obs_dim)
        """
        for remote in self.remotes:
            remote.send(('reset', None))
        
        obs = [remote.recv() for remote in self.remotes]
        return np.stack(obs)
    
    def step(self, actions: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, List]:
        """
        Step all environments with given actions
        
        Args:
            actions: Actions for each env, shape (num_workers, action_dim)
        
        Returns:
            observations: Shape (num_workers, obs_dim)
            rewards: Shape (num_workers,)
            dones: Shape (num_workers,)
            infos: List of info dicts
        """
        # Send actions to all workers
        for remote, action in zip(self.remotes, actions):
            remote.send(('step', action))
        
        # Receive results
        results = [remote.recv() for remote in self.remotes]
        
        obs, rewards, dones, infos = zip(*results)
        
        return (
            np.stack(obs),
            np.array(rewards),
            np.array(dones),
            list(infos)
        )
    
    def close(self):
        """Close all environments and terminate processes"""
        for remote in self.remotes:
            remote.send(('close', None))
        
        for process in self.processes:
            process.join(timeout=5)
            if process.is_alive():
                process.terminate()
        
        print("✅ ParallelEnv closed")
    
    def __del__(self):
        """Cleanup on deletion"""
        self.close()


class DummyParallelEnv:
    """
    Dummy parallel env that runs sequentially (for debugging)
    Same interface as ParallelEnv but without multiprocessing
    """
    
    def __init__(self, env_fn: Callable, num_workers: int = 4):
        self.num_workers = num_workers
        self.envs = [env_fn() for _ in range(num_workers)]
        self.observation_space = self.envs[0].observation_space
        self.action_space = self.envs[0].action_space
        
        print(f"⚠️  DummyParallelEnv (sequential) initialized with {num_workers} envs")
    
    def reset(self) -> np.ndarray:
        obs = [env.reset() for env in self.envs]
        return np.stack(obs)
    
    def step(self, actions: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, List]:
        results = []
        for env, action in zip(self.envs, actions):
            obs, reward, done, info = env.step(action)
            if done:
                obs = env.reset()
            results.append((obs, reward, done, info))
        
        obs, rewards, dones, infos = zip(*results)
        
        return (
            np.stack(obs),
            np.array(rewards),
            np.array(dones),
            list(infos)
        )
    
    def close(self):
        for env in self.envs:
            env.close()
