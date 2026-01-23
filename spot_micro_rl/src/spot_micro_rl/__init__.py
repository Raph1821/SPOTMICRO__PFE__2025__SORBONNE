"""
Spot Micro RL Package - Complete RL Training Suite
Reinforcement Learning for Spot Micro Quadruped Robot

Version 2.0.0 - All Phases Complete:
- Phase 1: Motor models, domain randomization, terrain
- Phase 2: Enhanced training, GMBC analysis, survival logging
- Phase 3: Deep RL (SAC & TD3), neural networks, replay buffers
"""

__version__ = '2.0.0'

# Import main classes for easy access
from .ars import ARSAgent, Policy, Normalizer
from .spot_kinematics import SpotModel
from .leg_kinematics import LegIK
from .bezier_gait import BezierGait
from .spot_env import SpotMicroEnv
from .lie_algebra import RpToTrans, TransToRp, TransInv, RPY

# Phase 1: Motor models et domain randomization
from .motor import MotorModel
from .env_randomizer_base import EnvRandomizerBase
from .spot_env_randomizer import (
    SpotEnvRandomizer,
    MinimalRandomizer,
    AggressiveRandomizer
)
from .heightfield import HeightField, FlatTerrain, GentleTerrain, RoughTerrain

# Phase 2: Advanced Training Components
from .ars import Policy, Normalizer, ARSAgent
from .parallel_env import ParallelEnv, DummyParallelEnv

# Phase 3: Deep RL Algorithms
try:
    import torch
    from .networks import Actor, Critic, GaussianActor, SoftCritic
    from .replay_buffer import ReplayBuffer, PrioritizedReplayBuffer
    from .sac import SACAgent, SACPolicy
    from .td3 import TD3Agent, TD3Policy
    _TORCH_AVAILABLE = True
except ImportError:
    _TORCH_AVAILABLE = False

__all__ = [
    # Core RL
    'ARSAgent',
    'Policy',
    'Normalizer',
    
    # Kinematics
    'SpotModel',
    'LegIK',
    'BezierGait',
    
    # Environment
    'SpotMicroEnv',
    
    # Math
    'RpToTrans',
    'TransToRp',
    'TransInv',
    'RPY',
    
    # Phase 1: Motor Models
    'MotorModel',
    
    # Phase 1: Domain Randomization
    'EnvRandomizerBase',
    'SpotEnvRandomizer',
    'MinimalRandomizer',
    'AggressiveRandomizer',
    
    # Phase 1: Terrain
    'HeightField',
    'FlatTerrain',
    'GentleTerrain',
    'RoughTerrain',
]

# Add Phase 3 components if PyTorch available
if _TORCH_AVAILABLE:
    __all__.extend([
        # Phase 3: Networks
        'Actor',
        'Critic',
        'GaussianActor',
        'SoftCritic',
        
        # Phase 3: Replay Buffers
        'ReplayBuffer',
        'PrioritizedReplayBuffer',
        
        # Phase 3: Agents
        'SACAgent',
        'SACPolicy',
        'TD3Agent',
        'TD3Policy',
    ])

