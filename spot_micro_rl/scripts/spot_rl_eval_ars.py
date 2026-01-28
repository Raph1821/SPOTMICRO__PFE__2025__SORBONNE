#!/usr/bin/env python3
"""
Spot Micro RL Evaluation Script
Evaluate trained ARS policy
"""

import argparse
import numpy as np
import pickle

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from spot_micro_rl.ars import ARSAgent
from spot_micro_rl.spot_env import make_env


def evaluate(args):
    """
    Evaluate trained Spot Micro policy
    
    Args:
        args: Command line arguments
    """
    print("=" * 60)
    print("Spot Micro RL Evaluation")
    print("=" * 60)
    
    # Load training config
    print(f"\nLoading model from: {args.model_path}")
    with open(args.model_path.replace('.pkl', '_config.pkl'), 'rb') as f:
        config = pickle.load(f)
    
    print(f"Model configuration:")
    for key, value in config.items():
        print(f"  {key}: {value}")
    
    # Create environment
    print(f"\nCreating environment...")
    print(f"  Render: {args.render}")
    env = make_env(render=args.render, use_ros=args.use_ros)
    
    # Create agent
    print(f"\nCreating agent...")
    agent = ARSAgent(
        state_dim=config['state_dim'],
        action_dim=config['action_dim'],
        learning_rate=config['learning_rate'],
        num_deltas=config['num_deltas'],
        num_best_deltas=config['num_best_deltas'],
        episode_steps=config.get('max_timesteps', args.rollout_length),
        expl_noise=config.get('expl_noise', 0.01),
        seed=config['seed']
    )
    
    # Load trained policy
    agent.load(args.model_path)
    print("✓ Model loaded successfully")
    
    print("\n" + "=" * 60)
    print("Starting Evaluation")
    print("=" * 60)
    
    # Run evaluation episodes
    episode_rewards = []
    
    for episode in range(args.num_episodes):
        print(f"\nEpisode {episode + 1}/{args.num_episodes}")
        
        state = env.reset()
        episode_reward = 0
        step = 0
        
        while step < args.rollout_length:
            # Get action from policy (no exploration noise)
            action = agent.policy.evaluate(state, delta=None, direction=None)
            
            # Take step
            state, reward, done, info = env.step(action)
            episode_reward += reward
            step += 1
            
            if done:
                break
        
        episode_rewards.append(episode_reward)
        print(f"  Total Reward: {episode_reward:.2f}")
        print(f"  Steps: {step}")
    
    # Print statistics
    print("\n" + "=" * 60)
    print("Evaluation Results")
    print("=" * 60)
    print(f"Number of episodes: {args.num_episodes}")
    print(f"Average Reward: {np.mean(episode_rewards):.2f}")
    print(f"Std Reward: {np.std(episode_rewards):.2f}")
    print(f"Min Reward: {np.min(episode_rewards):.2f}")
    print(f"Max Reward: {np.max(episode_rewards):.2f}")
    
    # Close environment
    env.close()


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description='Evaluate trained Spot Micro RL policy'
    )
    
    # Model arguments
    parser.add_argument('model_path', type=str,
                       help='Path to trained model (.pkl file)')
    
    # Environment arguments
    parser.add_argument('--render', action='store_true',
                       help='Render simulation')
    parser.add_argument('--use_ros', action='store_true',
                       help='Use ROS topics')
    
    # Evaluation arguments
    parser.add_argument('--num_episodes', type=int, default=10,
                       help='Number of evaluation episodes (default: 10)')
    parser.add_argument('--rollout_length', type=int, default=1000,
                       help='Episode length (default: 1000)')
    
    args = parser.parse_args()
    
    # Run evaluation
    evaluate(args)


if __name__ == '__main__':
    main()
