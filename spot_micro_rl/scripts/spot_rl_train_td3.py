#!/usr/bin/env python3
"""
SpotMicro RL Training - TD3 Algorithm
======================================

Train SpotMicro robot using Twin Delayed DDPG with:
- Deterministic policy with exploration noise
- Twin Q-networks
- Delayed policy updates
- Target policy smoothing

Usage:
    python scripts/spot_rl_train_td3.py --episodes 10000 --eval_freq 100
"""

import sys
import os
import argparse
import numpy as np
import torch
import time
from datetime import datetime
import csv

# Add src directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from spot_micro_rl.spot_env import SpotMicroEnv
from spot_micro_rl.td3 import TD3Agent
from spot_micro_rl.spot_env_randomizer import SpotEnvRandomizer
from spot_micro_rl.motor_model import MotorModel


def evaluate_policy(env, agent, eval_episodes=5):
    """Evaluate current policy"""
    avg_reward = 0.
    avg_survival = 0.
    
    for _ in range(eval_episodes):
        state = env.reset()
        done = False
        episode_reward = 0
        steps = 0
        
        while not done:
            action = agent.select_action(state, noise=0.0)  # Deterministic
            state, reward, done, _ = env.step(action)
            episode_reward += reward
            steps += 1
        
        avg_reward += episode_reward
        avg_survival += steps
    
    avg_reward /= eval_episodes
    avg_survival /= eval_episodes
    
    return avg_reward, avg_survival


def train_td3(args):
    """Main TD3 training loop"""
    
    # Set random seeds
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    
    # Device selection
    device = torch.device("cuda" if torch.cuda.is_available() and args.cuda else "cpu")
    print(f"Using device: {device}")
    
    # Create environment
    print("\n[1/5] Creating environment...")
    motor_models = {
        f'motor_{i}': MotorModel(
            kt=0.05, r=1.0, l=0.001,
            friction_coeff=0.01, damping=0.001,
            backlash=0.01
        )
        for i in range(12)
    }
    
    randomizer = SpotEnvRandomizer()
    
    env = SpotMicroEnv(
        render=args.render,
        on_rack=False,
        height_field=False,
        draw_foot_path=False,
        env_randomizer=randomizer,
        motor_models=motor_models
    )
    
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    max_action = float(env.action_space.high[0])
    
    print(f"✅ Environment ready")
    print(f"   State dim: {state_dim}")
    print(f"   Action dim: {action_dim}")
    print(f"   Max action: {max_action}")
    
    # Create TD3 agent
    print("\n[2/5] Initializing TD3 agent...")
    agent = TD3Agent(
        state_dim=state_dim,
        action_dim=action_dim,
        max_action=max_action,
        hidden_dims=[400, 300],
        lr_actor=args.lr_actor,
        lr_critic=args.lr_critic,
        gamma=args.gamma,
        tau=args.tau,
        policy_noise=args.policy_noise,
        noise_clip=args.noise_clip,
        policy_freq=args.policy_freq,
        buffer_size=args.buffer_size,
        device=device
    )
    
    print(f"✅ TD3 agent initialized")
    
    # Load checkpoint if specified
    if args.load_checkpoint:
        print(f"\n[3/5] Loading checkpoint: {args.load_checkpoint}")
        agent.load(args.load_checkpoint, load_buffer=True)
    else:
        print("\n[3/5] Starting from scratch")
    
    # Setup logging
    print("\n[4/5] Setting up logging...")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = os.path.join(os.path.dirname(__file__), '..', 'results', 'training_logs')
    os.makedirs(log_dir, exist_ok=True)
    
    log_file = os.path.join(log_dir, f'td3_training_{timestamp}.csv')
    checkpoint_dir = os.path.join(os.path.dirname(__file__), '..', 'models', 'checkpoints')
    os.makedirs(checkpoint_dir, exist_ok=True)
    
    # CSV header
    with open(log_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Episode', 'Steps', 'Reward', 'Survival', 
            'CriticLoss', 'ActorLoss', 'BufferSize',
            'ExplorationNoise', 'EvalReward', 'EvalSurvival'
        ])
    
    print(f"✅ Logging to: {log_file}")
    
    # Training loop
    print("\n[5/5] Starting TD3 training...")
    print("=" * 80)
    
    state = env.reset()
    episode_reward = 0
    episode_steps = 0
    episode_num = 0
    
    # Exploration noise schedule
    exploration_noise = args.exploration_noise
    noise_decay = 0.9999
    min_noise = 0.01
    
    start_time = time.time()
    
    for total_steps in range(1, args.max_timesteps + 1):
        
        # Select action with exploration noise
        if total_steps < args.start_timesteps:
            # Random actions for initial exploration
            action = env.action_space.sample()
        else:
            action = agent.select_action(state, noise=exploration_noise)
        
        # Execute action
        next_state, reward, done, info = env.step(action)
        
        # Store transition in replay buffer
        agent.replay_buffer.add(state, action, reward, next_state, float(done))
        
        state = next_state
        episode_reward += reward
        episode_steps += 1
        
        # Train agent (after initial random exploration)
        if total_steps >= args.start_timesteps:
            critic_loss, actor_loss = agent.train(batch_size=args.batch_size)
        else:
            critic_loss, actor_loss = None, None
        
        # End of episode
        if done:
            episode_num += 1
            
            # Decay exploration noise
            exploration_noise = max(min_noise, exploration_noise * noise_decay)
            
            # Logging
            elapsed = time.time() - start_time
            steps_per_sec = total_steps / elapsed
            
            print(f"Episode {episode_num:4d} | Steps: {episode_steps:4d} | "
                  f"Reward: {episode_reward:8.2f} | "
                  f"Noise: {exploration_noise:.3f} | "
                  f"Buffer: {len(agent.replay_buffer):6d} | "
                  f"Speed: {steps_per_sec:.1f} steps/s")
            
            # Evaluation
            eval_reward, eval_survival = None, None
            if episode_num % args.eval_freq == 0:
                print(f"\n{'='*60}")
                print(f"EVALUATION at episode {episode_num}")
                print(f"{'='*60}")
                eval_reward, eval_survival = evaluate_policy(env, agent, args.eval_episodes)
                print(f"Eval Reward: {eval_reward:.2f} | Eval Survival: {eval_survival:.1f}")
                print(f"{'='*60}\n")
            
            # Save checkpoint
            if episode_num % args.save_freq == 0:
                checkpoint_path = os.path.join(
                    checkpoint_dir, 
                    f'td3_episode_{episode_num}.pth'
                )
                agent.save(checkpoint_path)
                print(f"💾 Checkpoint saved: {checkpoint_path}")
            
            # Log to CSV
            with open(log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    episode_num, episode_steps, episode_reward, episode_steps,
                    critic_loss if critic_loss else '',
                    actor_loss if actor_loss else '',
                    len(agent.replay_buffer),
                    exploration_noise,
                    eval_reward if eval_reward else '',
                    eval_survival if eval_survival else ''
                ])
            
            # Reset for next episode
            state = env.reset()
            episode_reward = 0
            episode_steps = 0
        
        # Check if max episodes reached
        if args.max_episodes > 0 and episode_num >= args.max_episodes:
            print(f"\n✅ Reached max episodes ({args.max_episodes})")
            break
    
    # Final save
    final_path = os.path.join(checkpoint_dir, f'td3_final_{timestamp}.pth')
    agent.save(final_path)
    print(f"\n💾 Final model saved: {final_path}")
    
    # Final evaluation
    print(f"\n{'='*60}")
    print("FINAL EVALUATION")
    print(f"{'='*60}")
    eval_reward, eval_survival = evaluate_policy(env, agent, args.eval_episodes * 2)
    print(f"Final Eval Reward: {eval_reward:.2f}")
    print(f"Final Eval Survival: {eval_survival:.1f}")
    print(f"{'='*60}")
    
    env.close()
    print("\n✅ Training completed!")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Train SpotMicro with TD3')
    
    # Environment
    parser.add_argument('--render', action='store_true', help='Render environment')
    parser.add_argument('--seed', type=int, default=0, help='Random seed')
    
    # TD3 hyperparameters
    parser.add_argument('--lr_actor', type=float, default=3e-4, help='Actor learning rate')
    parser.add_argument('--lr_critic', type=float, default=3e-4, help='Critic learning rate')
    parser.add_argument('--gamma', type=float, default=0.99, help='Discount factor')
    parser.add_argument('--tau', type=float, default=0.005, help='Target network update rate')
    parser.add_argument('--policy_noise', type=float, default=0.2, help='Target policy smoothing noise')
    parser.add_argument('--noise_clip', type=float, default=0.5, help='Target policy noise clip')
    parser.add_argument('--policy_freq', type=int, default=2, help='Policy update frequency')
    parser.add_argument('--exploration_noise', type=float, default=0.1, help='Initial exploration noise')
    
    # Training
    parser.add_argument('--max_episodes', type=int, default=10000, help='Max training episodes (0=unlimited)')
    parser.add_argument('--max_timesteps', type=int, default=1000000, help='Max training timesteps')
    parser.add_argument('--start_timesteps', type=int, default=10000, help='Random exploration steps')
    parser.add_argument('--batch_size', type=int, default=256, help='Batch size')
    parser.add_argument('--buffer_size', type=int, default=1000000, help='Replay buffer size')
    
    # Evaluation & Logging
    parser.add_argument('--eval_freq', type=int, default=100, help='Evaluation frequency (episodes)')
    parser.add_argument('--eval_episodes', type=int, default=5, help='Evaluation episodes')
    parser.add_argument('--save_freq', type=int, default=500, help='Checkpoint save frequency')
    parser.add_argument('--load_checkpoint', type=str, default='', help='Load checkpoint path')
    
    # Hardware
    parser.add_argument('--cuda', action='store_true', help='Use CUDA if available')
    
    args = parser.parse_args()
    
    train_td3(args)
