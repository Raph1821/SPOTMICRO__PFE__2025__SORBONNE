#!/usr/bin/env python3
"""
SpotMicro RL Training - SAC with Multi-Worker Support
=====================================================

Train SpotMicro robot using Soft Actor-Critic with parallel data collection:
- Multiple PyBullet environments in parallel processes
- GPU-accelerated neural network training  
- Efficient experience collection + training pipeline

Usage:
    # 4 parallel workers + GPU
    python scripts/spot_rl_train_sac_parallel.py --num_workers 4 --cuda --episodes 10000
    
    # Single worker (fallback)
    python scripts/spot_rl_train_sac_parallel.py --num_workers 1 --cuda
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
from spot_micro_rl.sac import SACAgent
from spot_micro_rl.parallel_env import ParallelEnv, DummyParallelEnv
from spot_micro_rl.spot_env_randomizer import SpotEnvRandomizer


def make_env(render=False, terrain_randomization=True):
    """Factory function to create environment"""
    return SpotMicroEnv(
        render=render,
        motor_model_enabled=True,
        env_randomizer=SpotEnvRandomizer(),
        terrain_randomization=terrain_randomization,
        max_timesteps=1000
    )


def evaluate_policy(env, agent, eval_episodes=5):
    """Evaluate current policy (single env)"""
    avg_reward = 0.
    avg_survival = 0.
    
    for _ in range(eval_episodes):
        state = env.reset()
        done = False
        episode_reward = 0
        steps = 0
        
        while not done:
            action = agent.select_action(state, deterministic=True)
            state, reward, done, _ = env.step(action)
            episode_reward += reward
            steps += 1
        
        avg_reward += episode_reward
        avg_survival += steps
    
    avg_reward /= eval_episodes
    avg_survival /= eval_episodes
    
    return avg_reward, avg_survival


def train_sac_parallel(args):
    """Main SAC training loop with parallel workers"""
    
    # Set random seeds
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    
    # Device selection
    device = torch.device("cuda" if torch.cuda.is_available() and args.cuda else "cpu")
    print(f"Using device: {device}")
    
    # Create parallel environments for training
    print(f"\n[1/5] Creating {args.num_workers} parallel environments...")
    
    if args.num_workers > 1:
        vec_env = ParallelEnv(
            env_fn=lambda: make_env(render=False, terrain_randomization=True),
            num_workers=args.num_workers
        )
    else:
        # Fallback to single env
        vec_env = DummyParallelEnv(
            env_fn=lambda: make_env(render=args.render, terrain_randomization=True),
            num_workers=1
        )
    
    # Create separate env for evaluation (single, no randomization)
    eval_env = make_env(render=False, terrain_randomization=False)
    
    state_dim = vec_env.observation_space.shape[0]
    action_dim = vec_env.action_space.shape[0]
    max_action = float(vec_env.action_space.high[0])
    
    print(f"✅ Environments ready")
    print(f"   Workers: {args.num_workers}")
    print(f"   State dim: {state_dim}")
    print(f"   Action dim: {action_dim}")
    print(f"   Max action: {max_action}")
    
    # Create SAC agent
    print("\n[2/5] Initializing SAC agent...")
    agent = SACAgent(
        state_dim=state_dim,
        action_dim=action_dim,
        max_action=max_action,
        hidden_dims=[256, 256],
        lr_actor=args.lr_actor,
        lr_critic=args.lr_critic,
        lr_alpha=args.lr_alpha,
        gamma=args.gamma,
        tau=args.tau,
        alpha=args.alpha,
        auto_entropy_tuning=args.auto_entropy_tuning,
        buffer_size=args.buffer_size,
        device=device
    )
    
    print(f"✅ SAC agent initialized")
    print(f"   Auto entropy tuning: {args.auto_entropy_tuning}")
    print(f"   Initial alpha: {agent.alpha.item():.4f}")
    
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
    
    log_file = os.path.join(log_dir, f'sac_parallel_{args.num_workers}w_{timestamp}.csv')
    checkpoint_dir = os.path.join(os.path.dirname(__file__), '..', 'models', 'checkpoints')
    os.makedirs(checkpoint_dir, exist_ok=True)
    
    # CSV header
    with open(log_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Episode', 'TotalSteps', 'AvgReward', 'AvgSurvival',
            'CriticLoss', 'ActorLoss', 'AlphaLoss', 'Alpha',
            'BufferSize', 'EvalReward', 'EvalSurvival', 'StepsPerSec'
        ])
    
    print(f"✅ Logging to: {log_file}")
    
    # Training loop
    print("\n[5/5] Starting SAC parallel training...")
    print("=" * 80)
    
    states = vec_env.reset()  # Shape (num_workers, state_dim)
    episode_rewards = np.zeros(args.num_workers)
    episode_steps = np.zeros(args.num_workers, dtype=int)
    episode_num = 0
    total_steps = 0
    
    start_time = time.time()
    
    while total_steps < args.max_timesteps:
        
        # Select actions for all workers
        if total_steps < args.start_timesteps:
            # Random actions for initial exploration
            actions = np.array([vec_env.action_space.sample() for _ in range(args.num_workers)])
        else:
            # Stochastic policy for each worker
            actions = np.array([
                agent.select_action(states[i], deterministic=False)
                for i in range(args.num_workers)
            ])
        
        # Execute actions in parallel
        next_states, rewards, dones, infos = vec_env.step(actions)
        
        # Store transitions in replay buffer
        for i in range(args.num_workers):
            agent.replay_buffer.add(
                states[i], actions[i], rewards[i], next_states[i], float(dones[i])
            )
            episode_rewards[i] += rewards[i]
            episode_steps[i] += 1
        
        states = next_states
        total_steps += args.num_workers
        
        # Train agent (after initial random exploration)
        critic_loss, actor_loss, alpha_loss = None, None, None
        if total_steps >= args.start_timesteps:
            # Train multiple times per step for efficiency
            for _ in range(args.num_workers):
                c_loss, a_loss, al_loss = agent.train(batch_size=args.batch_size)
                if c_loss is not None:
                    critic_loss, actor_loss, alpha_loss = c_loss, a_loss, al_loss
        
        # Check for episode completion
        for i in range(args.num_workers):
            if dones[i]:
                episode_num += 1
                
                # Logging
                elapsed = time.time() - start_time
                steps_per_sec = total_steps / elapsed
                
                print(f"Episode {episode_num:4d} (Worker {i}) | "
                      f"Steps: {episode_steps[i]:4d} | "
                      f"Reward: {episode_rewards[i]:8.2f} | "
                      f"Buffer: {len(agent.replay_buffer):6d} | "
                      f"Speed: {steps_per_sec:.1f} steps/s")
                
                # Evaluation
                eval_reward, eval_survival = None, None
                if episode_num % args.eval_freq == 0:
                    print(f"\n{'='*60}")
                    print(f"EVALUATION at episode {episode_num}")
                    print(f"{'='*60}")
                    eval_reward, eval_survival = evaluate_policy(eval_env, agent, args.eval_episodes)
                    print(f"Eval Reward: {eval_reward:.2f} | Eval Survival: {eval_survival:.1f}")
                    print(f"{'='*60}\n")
                
                # Save checkpoint
                if episode_num % args.save_freq == 0:
                    checkpoint_path = os.path.join(
                        checkpoint_dir,
                        f'sac_parallel_episode_{episode_num}.pth'
                    )
                    agent.save(checkpoint_path)
                    print(f"💾 Checkpoint saved: {checkpoint_path}")
                
                # Log to CSV
                with open(log_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        episode_num, total_steps, episode_rewards[i], episode_steps[i],
                        critic_loss if critic_loss else '',
                        actor_loss if actor_loss else '',
                        alpha_loss if alpha_loss else '',
                        agent.alpha.item() if hasattr(agent, 'alpha') else '',
                        len(agent.replay_buffer),
                        eval_reward if eval_reward else '',
                        eval_survival if eval_survival else '',
                        steps_per_sec
                    ])
                
                # Reset episode counters for this worker
                episode_rewards[i] = 0
                episode_steps[i] = 0
                
                # Check if max episodes reached
                if args.max_episodes > 0 and episode_num >= args.max_episodes:
                    print(f"\n✅ Reached max episodes ({args.max_episodes})")
                    vec_env.close()
                    eval_env.close()
                    return
    
    # Final save
    final_path = os.path.join(checkpoint_dir, f'sac_parallel_final_{timestamp}.pth')
    agent.save(final_path)
    print(f"\n💾 Final model saved: {final_path}")
    
    # Final evaluation
    print(f"\n{'='*60}")
    print("FINAL EVALUATION")
    print(f"{'='*60}")
    eval_reward, eval_survival = evaluate_policy(eval_env, agent, args.eval_episodes * 2)
    print(f"Final Eval Reward: {eval_reward:.2f}")
    print(f"Final Eval Survival: {eval_survival:.1f}")
    print(f"{'='*60}")
    
    vec_env.close()
    eval_env.close()
    print("\n✅ Training completed!")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Train SpotMicro with SAC (Parallel)')
    
    # Environment
    parser.add_argument('--render', action='store_true', help='Render environment (only worker 0)')
    parser.add_argument('--seed', type=int, default=0, help='Random seed')
    parser.add_argument('--num_workers', type=int, default=4, help='Number of parallel workers')
    
    # SAC hyperparameters
    parser.add_argument('--lr_actor', type=float, default=3e-4, help='Actor learning rate')
    parser.add_argument('--lr_critic', type=float, default=3e-4, help='Critic learning rate')
    parser.add_argument('--lr_alpha', type=float, default=3e-4, help='Temperature learning rate')
    parser.add_argument('--gamma', type=float, default=0.99, help='Discount factor')
    parser.add_argument('--tau', type=float, default=0.005, help='Target network update rate')
    parser.add_argument('--alpha', type=float, default=0.2, help='Initial temperature')
    parser.add_argument('--auto_entropy_tuning', action='store_true', default=True,
                       help='Auto-tune temperature')
    
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
    
    train_sac_parallel(args)
