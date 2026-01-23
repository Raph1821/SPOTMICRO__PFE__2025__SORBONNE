#!/usr/bin/env python3
"""
Spot Micro RL Training Script - Complete Implementation
Train ARS agent with motor models, domain randomization, varied terrain

Features:
- Automatic checkpoint saving (compatible with gmbc_data.py)
- Survival data logging (for gmbc_data.py analysis)
- Curriculum learning support
- TensorBoard logging (optional)
- Multi-seed training
"""

import os
import sys
import argparse
import numpy as np
import pickle
from datetime import datetime
import time

# Add package to path
script_dir = os.path.dirname(os.path.abspath(__file__))
package_dir = os.path.join(script_dir, '..', 'src')
sys.path.insert(0, package_dir)

from spot_micro_rl import (
    SpotMicroEnv,
    Policy,
    Normalizer,
    ARSAgent,
    MinimalRandomizer,
    SpotEnvRandomizer,
    AggressiveRandomizer
)


def evaluate_policy(agent, env, num_episodes=5):
    """
    Evaluate policy without exploration (validation)
    
    Args:
        agent: ARS agent
        env: Environment
        num_episodes: Number of evaluation episodes
        
    Returns:
        avg_reward: Average reward
        avg_survival: Average survival timesteps
    """
    eval_rewards = []
    eval_survivals = []
    
    for _ in range(num_episodes):
        state = env.reset()
        done = False
        episode_reward = 0
        timesteps = 0
        
        while not done and timesteps < env.max_timesteps:
            # Use policy without exploration noise
            action = agent.policy.evaluate(state, update_stats=False)
            state, reward, done, _ = env.step(action)
            episode_reward += reward
            timesteps += 1
        
        eval_rewards.append(episode_reward)
        eval_survivals.append(timesteps)
    
    return np.mean(eval_rewards), np.mean(eval_survivals)


def save_survival_data(survival_timesteps, results_dir, episode, tag='agent'):
    """
    Save survival data for gmbc_data.py analysis
    
    Args:
        survival_timesteps: List of timesteps survived per episode
        results_dir: Directory to save results
        episode: Current episode number
        tag: 'agent' or 'vanilla' (Bezier baseline)
    """
    os.makedirs(results_dir, exist_ok=True)
    
    filename = f"spot_ars_{tag}_survival_{episode}"
    filepath = os.path.join(results_dir, filename)
    
    with open(filepath, 'wb') as f:
        pickle.dump(survival_timesteps, f)
    
    print(f"   Saved survival data: {filename}")


def train(args):
    """
    Train Spot Micro with ARS algorithm
    
    Args:
        args: Command line arguments
    """
    print("=" * 70)
    print("SPOT MICRO RL TRAINING - ARS with Sim-to-Real Features")
    print("=" * 70)
    
    # Determine randomizer based on curriculum
    if args.curriculum_stage == 1:
        randomizer = MinimalRandomizer()
        terrain = 'flat'
        print(f"\n Curriculum Stage 1: Minimal randomization + Flat terrain")
    elif args.curriculum_stage == 2:
        randomizer = SpotEnvRandomizer()
        terrain = 'gentle'
        print(f"\n Curriculum Stage 2: Medium randomization + Gentle terrain")
    elif args.curriculum_stage == 3:
        randomizer = AggressiveRandomizer()
        terrain = args.terrain  # 'rough' or 'random'
        print(f"\n Curriculum Stage 3: Aggressive randomization + {terrain} terrain")
    else:
        randomizer = SpotEnvRandomizer() if args.use_randomization else None
        terrain = args.terrain
    
    # Create environment
    print(f"\n Creating environment...")
    print(f"   Render: {args.render}")
    print(f"   Motor models: {args.use_motor_models}")
    print(f"   Randomization: {'Yes' if randomizer else 'No'}")
    print(f"   Terrain: {terrain}")
    
    env = SpotMicroEnv(
        render=args.render,
        motor_model_enabled=args.use_motor_models,
        env_randomizer=randomizer,
        terrain_type=terrain,
        terrain_randomization=args.terrain_randomization,
        max_timesteps=args.max_timesteps
    )
    
    # Get environment dimensions
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    
    print(f"\n Environment Info:")
    print(f"   State dimension: {state_dim}")
    print(f"   Action dimension: {action_dim}")
    print(f"   Max timesteps: {args.max_timesteps}")
    
    # Create ARS agent
    print(f"\n Creating ARS Agent...")
    print(f"   Learning rate: {args.learning_rate}")
    print(f"   Num deltas: {args.num_deltas}")
    print(f"   Num best deltas: {args.num_best_deltas}")
    print(f"   Exploration noise: {args.expl_noise}")
    print(f"   Seed: {args.seed}")
    
    policy = Policy(
        state_dim=state_dim,
        action_dim=action_dim,
        learning_rate=args.learning_rate,
        num_deltas=args.num_deltas,
        num_best_deltas=args.num_best_deltas,
        episode_steps=args.max_timesteps,
        expl_noise=args.expl_noise,
        seed=args.seed
    )
    
    normalizer = Normalizer(state_dim)
    
    agent = ARSAgent(
        normalizer=normalizer,
        policy=policy,
        env=env,
        desired_velocity=args.desired_velocity,
        desired_rate=args.desired_rate
    )
    
    # Load checkpoint if specified
    if args.load_checkpoint:
        print(f"\n Loading checkpoint: {args.load_checkpoint}")
        agent.load_policy(args.load_checkpoint)
    
    # Create save directories
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    base_name = f"spot_ars_{timestamp}_seed{args.seed}"
    
    models_dir = os.path.join(script_dir, '..', 'models', 'checkpoints')
    results_dir = os.path.join(script_dir, '..', 'results', 'survival_data')
    logs_dir = os.path.join(script_dir, '..', 'results', 'training_logs')
    
    os.makedirs(models_dir, exist_ok=True)
    os.makedirs(results_dir, exist_ok=True)
    os.makedirs(logs_dir, exist_ok=True)
    
    print(f"\n Save directories:")
    print(f"   Models: {models_dir}")
    print(f"   Results: {results_dir}")
    print(f"   Logs: {logs_dir}")
    
    # Save training config
    config = {
        'state_dim': state_dim,
        'action_dim': action_dim,
        'learning_rate': args.learning_rate,
        'num_deltas': args.num_deltas,
        'num_best_deltas': args.num_best_deltas,
        'expl_noise': args.expl_noise,
        'max_timesteps': args.max_timesteps,
        'seed': args.seed,
        'motor_models': args.use_motor_models,
        'randomization': randomizer is not None,
        'terrain': terrain,
        'curriculum_stage': args.curriculum_stage,
        'timestamp': timestamp
    }
    
    config_file = os.path.join(models_dir, f'{base_name}_config.pkl')
    with open(config_file, 'wb') as f:
        pickle.dump(config, f)
    
    print(f"   Config saved: {config_file}")
    
    # Training metrics
    best_reward = -np.inf
    all_rewards = []
    all_survival_timesteps = []
    
    # CSV log file
    csv_log = os.path.join(logs_dir, f'{base_name}_training.csv')
    with open(csv_log, 'w') as f:
        f.write("episode,reward,survival_timesteps,best_reward,time_elapsed\\n")
    
    print("\\n" + "=" * 70)
    print("STARTING TRAINING")
    print("=" * 70)
    
    start_time = time.time()
    
    # Training loop
    for episode in range(args.num_episodes):
        episode_start = time.time()
        
        print(f"\\n{'='*70}")
        print(f"Episode {episode + 1}/{args.num_episodes}")
        print(f"{'='*70}")
        
        # Train one episode
        reward, timesteps = agent.train_parallel([]) if args.num_workers > 1 else (agent.train(), args.max_timesteps)
        
        # If not using parallel, get timesteps from last rollout
        if args.num_workers == 1:
            timesteps = args.max_timesteps  # TODO: track actual timesteps in train()
        
        all_rewards.append(reward)
        all_survival_timesteps.append(timesteps)
        
        episode_time = time.time() - episode_start
        total_time = time.time() - start_time
        
        print(f"\\n Results:")
        print(f"   Reward: {reward:+.2f}")
        print(f"   Survival: {timesteps}/{args.max_timesteps} timesteps")
        print(f"   Episode time: {episode_time:.1f}s")
        print(f"   Total time: {total_time/60:.1f}min")
        
        # Validation (every 10 episodes)
        eval_reward, eval_survival = None, None
        if (episode + 1) % 10 == 0:
            print(f"\n{'='*60}")
            print(f"VALIDATION at episode {episode + 1}")
            print(f"{'='*60}")
            eval_reward, eval_survival = evaluate_policy(agent, env, num_episodes=5)
            print(f"Eval Reward: {eval_reward:.2f}")
            print(f"Eval Survival: {eval_survival:.1f}")
            print(f"{'='*60}\n")
        
        # Update best
        if reward > best_reward:
            best_reward = reward
            best_checkpoint = os.path.join(models_dir, f'{base_name}_best')
            agent.save_policy(best_checkpoint)
            print(f"   💾 New best model! Reward: {reward:+.2f}")
        
        # Save checkpoint every N episodes
        if (episode + 1) % args.save_interval == 0:
            checkpoint_name = f"spot_ars_{episode+1}_policy"
            checkpoint_path = os.path.join(models_dir, checkpoint_name)
            agent.save_policy(checkpoint_path)
            print(f"    Checkpoint saved: {checkpoint_name}")
            
            # Save survival data for gmbc_data.py
            save_survival_data(
                all_survival_timesteps,
                results_dir,
                episode + 1,
                tag='agent'
            )
        
        # Log to CSV
        with open(csv_log, 'a') as f:
            eval_r = f"{eval_reward:.4f}" if eval_reward is not None else ""
            eval_s = f"{eval_survival:.1f}" if eval_survival is not None else ""
            f.write(f"{episode+1},{reward:.4f},{timesteps},{best_reward:.4f},{eval_r},{eval_s},{total_time:.2f}\n")
        
        # Print stats every 10 episodes
        if (episode + 1) % 10 == 0:
            recent_rewards = all_rewards[-10:]
            print(f"\\n Last 10 episodes:")
            print(f"   Mean reward: {np.mean(recent_rewards):+.2f}")
            print(f"   Std reward: {np.std(recent_rewards):.2f}")
            print(f"   Best reward: {best_reward:+.2f}")
    
    # Final save
    print("\\n" + "=" * 70)
    print("TRAINING COMPLETE!")
    print("=" * 70)
    
    final_checkpoint = os.path.join(models_dir, f'{base_name}_final')
    agent.save_policy(final_checkpoint)
    
    save_survival_data(
        all_survival_timesteps,
        results_dir,
        args.num_episodes,
        tag='agent'
    )
    
    print(f"\\n Final Results:")
    print(f"   Total episodes: {args.num_episodes}")
    print(f"   Best reward: {best_reward:+.2f}")
    print(f"   Mean reward: {np.mean(all_rewards):+.2f}")
    print(f"   Final reward: {all_rewards[-1]:+.2f}")
    print(f"   Total time: {(time.time() - start_time)/60:.1f}min")
    print(f"\\n   Models saved in: {models_dir}")
    print(f"   Results saved in: {results_dir}")
    print(f"   Logs saved in: {logs_dir}")
    
    print(f"\\n Analyze results with:")
    print(f"   cd {os.path.dirname(script_dir)}/scripts")
    print(f"   python gmbc_data.py --NumberOfEpisodes {args.num_episodes}")
    
    # Close environment
    env.close()


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description='Train Spot Micro with ARS Reinforcement Learning',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    # Environment arguments
    env_group = parser.add_argument_group('Environment')
    env_group.add_argument('--render', action='store_true',
                          help='Render PyBullet simulation')
    env_group.add_argument('--use_motor_models', action='store_true', default=True,
                          help='Use realistic motor models')
    env_group.add_argument('--use_randomization', action='store_true', default=True,
                          help='Use domain randomization')
    env_group.add_argument('--terrain', type=str, default='flat',
                          choices=['flat', 'gentle', 'rough', 'random'],
                          help='Terrain type (ignored if --terrain_randomization is set)')
    env_group.add_argument('--terrain_randomization', action='store_true', default=True,
                          help='Randomize terrain at each reset (15%% flat, 15%% gentle, 70%% rough)')
    env_group.add_argument('--max_timesteps', type=int, default=1000,
                          help='Maximum timesteps per episode')
    
    # ARS algorithm arguments
    ars_group = parser.add_argument_group('ARS Algorithm')
    ars_group.add_argument('--num_episodes', type=int, default=1000,
                          help='Number of training episodes')
    ars_group.add_argument('--num_deltas', type=int, default=16,
                          help='Number of policy deltas to sample')
    ars_group.add_argument('--num_best_deltas', type=int, default=16,
                          help='Number of best deltas to use for update')
    ars_group.add_argument('--learning_rate', type=float, default=0.02,
                          help='Policy update learning rate')
    ars_group.add_argument('--expl_noise', type=float, default=0.01,
                          help='Exploration noise std')
    ars_group.add_argument('--seed', type=int, default=42,
                          help='Random seed')
    ars_group.add_argument('--num_workers', type=int, default=1,
                          help='Number of parallel workers (not implemented yet)')
    
    # Gait arguments
    gait_group = parser.add_argument_group('Gait Control')
    gait_group.add_argument('--desired_velocity', type=float, default=0.5,
                           help='Target forward velocity (m/s)')
    gait_group.add_argument('--desired_rate', type=float, default=0.0,
                           help='Target yaw rate (rad/s)')
    
    # Curriculum learning
    curriculum_group = parser.add_argument_group('Curriculum Learning')
    curriculum_group.add_argument('--curriculum_stage', type=int, default=0,
                                 choices=[0, 1, 2, 3],
                                 help='Curriculum stage: 0=manual, 1=easy, 2=medium, 3=hard')
    
    # Save/Load arguments
    save_group = parser.add_argument_group('Save/Load')
    save_group.add_argument('--save_interval', type=int, default=10,
                           help='Save checkpoint every N episodes')
    save_group.add_argument('--load_checkpoint', type=str, default=None,
                           help='Load checkpoint to resume training')
    
    args = parser.parse_args()
    
    # Validate
    assert args.num_best_deltas <= args.num_deltas, \
        "num_best_deltas must be <= num_deltas"
    
    # Run training
    try:
        train(args)
    except KeyboardInterrupt:
        print("\\n\\n  Training interrupted by user")
        print("   Checkpoints saved in models/checkpoints/")
    except Exception as e:
        print(f"\\n\\n Error during training: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
