#!/usr/bin/env python3
"""
SpotMicro RL - Test Protocol
=============================

Test rigoureux des modèles entraînés sur conditions jamais vues.

Usage:
    # Test ARS
    python scripts/spot_rl_test.py --policy models/checkpoints/spot_ars_best.pkl --algorithm ars
    
    # Test SAC
    python scripts/spot_rl_test.py --policy models/checkpoints/sac_best.pth --algorithm sac --cuda
    
    # Test TD3
    python scripts/spot_rl_test.py --policy models/checkpoints/td3_best.pth --algorithm td3 --cuda
"""

import sys
import os
import argparse
import numpy as np
import torch
import pickle
import json
from datetime import datetime
from pathlib import Path

# Add src to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from spot_micro_rl import (
    SpotMicroEnv,
    Policy,
    Normalizer,
    ARSAgent,
    SACAgent,
    TD3Agent,
    SpotEnvRandomizer,
    AggressiveRandomizer,
    MotorModel
)


class TestProtocol:
    """Protocol de test rigoureux pour évaluation finale"""
    
    def __init__(self, algorithm, policy_path, device='cpu'):
        """
        Args:
            algorithm: 'ars', 'sac', or 'td3'
            policy_path: Chemin vers le checkpoint
            device: 'cpu' or 'cuda'
        """
        self.algorithm = algorithm.lower()
        self.policy_path = policy_path
        self.device = device
        self.agent = None
        
        # Test configurations
        self.test_configs = [
            # Config 1: Flat terrain, no randomization (baseline)
            {
                'name': 'Baseline',
                'terrain': 'flat',
                'randomizer': None,
                'num_episodes': 20,
                'seeds': [100, 200, 300]
            },
            # Config 2: Gentle terrain with standard randomization
            {
                'name': 'Gentle_Terrain',
                'terrain': 'gentle',
                'randomizer': SpotEnvRandomizer(),
                'num_episodes': 20,
                'seeds': [400, 500, 600]
            },
            # Config 3: Rough terrain with aggressive randomization
            {
                'name': 'Rough_Aggressive',
                'terrain': 'rough',
                'randomizer': AggressiveRandomizer(),
                'num_episodes': 30,
                'seeds': [700, 800, 900, 1000]
            },
            # Config 4: Random terrain (stress test)
            {
                'name': 'Stress_Test',
                'terrain': 'random',
                'randomizer': AggressiveRandomizer(),
                'num_episodes': 30,
                'seeds': [1100, 1200, 1300, 1400]
            }
        ]
        
        self.results = {
            'algorithm': self.algorithm,
            'policy_path': str(self.policy_path),
            'test_date': datetime.now().isoformat(),
            'configs': []
        }
    
    def load_agent(self):
        """Charge l'agent depuis le checkpoint"""
        print(f"\n{'='*70}")
        print(f"LOADING {self.algorithm.upper()} AGENT")
        print(f"{'='*70}")
        print(f"Policy: {self.policy_path}")
        
        if self.algorithm == 'ars':
            # Load ARS policy
            with open(self.policy_path, 'rb') as f:
                data = pickle.load(f)
            
            state_dim = data['state_dim']
            action_dim = data['action_dim']
            
            policy = Policy(
                state_dim=state_dim,
                action_dim=action_dim,
                learning_rate=0.02,
                num_deltas=16,
                num_best_deltas=16
            )
            policy.theta = data['theta']
            
            normalizer = Normalizer(state_dim)
            normalizer.n = data.get('normalizer_n', 0)
            normalizer.mean = data.get('normalizer_mean', np.zeros(state_dim))
            normalizer.mean_diff = data.get('normalizer_mean_diff', np.zeros(state_dim))
            normalizer.var = data.get('normalizer_var', np.ones(state_dim))
            
            # Create dummy env for agent initialization
            env = SpotMicroEnv(render=False, max_timesteps=1000)
            self.agent = ARSAgent(normalizer, policy, env)
            env.close()
            
            print(f"✅ ARS policy loaded")
            print(f"   State dim: {state_dim}")
            print(f"   Action dim: {action_dim}")
            
        elif self.algorithm == 'sac':
            # Load SAC agent
            checkpoint = torch.load(self.policy_path, map_location=self.device)
            
            state_dim = checkpoint['state_dim']
            action_dim = checkpoint['action_dim']
            max_action = checkpoint['max_action']
            
            self.agent = SACAgent(
                state_dim=state_dim,
                action_dim=action_dim,
                max_action=max_action,
                device=self.device
            )
            self.agent.load(self.policy_path, load_buffer=False)
            
            print(f"✅ SAC agent loaded")
            print(f"   State dim: {state_dim}")
            print(f"   Action dim: {action_dim}")
            print(f"   Device: {self.device}")
            
        elif self.algorithm == 'td3':
            # Load TD3 agent
            checkpoint = torch.load(self.policy_path, map_location=self.device)
            
            state_dim = checkpoint['state_dim']
            action_dim = checkpoint['action_dim']
            max_action = checkpoint['max_action']
            
            self.agent = TD3Agent(
                state_dim=state_dim,
                action_dim=action_dim,
                max_action=max_action,
                device=self.device
            )
            self.agent.load(self.policy_path, load_buffer=False)
            
            print(f"✅ TD3 agent loaded")
            print(f"   State dim: {state_dim}")
            print(f"   Action dim: {action_dim}")
            print(f"   Device: {self.device}")
        
        else:
            raise ValueError(f"Unknown algorithm: {self.algorithm}")
    
    def test_configuration(self, config):
        """Test agent on specific configuration"""
        print(f"\n{'='*70}")
        print(f"TEST: {config['name']}")
        print(f"{'='*70}")
        print(f"Terrain: {config['terrain']}")
        print(f"Randomizer: {type(config['randomizer']).__name__ if config['randomizer'] else 'None'}")
        print(f"Episodes: {config['num_episodes']}")
        print(f"Seeds: {config['seeds']}")
        
        rewards = []
        survivals = []
        velocities = []
        
        for seed in config['seeds']:
            print(f"\n  Testing with seed {seed}...")
            
            # Create environment
            env = SpotMicroEnv(
                render=False,
                terrain_type=config['terrain'],
                terrain_randomization=False,  # Fixed terrain type
                env_randomizer=config['randomizer'],
                max_timesteps=1000
            )
            
            # Set seed
            env.seed(seed)
            np.random.seed(seed)
            if self.algorithm in ['sac', 'td3']:
                torch.manual_seed(seed)
            
            # Run episodes
            for ep in range(config['num_episodes'] // len(config['seeds'])):
                state = env.reset()
                done = False
                episode_reward = 0
                timesteps = 0
                positions = []
                
                while not done and timesteps < env.max_timesteps:
                    # Select action (deterministic for test)
                    if self.algorithm == 'ars':
                        action = self.agent.policy.evaluate(state, update_stats=False)
                    elif self.algorithm == 'sac':
                        action = self.agent.select_action(state, deterministic=True)
                    elif self.algorithm == 'td3':
                        action = self.agent.select_action(state, noise=0.0)
                    
                    # Step
                    state, reward, done, _ = env.step(action)
                    episode_reward += reward
                    timesteps += 1
                    positions.append(state[40:43])  # x, y, z position
                
                # Compute metrics
                rewards.append(episode_reward)
                survivals.append(timesteps)
                
                # Average forward velocity
                if len(positions) > 1:
                    positions = np.array(positions)
                    distances = np.linalg.norm(np.diff(positions[:, :2], axis=0), axis=1)
                    avg_velocity = np.mean(distances) / env.dt
                    velocities.append(avg_velocity)
            
            env.close()
        
        # Compute statistics
        result = {
            'name': config['name'],
            'terrain': config['terrain'],
            'num_episodes': len(rewards),
            'reward_mean': float(np.mean(rewards)),
            'reward_std': float(np.std(rewards)),
            'reward_min': float(np.min(rewards)),
            'reward_max': float(np.max(rewards)),
            'survival_mean': float(np.mean(survivals)),
            'survival_std': float(np.std(survivals)),
            'survival_min': int(np.min(survivals)),
            'survival_max': int(np.max(survivals)),
            'success_rate': float(np.mean([s >= 900 for s in survivals]) * 100),  # 90% of max
            'velocity_mean': float(np.mean(velocities)) if velocities else 0.0,
            'velocity_std': float(np.std(velocities)) if velocities else 0.0
        }
        
        # Print results
        print(f"\n  Results:")
        print(f"    Reward: {result['reward_mean']:.2f} ± {result['reward_std']:.2f} (min: {result['reward_min']:.2f}, max: {result['reward_max']:.2f})")
        print(f"    Survival: {result['survival_mean']:.1f} ± {result['survival_std']:.1f} (min: {result['survival_min']}, max: {result['survival_max']})")
        print(f"    Success Rate: {result['success_rate']:.1f}%")
        print(f"    Avg Velocity: {result['velocity_mean']:.3f} ± {result['velocity_std']:.3f} m/s")
        
        return result
    
    def run_full_test(self):
        """Run complete test protocol"""
        print(f"\n{'#'*70}")
        print(f"# SPOTMICRO RL - FULL TEST PROTOCOL")
        print(f"# Algorithm: {self.algorithm.upper()}")
        print(f"{'#'*70}")
        
        # Load agent
        self.load_agent()
        
        # Run all test configurations
        for config in self.test_configs:
            result = self.test_configuration(config)
            self.results['configs'].append(result)
        
        # Aggregate results
        self.compute_aggregate_stats()
        
        # Save results
        self.save_results()
        
        # Print summary
        self.print_summary()
    
    def compute_aggregate_stats(self):
        """Compute overall statistics"""
        all_rewards = []
        all_survivals = []
        
        for config_result in self.results['configs']:
            # Approximate reconstruction from mean/std (not exact but good enough)
            all_rewards.append(config_result['reward_mean'])
            all_survivals.append(config_result['survival_mean'])
        
        self.results['overall'] = {
            'avg_reward': float(np.mean(all_rewards)),
            'avg_survival': float(np.mean(all_survivals)),
            'total_episodes_tested': sum(c['num_episodes'] for c in self.results['configs'])
        }
    
    def save_results(self):
        """Save test results to JSON"""
        results_dir = Path(__file__).parent.parent / 'results' / 'test_results'
        results_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.algorithm}_test_{timestamp}.json"
        filepath = results_dir / filename
        
        with open(filepath, 'w') as f:
            json.dump(self.results, f, indent=2)
        
        print(f"\n✅ Test results saved: {filepath}")
    
    def print_summary(self):
        """Print test summary"""
        print(f"\n{'='*70}")
        print(f"TEST SUMMARY - {self.algorithm.upper()}")
        print(f"{'='*70}")
        
        print(f"\nTotal Episodes Tested: {self.results['overall']['total_episodes_tested']}")
        print(f"Average Reward (all configs): {self.results['overall']['avg_reward']:.2f}")
        print(f"Average Survival (all configs): {self.results['overall']['avg_survival']:.1f} timesteps")
        
        print(f"\n{'Config':<20} {'Success%':<12} {'Reward':<15} {'Survival':<15}")
        print(f"{'-'*70}")
        
        for config in self.results['configs']:
            print(f"{config['name']:<20} {config['success_rate']:>6.1f}%     "
                  f"{config['reward_mean']:>7.2f}±{config['reward_std']:<5.2f} "
                  f"{config['survival_mean']:>7.1f}±{config['survival_std']:<5.1f}")
        
        print(f"\n{'='*70}")
        
        # Robustness assessment
        success_rates = [c['success_rate'] for c in self.results['configs']]
        avg_success = np.mean(success_rates)
        
        print(f"\n🎯 ROBUSTNESS ASSESSMENT:")
        if avg_success >= 80:
            print(f"   ✅ EXCELLENT ({avg_success:.1f}% avg success)")
        elif avg_success >= 60:
            print(f"   ⚠️ GOOD ({avg_success:.1f}% avg success)")
        elif avg_success >= 40:
            print(f"   ⚠️ MODERATE ({avg_success:.1f}% avg success)")
        else:
            print(f"   ❌ POOR ({avg_success:.1f}% avg success)")


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description='Test SpotMicro RL trained models',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    parser.add_argument('--policy', type=str, required=True,
                       help='Path to policy checkpoint (.pkl for ARS, .pth for SAC/TD3)')
    parser.add_argument('--algorithm', type=str, required=True,
                       choices=['ars', 'sac', 'td3'],
                       help='Algorithm used to train the policy')
    parser.add_argument('--cuda', action='store_true',
                       help='Use CUDA (for SAC/TD3)')
    
    args = parser.parse_args()
    
    # Determine device
    if args.algorithm in ['sac', 'td3'] and args.cuda:
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        if device == 'cpu':
            print("⚠️ CUDA requested but not available, using CPU")
    else:
        device = 'cpu'
    
    # Run test protocol
    tester = TestProtocol(
        algorithm=args.algorithm,
        policy_path=args.policy,
        device=device
    )
    
    tester.run_full_test()


if __name__ == '__main__':
    main()
