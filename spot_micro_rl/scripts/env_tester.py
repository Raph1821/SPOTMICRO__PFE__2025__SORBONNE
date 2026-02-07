#!/usr/bin/env python3
"""
Environment Tester for SpotMicro RL
====================================

Test visual de l'initialisation du robot et des poses.
Permet de vérifier que le robot démarre dans une pose stable.

Usage:
    python scripts/env_tester.py
    python scripts/env_tester.py --episodes 5
"""

import sys
import os
import argparse
import time
import numpy as np

# Add src to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

from spot_env import SpotMicroEnv


def test_env(num_episodes=3, render=True, max_steps=500):
    """
    Test l'environnement avec visualisation PyBullet.
    
    Args:
        num_episodes: Nombre d'épisodes de test
        render: Activer le rendu PyBullet GUI
        max_steps: Nombre max de steps par épisode
    """
    print("=" * 70)
    print("SPOT MICRO RL - ENVIRONMENT TESTER")
    print("=" * 70)
    print("\nCe script teste l'initialisation et la stabilité du robot.")
    print("Vérifiez visuellement que le robot :")
    print("  ✓ Démarre debout (pas couché ou tombé)")
    print("  ✓ Les pattes sont pliées de manière réaliste")
    print("  ✓ Le corps est horizontal à ~15-20cm du sol")
    print("=" * 70)
    
    # Créer l'environnement
    print(f"\n[1/3] Création de l'environnement...")
    env = SpotMicroEnv(
        render=render,
        motor_model_enabled=False,  # Pas de motor models pour test rapide
        env_randomizer=None,        # Pas de randomization pour test
        terrain_type='flat',        # Terrain plat pour test
        max_timesteps=max_steps
    )
    
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    
    print(f"✅ Environnement créé")
    print(f"   State dim: {state_dim}")
    print(f"   Action dim: {action_dim}")
    
    # Test de reset
    print(f"\n[2/3] Test de reset initial...")
    state = env.reset()
    print(f"✅ Reset réussi")
    print(f"   Observation shape: {state.shape}")
    print(f"   Position du corps (x, y, z): {state[40:43]}")
    
    input("\nAppuyez sur ENTRÉE pour voir le robot tenir sa pose pendant 5 secondes...")
    
    # Tenir la pose (action nulle)
    print(f"\n[3/3] Test de maintien de pose (5 secondes)...")
    for step in range(250):  # 5 sec à 50Hz
        action = np.zeros(action_dim)  # Action nulle = tenir la pose
        state, reward, done, info = env.step(action)
        
        if step % 50 == 0:
            print(f"  Step {step}/250 - Position Z: {state[42]:.3f}m - Reward: {reward:.2f}")
        
        if done:
            print(f"⚠️  Robot tombé à step {step}!")
            break
        
        time.sleep(0.02)  # 50Hz
    
    if not done:
        print(f"✅ Robot stable pendant 5 secondes!")
    
    input("\nAppuyez sur ENTRÉE pour tester plusieurs resets...")
    
    # Test de plusieurs épisodes
    print(f"\n{'='*70}")
    print(f"TEST DE PLUSIEURS ÉPISODES")
    print(f"{'='*70}")
    
    for ep in range(num_episodes):
        print(f"\n📍 Épisode {ep+1}/{num_episodes}")
        state = env.reset()
        
        episode_reward = 0
        survived_steps = 0
        
        for step in range(max_steps):
            # Action aléatoire petite (pour tester stabilité)
            action = np.random.randn(action_dim) * 0.1
            action = np.clip(action, -1.0, 1.0)
            
            state, reward, done, info = env.step(action)
            episode_reward += reward
            survived_steps = step + 1
            
            if done:
                break
        
        print(f"  Récompense totale: {episode_reward:.2f}")
        print(f"  Steps survécus: {survived_steps}/{max_steps}")
        
        if survived_steps < 50:
            print(f"  ⚠️  Robot instable (tombé rapidement)")
        elif survived_steps < 200:
            print(f"  ⚙️  Stabilité moyenne")
        else:
            print(f"  ✅ Robot stable")
        
        time.sleep(1.0)
    
    # Fermer l'environnement
    env.close()
    
    print(f"\n{'='*70}")
    print("TEST TERMINÉ")
    print(f"{'='*70}")
    print("\nSi le robot démarre debout et tient sa pose, l'entraînement peut commencer.")
    print("Si le robot tombe immédiatement, vérifiez les angles initiaux dans spot_env.py.")


def main():
    parser = argparse.ArgumentParser(
        description='Test SpotMicro RL Environment',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    parser.add_argument('--episodes', type=int, default=3,
                       help='Nombre d\'épisodes de test')
    parser.add_argument('--steps', type=int, default=500,
                       help='Nombre max de steps par épisode')
    parser.add_argument('--no_render', action='store_true',
                       help='Désactiver le rendu (mode headless)')
    
    args = parser.parse_args()
    
    test_env(
        num_episodes=args.episodes,
        render=not args.no_render,
        max_steps=args.steps
    )


if __name__ == '__main__':
    main()
