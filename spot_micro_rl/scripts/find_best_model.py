#!/usr/bin/env python
"""
find_best_model.py
------------------
Évalue tous les modèles sauvegardés dans ../models/ et affiche un classement
par récompense moyenne. Aucun rendu — s'exécute rapidement en mode headless.

Usage:
    python find_best_model.py           # 3 épisodes par modèle (rapide)
    python find_best_model.py -n 10     # 10 épisodes par modèle (précis)
    python find_best_model.py -hf       # avec HeightField (terrain accidenté)
"""

import numpy as np
import sys
import os
import argparse
import glob

sys.path.append('../')

from src.ars_lib.ars import ARSAgent, Normalizer, Policy
from src.Kinematics.SpotKinematics import SpotModel
from src.GaitGenerator.Bezier import BezierGait
from src.OpenLoopSM.SpotOL import BezierStepper
from src.GymEnvs.spot_bezier_env import spotBezierEnv

# ── Arguments ──────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="Benchmark all saved models.")
parser.add_argument("-n", "--NumEpisodes", type=int, default=3,
                    help="Nombre d'épisodes d'évaluation par modèle (défaut: 3)")
parser.add_argument("-hf", "--HeightField", action='store_true',
                    help="Utiliser HeightField (terrain accidenté)")
parser.add_argument("-steps", "--EpisodeSteps", type=int, default=2000,
                    help="Nombre de pas par épisode (défaut: 2000)")
ARGS = parser.parse_args()


def evaluate_model(agent, env, n_episodes, episode_steps):
    """Évalue un modèle sur n_episodes et retourne (mean_reward, std_reward)."""
    rewards = []
    agent.policy.episode_steps = episode_steps
    for _ in range(n_episodes):
        reward, _ = agent.deployTG()
        rewards.append(reward)
    return np.mean(rewards), np.std(rewards)


def main():
    seed = 0
    my_path = os.path.abspath(os.path.dirname(__file__))
    models_path = os.path.join(my_path, "../models")

    # ── Trouver tous les modèles disponibles ───────────────────────────────────
    pattern = os.path.join(models_path, "spot_ars_*_policy")
    model_files = sorted(glob.glob(pattern))

    if not model_files:
        print(f"Aucun modèle trouvé dans : {models_path}")
        print("Lance d'abord l'entraînement avec spot_ars.py")
        return

    # Extraire les numéros d'épisode depuis les noms de fichiers
    model_nums = []
    for f in model_files:
        basename = os.path.basename(f)        # ex: spot_ars_1549_policy
        num_str = basename.replace("spot_ars_", "").replace("_policy", "")
        try:
            model_nums.append(int(num_str))
        except ValueError:
            pass

    model_nums.sort()
    print(f"\n{'='*60}")
    print(f"  {len(model_nums)} modèles trouvés dans {models_path}")
    print(f"  {ARGS.NumEpisodes} épisodes × {ARGS.EpisodeSteps} pas par modèle")
    print(f"  HeightField: {ARGS.HeightField}")
    print(f"{'='*60}\n")

    # ── Créer l'environnement (une seule fois, partagé) ────────────────────────
    env = spotBezierEnv(render=False,
                        on_rack=False,
                        height_field=ARGS.HeightField,
                        draw_foot_path=False)
    env.seed(seed)
    np.random.seed(seed)

    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]

    spot = SpotModel(
        shoulder_length=0.052,
        elbow_length=0.12,
        wrist_length=0.115,
        hip_x=0.186,
        hip_y=0.072,
        foot_x=0.186,
        foot_y=0.176,
        height=0.16,
    )
    bz_step = BezierStepper(dt=env._time_step)
    bzg = BezierGait(dt=env._time_step)

    # ── Évaluer chaque modèle ──────────────────────────────────────────────────
    results = []  # liste de (episode_num, mean_reward, std_reward)

    for i, ep_num in enumerate(model_nums):
        model_path = os.path.join(models_path, f"spot_ars_{ep_num}")
        policy_file = model_path + "_policy"

        if not os.path.exists(policy_file):
            continue

        normalizer = Normalizer(state_dim)
        policy = Policy(state_dim, action_dim)
        agent = ARSAgent(normalizer, policy, env, bz_step, bzg, spot)
        agent.load(model_path)

        mean_r, std_r = evaluate_model(agent, env, ARGS.NumEpisodes,
                                       ARGS.EpisodeSteps)
        results.append((ep_num, mean_r, std_r))

        bar = "█" * int((i + 1) / len(model_nums) * 20)
        bar = bar.ljust(20)
        print(f"[{bar}] {i+1:3d}/{len(model_nums)} | "
              f"Ep {ep_num:5d} | Reward: {mean_r:8.1f} ± {std_r:.1f}")

    # ── Classement ─────────────────────────────────────────────────────────────
    results.sort(key=lambda x: x[1], reverse=True)

    print(f"\n{'='*60}")
    print(f"  CLASSEMENT — Top 10 meilleurs modèles")
    print(f"{'='*60}")
    print(f"  {'Rang':<5} {'Episode':>8} {'Reward moyen':>14} {'Std':>8}")
    print(f"  {'-'*40}")
    for rank, (ep_num, mean_r, std_r) in enumerate(results[:10], 1):
        marker = " ◀ MEILLEUR" if rank == 1 else ""
        print(f"  {rank:<5} {ep_num:>8} {mean_r:>14.1f} {std_r:>8.1f}{marker}")

    best_ep, best_r, best_std = results[0]
    print(f"\n  ✓ Meilleur modèle : spot_ars_{best_ep}_policy")
    print(f"  ✓ Reward moyen    : {best_r:.1f} ± {best_std:.1f}")
    print(f"\n  Pour le visualiser :")
    print(f"    python spot_ars_eval.py -a {best_ep} -r")
    print(f"{'='*60}\n")


if __name__ == '__main__':
    main()
