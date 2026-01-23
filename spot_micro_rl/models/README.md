# Models Directory

Cette structure organise tous les fichiers nécessaires pour l'entraînement RL de SpotMicro.

## Structure

```
models/
├── checkpoints/               # Policies sauvegardées (ARS, SAC, TD3)
│   ├── spot_ars_100_policy   # Checkpoint ARS episode 100
│   ├── sac_episode_500.pth   # Checkpoint SAC episode 500
│   ├── td3_episode_500.pth   # Checkpoint TD3 episode 500
│   └── ...
└── README.md                  # Ce fichier

results/                       # Résultats d'entraînement
├── training_logs/            # CSV logs (ARS, SAC, TD3)
├── plots/                    # Graphiques de performance
└── survival_data/            # Données de survie (gmbc_data)
```

**Note:** Les fichiers URDF du robot sont dans `spot_micro_pybullet/urdf/` et utilisés directement par l'environnement.

## Checkpoints

Les policies sont sauvegardées automatiquement:

**ARS (pickle format):**
- Format: `spot_ars_{episode}_policy` ou `.pkl`
- Contient: theta, normalizer, hyperparamètres

**SAC/TD3 (PyTorch format):**
- Format: `sac_episode_{num}.pth` / `td3_episode_{num}.pth`
- Contient: actor, critic, optimizer states, replay buffer

## URDF Robot

Les fichiers URDF sont dans `../spot_micro_pybullet/urdf/`:
- `spot_micro_pybullet_gen_ros.urdf` (utilisé pour RL)
- Dimensions: shoulder=0.055m, elbow=0.1075m, wrist=0.130m
- Masse: base=1.2kg, pattes=~0.05kg chacune

## Usage

### Sauvegarder policy
```python
from spot_micro_rl import ARS

ars = ARS(state_dim=33, action_dim=14)
ars.save_policy("models/checkpoints/spot_ars_1000_policy")
```

### Charger policy
```python
ars.load_policy("models/checkpoints/spot_ars_1000_policy")
```

### Utiliser URDF custom
```python
from spot_micro_rl import SpotMicroEnv

env = SpotMicroEnv(urdf_file="models/urdf/spot_custom.urdf")
```

## Notes

- **Checkpoints**: Sauvegarder régulièrement (tous les 10-100 épisodes)
- **URDF**: Vérifier que les meshes STL sont accessibles
- **Results**: Analyser avec gmbc_data.py pour comparer performances
