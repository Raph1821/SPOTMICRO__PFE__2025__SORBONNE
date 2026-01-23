# Results Directory

Stockage des résultats d'entraînement RL.

## Structure

```
results/
├── training_logs/             # Logs d'entraînement
│   ├── tensorboard/          # Logs TensorBoard
│   └── csv/                  # Logs CSV (rewards, losses, etc.)
├── plots/                     # Graphiques générés
│   ├── reward_curves.png     # Courbes de reward
│   ├── survival_dist.png     # Distribution survie (gmbc_data)
│   └── loss_curves.png       # Courbes de loss (SAC/TD3)
└── survival_data/             # Données brutes survie
    ├── spot_ars_vanilla_survival_1000  # Bezier baseline
    └── spot_ars_agent_survival_1000    # Policy ARS/GMBC
```

## Fichiers générés

### Training Logs
- `rewards.csv`: Reward par épisode
- `timesteps.csv`: Timesteps survie par épisode
- `tensorboard/`: Events TensorBoard (tensorboard --logdir results/training_logs/tensorboard)

### Plots
- `reward_curves.png`: Evolution reward (Bezier vs Agent)
- `survival_dist.png`: Distribution KDE survie (gmbc_data.py)
- `loss_curves.png`: Courbes loss critic/actor (SAC/TD3)

### Survival Data
Format pickle Python:
```python
import pickle
with open('results/survival_data/spot_ars_agent_survival_1000', 'rb') as f:
    survival_timesteps = pickle.load(f)  # Liste timesteps survie
```

## Usage

### Analyser résultats
```bash
# Comparer Bezier vs GMBC
cd spot_micro_rl
python scripts/gmbc_data.py --NumberOfEpisodes 1000

# Visualiser TensorBoard
tensorboard --logdir results/training_logs/tensorboard
```

### Générer plots
```python
from spot_micro_rl.utils import plot_training_results

plot_training_results(
    csv_path='results/training_logs/rewards.csv',
    output_path='results/plots/reward_curves.png'
)
```

## Notes

- **Survival Data**: Généré automatiquement par spot_rl_train_ars.py
- **TensorBoard**: Accessible sur http://localhost:6006
- **CSV**: Compatible pandas pour analyse custom
