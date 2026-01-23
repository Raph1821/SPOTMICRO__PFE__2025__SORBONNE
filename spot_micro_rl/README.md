# SpotMicro RL Package - Complete Training Suite

**Version 2.0.0** - Package ROS complet pour l'entraînement par apprentissage par renforcement de SpotMicro avec PyBullet.

📘 **[Guide d'Intégration Complet](INTEGRATION_GUIDE.md)** ← Documentation détaillée Phases 1-3

---

## 🎯 Caractéristiques

### Observation Space: **46 Dimensions**

```
Observation = [
    IMU (8) +           # Roll, Pitch, Yaw, Gyro XYZ, Accel Z, Placeholder
    Joints (12) +       # Positions des 12 servos
    Joint_Vel (12) +    # Vitesses des 12 servos
    Contacts (4) +      # États contact des 4 pattes (0/1)
    Phases (4) +        # Phases de démarche (0-1)
    Position (3) +      # X, Y, Z du corps
    Velocity (3)        # Vx, Vy, Vz du corps
]
= 8 + 12 + 12 + 4 + 4 + 3 + 3 = 46 dimensions
```

### Action Space: 14 Dimensions

```
Action = [
```

### Algorithms: ARS / SAC / TD3

**3 algorithmes disponibles:**

| Algorithm | Type | Policy | Best For |
|-----------|------|--------|----------|
| **ARS** | On-policy | Linear | Prototyping rapide, CPU |
| **SAC** | Off-policy | Stochastic NN | Performance maximale |
| **TD3** | Off-policy | Deterministic NN | Stabilité + performance |

### Physics Realism (Phase 1)

- ✅ **Motor Models**: DC motor dynamics (kt, R, L, friction, backlash)
- ✅ **Domain Randomization**: 12 paramètres physiques
- ✅ **Terrain Generation**: Flat, gentle, rough terrains

---

## 🚀 Quick Start

### Installation

```bash
# 1. Clone repository
cd ~/catkin_ws/src
git clone <your_repo>/spot_micro_rl.git

# 2. Install dependencies
cd spot_micro_rl
pip install -r requirements.txt

# 3. (Optional) Install PyTorch for SAC/TD3
pip install torch

# 4. Build ROS workspace
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Validation

```bash
# Test all phases
python scripts/test_phase1.py  # Motor models, randomization, terrain
python scripts/test_phase2.py  # Training v2, GMBC, survival data
python scripts/test_phase3.py  # SAC, TD3, neural networks

# Expected: 18/18 tests passed
```

---

## 🏋️ Training

### Option 1: ARS (Rapide - CPU)

```bash
# Entraînement ARS avec curriculum learning
python scripts/spot_rl_train_ars.py \
    --num_episodes 1000 \
    --curriculum_stage 2 \
    --use_motor_models \
    --save_frequency 500

# Temps: ~6-8h (4 CPU cores)
```

### Option 2: SAC (Performance - GPU)

```bash
# Entraînement SAC (maximum entropy RL)
python scripts/spot_rl_train_sac.py \
    --episodes 10000 \
    --eval_freq 100 \
    --cuda

# Temps: ~12-16h (GPU), ~48h (CPU)
```

### Option 3: TD3 (Stabilité - GPU)

```bash
# Entraînement TD3 (deterministic policy)
python scripts/spot_rl_train_td3.py \
    --episodes 10000 \
    --eval_freq 100 \
    --cuda

# Temps: ~10-14h (GPU), ~40h (CPU)
```

---

## 📊 Evaluation

```bash
# Évaluer policy ARS
python scripts/spot_rl_eval.py \
    --policy_path models/checkpoints/spot_ars_10000.pkl \
    --num_episodes 10

# Comparer Bezier vs GMBC
python scripts/gmbc_data.py
```

---

## 📚 Documentation

**Fichiers disponibles:**

- 📘 **[INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md)**: Guide complet Phases 1-3
- 📄 **[README.md](README.md)**: Ce fichier (quick reference)
- 📁 **[models/README.md](models/README.md)**: Format checkpoints
- 📁 **[results/README.md](results/README.md)**: Structure logs

**Scripts de validation:**

```bash
python tests/test_phase1.py  # Motor models, randomization (5 tests)
python tests/test_phase2.py  # Training v2, GMBC (6 tests)
python tests/test_phase3.py  # SAC, TD3, networks (7 tests)
```

---

## 🔧 Package Structure

```
spot_micro_rl/
├── src/spot_micro_rl/          # Python package
│   ├── CORE                    # Base components
│   │   ├── spot_env.py         # Gym environment (46 dims)
│   │   ├── spot_kinematics.py  # Robot kinematics
│   │   ├── leg_kinematics.py   # IK per leg
│   │   ├── bezier_gait.py      # Gait generator
│   │   └── lie_algebra.py      # 3D transforms
│   │
│   ├── PHASE 1                 # Physics realism
│   │   ├── motor.py            # DC motor models
│   │   ├── spot_env_randomizer.py  # Domain randomization
│   │   └── heightfield.py      # Terrain generation
│   │
│   ├── PHASE 2                 # ARS algorithm
│   │   └── ars.py              # Augmented Random Search
│   │
│   └── PHASE 3                 # Deep RL
│       ├── networks.py         # Actor/Critic networks
│       ├── replay_buffer.py    # Experience replay
│       ├── sac.py              # Soft Actor-Critic
│       └── td3.py              # Twin Delayed DDPG
│
├── scripts/                    # Executables
│   ├── spot_rl_train_ars.py    # ARS training
│   ├── spot_rl_train_sac.py   # SAC training
│   ├── spot_rl_train_td3.py   # TD3 training
│   ├── spot_rl_eval.py        # Policy evaluation
│   └── gmbc_data.py           # GMBC analysis
│
├── tests/                      # Validation tests
│   ├── test_phase1.py         # Phase 1 validation
│   ├── test_phase2.py         # Phase 2 validation
│   └── test_phase3.py         # Phase 3 validation
│
├── models/                     # Saved models
│   ├── checkpoints/           # .pkl (ARS), .pth (SAC/TD3)
│   └── README.md
│
├── results/                    # Training results
│   ├── training_logs/         # CSV logs
│   ├── survival_data/         # Pickle files
│   └── plots/                 # Matplotlib graphs
│
├── config/                     # ROS configs
├── launch/                     # ROS launch files
├── CMakeLists.txt
├── setup.py
├── package.xml
├── requirements.txt
├── INTEGRATION_GUIDE.md        # 📘 Documentation complète
└── README.md                   # Ce fichier
```

---

## 🧪 Quick Test

```python
from spot_micro_rl import SpotMicroEnv

# Créer environnement
env = SpotMicroEnv(render=True)
obs = env.reset()

print(f"Observation: {obs.shape}")  # (46,)
print(f"Action space: {env.action_space}")  # Box(14,)

# Rollout aléatoire
for _ in range(100):
    action = env.action_space.sample()
    obs, reward, done, _ = env.step(action)
    if done:
        obs = env.reset()

env.close()
```

---

## 📋 Requirements

```txt
numpy>=1.19.0
scipy>=1.5.0
gym==0.17.3
pybullet>=3.0.0
matplotlib>=3.3.0

# Optional (Phase 3)
torch>=1.10.0
```

**Installation:**

```bash
pip install -r requirements.txt

# Pour SAC/TD3
pip install torch
```

---

## 🎯 Performance Benchmarks

**Configuration test:** Intel i7-10700K, RTX 3070

| Algorithm | Throughput | Memory | Convergence |
|-----------|------------|--------|-------------|
| ARS (1 CPU) | ~500 steps/s | 50 MB | 5000-8000 ep |
| ARS (4 CPU) | ~1800 steps/s | 200 MB | 5000-8000 ep |
| SAC (GPU) | ~1200 steps/s | 600 MB | 3000-5000 ep |
| TD3 (GPU) | ~1400 steps/s | 600 MB | 3000-5000 ep |

---

## 🐛 Troubleshooting

**Observation dimension error:**
```python
# Vérifier spot_env.py ligne 131-136
# observation_space doit avoir shape=(46,)
```

**PyTorch not found:**
```bash
pip install torch
# Phase 3 nécessite PyTorch
```

**CUDA out of memory:**
```bash
# Réduire buffer_size et batch_size
python scripts/spot_rl_train_sac.py --buffer_size 100000 --batch_size 128
```

---

## 📝 License

MIT License

---

## 👥 Contributors

Basé sur:
- [spot_mini_mini](https://github.com/moribots/spot_mini_mini)
- [spot_micro_kinematics_python](https://github.com/mike4192/spotMicro)

**Version 2.0.0** - SpotMicro RL Complete Training Suite

## 🔗 Liens Utiles

- [ARS Paper](https://arxiv.org/abs/1803.07055)
- [PyBullet Documentation](https://pybullet.org)
- [Gym Documentation](https://gym.openai.com/)
- [SpotMicro Community](https://spotmicroai.readthedocs.io/)
