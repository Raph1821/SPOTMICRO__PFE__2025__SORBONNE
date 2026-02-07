# Correction de l'Initialisation du Robot - README

**Date:** 2026-02-07  
**Problème identifié:** Initialisation incorrecte de la pose du robot lors de la migration de src_RL vers src_main

---

## 🔴 Problème Critique Résolu

### Symptôme
Lors de l'entraînement RL, le robot tombait immédiatement à chaque reset, rendant l'entraînement inutile car l'agent démarrait toujours dans un état d'échec.

### Cause Racine
Dans la version migrée (`spot_env.py`), les joints étaient initialisés à **zéro** :
```python
# ❌ INCORRECT - Pose instable
neutral_angles = np.zeros(12)  # Toutes les pattes droites → Robot tombe
```

Dans la version originale (`src_RL/spot.py`), le robot utilisait une **pose "stand" stable** :
```python
# ✅ CORRECT - Pose stable
stand_angles = np.array([
    -0.15192765, -0.7552236, 1.5104472,  # Patte avant-gauche
    0.15192765, -0.7552236, 1.5104472,   # Patte avant-droite
    -0.15192765, -0.7552236, 1.5104472,  # Patte arrière-gauche
    0.15192765, -0.7552236, 1.5104472    # Patte arrière-droite
])
```

---

## ✅ Corrections Appliquées

### 1. Fichier: `spot_env.py` (lignes ~353-370)

**Avant:**
```python
# Configurer les joints à leur position neutre
neutral_angles = np.zeros(12)
for i, joint_id in enumerate(self.joint_ids):
    if i < len(neutral_angles):
        p.resetJointState(self.robot_id, joint_id, neutral_angles[i])
```

**Après:**
```python
# Configurer les joints à leur position de STAND (pose stable)
# Ces angles viennent de src_RL/spot.py INIT_POSES['stand']
# Format: [shoulder, elbow, wrist] x 4 pattes (FL, FR, BL, BR)
stand_angles = np.array([
    -0.15192765, -0.7552236, 1.5104472,  # Front-left
    0.15192765, -0.7552236, 1.5104472,   # Front-right
    -0.15192765, -0.7552236, 1.5104472,  # Back-left
    0.15192765, -0.7552236, 1.5104472    # Back-right
])

for i, joint_id in enumerate(self.joint_ids):
    if i < len(stand_angles):
        p.resetJointState(self.robot_id, joint_id, stand_angles[i])

# Laisser le robot se stabiliser dans cette pose (important!)
for _ in range(100):
    p.stepSimulation()
```

### 2. Nouveau Fichier: `scripts/env_tester.py`

Script de test visuel pour vérifier l'initialisation du robot avant l'entraînement.

**Usage:**
```bash
python scripts/env_tester.py
python scripts/env_tester.py --episodes 5
```

**Checklist visuelle:**
- ✅ Robot démarre debout (pas couché)
- ✅ Pattes pliées de manière réaliste
- ✅ Corps horizontal à ~15-20cm du sol
- ✅ Robot tient sa pose pendant 5 secondes

### 3. Mise à jour: `INTEGRATION_GUIDE.md`

Ajout d'un **Scénario 0** (vérification critique) avant tous les entraînements :
- Explication du problème d'initialisation
- Instructions pour utiliser `env_tester.py`
- Checklist visuelle
- Note d'avertissement sur l'inutilité d'un entraînement avec mauvaise initialisation

---

## 🧪 Validation

### Étape 1: Tester l'initialisation
```bash
cd spot_micro_rl
python scripts/env_tester.py
```

**Attendu:**
- Robot démarre **debout** dans PyBullet GUI
- Robot reste **stable** pendant 5 secondes
- Pas de message "Robot tombé"

### Étape 2: Lancer un entraînement court de test
```bash
python scripts/spot_rl_train_ars.py --num_episodes 100 --render
```

**Attendu:**
- Robot démarre debout à chaque reset
- Récompenses augmentent progressivement
- Pas de "Done" immédiat à step 1-2

---

## 📊 Impact Attendu

### Avant Correction
- Robot tombe immédiatement (steps < 10)
- Récompenses toujours très négatives (~-100)
- Entraînement ne converge jamais
- Temps perdu : plusieurs heures voire jours

### Après Correction
- Robot démarre stable
- Survie initiale : 100-500 steps
- Récompenses progressent normalement
- Convergence attendue : 2-3h (ARS), 5-8h (SAC/TD3)

---

## 🔍 Diagnostic Si Problèmes Persistent

### Si le robot tombe toujours:

1. **Vérifier les angles dans spot_env.py:**
   ```bash
   grep -A 10 "stand_angles" src/spot_micro_rl/spot_env.py
   ```
   Doivent correspondre aux valeurs ci-dessus.

2. **Vérifier l'URDF chargé:**
   Chercher dans les logs au démarrage:
   ```
   [INFO] URDF chargé: /path/to/spot_micro_pybullet_gen_ros.urdf
   ```

3. **Vérifier la gravité et le timestep:**
   ```python
   p.setGravity(0, 0, -9.81)  # Correct
   p.setTimeStep(0.02)        # 50Hz
   ```

4. **Tester avec on_rack=True:**
   ```python
   env = SpotMicroEnv(render=True, on_rack=True)  # Robot suspendu
   ```

---

## 📚 Références

- **Fichier original:** `src_RL/spot_mini_mini/spotmicro/spot.py` (ligne 80)
- **Pose "stand":** INIT_POSES dict
- **Script de test original:** `src_RL/spot_bullet/src/env_tester.py`

---

## ✨ Prochaines Étapes

1. ✅ Vérifier initialisation : `python scripts/env_tester.py`
2. ✅ Lancer entraînement ARS : `python scripts/spot_rl_train_ars.py`
3. ⏭️ Suivre guide INTEGRATION_GUIDE.md pour SAC/TD3

**L'entraînement peut maintenant commencer dans de bonnes conditions !**
