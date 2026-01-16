# SpotMicro PyBullet 测试环境文件结构说明

## 📁 文件结构树

```
spot_micro_pybullet/scripts/
│
├── 🎮 env_tester.py                    # 【主程序】PyBullet 交互式测试环境
│   ├── 功能：提供 GUI 界面，实时调整机器人参数
│   ├── 职责：
│   │   ├── 初始化 PyBullet 物理引擎
│   │   ├── 加载 URDF 机器人模型
│   │   ├── 创建 GUI 参数滑块
│   │   ├── 主循环：读取 GUI 输入 → 计算步态 → IK 计算 → 控制关节
│   │   └── 处理键盘和鼠标输入
│   └── 依赖：spot_micro_kinematics_fixed, simple_gait_generator
│
├── 🔧 spot_micro_kinematics_fixed.py  # 【运动学核心】逆运动学计算（修复版）
│   ├── 功能：将足端位置转换为关节角度
│   ├── 职责：
│   │   ├── compute_leg_ik()          # 单腿逆运动学计算
│   │   ├── compute_all_joint_angles() # 所有腿的 IK 计算
│   │   ├── compute_stand_pose_angles() # 计算初始站立姿态
│   │   ├── get_default_foot_positions() # 获取默认足端位置
│   │   └── 角度合理性检查和平滑处理
│   └── 关键参数：
│       ├── hip_link_length = 0.055m
│       ├── upper_leg_link_length = 0.1075m
│       ├── lower_leg_link_length = 0.13m
│       └── body_width/length = 0.078m / 0.186m
│
├── 🚶 simple_gait_generator.py        # 【步态生成器】8 相步态轨迹生成
│   ├── 功能：根据步态参数生成足端轨迹
│   ├── 职责：
│   │   ├── get_foot_positions()      # 计算当前时刻的足端位置
│   │   ├── update_time()             # 更新步态时间
│   │   ├── reset_time()              # 重置步态时间
│   │   └── 处理摆动相和支撑相的轨迹
│   └── 关键参数：
│       ├── swing_time = 0.36s        # 摆动时间
│       ├── stance_time = 1.08s        # 支撑时间（3倍摆动时间）
│       └── phase_offsets              # 各腿的相位偏移
│
├── 📄 spot_micro_kinematics.py        # 【原版运动学】（未使用，保留作为参考）
│
├── 🤖 spot_micro_pybullet_sim.py      # 【ROS 节点】原始 PyBullet 仿真节点
│   ├── 功能：ROS 环境下的 PyBullet 仿真
│   ├── 特点：依赖 ROS，订阅 /servos_proportional 话题
│   └── 状态：不用于交互式测试，保留作为参考
│
└── 📚 文档文件
    ├── README.md                      # 使用说明
    ├── 机器人差异说明.md              # 机器人参数差异
    ├── 诊断问题.md                    # 问题诊断记录
    ├── 问题修复说明.md                # 修复历史
    └── 项目差异和问题分析.md          # 项目对比分析
```

## 🔄 数据流程图

```
┌─────────────────────────────────────────────────────────────────┐
│                        env_tester.py                            │
│                      (主程序入口)                                │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────┐
        │  1. 初始化阶段                           │
        │  - 加载 URDF 模型                        │
        │  - 创建 GUI 界面                         │
        │  - 初始化运动学和步态生成器              │
        └─────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────┐
        │  2. 主循环 (每帧执行)                    │
        │                                          │
        │  ┌──────────────────────────────────┐  │
        │  │ 读取 GUI 输入                      │  │
        │  │ - StepLength, StepVelocity        │  │
        │  │ - LateralFraction, YawRate       │  │
        │  │ - Body Position/Orientation        │  │
        │  └──────────────────────────────────┘  │
        │              │                            │
        │              ▼                            │
        │  ┌──────────────────────────────────┐  │
        │  │ simple_gait_generator             │  │
        │  │ get_foot_positions()              │  │
        │  │  ↓                                │  │
        │  │ 计算足端位置增量 (step_delta)     │  │
        │  │ 处理摆动/支撑相轨迹               │  │
        │  └──────────────────────────────────┘  │
        │              │                            │
        │              ▼                            │
        │  ┌──────────────────────────────────┐  │
        │  │ spot_micro_kinematics_fixed        │  │
        │  │ compute_all_joint_angles()        │  │
        │  │  ↓                                │  │
        │  │ 坐标转换：世界 → 身体 → 髋关节     │  │
        │  │ compute_leg_ik() (每条腿)         │  │
        │  │  ↓                                │  │
        │  │ 计算关节角度：                     │  │
        │  │ - shoulder_angle                  │  │
        │  │ - leg_angle                       │  │
        │  │ - foot_angle                      │  │
        │  └──────────────────────────────────┘  │
        │              │                            │
        │              ▼                            │
        │  ┌──────────────────────────────────┐  │
        │  │ PyBullet 关节控制                  │  │
        │  │ setJointMotorControl2()           │  │
        │  │ POSITION_CONTROL                   │  │
        │  └──────────────────────────────────┘  │
        │              │                            │
        │              ▼                            │
        │  ┌──────────────────────────────────┐  │
        │  │ 物理引擎步进                      │  │
        │  │ stepSimulation()                  │  │
        │  └──────────────────────────────────┘  │
        └─────────────────────────────────────────┘
```

## 📋 各模块详细说明

### 1. env_tester.py（主程序）

**核心功能：**
- 提供交互式 PyBullet 测试环境
- 实时调整机器人参数并观察效果

**主要组件：**

#### 1.1 URDF 路径解析
```python
resolve_urdf_paths(urdf_path)
```
- **功能**：将 ROS `package://` 路径转换为绝对路径
- **原因**：PyBullet 不支持 ROS 路径格式

#### 1.2 GUI 类
```python
class SpotMicroGUI
```
- **功能**：创建参数滑块界面
- **参数**：
  - Body Position (X, Y, Z)
  - Body Orientation (Roll, Pitch, Yaw)
  - StepLength, StepVelocity
  - LateralFraction, YawRate
  - ClearanceHeight, PenetrationDepth

#### 1.3 主循环
```python
while True:
    # 1. 读取 GUI 输入
    # 2. 检测运动状态变化
    # 3. 计算步态（如果有运动）
    # 4. 计算 IK
    # 5. 应用关节角度
    # 6. 物理引擎步进
```

### 2. spot_micro_kinematics_fixed.py（运动学核心）

**核心功能：**
- 将足端位置（世界坐标）转换为关节角度

**关键函数：**

#### 2.1 compute_leg_ik()
```python
def compute_leg_ik(self, leg_name, foot_pos_hip_frame):
    # 输入：足端位置（髋关节坐标系）
    # 输出：[shoulder_angle, leg_angle, foot_angle]
```
- **算法**：3 连杆逆运动学
- **步骤**：
  1. 计算 `foot_angle`（使用余弦定理）
  2. 计算 `leg_angle`（使用 atan2）
  3. 计算 `shoulder_angle`（使用 atan2）

#### 2.2 compute_all_joint_angles()
```python
def compute_all_joint_angles(self, body_pos, body_orn, foot_positions_world):
    # 输入：身体位置/姿态，足端世界坐标
    # 输出：所有关节角度字典
```
- **坐标转换流程**：
  1. 世界坐标 → 身体坐标（使用旋转矩阵）
  2. 身体坐标 → 髋关节坐标（减去髋关节位置）
  3. 对每条腿调用 `compute_leg_ik()`

#### 2.3 compute_stand_pose_angles()
```python
def compute_stand_pose_angles(self, body_height=None):
    # 计算初始站立姿态的关节角度
```
- **步骤**：
  1. 获取默认足端位置（身体坐标系）
  2. 转换到世界坐标系
  3. 调用 `compute_all_joint_angles()`

### 3. simple_gait_generator.py（步态生成器）

**核心功能：**
- 根据步态参数生成足端轨迹

**关键函数：**

#### 3.1 get_foot_positions()
```python
def get_foot_positions(self, step_length, lateral_fraction, yaw_rate, 
                       step_velocity, clearance_height, penetration_depth, 
                       body_height):
    # 输入：步态参数
    # 输出：足端位置字典（身体坐标系）
```
- **算法**：8 相步态
- **步骤**：
  1. 计算每条腿的相位
  2. 判断是摆动相还是支撑相
  3. 计算 `step_delta`（相对于默认位置的增量）
  4. 摆动相：抛物线轨迹（向上抬起）
  5. 支撑相：线性轨迹（跟随身体）

#### 3.2 相位管理
```python
phase_offsets = {
    'RB': 0.0,   # 相位 0
    'RF': 0.25,  # 相位 2
    'LF': 0.5,   # 相位 4
    'LB': 0.75   # 相位 6
}
```
- **8 相步态**：每次只有一条腿在摆动，其他三条腿支撑

## 🔗 模块间交互

```
env_tester.py
    │
    ├─→ simple_gait_generator.get_foot_positions()
    │   └─→ 返回：足端位置（身体坐标系）
    │
    ├─→ spot_micro_kinematics_fixed.compute_all_joint_angles()
    │   ├─→ 坐标转换：身体 → 髋关节
    │   └─→ compute_leg_ik() × 4（每条腿）
    │       └─→ 返回：关节角度
    │
    └─→ PyBullet.setJointMotorControl2()
        └─→ 应用关节角度到机器人
```

## 🎯 关键数据流

### 输入 → 输出流程

1. **用户输入**（GUI 滑块）
   ```
   StepLength = 0.05m
   StepVelocity = 0.5m/s
   ```

2. **步态生成器**
   ```
   step_delta = [0.02, 0.01, 0.03]  # 相对于默认位置的增量
   foot_pos_body = default_pos + step_delta
   ```

3. **坐标转换**
   ```
   foot_pos_world = body_pos + R @ foot_pos_body
   foot_pos_hip = foot_pos_body - hip_pos_body
   ```

4. **IK 计算**
   ```
   [shoulder, leg, foot] = compute_leg_ik(foot_pos_hip)
   ```

5. **关节控制**
   ```
   setJointMotorControl2(joint_idx, POSITION_CONTROL, targetPosition=angle)
   ```

## 🐛 常见问题处理

### 问题 1：右侧腿 y=0 导致极端角度
- **位置**：`spot_micro_kinematics_fixed.py` → `compute_leg_ik()`
- **处理**：当 y=0 时，使用特殊逻辑计算 `shoulder_angle`

### 问题 2：初始姿态不正确
- **位置**：`env_tester.py` → 初始化部分
- **处理**：使用参考角度而不是计算出的角度

### 问题 3：步态开始/停止时机器人摔倒
- **位置**：`simple_gait_generator.py` → `get_foot_positions()`
- **处理**：使用 `smooth_factor` 平滑过渡

## 📝 使用建议

1. **调试模式**：在 `env_tester.py` 中启用 `_debug_ik = True` 查看 IK 计算过程
2. **参数调整**：通过 GUI 滑块实时调整参数，观察机器人反应
3. **测试模式**：使用 `--test-joints` 参数测试关节控制是否工作

---

# Documentation de la structure des fichiers de l'environnement de test SpotMicro PyBullet

## 📁 Arborescence des fichiers

```
spot_micro_pybullet/scripts/
│
├── 🎮 env_tester.py                    # 【Programme principal】Environnement de test interactif PyBullet
│   ├── Fonction : Fournit une interface GUI pour ajuster les paramètres du robot en temps réel
│   ├── Responsabilités :
│   │   ├── Initialiser le moteur physique PyBullet
│   │   ├── Charger le modèle de robot URDF
│   │   ├── Créer les curseurs de paramètres GUI
│   │   ├── Boucle principale : Lire entrée GUI → Calculer démarche → Calcul IK → Contrôler articulations
│   │   └── Traiter les entrées clavier et souris
│   └── Dépendances : spot_micro_kinematics_fixed, simple_gait_generator
│
├── 🔧 spot_micro_kinematics_fixed.py  # 【Cœur cinématique】Calcul cinématique inverse (version corrigée)
│   ├── Fonction : Convertir les positions des pieds en angles d'articulation
│   ├── Responsabilités :
│   │   ├── compute_leg_ik()          # Calcul IK pour une seule jambe
│   │   ├── compute_all_joint_angles() # Calcul IK pour toutes les jambes
│   │   ├── compute_stand_pose_angles() # Calculer la pose debout initiale
│   │   ├── get_default_foot_positions() # Obtenir les positions par défaut des pieds
│   │   └── Vérification de validité et lissage des angles
│   └── Paramètres clés :
│       ├── hip_link_length = 0.055m
│       ├── upper_leg_link_length = 0.1075m
│       ├── lower_leg_link_length = 0.13m
│       └── body_width/length = 0.078m / 0.186m
│
├── 🚶 simple_gait_generator.py        # 【Générateur de démarche】Génération de trajectoire à 8 phases
│   ├── Fonction : Générer les trajectoires des pieds selon les paramètres de démarche
│   ├── Responsabilités :
│   │   ├── get_foot_positions()      # Calculer les positions des pieds à l'instant actuel
│   │   ├── update_time()             # Mettre à jour le temps de démarche
│   │   ├── reset_time()              # Réinitialiser le temps de démarche
│   │   └── Traiter les trajectoires de phase d'oscillation et de support
│   └── Paramètres clés :
│       ├── swing_time = 0.36s        # Temps d'oscillation
│       ├── stance_time = 1.08s        # Temps de support (3x temps d'oscillation)
│       └── phase_offsets              # Décalages de phase pour chaque jambe
│
├── 📄 spot_micro_kinematics.py        # 【Cinématique originale】(Non utilisé, conservé comme référence)
│
├── 🤖 spot_micro_pybullet_sim.py      # 【Nœud ROS】Nœud de simulation PyBullet original
│   ├── Fonction : Simulation PyBullet dans l'environnement ROS
│   ├── Caractéristiques : Dépend de ROS, s'abonne au topic /servos_proportional
│   └── État : Non utilisé pour les tests interactifs, conservé comme référence
│
└── 📚 Fichiers de documentation
    ├── README.md                      # Instructions d'utilisation
    ├── 机器人差异说明.md              # Différences de paramètres du robot
    ├── 诊断问题.md                    # Enregistrement du diagnostic des problèmes
    ├── 问题修复说明.md                # Historique des corrections
    └── 项目差异和问题分析.md          # Analyse comparative des projets
```

## 🔄 Diagramme de flux de données

```
┌─────────────────────────────────────────────────────────────────┐
│                        env_tester.py                            │
│                      (Point d'entrée principal)                │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────┐
        │  1. Phase d'initialisation              │
        │  - Charger le modèle URDF                │
        │  - Créer l'interface GUI                 │
        │  - Initialiser cinématique et générateur │
        └─────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────────┐
        │  2. Boucle principale (exécutée chaque frame)│
        │                                          │
        │  ┌──────────────────────────────────┐  │
        │  │ Lire entrée GUI                   │  │
        │  │ - StepLength, StepVelocity        │  │
        │  │ - LateralFraction, YawRate       │  │
        │  │ - Body Position/Orientation        │  │
        │  └──────────────────────────────────┘  │
        │              │                            │
        │              ▼                            │
        │  ┌──────────────────────────────────┐  │
        │  │ simple_gait_generator             │  │
        │  │ get_foot_positions()              │  │
        │  │  ↓                                │  │
        │  │ Calculer incrément position pieds │  │
        │  │ Traiter trajectoires phase        │  │
        │  └──────────────────────────────────┘  │
        │              │                            │
        │              ▼                            │
        │  ┌──────────────────────────────────┐  │
        │  │ spot_micro_kinematics_fixed        │  │
        │  │ compute_all_joint_angles()        │  │
        │  │  ↓                                │  │
        │  │ Conversion : Monde → Corps → Hanche│
        │  │ compute_leg_ik() (chaque jambe)   │  │
        │  │  ↓                                │  │
        │  │ Calculer angles d'articulation :  │  │
        │  │ - shoulder_angle                  │  │
        │  │ - leg_angle                       │  │
        │  │ - foot_angle                      │  │
        │  └──────────────────────────────────┘  │
        │              │                            │
        │              ▼                            │
        │  ┌──────────────────────────────────┐  │
        │  │ Contrôle des articulations        │  │
        │  │ setJointMotorControl2()           │  │
        │  │ POSITION_CONTROL                   │  │
        │  └──────────────────────────────────┘  │
        │              │                            │
        │              ▼                            │
        │  ┌──────────────────────────────────┐  │
        │  │ Avancement moteur physique        │  │
        │  │ stepSimulation()                  │  │
        │  └──────────────────────────────────┘  │
        └─────────────────────────────────────────┘
```

## 📋 Description détaillée des modules

### 1. env_tester.py（Programme principal）

**Fonctionnalités principales :**
- Fournir un environnement de test PyBullet interactif
- Ajuster les paramètres du robot en temps réel et observer les effets

**Composants principaux :**

#### 1.1 Résolution des chemins URDF
```python
resolve_urdf_paths(urdf_path)
```
- **Fonction**：Convertir les chemins ROS `package://` en chemins absolus
- **Raison**：PyBullet ne supporte pas le format de chemin ROS

#### 1.2 Classe GUI
```python
class SpotMicroGUI
```
- **Fonction**：Créer une interface avec curseurs de paramètres
- **Paramètres**：
  - Body Position (X, Y, Z)
  - Body Orientation (Roll, Pitch, Yaw)
  - StepLength, StepVelocity
  - LateralFraction, YawRate
  - ClearanceHeight, PenetrationDepth

#### 1.3 Boucle principale
```python
while True:
    # 1. Lire entrée GUI
    # 2. Détecter changement d'état de mouvement
    # 3. Calculer démarche (si mouvement)
    # 4. Calculer IK
    # 5. Appliquer angles d'articulation
    # 6. Avancer moteur physique
```

### 2. spot_micro_kinematics_fixed.py（Cœur cinématique）

**Fonctionnalités principales :**
- Convertir les positions des pieds (coordonnées monde) en angles d'articulation

**Fonctions clés :**

#### 2.1 compute_leg_ik()
```python
def compute_leg_ik(self, leg_name, foot_pos_hip_frame):
    # Entrée : Position du pied (repère hanche)
    # Sortie : [shoulder_angle, leg_angle, foot_angle]
```
- **Algorithme**：Cinématique inverse à 3 maillons
- **Étapes**：
  1. Calculer `foot_angle` (théorème du cosinus)
  2. Calculer `leg_angle` (atan2)
  3. Calculer `shoulder_angle` (atan2)

#### 2.2 compute_all_joint_angles()
```python
def compute_all_joint_angles(self, body_pos, body_orn, foot_positions_world):
    # Entrée : Position/orientation du corps, positions des pieds (monde)
    # Sortie : Dictionnaire de tous les angles d'articulation
```
- **Processus de conversion de coordonnées**：
  1. Coordonnées monde → Coordonnées corps (matrice de rotation)
  2. Coordonnées corps → Coordonnées hanche (soustraire position hanche)
  3. Appeler `compute_leg_ik()` pour chaque jambe

#### 2.3 compute_stand_pose_angles()
```python
def compute_stand_pose_angles(self, body_height=None):
    # Calculer les angles d'articulation pour la pose debout initiale
```
- **Étapes**：
  1. Obtenir positions par défaut des pieds (repère corps)
  2. Convertir en coordonnées monde
  3. Appeler `compute_all_joint_angles()`

### 3. simple_gait_generator.py（Générateur de démarche）

**Fonctionnalités principales :**
- Générer les trajectoires des pieds selon les paramètres de démarche

**Fonctions clés :**

#### 3.1 get_foot_positions()
```python
def get_foot_positions(self, step_length, lateral_fraction, yaw_rate, 
                       step_velocity, clearance_height, penetration_depth, 
                       body_height):
    # Entrée : Paramètres de démarche
    # Sortie : Dictionnaire de positions des pieds (repère corps)
```
- **Algorithme**：Démarche à 8 phases
- **Étapes**：
  1. Calculer la phase de chaque jambe
  2. Déterminer si phase d'oscillation ou de support
  3. Calculer `step_delta` (incrément par rapport à la position par défaut)
  4. Phase d'oscillation : Trajectoire parabolique (soulèvement)
  5. Phase de support : Trajectoire linéaire (suivre le corps)

#### 3.2 Gestion des phases
```python
phase_offsets = {
    'RB': 0.0,   # Phase 0
    'RF': 0.25,  # Phase 2
    'LF': 0.5,   # Phase 4
    'LB': 0.75   # Phase 6
}
```
- **Démarche à 8 phases**：À chaque instant, une seule jambe oscille, les trois autres supportent

## 🔗 Interactions entre modules

```
env_tester.py
    │
    ├─→ simple_gait_generator.get_foot_positions()
    │   └─→ Retour : Positions des pieds (repère corps)
    │
    ├─→ spot_micro_kinematics_fixed.compute_all_joint_angles()
    │   ├─→ Conversion : Corps → Hanche
    │   └─→ compute_leg_ik() × 4 (chaque jambe)
    │       └─→ Retour : Angles d'articulation
    │
    └─→ PyBullet.setJointMotorControl2()
        └─→ Appliquer les angles au robot
```

## 🎯 Flux de données clés

### Flux Entrée → Sortie

1. **Entrée utilisateur**（Curseurs GUI）
   ```
   StepLength = 0.05m
   StepVelocity = 0.5m/s
   ```

2. **Générateur de démarche**
   ```
   step_delta = [0.02, 0.01, 0.03]  # Incrément par rapport à la position par défaut
   foot_pos_body = default_pos + step_delta
   ```

3. **Conversion de coordonnées**
   ```
   foot_pos_world = body_pos + R @ foot_pos_body
   foot_pos_hip = foot_pos_body - hip_pos_body
   ```

4. **Calcul IK**
   ```
   [shoulder, leg, foot] = compute_leg_ik(foot_pos_hip)
   ```

5. **Contrôle des articulations**
   ```
   setJointMotorControl2(joint_idx, POSITION_CONTROL, targetPosition=angle)
   ```

## 🐛 Traitement des problèmes courants

### Problème 1 : y=0 pour la jambe droite cause des angles extrêmes
- **Localisation**：`spot_micro_kinematics_fixed.py` → `compute_leg_ik()`
- **Traitement**：Quand y=0, utiliser une logique spéciale pour calculer `shoulder_angle`

### Problème 2 : Pose initiale incorrecte
- **Localisation**：`env_tester.py` → Section d'initialisation
- **Traitement**：Utiliser des angles de référence au lieu des angles calculés

### Problème 3 : Le robot tombe au démarrage/arrêt de la démarche
- **Localisation**：`simple_gait_generator.py` → `get_foot_positions()`
- **Traitement**：Utiliser `smooth_factor` pour une transition en douceur

## 📝 Suggestions d'utilisation

1. **Mode débogage**：Activer `_debug_ik = True` dans `env_tester.py` pour voir le processus de calcul IK
2. **Ajustement des paramètres**：Ajuster les paramètres en temps réel via les curseurs GUI et observer la réaction du robot
3. **Mode test**：Utiliser l'argument `--test-joints` pour tester si le contrôle des articulations fonctionne

