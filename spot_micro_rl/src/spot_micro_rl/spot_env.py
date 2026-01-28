#!/usr/bin/env python3
"""Environment Gym pour l'entraînement RL de SpotMicro avec PyBullet.

Espace d'observation: 33 dimensions
- IMU (8): roll, pitch, yaw, gyro_x, gyro_y, gyro_z, accel_z, placeholder
- Angles joints (12): positions des 12 servos
- Vitesses joints (12): vitesses des 12 servos
- Contacts (4): états de contact des 4 pattes
- Phases (4): phases de démarche pour chaque patte
- Position (3): x, y, z du corps
- Vélocité (3): vx, vy, vz du corps

Espace d'action: 14 dimensions
- Résidus joints (12): corrections pour chaque servo
- Clearance height (1): hauteur de levée de patte
- Penetration depth (1): profondeur de pénétration au sol
"""

import gym
from gym import spaces
import numpy as np
import pybullet as p
import pybullet_data
import time
from typing import Tuple, Dict, Any, Optional

try:
    import rospy
    from sensor_msgs.msg import Imu, JointState
    from geometry_msgs.msg import Twist, Pose
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("[WARNING] ROS non disponible, mode PyBullet pur")

from .bezier_gait import BezierGait
from .spot_kinematics import SpotModel
from .lie_algebra import RpToTrans, TransToRp, TransInv, RPY
from .motor import MotorModel
from .spot_env_randomizer import SpotEnvRandomizer
from .heightfield import HeightField

def make_env(render=False, use_ros=False, max_timesteps=1000, urdf_path=None, **kwargs):
    """
    Factory pour créer un SpotMicroEnv avec les paramètres courants.
    Args:
        render (bool): Afficher PyBullet GUI
        use_ros (bool): Activer interface ROS
        max_timesteps (int): Longueur max épisode
        urdf_path (str): Chemin URDF custom (optionnel)
        kwargs: Autres paramètres avancés
    Returns:
        SpotMicroEnv
    """
    return SpotMicroEnv(
        urdf_path=urdf_path,
        render=render,
        max_timesteps=max_timesteps,
        use_ros=use_ros,
        **kwargs
    )

class SpotMicroEnv(gym.Env):
    """Environnement Gym pour SpotMicro avec PyBullet et support ROS optionnel.
    
    Cet environnement implémente:
    - Simulation physique complète avec PyBullet
    - Génération de trajectoires avec courbes de Bézier
    - Cinématique inverse adaptée aux dimensions RL_app
    - Récompenses pour marche stable et efficace
    - Interface ROS optionnelle pour déploiement réel
    """
    
    metadata = {'render.modes': ['human', 'rgb_array']}
    
    def __init__(self,
                 urdf_path: str = None,
                 render: bool = False,
                 max_timesteps: int = 4000,
                 dt: float = 0.01,
                 use_ros: bool = False,
                 action_scale: float = 0.3,
                 energy_weight: float = 0.001,
                 motor_model_enabled: bool = True,
                 env_randomizer = None,
                 terrain_type: str = 'flat',
                 terrain_randomization: bool = True):
        """
        Args:
            urdf_path: Chemin vers le URDF du robot
            render: Activer le rendu graphique PyBullet
            max_timesteps: Nombre max de pas par épisode
            dt: Pas de temps de simulation (secondes)
            use_ros: Utiliser ROS pour communication (déploiement)
            action_scale: Échelle des actions (limite les mouvements brusques)
            energy_weight: Poids de la pénalité d'énergie dans la récompense
            motor_model_enabled: Utiliser motor model réaliste (CRITIQUE pour sim-to-real)
            env_randomizer: Domain randomizer (SpotEnvRandomizer instance ou None)
            terrain_type: Type de terrain ('flat', 'gentle', 'rough', 'random') - ignoré si terrain_randomization=True
            terrain_randomization: Si True, choisit aléatoirement terrain à chaque reset (15% flat, 15% gentle, 70% rough)
        """
        super(SpotMicroEnv, self).__init__()
        
        # Configuration
        self.urdf_path = urdf_path or "spot_micro.urdf"  # TODO: Charger depuis RL_app
        self.render_enabled = render
        self.max_timesteps = max_timesteps
        self.dt = dt
        self.use_ros = use_ros and ROS_AVAILABLE
        self.action_scale = action_scale
        self.energy_weight = energy_weight
        self.motor_model_enabled = motor_model_enabled
        self.terrain_type = terrain_type
        self.terrain_randomization = terrain_randomization
        
        # Domain randomizer (CRITIQUE pour sim-to-real)
        if env_randomizer is None and motor_model_enabled:
            # Use default randomizer
            self.env_randomizer = SpotEnvRandomizer()
        else:
            self.env_randomizer = env_randomizer
        
        # Dimensions du robot (depuis RL_app)
        self.shoulder_length = 0.055  # l1
        self.elbow_length = 0.1075    # l2
        self.wrist_length = 0.130     # l3
        self.hip_x = 0.186 / 2        # Écartement avant-arrière / 2
        self.hip_y = 0.078 / 2        # Écartement gauche-droite / 2
        self.default_height = 0.155   # Hauteur par défaut du corps
        
        # Cinématique et génération de trajectoires
        self.kinematics = SpotModel(
            shoulder_length=self.shoulder_length,
            elbow_length=self.elbow_length,
            wrist_length=self.wrist_length,
            hip_x=self.hip_x * 2,  # SpotModel expects full length
            hip_y=self.hip_y * 2,  # SpotModel expects full width
            height=self.default_height  # Parameter is 'height' not 'default_height'
        )
        
        self.gait_generator = BezierGait(
            dt=self.dt,
            shoulder_length=self.shoulder_length,
            elbow_length=self.elbow_length,
            wrist_length=self.wrist_length,
            hip_x=self.hip_x,
            hip_y=self.hip_y,
            height=self.default_height
        )
        
        # Espaces Gym: 33 dimensions d'observation
        # Espace d'observation: 46 dimensions
        # IMU (8) + Joints (12) + Joint_Vel (12) + Contacts (4) + Phases (4) + Pos (3) + Vel (3) = 46
        obs_high = np.inf * np.ones(46)
        self.observation_space = spaces.Box(
            low=-obs_high,
            high=obs_high,
            dtype=np.float32
        )
        
        # Espaces Gym: 14 dimensions d'action
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(14,),
            dtype=np.float32
        )
        
        # État de simulation
        self.physics_client = None
        self.robot_id = None
        self.plane_id = None
        self.timestep = 0
        self.episode = 0
        
        # État du robot
        self.joint_ids = []
        self.joint_names = []
        self.foot_links = []  # IDs des liens des pieds pour détection contact
        
        # Motor models (un par joint)
        self.motor_models = None
        if self.motor_model_enabled:
            self.motor_models = [MotorModel(kp=1.2, kd=0.0) for _ in range(12)]
        
        # Terrain generator
        self.terrain_generator = None
        if terrain_type != 'flat' or terrain_randomization:
            self.terrain_generator = HeightField()
        
        # Historique pour calcul de récompenses
        self.last_action = np.zeros(14)
        self.cumulative_reward = 0.0
        self.base_position_history = []
        
        # ROS (optionnel)
        if self.use_ros:
            self._init_ros()
    
    def _init_ros(self):
        """Initialise les publishers/subscribers ROS."""
        rospy.init_node('spot_rl_env', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/spot/cmd_vel', Twist, queue_size=1)
        self.joint_state_pub = rospy.Publisher('/spot/joint_states', JointState, queue_size=1)
    
    def _setup_joints(self):
        """Identifie les joints du robot depuis le URDF chargé."""
        self.joint_ids = []
        self.joint_names = []
        self.foot_links = []
        
        # Noms des joints dans l'ordre (RL_app convention)
        expected_joint_names = [
            # Front Left
            'front_left_shoulder', 'front_left_leg', 'front_left_foot',
            # Front Right  
            'front_right_shoulder', 'front_right_leg', 'front_right_foot',
            # Rear Left
            'rear_left_shoulder', 'rear_left_leg', 'rear_left_foot',
            # Rear Right
            'rear_right_shoulder', 'rear_right_leg', 'rear_right_foot'
        ]
        
        # Noms des liens de pieds pour détection contact
        foot_link_names = [
            'front_left_toe_link', 'front_right_toe_link',
            'rear_left_toe_link', 'rear_right_toe_link'
        ]
        
        # Scanner tous les joints du robot
        num_joints = p.getNumJoints(self.robot_id)
        joint_name_to_id = {}
        
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            joint_name_to_id[joint_name] = i
        
        # Mapper les joints attendus
        for joint_name in expected_joint_names:
            if joint_name in joint_name_to_id:
                joint_id = joint_name_to_id[joint_name]
                self.joint_ids.append(joint_id)
                self.joint_names.append(joint_name)
            else:
                print(f"[WARNING] Joint '{joint_name}' non trouvé dans URDF")
        
        # Mapper les liens de pieds
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            link_name = joint_info[12].decode('utf-8')
            if link_name in foot_link_names:
                self.foot_links.append(i)
        
        print(f"[INFO] {len(self.joint_ids)} joints configurés")
        print(f"[INFO] {len(self.foot_links)} pieds identifiés")
    
    def reset(self) -> np.ndarray:
        """Réinitialise l'environnement.
        
        Returns:
            Observation initiale (33 dims)
        """
        self.timestep = 0
        self.episode += 1
        self.cumulative_reward = 0.0
        self.last_action = np.zeros(14)
        self.base_position_history = []
        
        # Connexion PyBullet
        if self.physics_client is None:
            if self.render_enabled:
                self.physics_client = p.connect(p.GUI)
            else:
                self.physics_client = p.connect(p.DIRECT)
            
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.81)
            p.setTimeStep(self.dt)
        else:
            p.resetSimulation()
            p.setGravity(0, 0, -9.81)
        
        # Choisir type de terrain
        current_terrain = self.terrain_type
        
        if self.terrain_randomization:
            # Distribution: 15% flat, 15% gentle, 70% rough
            rand = np.random.random()
            if rand < 0.15:
                current_terrain = 'flat'
            elif rand < 0.30:  # 0.15 + 0.15
                current_terrain = 'gentle'
            else:
                current_terrain = 'rough'
        
        # Charger plan ou terrain
        if current_terrain == 'flat':
            self.plane_id = p.loadURDF("plane.urdf")
        elif self.terrain_generator is not None:
            # Générer terrain varié
            if current_terrain == 'gentle':
                self.plane_id = self.terrain_generator.generate_terrain(self, 0.05)
            elif current_terrain == 'rough':
                # Hauteur variée pour rough: 0.08-0.20m
                height_range = np.random.uniform(0.08, 0.20)
                self.plane_id = self.terrain_generator.generate_terrain(self, height_range)
            else:  # 'random' (legacy)
                height_range = np.random.uniform(0.03, 0.12)
                self.plane_id = self.terrain_generator.generate_terrain(self, height_range)
        else:
            self.plane_id = p.loadURDF("plane.urdf")
        
        # Charger le robot URDF depuis RL_app
        start_position = [0, 0, 0.3]  # 30cm au-dessus du sol
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        
        # Essayer de charger le URDF de RL_app, sinon fallback
        import os
        urdf_candidates = [
            self.urdf_path,
            os.path.join(os.path.dirname(__file__), "../../../spot_micro_pybullet/urdf/spot_micro_pybullet_gen_ros.urdf"),
            os.path.expanduser("~/catkin_ws/src/SPOTMICRO__PFE__2025__SORBONNE/spot_micro_pybullet/urdf/spot_micro_pybullet_gen_ros.urdf"),
        ]
        
        urdf_loaded = False
        for urdf_path in urdf_candidates:
            if os.path.exists(urdf_path):
                try:
                    self.robot_id = p.loadURDF(
                        urdf_path,
                        start_position,
                        start_orientation,
                        useFixedBase=False
                    )
                    print(f"[INFO] URDF chargé: {urdf_path}")
                    urdf_loaded = True
                    break
                except Exception as e:
                    print(f"[WARNING] Échec chargement {urdf_path}: {e}")
        
        if not urdf_loaded:
            raise FileNotFoundError(
                f"URDF non trouvé. Essayé: {urdf_candidates}\n"
                "Veuillez spécifier urdf_path dans SpotMicroEnv()"
            )
        
        # Identifier les joints
        self._setup_joints()
        
        # Configurer les joints à leur position neutre
        neutral_angles = np.zeros(12)
        for i, joint_id in enumerate(self.joint_ids):
            if i < len(neutral_angles):
                p.resetJointState(self.robot_id, joint_id, neutral_angles[i])
        
        # Domain Randomization (CRITIQUE pour sim-to-real)
        if self.env_randomizer is not None:
            self.env_randomizer.randomize_env(self)
        
        # Réinitialiser le générateur de démarche
        self.gait_generator.reset()
        
        return self._get_observation()
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict]:
        """Execute une action.
        
        Args:
            action: Array (14,) contenant:
                - [0:12] résidus pour chaque joint
                - [12] clearance height multiplier
                - [13] penetration depth multiplier
        
        Returns:
            observation (33,): Nouvel état
            reward (float): Récompense
            done (bool): Épisode terminé?
            info (dict): Infos supplémentaires
        """
        self.timestep += 1
        
        # Clipper et scaler l'action
        action = np.clip(action, -1.0, 1.0) * self.action_scale
        
        # Extraire les composantes de l'action
        joint_residuals = action[:12]
        clearance_mult = 1.0 + action[12] * 0.2  # ±20%
        penetration_mult = 1.0 + action[13] * 0.2
        
        # Générer positions cibles via Bézier + résidus
        # Le gait generator produit les angles de base pour la démarche
        if hasattr(self.gait_generator, 'step'):
            # Velocity command pour gait generator (peut être modifié)
            vel_cmd = np.array([0.5, 0.0, 0.0])  # [vx, vy, vyaw] target
            
            # Générer trajectoire de base
            base_joint_angles = self.gait_generator.step(
                vel_cmd=vel_cmd,
                clearance_height=0.04 * clearance_mult,
                penetration_depth=0.01 * penetration_mult
            )
            
            # Ajouter résidus RL
            target_joint_positions = base_joint_angles + joint_residuals
        else:
            # Fallback: utiliser seulement les résidus
            target_joint_positions = joint_residuals
        
        # Clipper pour sécurité (limites articulaires)
        target_joint_positions = np.clip(target_joint_positions, -1.8, 1.8)
        
        # Appliquer les positions aux joints
        if self.motor_model_enabled and self.motor_models is not None:
            # Utiliser motor models réalistes
            for i, joint_id in enumerate(self.joint_ids[:12]):
                # Obtenir état actuel
                joint_state = p.getJointState(self.robot_id, joint_id)
                current_angle = joint_state[0]
                current_velocity = joint_state[1]
                
                # Calculer torque via motor model
                actual_torque, observed_torque = self.motor_models[i].convert_to_torque(
                    target_joint_positions[i],
                    current_angle,
                    current_velocity
                )
                
                # Appliquer torque
                p.setJointMotorControl2(
                    self.robot_id,
                    joint_id,
                    p.TORQUE_CONTROL,
                    force=actual_torque
                )
        else:
            # Position control directe (moins réaliste)
            for i, joint_id in enumerate(self.joint_ids[:12]):
                p.setJointMotorControl2(
                    self.robot_id,
                    joint_id,
                    p.POSITION_CONTROL,
                    targetPosition=target_joint_positions[i],
                    force=3.0,
                    maxVelocity=10.0
                )
        
        # Simuler
        p.stepSimulation()
        if self.render_enabled:
            time.sleep(self.dt)
        
        # Sauvegarder l'action
        self.last_action = action
        
        # Observation, récompense, terminaison
        observation = self._get_observation()
        reward = self._calc_reward(observation, action)
        done = self._is_done(observation)
        
        self.cumulative_reward += reward
        
        info = {
            'timestep': self.timestep,
            'episode': self.episode,
            'cumulative_reward': self.cumulative_reward
        }
        
        return observation, reward, done, info
    
    def _get_observation(self) -> np.ndarray:
        """Récupère l'état actuel du robot (33 dimensions).
        
        Returns:
            Array (33,) contenant:
            - [0:8]   IMU (roll, pitch, yaw, gyro_xyz, accel_z, placeholder)
            - [8:20]  Angles joints (12)
            - [20:32] Vitesses joints (12)
            - [32:36] Contacts pieds (4)
            - [36:40] Phases démarche (4)
            - [40:43] Position corps (x, y, z)
            - [43:46] Vélocité corps (vx, vy, vz)
        """
        obs = np.zeros(46, dtype=np.float32)
        
        # Récupérer position et orientation du corps
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(orn)  # (roll, pitch, yaw)
        lin_vel, ang_vel = p.getBaseVelocity(self.robot_id)
        
        # IMU (8)
        obs[0] = euler[0]    # roll
        obs[1] = euler[1]    # pitch
        obs[2] = euler[2]    # yaw
        obs[3] = ang_vel[0]  # gyro_x (vitesse angulaire roll)
        obs[4] = ang_vel[1]  # gyro_y (vitesse angulaire pitch)
        obs[5] = ang_vel[2]  # gyro_z (vitesse angulaire yaw)
        
        # Accélération verticale (approximation via vélocité)
        if hasattr(self, '_last_lin_vel_z'):
            obs[6] = (lin_vel[2] - self._last_lin_vel_z) / self.dt
        else:
            obs[6] = 0.0
        self._last_lin_vel_z = lin_vel[2]
        
        obs[7] = 0.0  # placeholder pour future extension
        
        # Angles joints (12)
        for i, joint_id in enumerate(self.joint_ids[:12]):
            joint_state = p.getJointState(self.robot_id, joint_id)
            obs[8 + i] = joint_state[0]  # position angulaire
        
        # Vitesses joints (12)
        for i, joint_id in enumerate(self.joint_ids[:12]):
            joint_state = p.getJointState(self.robot_id, joint_id)
            obs[20 + i] = joint_state[1]  # vélocité angulaire
        
        # Contacts (4) - détection via PyBullet
        for i, foot_link in enumerate(self.foot_links[:4]):
            contact_points = p.getContactPoints(
                bodyA=self.robot_id,
                bodyB=self.plane_id,
                linkIndexA=foot_link
            )
            obs[32 + i] = 1.0 if len(contact_points) > 0 else 0.0
        
        # Phases (4) - depuis gait_generator
        # Note: BezierGait doit implémenter get_phases()
        if hasattr(self.gait_generator, 'get_phases'):
            phases = self.gait_generator.get_phases()
            obs[36:40] = phases[:4]
        else:
            # Fallback: estimer depuis le timestep
            # Trot: FL/BR ensemble (phase 0), FR/BL ensemble (phase 0.5)
            phase = (self.timestep * self.dt * 2.0) % 1.0  # 0.5 Hz
            obs[36] = phase           # FL
            obs[37] = (phase + 0.5) % 1.0  # FR
            obs[38] = (phase + 0.5) % 1.0  # BL
            obs[39] = phase           # BR
        
        # Position (3)
        obs[40] = pos[0]  # x
        obs[41] = pos[1]  # y
        obs[42] = pos[2]  # z
        
        # Vélocité (3)
        obs[43] = lin_vel[0]  # vx
        obs[44] = lin_vel[1]  # vy
        obs[45] = lin_vel[2]  # vz
        
        return obs
    
    def _calc_reward(self, observation: np.ndarray, action: np.ndarray) -> float:
        """Calcule la récompense.
        
        Composantes:
        - Forward progress: récompense la progression vers l'avant
        - Energy penalty: pénalise les mouvements brusques
        - Drift penalty: pénalise la dérive latérale
        - Shake penalty: pénalise les oscillations
        - Height penalty: pénalise l'écart à la hauteur cible
        - Orientation penalty: pénalise les inclinaisons excessives
        - Alive bonus: bonus pour rester debout
        - Joint limits: pénalise angles extrêmes
        
        Args:
            observation: État actuel (33,)
            action: Action appliquée (14,)
        
        Returns:
            Récompense scalaire
        """
        reward = 0.0
        
        # Extraire composantes de l'observation
        roll, pitch, yaw = observation[0], observation[1], observation[2]
        gyro = observation[3:6]
        joint_angles = observation[8:20]
        joint_vels = observation[20:32]
        contacts = observation[32:36]
        pos = observation[40:43]
        vel = observation[43:46]
        vx, vy, vz = vel[0], vel[1], vel[2]
        x, y, z = pos[0], pos[1], pos[2]
        
        # 1. Forward progress: récompense vitesse vers l'avant
        # Cible: 0.4-0.6 m/s
        target_vx = 0.5
        forward_reward = -abs(vx - target_vx) * 3.0
        if vx > 0.3:  # Bonus si vitesse raisonnable
            forward_reward += 1.0
        reward += forward_reward
        
        # 2. Energy penalty: pénalise changements brusques d'action
        action_diff = np.sum(np.square(action[:12] - self.last_action[:12]))
        energy_penalty = self.energy_weight * action_diff
        reward -= energy_penalty
        
        # 3. Joint velocity penalty: pénalise vitesses articulaires élevées
        joint_vel_penalty = 0.0001 * np.sum(np.square(joint_vels))
        reward -= joint_vel_penalty
        
        # 4. Drift penalty: pénalise dérive latérale
        lateral_drift = abs(vy) * 2.0 + abs(y) * 0.5
        reward -= lateral_drift
        
        # 5. Shake penalty: pénalise oscillations (roll/pitch)
        shake_penalty = (abs(roll) + abs(pitch)) * 1.0
        shake_penalty += np.sum(np.square(gyro)) * 0.1  # Pénalise vitesses angulaires
        reward -= shake_penalty
        
        # 6. Height penalty: maintenir hauteur cible
        height_error = abs(z - self.default_height)
        height_penalty = height_error * 5.0
        if height_error > 0.05:  # > 5cm d'écart
            height_penalty += 2.0  # Pénalité supplémentaire
        reward -= height_penalty
        
        # 7. Orientation penalty: rester horizontal
        orientation_penalty = 0.0
        if abs(roll) > 0.5 or abs(pitch) > 0.5:  # > ~30°
            orientation_penalty += 5.0
        reward -= orientation_penalty
        
        # 8. Joint limits penalty: éviter angles extrêmes
        joint_limit_penalty = 0.0
        for angle in joint_angles:
            if abs(angle) > 1.5:  # > ~85° (limite douce)
                joint_limit_penalty += (abs(angle) - 1.5) ** 2
        reward -= joint_limit_penalty * 0.5
        
        # 9. Contact pattern reward: encourager bon patron de contact
        # Trot: 2 pattes au sol (diagonales)
        num_contacts = int(np.sum(contacts))
        if num_contacts == 2:
            # Vérifier si diagonales (FL+BR ou FR+BL)
            if (contacts[0] and contacts[3]) or (contacts[1] and contacts[2]):
                reward += 0.5  # Bon patron de trot
        elif num_contacts < 1:
            reward -= 2.0  # Pénalité si aucun contact (chute)
        
        # 10. Alive bonus: encourager à rester debout
        alive_bonus = 0.1
        reward += alive_bonus
        
        # 11. Yaw penalty: décourager rotation excessive
        if hasattr(self, '_initial_yaw'):
            yaw_drift = abs(yaw - self._initial_yaw)
            if yaw_drift > np.pi:  # Normaliser [-π, π]
                yaw_drift = 2 * np.pi - yaw_drift
            reward -= yaw_drift * 0.2
        else:
            self._initial_yaw = yaw
        
        return reward
    
    def _is_done(self, observation: np.ndarray) -> bool:
        """Détermine si l'épisode est terminé.
        
        Conditions de terminaison:
        - Robot tombé (pitch/roll > seuil)
        - Hauteur trop basse
        - Timestep max atteint
        
        Args:
            observation: État actuel (33,)
        
        Returns:
            True si épisode terminé
        """
        # Max timesteps
        if self.timestep >= self.max_timesteps:
            return True
        
        # Robot tombé (inclinaison)
        roll, pitch = observation[0], observation[1]
        if abs(roll) > np.pi/3 or abs(pitch) > np.pi/3:  # > 60°
            return True
        
        # Hauteur trop basse
        z = observation[42]
        if z < 0.08:  # < 8cm
            return True
        
        return False
    
    def render(self, mode='human'):
        """Rendu visuel (déjà géré par PyBullet GUI)."""
        pass
    
    def close(self):
        """Ferme l'environnement."""
        if self.physics_client is not None:
            p.disconnect()
            self.physics_client = None
    
    def seed(self, seed=None):
        """Définit la graine aléatoire."""
        np.random.seed(seed)
        return [seed]
