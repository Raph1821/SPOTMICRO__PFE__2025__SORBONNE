#!/usr/bin/env python3
"""Environment randomizer for SpotMicro - CRITICAL for sim-to-real transfer.

This randomizer changes physical parameters at every reset() to make the
learned policy robust to:
- Model uncertainties (mass, inertia variations)
- Actuator variations (motor properties)
- Environmental uncertainties (ground friction)

Without domain randomization, policies trained in simulation typically
FAIL on real hardware due to the reality gap.

Adapted from spot_mini_mini for RL_app
"""

import numpy as np
import pybullet as p
from .env_randomizer_base import EnvRandomizerBase

# Randomization ranges (relative)
SPOT_BASE_MASS_ERROR_RANGE = (-0.2, 0.2)    # ±20% base mass variation
SPOT_LEG_MASS_ERROR_RANGE = (-0.2, 0.2)     # ±20% leg mass variation

# Randomization ranges (absolute)
BATTERY_VOLTAGE_RANGE = (14.8, 16.8)        # Volts - battery discharge range
MOTOR_VISCOUS_DAMPING_RANGE = (0, 0.01)     # N·m·s/rad - friction variation
SPOT_LEG_FRICTION_RANGE = (0.8, 1.5)        # Dimensionless - ground friction
MOTOR_KP_RANGE = (1.0, 1.5)                 # Position control gain variation
GROUND_RESTITUTION_RANGE = (0.0, 0.1)       # Bounciness of ground


class SpotEnvRandomizer(EnvRandomizerBase):
    """Randomizer for SpotMicro environment.
    
    Randomizes physical parameters at each reset() to improve policy robustness.
    This is based on "Domain Randomization for Transferring Deep Neural Networks
    from Simulation to the Real World" (Tobin et al., 2017).
    
    Parameters randomized:
    1. **Base mass**: Body weight variation (payload, battery charge)
    2. **Leg masses**: Component weight tolerances
    3. **Battery voltage**: Discharge state (affects motor torque)
    4. **Motor damping**: Bearing friction, temperature effects
    5. **Ground friction**: Different surfaces (concrete, carpet, grass)
    6. **Motor gains**: Controller variations, temperature drift
    7. **Ground restitution**: Surface compliance
    """
    
    def __init__(self,
                 base_mass_err_range=SPOT_BASE_MASS_ERROR_RANGE,
                 leg_mass_err_range=SPOT_LEG_MASS_ERROR_RANGE,
                 battery_voltage_range=BATTERY_VOLTAGE_RANGE,
                 motor_viscous_damping_range=MOTOR_VISCOUS_DAMPING_RANGE,
                 leg_friction_range=SPOT_LEG_FRICTION_RANGE,
                 motor_kp_range=MOTOR_KP_RANGE,
                 ground_restitution_range=GROUND_RESTITUTION_RANGE,
                 seed=None):
        """Initialize randomizer.
        
        Args:
            base_mass_err_range: (min, max) relative error for base mass
            leg_mass_err_range: (min, max) relative error for leg masses
            battery_voltage_range: (min, max) battery voltage in Volts
            motor_viscous_damping_range: (min, max) motor damping in N·m·s/rad
            leg_friction_range: (min, max) foot-ground friction coefficient
            motor_kp_range: (min, max) motor position control gain
            ground_restitution_range: (min, max) ground bounciness [0, 1]
            seed: Random seed for reproducibility (None = random)
        """
        self._base_mass_err_range = base_mass_err_range
        self._leg_mass_err_range = leg_mass_err_range
        self._battery_voltage_range = battery_voltage_range
        self._motor_viscous_damping_range = motor_viscous_damping_range
        self._leg_friction_range = leg_friction_range
        self._motor_kp_range = motor_kp_range
        self._ground_restitution_range = ground_restitution_range
        
        if seed is not None:
            np.random.seed(seed)
    
    def randomize_env(self, env):
        """Randomize environment physics.
        
        Called automatically in env.reset().
        
        Args:
            env: SpotMicroEnv instance
        """
        self._randomize_robot(env)
        self._randomize_ground(env)
    
    def _randomize_robot(self, env):
        """Randomize robot physical properties.
        
        Args:
            env: SpotMicroEnv instance
        """
        robot_id = env.robot_id
        
        # 1. Randomize base mass
        # Get base link mass from URDF
        base_dynamics = p.getDynamicsInfo(robot_id, -1)  # -1 = base link
        nominal_base_mass = base_dynamics[0]
        
        randomized_base_mass = np.random.uniform(
            nominal_base_mass * (1.0 + self._base_mass_err_range[0]),
            nominal_base_mass * (1.0 + self._base_mass_err_range[1])
        )
        
        p.changeDynamics(
            robot_id,
            -1,  # base link
            mass=randomized_base_mass
        )
        
        # 2. Randomize leg masses
        num_joints = p.getNumJoints(robot_id)
        
        for joint_id in range(num_joints):
            joint_info = p.getJointInfo(robot_id, joint_id)
            link_name = joint_info[12].decode('utf-8')
            
            # Only randomize leg links (not sensors, etc.)
            if any(keyword in link_name.lower() for keyword in 
                   ['shoulder', 'leg', 'foot', 'toe', 'knee', 'hip']):
                
                dynamics_info = p.getDynamicsInfo(robot_id, joint_id)
                nominal_mass = dynamics_info[0]
                
                if nominal_mass > 0:  # Skip massless links
                    randomized_mass = np.random.uniform(
                        nominal_mass * (1.0 + self._leg_mass_err_range[0]),
                        nominal_mass * (1.0 + self._leg_mass_err_range[1])
                    )
                    
                    p.changeDynamics(
                        robot_id,
                        joint_id,
                        mass=randomized_mass
                    )
        
        # 3. Randomize motor properties (if env has motor models)
        if hasattr(env, 'motor_models') and env.motor_models is not None:
            # Randomize battery voltage
            randomized_voltage = np.random.uniform(
                self._battery_voltage_range[0],
                self._battery_voltage_range[1]
            )
            
            # Randomize viscous damping
            randomized_damping = np.random.uniform(
                self._motor_viscous_damping_range[0],
                self._motor_viscous_damping_range[1]
            )
            
            # Randomize kp gain
            randomized_kp = np.random.uniform(
                self._motor_kp_range[0],
                self._motor_kp_range[1]
            )
            
            for motor in env.motor_models:
                motor.set_voltage(randomized_voltage)
                motor.set_viscous_damping(randomized_damping)
                motor.set_gains(randomized_kp, motor._kd)  # Keep kd fixed
        
        # 4. Randomize foot friction
        randomized_friction = np.random.uniform(
            self._leg_friction_range[0],
            self._leg_friction_range[1]
        )
        
        # Apply to all foot links
        for foot_link_id in env.foot_links:
            p.changeDynamics(
                robot_id,
                foot_link_id,
                lateralFriction=randomized_friction
            )
    
    def _randomize_ground(self, env):
        """Randomize ground properties.
        
        Args:
            env: SpotMicroEnv instance
        """
        if env.plane_id is None:
            return
        
        # Randomize ground friction
        ground_friction = np.random.uniform(
            self._leg_friction_range[0],
            self._leg_friction_range[1]
        )
        
        # Randomize ground restitution (bounciness)
        ground_restitution = np.random.uniform(
            self._ground_restitution_range[0],
            self._ground_restitution_range[1]
        )
        
        p.changeDynamics(
            env.plane_id,
            -1,
            lateralFriction=ground_friction,
            restitution=ground_restitution
        )


# Preset randomizers for different training phases

class MinimalRandomizer(SpotEnvRandomizer):
    """Minimal randomization for initial training.
    
    Use this for:
    - Early training stages
    - Debugging
    - Fast convergence on flat terrain
    """
    def __init__(self):
        super().__init__(
            base_mass_err_range=(-0.05, 0.05),      # ±5%
            leg_mass_err_range=(-0.05, 0.05),       # ±5%
            battery_voltage_range=(15.5, 16.0),     # Narrow
            motor_viscous_damping_range=(0, 0.002), # Minimal
            leg_friction_range=(0.9, 1.1),          # Almost constant
            motor_kp_range=(1.15, 1.25),            # Narrow
            ground_restitution_range=(0.0, 0.02)    # Almost rigid
        )


class AggressiveRandomizer(SpotEnvRandomizer):
    """Aggressive randomization for robust policies.
    
    Use this for:
    - Final training stages
    - Real robot deployment preparation
    - Maximum robustness
    """
    def __init__(self):
        super().__init__(
            base_mass_err_range=(-0.3, 0.3),        # ±30%
            leg_mass_err_range=(-0.3, 0.3),         # ±30%
            battery_voltage_range=(14.0, 17.0),     # Wide
            motor_viscous_damping_range=(0, 0.02),  # Full range
            leg_friction_range=(0.6, 1.8),          # Very wide
            motor_kp_range=(0.8, 1.6),              # Wide
            ground_restitution_range=(0.0, 0.15)    # Include bouncy
        )
