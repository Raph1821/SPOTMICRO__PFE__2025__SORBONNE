#!/usr/bin/env python3
"""Accurate motor model for SpotMicro.

This file implements a realistic DC motor model that accounts for:
- PD control gains
- Viscous friction
- Back-EMF voltage
- Current-torque saturation profile
- Voltage clipping

Adapted from spot_mini_mini for RL_app
"""

import numpy as np

# Motor specifications (adjust for your servos)
VOLTAGE_CLIPPING = 8.4          # Volts - diode clipping on motor controller
OBSERVED_TORQUE_LIMIT = 5.7     # N·m - maximum observable torque
MOTOR_VOLTAGE = 7.4             # Volts - nominal operating voltage
MOTOR_RESISTANCE = 0.086        # Ohms - motor winding resistance
MOTOR_TORQUE_CONSTANT = 0.00954 # N·m/A - torque constant Kt
MOTOR_VISCOUS_DAMPING = 0.0     # N·m·s/rad - viscous friction coefficient
MOTOR_SPEED_LIMIT = 9.5         # rad/s - maximum motor speed


class MotorModel(object):
    """Accurate motor model based on DC motor physics.

    The motor model supports two control modes:
    
    1. **Position Control** (default):
       - Desired motor angle is specified
       - Torque computed via PD controller + motor physics
       - More realistic for servo-based robots
    
    2. **Torque Control**:
       - PWM signal [-1.0, 1.0] directly converted to torque
       - Useful for low-level control
    
    The model includes:
    - PD gains (kp, kd)
    - Viscous friction (damping)
    - Back-EMF voltage (speed-dependent)
    - Empirical current-torque saturation
    
    This is CRITICAL for sim-to-real transfer - naive PyBullet motors
    are too ideal and don't capture real servo behavior.
    """
    
    def __init__(self, torque_control_enabled=False, kp=1.2, kd=0.0):
        """Initialize motor model.
        
        Args:
            torque_control_enabled: If True, use direct torque control.
                                   If False, use position control (default).
            kp: Proportional gain for position control (N·m/rad)
            kd: Derivative gain for position control (N·m·s/rad)
        """
        self._torque_control_enabled = torque_control_enabled
        self._kp = kp
        self._kd = kd
        self._resistance = MOTOR_RESISTANCE
        self._voltage = MOTOR_VOLTAGE
        self._torque_constant = MOTOR_TORQUE_CONSTANT
        self._viscous_damping = MOTOR_VISCOUS_DAMPING
        
        # Empirical current-torque relationship
        # Based on real motor measurements
        self._current_table = [0, 10, 20, 30, 40, 50, 60]      # Amperes
        self._torque_table = [0, 1, 1.9, 2.45, 3.0, 3.25, 3.5] # N·m

    def set_voltage(self, voltage):
        """Set battery/supply voltage.
        
        Args:
            voltage: Battery voltage in Volts (typically 14.8-16.8V)
        """
        self._voltage = voltage

    def get_voltage(self):
        """Get current supply voltage.
        
        Returns:
            float: Voltage in Volts
        """
        return self._voltage

    def set_viscous_damping(self, viscous_damping):
        """Set viscous damping coefficient.
        
        Args:
            viscous_damping: Damping coefficient (N·m·s/rad)
                           Typical range: 0.0 - 0.01
        """
        self._viscous_damping = viscous_damping

    def get_viscous_damping(self):
        """Get viscous damping coefficient.
        
        Returns:
            float: Damping coefficient (N·m·s/rad)
        """
        return self._viscous_damping
    
    def set_gains(self, kp, kd):
        """Set PD control gains.
        
        Args:
            kp: Proportional gain
            kd: Derivative gain
        """
        self._kp = kp
        self._kd = kd

    def convert_to_torque(self, motor_commands, current_motor_angle,
                          current_motor_velocity):
        """Convert motor commands to actual torque.

        This is the main interface - converts desired commands to torque
        that should be applied in PyBullet, accounting for motor physics.

        Args:
            motor_commands: Desired angle (position control) or PWM (torque control)
            current_motor_angle: Current motor angle (rad)
            current_motor_velocity: Current motor velocity (rad/s)

        Returns:
            tuple: (actual_torque, observed_torque)
                - actual_torque: Torque to apply in simulation (N·m)
                - observed_torque: Torque that would be measured by sensor (N·m)
        """
        if self._torque_control_enabled:
            # Direct torque control: command is PWM signal
            pwm = motor_commands
        else:
            # Position control: compute PWM from PD controller
            position_error = motor_commands - current_motor_angle
            velocity_error = -current_motor_velocity  # Target velocity = 0
            
            pwm = self._kp * position_error + self._kd * velocity_error
        
        # Clip PWM to [-1.0, 1.0]
        pwm = np.clip(pwm, -1.0, 1.0)
        
        return self._convert_to_torque_from_pwm(pwm, current_motor_velocity)

    def _convert_to_torque_from_pwm(self, pwm, current_motor_velocity):
        """Convert PWM signal to torque using motor physics.

        Models:
        1. Back-EMF: voltage proportional to speed
        2. Current: (V_applied - V_back_emf) / R
        3. Torque saturation: empirical current-torque curve

        Args:
            pwm: Pulse width modulation signal [-1.0, 1.0]
            current_motor_velocity: Motor angular velocity (rad/s)

        Returns:
            tuple: (actual_torque, observed_torque) both in N·m
        """
        # Observed torque (what sensor would measure)
        # Simplified linear model
        observed_torque = np.clip(
            self._torque_constant * (pwm * self._voltage / self._resistance),
            -OBSERVED_TORQUE_LIMIT,
            OBSERVED_TORQUE_LIMIT
        )

        # Net voltage after back-EMF and viscous damping
        # V_net = V_applied - (Kt + b) * omega
        voltage_net = np.clip(
            pwm * self._voltage -
            (self._torque_constant + self._viscous_damping) * current_motor_velocity,
            -VOLTAGE_CLIPPING,
            VOLTAGE_CLIPPING
        )
        
        # Current from Ohm's law
        current = voltage_net / self._resistance
        current_sign = np.sign(current)
        current_magnitude = np.abs(current)

        # Actual torque from empirical current-torque relationship
        # Accounts for saturation effects
        actual_torque = np.interp(
            current_magnitude,
            self._current_table,
            self._torque_table
        )
        actual_torque = current_sign * actual_torque
        
        return actual_torque, observed_torque
