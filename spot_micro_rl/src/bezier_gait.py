#!/usr/bin/env python3
"""
Bezier Gait Generator
Adapted from spot_mini_mini for RL_app

Based on:
- Bezier Curves: https://dspace.mit.edu/handle/1721.1/98270
- Rotation Logic: http://www.inase.org/library/2014/santorini/bypaper/ROBCIRC/ROBCIRC-54.pdf
"""

import numpy as np
from .lie_algebra import TransToRp
import copy

STANCE = 0
SWING = 1


class BezierGait:
    """
    Generates smooth foot trajectories using Bezier curves for swing
    and sinusoidal curves for stance phases.
    
    Coordinates multiple legs with phase offsets for different gait patterns.
    """
    
    def __init__(self, dSref=[0.0, 0.5, 0.5, 0.0], dt=0.01, Tswing=0.15,
                 shoulder_length=0.055, elbow_length=0.1075, wrist_length=0.130,
                 hip_x=0.093, hip_y=0.039, height=0.155):
        """
        Initialize Bezier Gait Generator
        
        Args:
            dSref: Phase lag per leg [FL, FR, BL, BR]
                   Reference leg (FL) is always 0.0
                   Common patterns:
                   - Trot: [0.0, 0.5, 0.5, 0.0] (diagonal pairs)
                   - Walk: [0.0, 0.25, 0.5, 0.75]
                   - Pace: [0.0, 0.5, 0.0, 0.5] (lateral pairs)
            dt: Timestep (s)
            Tswing: Swing phase duration (s), typically 0.15-0.20
            shoulder_length: L1 link length (m)
            elbow_length: L2 link length (m)
            wrist_length: L3 link length (m)
            hip_x: Hip spacing X (m)
            hip_y: Hip spacing Y (m)
            height: Default body height (m)
        """
        # Phase Lag Per Leg: FL, FR, BL, BR
        self.dSref = dSref
        self.ModulatedRotation = [0.0, 0.0, 0.0, 0.0]
        
        # Number of control points is n + 1 = 11 + 1 = 12
        self.NumBezierPoints = 11
        
        # Timestep
        self.dt = dt
        
        # Robot dimensions
        self.shoulder_length = shoulder_length
        self.elbow_length = elbow_length
        self.wrist_length = wrist_length
        self.hip_x = hip_x
        self.hip_y = hip_y
        self.height = height
        
        # Total Elapsed Time
        self.time = 0.0
        # Touchdown Time
        self.TD_time = 0.0
        # Time Since Last Touchdown
        self.time_since_last_TD = 0.0
        # Trajectory Mode
        self.StanceSwing = SWING
        # Swing Phase value [0, 1] of Reference Foot
        self.SwRef = 0.0
        self.Stref = 0.0
        # Whether Reference Foot has Touched Down
        self.TD = False
        
        # Swing Time
        self.Tswing = Tswing
        
        # Reference Leg Index
        self.ref_idx = 0
        
        # Store all leg phases
        self.Phases = list(self.dSref)
    
    def reset(self):
        """Reset the parameters of the Bezier Gait Generator"""
        self.ModulatedRotation = [0.0, 0.0, 0.0, 0.0]
        
        # Total Elapsed Time
        self.time = 0.0
        # Touchdown Time
        self.TD_time = 0.0
        # Time Since Last Touchdown
        self.time_since_last_TD = 0.0
        # Trajectory Mode
        self.StanceSwing = SWING
        # Swing Phase value [0, 1] of Reference Foot
        self.SwRef = 0.0
        self.Stref = 0.0
        # Whether Reference Foot has Touched Down
        self.TD = False
    
    def GetPhase(self, index, Tstance, Tswing):
        """
        Retrieves the phase of an individual leg.
        
        Modified from original paper to avoid phase discontinuity:
        if ti < -Tswing:
            ti += Tstride
        
        Args:
            index: Leg index (0-3)
            Tstance: Current stance period duration
            Tswing: Swing period duration (constant)
        
        Returns:
            phase: Leg phase [0, 1]
            StanceSwing: STANCE or SWING mode indicator
        """
        StanceSwing = STANCE
        Sw_phase = 0.0
        Tstride = Tstance + Tswing
        ti = self.Get_ti(index, Tstride)
        
        # Avoid phase discontinuity
        if ti < -Tswing:
            ti += Tstride
        
        # STANCE PHASE
        if ti >= 0.0 and ti <= Tstance:
            StanceSwing = STANCE
            if Tstance == 0.0:
                Stnphase = 0.0
            else:
                Stnphase = ti / float(Tstance)
            if index == self.ref_idx:
                self.StanceSwing = StanceSwing
            return Stnphase, StanceSwing
        
        # SWING PHASE
        elif ti >= -Tswing and ti < 0.0:
            StanceSwing = SWING
            Sw_phase = (ti + Tswing) / Tswing
        elif ti > Tstance and ti <= Tstride:
            StanceSwing = SWING
            Sw_phase = (ti - Tstance) / Tswing
        
        # Touchdown at End of Swing
        if Sw_phase >= 1.0:
            Sw_phase = 1.0
        
        if index == self.ref_idx:
            self.StanceSwing = StanceSwing
            self.SwRef = Sw_phase
            # Reference Touchdown at End of Swing
            if self.SwRef >= 0.999:
                self.TD = True
        
        return Sw_phase, StanceSwing
    
    def Get_ti(self, index, Tstride):
        """
        Retrieves the time index for the individual leg
        
        Args:
            index: Leg index
            Tstride: Total leg movement period (Tstance + Tswing)
        
        Returns:
            Time index for the leg
        """
        # Force reference leg to 0.0 to avoid numerical issues
        if index == self.ref_idx:
            self.dSref[index] = 0.0
        return self.time_since_last_TD - self.dSref[index] * Tstride
    
    def Increment(self, dt, Tstride):
        """
        Increments the Bezier gait generator's internal clock
        
        Args:
            dt: Time step
            Tstride: Total leg movement period (Tstance + Tswing)
        """
        self.CheckTouchDown()
        self.time_since_last_TD = self.time - self.TD_time
        
        if self.time_since_last_TD > Tstride:
            self.time_since_last_TD = Tstride
        elif self.time_since_last_TD < 0.0:
            self.time_since_last_TD = 0.0
        
        # Increment time at the end (in case TD just happened)
        # So that we get time_since_last_TD = 0.0
        self.time += dt
        
        # If Tstride = Tswing, Tstance = 0
        # RESET ALL
        if Tstride < self.Tswing + dt:
            self.time = 0.0
            self.time_since_last_TD = 0.0
            self.TD_time = 0.0
            self.SwRef = 0.0
    
    def CheckTouchDown(self):
        """
        Checks whether a reference leg touchdown has occurred,
        and whether this warrants resetting the touchdown time
        """
        if self.SwRef >= 0.9 and self.TD:
            self.TD_time = self.time
            self.TD = False
            self.SwRef = 0.0
    
    def BezierPoint(self, t, k, point):
        """
        Calculate the point on the Bezier curve
        
        Args:
            t: Phase [0, 1]
            k: Point number [0, 11]
            point: Point value
        
        Returns:
            Value through Bezier Curve
        """
        return point * self.Binomial(k) * np.power(t, k) * np.power(
            1 - t, self.NumBezierPoints - k)
    
    def Binomial(self, k):
        """
        Solves the binomial theorem given a Bezier point number
        
        Args:
            k: Bezier point number
        
        Returns:
            Binomial coefficient
        """
        return np.math.factorial(self.NumBezierPoints) / (
            np.math.factorial(k) * np.math.factorial(self.NumBezierPoints - k))
    
    def BezierSwing(self, phase, L, LateralFraction, clearance_height=0.04):
        """
        Calculates the step coordinates for the Bezier (swing) period
        
        Args:
            phase: Current trajectory phase [0, 1]
            L: Step length (m) - NOTE: L is HALF of full stride length
            LateralFraction: Lateral movement angle (rad)
            clearance_height: Foot clearance height during swing (m)
        
        Returns:
            stepX, stepY, stepZ: Foot coordinates relative to body
        """
        # Polar Leg Coords
        X_POLAR = np.cos(LateralFraction)
        Y_POLAR = np.sin(LateralFraction)
        
        # Bezier Curve Points (12 control points)
        # NOTE: L is HALF of STEP LENGTH
        
        # Forward Component
        STEP = np.array([
            -L,           # Ctrl Point 0
            -L * 1.4,     # Ctrl Point 1: Lift velocity
            -L * 1.5,     # Ctrl Points 2, 3, 4: overlapped for
            -L * 1.5,     # direction change after
            -L * 1.5,     # follow-through
            0.0,          # Ctrl Points 5, 6, 7: Change acceleration
            0.0,          # during protraction
            0.0,
            L * 1.5,      # Ctrl Points 8, 9: Direction change
            L * 1.5,      # for swing-leg retraction
            L * 1.4,      # Ctrl Point 10: Retraction velocity
            L             # Ctrl Point 11
        ])
        
        # Apply lateral component
        X = STEP * X_POLAR
        Y = STEP * Y_POLAR
        
        # Vertical Component
        Z = np.array([
            0.0,                      # Ctrl Points 0, 1: Zero lift velocity
            0.0,
            clearance_height * 0.9,   # Ctrl Points 2, 3, 4: Transition
            clearance_height * 0.9,
            clearance_height * 0.9,
            clearance_height * 0.9,   # Ctrl Points 5, 6: Direction change
            clearance_height * 0.9,
            clearance_height * 1.1,   # Ctrl Point 7: Maximum clearance
            clearance_height * 1.1,   # Ctrl Points 8, 9: Smooth transition
            clearance_height * 1.1,
            0.0,                      # Ctrl Points 10, 11: Zero touchdown vel
            0.0
        ])
        
        # Calculate Bezier curve values
        stepX = 0.0
        stepY = 0.0
        stepZ = 0.0
        for i in range(len(X)):
            stepX += self.BezierPoint(phase, i, X[i])
            stepY += self.BezierPoint(phase, i, Y[i])
            stepZ += self.BezierPoint(phase, i, Z[i])
        
        return stepX, stepY, stepZ
    
    def SineStance(self, phase, L, LateralFraction, penetration_depth=0.00):
        """
        Calculates the step coordinates for the Sinusoidal stance period
        
        Args:
            phase: Current trajectory phase [0, 1]
            L: Step length (m)
            LateralFraction: Lateral movement angle (rad)
            penetration_depth: Foot penetration depth during stance (m)
        
        Returns:
            stepX, stepY, stepZ: Foot coordinates relative to body
        """
        X_POLAR = np.cos(LateralFraction)
        Y_POLAR = np.sin(LateralFraction)
        
        # Moves from +L to -L during stance
        step = L * (1.0 - 2.0 * phase)
        stepX = step * X_POLAR
        stepY = step * Y_POLAR
        
        # Sinusoidal penetration for compliance
        if L != 0.0:
            stepZ = -penetration_depth * np.cos(
                (np.pi * (stepX + stepY)) / (2.0 * L))
        else:
            stepZ = 0.0
        
        return stepX, stepY, stepZ
    
    def SwingStep(self, phase, L, LateralFraction, YawRate, clearance_height,
                  T_bf, key, index):
        """
        Calculates swing step coordinates combining forward and rotational motion
        
        Args:
            phase: Current trajectory phase
            L: Step length (m)
            LateralFraction: Lateral movement angle (rad)
            YawRate: Desired body yaw rate (rad)
            clearance_height: Foot clearance height (m)
            T_bf: Default body-to-foot vector
            key: Foot name ('FL', 'FR', 'BL', 'BR')
            index: Foot index (0-3)
        
        Returns:
            coord: Foot coordinates [x, y, z] relative to body
        """
        DefaultBodyToFoot_Magnitude = np.sqrt(T_bf[0]**2 + T_bf[1]**2)
        
        # Rotation angle depending on leg position
        DefaultBodyToFoot_Direction = np.arctan2(T_bf[1], T_bf[0])
        
        # Angle traced by foot for rotation
        FootArcAngle = (np.pi / 2.0 + DefaultBodyToFoot_Direction + 
                       self.ModulatedRotation[index])
        
        # Get foot coordinates for forward motion
        X_delta_lin, Y_delta_lin, Z_delta_lin = self.BezierSwing(
            phase, L, LateralFraction, clearance_height)
        
        # Get foot coordinates for rotational motion
        X_delta_rot, Y_delta_rot, Z_delta_rot = self.BezierSwing(
            phase, YawRate, FootArcAngle, clearance_height)
        
        # Modulate magnitude to keep tracing circle
        ModulatedBodyToFoot_Magnitude = np.sqrt(
            (X_delta_rot + X_delta_lin)**2 + 
            (Y_delta_rot + Y_delta_lin)**2)
        mod = np.arctan2(ModulatedBodyToFoot_Magnitude,
                        DefaultBodyToFoot_Magnitude)
        self.ModulatedRotation[index] = mod
        
        coord = np.array([
            X_delta_lin + X_delta_rot,
            Y_delta_lin + Y_delta_rot,
            Z_delta_lin + Z_delta_rot
        ])
        
        return coord
    
    def StanceStep(self, phase, L, LateralFraction, YawRate, penetration_depth,
                   T_bf, key, index):
        """
        Calculates stance step coordinates combining forward and rotational motion
        
        Args:
            phase: Current trajectory phase
            L: Step length (m)
            LateralFraction: Lateral movement angle (rad)
            YawRate: Desired body yaw rate (rad)
            penetration_depth: Foot penetration depth (m)
            T_bf: Default body-to-foot vector
            key: Foot name ('FL', 'FR', 'BL', 'BR')
            index: Foot index (0-3)
        
        Returns:
            coord: Foot coordinates [x, y, z] relative to body
        """
        DefaultBodyToFoot_Magnitude = np.sqrt(T_bf[0]**2 + T_bf[1]**2)
        
        # Rotation angle depending on leg position
        DefaultBodyToFoot_Direction = np.arctan2(T_bf[1], T_bf[0])
        
        # Angle traced by foot for rotation
        FootArcAngle = (np.pi / 2.0 + DefaultBodyToFoot_Direction + 
                       self.ModulatedRotation[index])
        
        # Get foot coordinates for forward motion
        X_delta_lin, Y_delta_lin, Z_delta_lin = self.SineStance(
            phase, L, LateralFraction, penetration_depth)
        
        # Get foot coordinates for rotational motion
        X_delta_rot, Y_delta_rot, Z_delta_rot = self.SineStance(
            phase, YawRate, FootArcAngle, penetration_depth)
        
        # Modulate magnitude to keep tracing circle
        ModulatedBodyToFoot_Magnitude = np.sqrt(
            (X_delta_rot + X_delta_lin)**2 + 
            (Y_delta_rot + Y_delta_lin)**2)
        mod = np.arctan2(ModulatedBodyToFoot_Magnitude,
                        DefaultBodyToFoot_Magnitude)
        self.ModulatedRotation[index] = mod
        
        coord = np.array([
            X_delta_lin + X_delta_rot,
            Y_delta_lin + Y_delta_rot,
            Z_delta_lin + Z_delta_rot
        ])
        
        return coord
    
    def GetFootStep(self, L, LateralFraction, YawRate, clearance_height,
                    penetration_depth, Tstance, T_bf, index, key):
        """
        Calculates step coordinates in either Bezier (swing) or Sine (stance)
        portion of the trajectory depending on current phase
        
        Args:
            L: Step length (m)
            LateralFraction: Lateral movement angle (rad)
            YawRate: Desired body yaw rate (rad)
            clearance_height: Foot clearance height (m)
            penetration_depth: Foot penetration depth (m)
            Tstance: Current stance period duration
            T_bf: Default body-to-foot vector
            index: Foot index (0-3)
            key: Foot name ('FL', 'FR', 'BL', 'BR')
        
        Returns:
            Foot coordinates relative to body
        """
        phase, StanceSwing = self.GetPhase(index, Tstance, self.Tswing)
        
        # Store phase for tracking (swing = phase + 1.0)
        if StanceSwing == SWING:
            stored_phase = phase + 1.0
        else:
            stored_phase = phase
        self.Phases[index] = stored_phase
        
        # Calculate step based on phase
        if StanceSwing == STANCE:
            return self.StanceStep(phase, L, LateralFraction, YawRate,
                                 penetration_depth, T_bf, key, index)
        elif StanceSwing == SWING:
            return self.SwingStep(phase, L, LateralFraction, YawRate,
                                clearance_height, T_bf, key, index)
    
    def GenerateTrajectory(self,
                          L,
                          LateralFraction,
                          YawRate,
                          vel,
                          T_bf_,
                          T_bf_curr,
                          clearance_height=0.06,
                          penetration_depth=0.01,
                          contacts=[0, 0, 0, 0],
                          dt=None):
        """
        Calculates step coordinates for each foot
        
        Args:
            L: Step length (m)
            LateralFraction: Lateral movement angle (rad)
            YawRate: Desired body yaw rate (rad/s)
            vel: Desired step velocity (m/s)
            T_bf_: Default body-to-foot transforms (dict)
            T_bf_curr: Current body-to-foot transforms (dict)
            clearance_height: Foot clearance height (m)
            penetration_depth: Foot penetration depth (m)
            contacts: Contact state [1=contact, 0=no contact] for each foot
            dt: Time step (s)
        
        Returns:
            T_bf: Updated body-to-foot transforms for all feet
        """
        # Calculate stance time from desired speed and stride length
        # NOTE: L is HALF of stride length
        if vel != 0.0:
            Tstance = 2.0 * abs(L) / abs(vel)
        else:
            Tstance = 0.0
            L = 0.0
            self.TD = False
            self.time = 0.0
            self.time_since_last_TD = 0.0
        
        # Use default timestep if not specified
        if dt is None:
            dt = self.dt
        
        YawRate *= dt
        
        # Catch infeasible timesteps
        if Tstance < dt:
            Tstance = 0.0
            L = 0.0
            self.TD = False
            self.time = 0.0
            self.time_since_last_TD = 0.0
            YawRate = 0.0
        # Limit stance time for stability
        elif Tstance > 1.3 * self.Tswing:
            Tstance = 1.3 * self.Tswing
        
        # Check contacts (use FL as reference)
        if contacts[0] == 1 and Tstance > dt:
            self.TD = True
        
        # Increment time
        self.Increment(dt, Tstance + self.Tswing)
        
        # Calculate new foot positions
        T_bf = copy.deepcopy(T_bf_)
        for i, (key, Tbf_in) in enumerate(T_bf_.items()):
            # Set phase lags for each leg
            if key == "FL":
                self.ref_idx = i
                self.dSref[i] = 0.0
            elif key == "FR":
                self.dSref[i] = 0.5
            elif key == "BL":
                self.dSref[i] = 0.5
            elif key == "BR":
                self.dSref[i] = 0.0
            
            # Extract position from transform
            _, p_bf = TransToRp(Tbf_in)
            
            # Calculate step coordinates
            if Tstance > 0.0:
                step_coord = self.GetFootStep(L, LateralFraction, YawRate,
                                             clearance_height,
                                             penetration_depth, Tstance, p_bf,
                                             i, key)
            else:
                step_coord = np.array([0.0, 0.0, 0.0])
            
            # Update transform with new position
            T_bf[key][0, 3] = Tbf_in[0, 3] + step_coord[0]
            T_bf[key][1, 3] = Tbf_in[1, 3] + step_coord[1]
            T_bf[key][2, 3] = Tbf_in[2, 3] + step_coord[2]
        
        return T_bf
    
    def get_phases(self):
        """
        Returns the current gait phases for all 4 legs.
        
        Returns:
            np.ndarray: Array of 4 phase values [FL, FR, BL, BR], each in [0, 1]
        """
        return np.array(self.Phases[:4], dtype=np.float32)
    
    def step(self, vel_cmd=np.array([0.5, 0.0, 0.0]), 
             clearance_height=0.04, penetration_depth=0.01):
        """
        Generate joint angles for one timestep.
        
        Simplified version that returns neutral stance + small oscillation.
        For full implementation, integrate with spot_kinematics IK.
        
        Args:
            vel_cmd: [vx, vy, vyaw] desired velocities
            clearance_height: Height to lift feet during swing (m)
            penetration_depth: Depth to push into ground during stance (m)
        
        Returns:
            np.ndarray: 12 joint angles [FL_shoulder, FL_leg, FL_foot, 
                                         FR_shoulder, FR_leg, FR_foot,
                                         BL_shoulder, BL_leg, BL_foot,
                                         BR_shoulder, BR_leg, BR_foot]
        """
        # Compute stance time from velocity
        # Faster velocity = shorter stance time
        L = np.linalg.norm(vel_cmd[:2])  # Forward/lateral speed
        if L > 0.01:
            Tstance = 0.3  # 300ms stance for normal walking
        else:
            Tstance = 0.5  # 500ms stance for standing
        
        Tstride = Tstance + self.Tswing
        
        # Update phases for all legs
        for i in range(4):
            phase, mode = self.GetPhase(i, Tstance, self.Tswing)
            self.Phases[i] = phase
        
        # Increment time
        self.Increment(self.dt, Tstride)
        
        # Generate simple joint angles (placeholder)
        # For full implementation, compute IK from foot positions
        joint_angles = np.zeros(12)
        
        # Simple oscillation for each leg based on phase
        for i in range(4):
            phase = self.Phases[i]
            base_idx = i * 3
            
            # Shoulder joint: small side-to-side
            joint_angles[base_idx] = 0.0
            
            # Leg joint: oscillate between bent and straight
            # Swing phase: bend leg (lift foot)
            # Stance phase: straighten leg (push ground)
            if phase < 0.5:  # Swing
                joint_angles[base_idx + 1] = -0.8 + 0.3 * np.sin(phase * 2 * np.pi)
            else:  # Stance
                joint_angles[base_idx + 1] = -0.5 - 0.2 * np.sin((phase - 0.5) * 2 * np.pi)
            
            # Foot joint: complement leg joint to maintain height
            joint_angles[base_idx + 2] = 1.2 - joint_angles[base_idx + 1]
        
        return joint_angles
