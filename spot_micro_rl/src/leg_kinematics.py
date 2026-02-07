#!/usr/bin/env python3
"""
Leg Inverse Kinematics for Spot Micro
Based on: https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot
"""

import numpy as np


class LegIK:
    """
    Leg Inverse Kinematics Solver
    Solves IK for a single 3-DOF leg
    """
    
    def __init__(self,
                 legtype="RIGHT",
                 shoulder_length=0.055,
                 elbow_length=0.1075,
                 wrist_length=0.130,
                 hip_lim=[-0.548, 0.548],
                 shoulder_lim=[-2.17, 0.97],
                 leg_lim=[-0.1, 2.59]):
        """
        Initialize Leg IK Solver
        
        Args:
            legtype: "LEFT" or "RIGHT"
            shoulder_length: Length of hip link (m)
            elbow_length: Length of upper leg link (m)
            wrist_length: Length of lower leg link (m)
            hip_lim: Hip joint limits [min, max] (rad)
            shoulder_lim: Shoulder joint limits [min, max] (rad)
            leg_lim: Knee joint limits [min, max] (rad)
        """
        self.legtype = legtype
        self.shoulder_length = shoulder_length
        self.elbow_length = elbow_length
        self.wrist_length = wrist_length
        self.hip_lim = hip_lim
        self.shoulder_lim = shoulder_lim
        self.leg_lim = leg_lim
    
    def get_domain(self, x, y, z):
        """
        Calculate leg domain and clip if necessary
        
        Args:
            x, y, z: Hip-to-foot distances
        
        Returns:
            D: Leg domain (clipped to [-1, 1])
        """
        D = (y**2 + (-z)**2 - self.shoulder_length**2 +
             (-x)**2 - self.elbow_length**2 - self.wrist_length**2) / (
                 2 * self.wrist_length * self.elbow_length)
        
        if D > 1 or D < -1:
            # Domain breach - clip to valid range
            # print("Warning: Domain breach, clipping...")
            D = np.clip(D, -1.0, 1.0)
        
        return D
    
    def solve(self, xyz_coord):
        """
        Solve IK for desired foot position
        
        Args:
            xyz_coord: [x, y, z] hip-to-foot position
        
        Returns:
            joint_angles: [hip, shoulder, knee] angles (rad)
        """
        x = xyz_coord[0]
        y = xyz_coord[1]
        z = xyz_coord[2]
        
        D = self.get_domain(x, y, z)
        
        if self.legtype == "RIGHT":
            return self.RightIK(x, y, z, D)
        else:
            return self.LeftIK(x, y, z, D)
    
    def RightIK(self, x, y, z, D):
        """
        Right leg IK solver
        
        Args:
            x, y, z: Hip-to-foot distances
            D: Leg domain
        
        Returns:
            joint_angles: [hip, shoulder, knee] angles (rad)
        """
        # Knee angle
        wrist_angle = np.arctan2(-np.sqrt(1 - D**2), D)
        
        # Calculate sqrt component (check for negative)
        sqrt_component = y**2 + (-z)**2 - self.shoulder_length**2
        if sqrt_component < 0.0:
            print("Warning: Negative sqrt component")
            sqrt_component = 0.0
        
        # Hip angle
        shoulder_angle = -np.arctan2(z, y) - np.arctan2(
            np.sqrt(sqrt_component), -self.shoulder_length)
        
        # Shoulder angle
        elbow_angle = np.arctan2(-x, np.sqrt(sqrt_component)) - np.arctan2(
            self.wrist_length * np.sin(wrist_angle),
            self.elbow_length + self.wrist_length * np.cos(wrist_angle))
        
        joint_angles = np.array([shoulder_angle, -elbow_angle, -wrist_angle])
        return joint_angles
    
    def LeftIK(self, x, y, z, D):
        """
        Left leg IK solver
        
        Args:
            x, y, z: Hip-to-foot distances
            D: Leg domain
        
        Returns:
            joint_angles: [hip, shoulder, knee] angles (rad)
        """
        # Knee angle
        wrist_angle = np.arctan2(-np.sqrt(1 - D**2), D)
        
        # Calculate sqrt component (check for negative)
        sqrt_component = y**2 + (-z)**2 - self.shoulder_length**2
        if sqrt_component < 0.0:
            print("Warning: Negative sqrt component")
            sqrt_component = 0.0
        
        # Hip angle
        shoulder_angle = -np.arctan2(z, y) - np.arctan2(
            np.sqrt(sqrt_component), self.shoulder_length)
        
        # Shoulder angle
        elbow_angle = np.arctan2(-x, np.sqrt(sqrt_component)) - np.arctan2(
            self.wrist_length * np.sin(wrist_angle),
            self.elbow_length + self.wrist_length * np.cos(wrist_angle))
        
        joint_angles = np.array([shoulder_angle, -elbow_angle, -wrist_angle])
        return joint_angles
