#!/usr/bin/env python3
"""
Spot Micro Kinematics Model
Adapted for RL_app robot dimensions from config file
"""

import numpy as np
from collections import OrderedDict
from .leg_kinematics import LegIK
from .lie_algebra import RpToTrans, TransToRp, TransInv, RPY


class SpotModel:
    """
    Spot Micro Kinematics Model
    Manages body-level IK and coordinates all 4 legs
    """
    
    def __init__(self,
                 shoulder_length=0.055,      # hip_link_length from config
                 elbow_length=0.1075,        # upper_leg_link_length from config
                 wrist_length=0.130,         # lower_leg_link_length from config
                 hip_x=0.186,                # body_length from config
                 hip_y=0.078,                # body_width from config
                 foot_x=0.186,               # Same as hip_x (default stance)
                 foot_y=0.17,                # Wider stance for stability
                 height=0.155,               # default_stand_height from config
                 shoulder_lim=[-0.548, 0.548],
                 elbow_lim=[-2.17, 0.97],
                 wrist_lim=[-0.1, 2.59]):
        """
        Initialize Spot Micro Kinematics Model
        
        Dimensions are adapted from RL_app config file:
        - shoulder_length: 0.055m (hip_link_length)
        - elbow_length: 0.1075m (upper_leg_link_length)
        - wrist_length: 0.130m (lower_leg_link_length)
        - hip_x: 0.186m (body_length)
        - hip_y: 0.078m (body_width)
        - height: 0.155m (default_stand_height)
        
        Args:
            shoulder_length: Hip link length (m)
            elbow_length: Upper leg link length (m)
            wrist_length: Lower leg link length (m)
            hip_x: Distance between front and back hips (m)
            hip_y: Distance between left and right hips (m)
            foot_x: Default foot separation front-to-back (m)
            foot_y: Default foot separation left-to-right (m)
            height: Default body height (m)
            shoulder_lim: Hip joint limits [min, max] (rad)
            elbow_lim: Shoulder joint limits [min, max] (rad)
            wrist_lim: Knee joint limits [min, max] (rad)
        """
        # Leg link lengths
        self.shoulder_length = shoulder_length
        self.elbow_length = elbow_length
        self.wrist_length = wrist_length
        
        # Hip positions (distance between hip joints)
        self.hip_x = hip_x  # Length (front-to-back)
        self.hip_y = hip_y  # Width (left-to-right)
        
        # Default foot positions
        self.foot_x = foot_x  # Length
        self.foot_y = foot_y  # Width
        
        # Default body height
        self.height = height
        
        # Joint limits
        self.shoulder_lim = shoulder_lim
        self.elbow_lim = elbow_lim
        self.wrist_lim = wrist_lim
        
        # Create Leg IK Solvers for each leg
        self.Legs = OrderedDict()
        
        # Front Left (FL)
        self.Legs["FL"] = LegIK("LEFT", 
                               self.shoulder_length,
                               self.elbow_length, 
                               self.wrist_length,
                               self.shoulder_lim, 
                               self.elbow_lim,
                               self.wrist_lim)
        
        # Front Right (FR)
        self.Legs["FR"] = LegIK("RIGHT", 
                               self.shoulder_length,
                               self.elbow_length, 
                               self.wrist_length,
                               self.shoulder_lim, 
                               self.elbow_lim,
                               self.wrist_lim)
        
        # Back Left (BL)
        self.Legs["BL"] = LegIK("LEFT", 
                               self.shoulder_length,
                               self.elbow_length, 
                               self.wrist_length,
                               self.shoulder_lim, 
                               self.elbow_lim,
                               self.wrist_lim)
        
        # Back Right (BR)
        self.Legs["BR"] = LegIK("RIGHT", 
                               self.shoulder_length,
                               self.elbow_length, 
                               self.wrist_length,
                               self.shoulder_lim, 
                               self.elbow_lim,
                               self.wrist_lim)
        
        # Initialize World-to-Hip Transforms
        # Hip positions relative to body center
        Rwb = np.eye(3)  # Identity rotation
        self.WorldToHip = OrderedDict()
        
        # Front Left hip
        self.ph_FL = np.array([self.hip_x / 2.0, self.hip_y / 2.0, 0])
        self.WorldToHip["FL"] = RpToTrans(Rwb, self.ph_FL)
        
        # Front Right hip
        self.ph_FR = np.array([self.hip_x / 2.0, -self.hip_y / 2.0, 0])
        self.WorldToHip["FR"] = RpToTrans(Rwb, self.ph_FR)
        
        # Back Left hip
        self.ph_BL = np.array([-self.hip_x / 2.0, self.hip_y / 2.0, 0])
        self.WorldToHip["BL"] = RpToTrans(Rwb, self.ph_BL)
        
        # Back Right hip
        self.ph_BR = np.array([-self.hip_x / 2.0, -self.hip_y / 2.0, 0])
        self.WorldToHip["BR"] = RpToTrans(Rwb, self.ph_BR)
        
        # Initialize World-to-Foot Transforms (default stance)
        self.WorldToFoot = OrderedDict()
        
        # Front Left foot
        self.pf_FL = np.array([self.foot_x / 2.0, self.foot_y / 2.0, -self.height])
        self.WorldToFoot["FL"] = RpToTrans(Rwb, self.pf_FL)
        
        # Front Right foot
        self.pf_FR = np.array([self.foot_x / 2.0, -self.foot_y / 2.0, -self.height])
        self.WorldToFoot["FR"] = RpToTrans(Rwb, self.pf_FR)
        
        # Back Left foot
        self.pf_BL = np.array([-self.foot_x / 2.0, self.foot_y / 2.0, -self.height])
        self.WorldToFoot["BL"] = RpToTrans(Rwb, self.pf_BL)
        
        # Back Right foot
        self.pf_BR = np.array([-self.foot_x / 2.0, -self.foot_y / 2.0, -self.height])
        self.WorldToFoot["BR"] = RpToTrans(Rwb, self.pf_BR)
    
    def HipToFoot(self, orn, pos, T_bf):
        """
        Convert desired body pose and foot positions to hip-to-foot vectors
        
        This is the key function that:
        1. Takes desired body orientation (roll, pitch, yaw)
        2. Takes desired body position (x, y, z)
        3. Takes desired foot positions relative to body
        4. Returns hip-to-foot vectors for each leg
        
        Args:
            orn: [roll, pitch, yaw] body orientation (rad)
            pos: [x, y, z] body position (m)
            T_bf: Dictionary of body-to-foot transforms for each leg
        
        Returns:
            HipToFoot_List: Dictionary of hip-to-foot vectors for each leg
        """
        # Get rotation matrix from RPY
        Rb, _ = TransToRp(RPY(orn[0], orn[1], orn[2]))
        pb = pos
        
        # World-to-body transform
        T_wb = RpToTrans(Rb, pb)
        
        # Dictionary to store hip-to-foot vectors
        HipToFoot_List = OrderedDict()
        
        for key, T_wh in self.WorldToHip.items():
            # Extract position from body-to-foot transform
            _, p_bf = TransToRp(T_bf[key])
            
            # Get body-to-hip transform
            T_bh = np.dot(TransInv(T_wb), T_wh)
            
            # Get hip-to-foot transform
            T_hf = np.dot(TransInv(T_bh), T_bf[key])
            
            # Extract position (hip-to-foot vector)
            _, p_hf = TransToRp(T_hf)
            
            HipToFoot_List[key] = p_hf
        
        return HipToFoot_List
    
    def IK(self, orn, pos, T_bf):
        """
        Full Body Inverse Kinematics
        
        Computes joint angles for all 4 legs given desired body pose
        and foot positions
        
        Args:
            orn: [roll, pitch, yaw] body orientation (rad)
            pos: [x, y, z] body position (m)
            T_bf: Dictionary of body-to-foot transforms
        
        Returns:
            joint_angles: 4x3 array of joint angles
                         [[FL_hip, FL_shoulder, FL_knee],
                          [FR_hip, FR_shoulder, FR_knee],
                          [BL_hip, BL_shoulder, BL_knee],
                          [BR_hip, BR_shoulder, BR_knee]]
        """
        # 4 legs × 3 joints per leg
        joint_angles = np.zeros((4, 3))
        
        # Get hip-to-foot vectors
        HipToFoot = self.HipToFoot(orn, pos, T_bf)
        
        # Solve IK for each leg
        for i, (key, p_hf) in enumerate(HipToFoot.items()):
            # Solve leg IK
            joint_angles[i, :] = self.Legs[key].solve(p_hf)
        
        return joint_angles
    
    def get_leg_order(self):
        """
        Return the order of legs as used in this model
        
        Returns:
            List of leg names in order: ['FL', 'FR', 'BL', 'BR']
        """
        return list(self.Legs.keys())
