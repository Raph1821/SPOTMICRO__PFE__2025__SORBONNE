#!/usr/bin/env python3
"""
Lie Algebra utilities for transformations
Based on Modern Robotics from Northwestern University
"""

import numpy as np


def RpToTrans(R, p):
    """
    Converts rotation matrix and position vector to homogeneous transform
    
    Args:
        R: 3x3 rotation matrix
        p: 3-vector position
    
    Returns:
        4x4 homogeneous transformation matrix
    """
    return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]


def TransToRp(T):
    """
    Converts homogeneous transform to rotation matrix and position
    
    Args:
        T: 4x4 homogeneous transformation matrix
    
    Returns:
        (R, p): 3x3 rotation matrix, 3-vector position
    """
    T = np.array(T)
    return T[0:3, 0:3], T[0:3, 3]


def TransInv(T):
    """
    Inverts a homogeneous transformation matrix
    
    Args:
        T: 4x4 homogeneous transformation matrix
    
    Returns:
        4x4 inverse transformation matrix
    """
    R, p = TransToRp(T)
    Rt = np.array(R).T
    return np.r_[np.c_[Rt, -np.dot(Rt, p)], [[0, 0, 0, 1]]]


def RPY(roll, pitch, yaw):
    """
    Creates Roll-Pitch-Yaw transformation matrix
    
    Args:
        roll: Roll angle (rad)
        pitch: Pitch angle (rad)
        yaw: Yaw angle (rad)
    
    Returns:
        4x4 transformation matrix
    """
    Roll = np.array([
        [1, 0, 0, 0],
        [0, np.cos(roll), -np.sin(roll), 0],
        [0, np.sin(roll), np.cos(roll), 0],
        [0, 0, 0, 1]
    ])
    
    Pitch = np.array([
        [np.cos(pitch), 0, np.sin(pitch), 0],
        [0, 1, 0, 0],
        [-np.sin(pitch), 0, np.cos(pitch), 0],
        [0, 0, 0, 1]
    ])
    
    Yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0, 0],
        [np.sin(yaw), np.cos(yaw), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    return np.matmul(np.matmul(Roll, Pitch), Yaw)


def TransformVector(xyz_coord, rotation, translation):
    """
    Transforms a vector by rotation THEN translation
    
    Args:
        xyz_coord: 3-vector to transform
        rotation: 4x4 rotation matrix
        translation: 3-vector translation
    
    Returns:
        Transformed 3-vector
    """
    xyz_vec = np.append(xyz_coord, 1.0)
    
    trans = np.eye(4)
    trans[0, 3] = translation[0]
    trans[1, 3] = translation[1]
    trans[2, 3] = translation[2]
    
    Transformed = np.dot(np.dot(rotation, trans), xyz_vec)
    return Transformed[:3]
