#!/usr/bin/env python3
"""
SPOTMICRO 运动学计算类
适配 SPOTMICRO 项目的实际参数和关节命名
"""
import numpy as np
import math


class SpotMicroKinematics:
    """
    SPOTMICRO 运动学类
    使用来自 spot_micro_motion_cmd.yaml 的参数
    """
    def __init__(self):
        # 从 spot_micro_motion_cmd.yaml 中的参数
        self.hip_link_length = 0.055      # 髋关节连杆长度（水平）
        self.upper_leg_link_length = 0.1075  # 上腿长度
        self.lower_leg_link_length = 0.130   # 下腿长度
        self.body_width = 0.078            # 身体宽度（髋关节间距）
        self.body_length = 0.186           # 身体长度（肩关节间距）
        self.default_stand_height = 0.155  # 默认站立高度
        
        # 腿的位置（相对于身体中心，单位：米）
        # 注意：SPOTMICRO 使用 RF, RB, LB, LF 命名
        # RF = Right Front, RB = Right Back, LB = Left Back, LF = Left Front
        self.leg_positions = {
            'RF': [self.body_length/2, -self.body_width/2, 0],   # Right Front
            'RB': [-self.body_length/2, -self.body_width/2, 0],  # Right Back
            'LB': [-self.body_length/2, self.body_width/2, 0],    # Left Back
            'LF': [self.body_length/2, self.body_width/2, 0]       # Left Front
        }
        
        # 关节名称映射（SPOTMICRO 的命名方式）
        # 注意：SPOTMICRO 使用 front_left_shoulder, front_left_leg, front_left_foot
        # 而不是 spot_mini_mini 的 motor_front_left_shoulder 等
        self.joint_names = {
            'RF': ['front_right_shoulder', 'front_right_leg', 'front_right_foot'],
            'RB': ['rear_right_shoulder', 'rear_right_leg', 'rear_right_foot'],
            'LB': ['rear_left_shoulder', 'rear_left_leg', 'rear_left_foot'],
            'LF': ['front_left_shoulder', 'front_left_leg', 'front_left_foot']
        }
    
    def compute_leg_ik(self, leg_name, foot_pos_hip_frame):
        """
        计算单腿的逆运动学（相对于髋关节坐标系）
        
        Args:
            leg_name: 'RF', 'RB', 'LB', 'LF'
            foot_pos_hip_frame: 足端在髋关节坐标系中的位置 [x, y, z]
        
        Returns:
            [shoulder_angle, leg_angle, foot_angle] (弧度)
        """
        x, y, z = foot_pos_hip_frame
        
        # 计算到髋关节的距离（在水平面投影）
        # hip_link_length 是水平连杆长度
        proj_x = x - self.hip_link_length
        proj_y = y
        proj_dist = math.sqrt(proj_x**2 + proj_y**2)
        
        # 计算肩关节角度（绕 Z 轴旋转，控制腿的左右摆动）
        shoulder_angle = math.atan2(proj_y, proj_x)
        
        # 计算到足端的距离（在肩关节坐标系中）
        # 在肩关节坐标系中，x 方向是沿着上腿的方向
        leg_dist = math.sqrt(proj_dist**2 + z**2)
        
        # 使用余弦定理计算上腿角度
        # leg_dist^2 = upper_leg^2 + lower_leg^2 - 2*upper_leg*lower_leg*cos(pi - leg_angle)
        # 其中 leg_angle 是上腿与水平面的夹角（负值表示向下）
        cos_angle = (self.upper_leg_link_length**2 + self.lower_leg_link_length**2 - leg_dist**2) / \
                   (2 * self.upper_leg_link_length * self.lower_leg_link_length)
        
        # 限制在有效范围内
        cos_angle = max(-1.0, min(1.0, cos_angle))
        leg_joint_angle = math.acos(cos_angle) - math.pi  # 转换为向下为负的角度
        
        # 计算下腿角度（相对于上腿）
        # 使用正弦定理或直接计算
        sin_alpha = z / leg_dist if leg_dist > 0 else 0
        alpha = math.asin(sin_alpha) if abs(sin_alpha) <= 1.0 else (math.pi/2 if sin_alpha > 0 else -math.pi/2)
        
        # 计算下腿相对于上腿的角度
        # 这需要根据实际的关节配置调整
        # 简化处理：假设下腿角度使得足端到达目标位置
        beta = math.atan2(z, proj_dist) - leg_joint_angle
        foot_angle = beta
        
        return [shoulder_angle, leg_joint_angle, foot_angle]
    
    def compute_all_joint_angles(self, body_pos, body_orn, foot_positions_world):
        """
        计算所有关节角度
        
        Args:
            body_pos: 身体位置 [x, y, z] (世界坐标系)
            body_orn: 身体姿态 [roll, pitch, yaw] (弧度)
            foot_positions_world: 字典，键为腿名称 ('RF', 'RB', 'LB', 'LF')，
                                 值为足端世界坐标位置 [x, y, z]
        
        Returns:
            字典，键为关节名称，值为角度（弧度）
        """
        # 计算身体旋转矩阵
        roll, pitch, yaw = body_orn
        cos_r, sin_r = math.cos(roll), math.sin(roll)
        cos_p, sin_p = math.cos(pitch), math.sin(pitch)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        
        # ZYX 欧拉角旋转矩阵
        R = np.array([
            [cos_y*cos_p, cos_y*sin_p*sin_r - sin_y*cos_r, cos_y*sin_p*cos_r + sin_y*sin_r],
            [sin_y*cos_p, sin_y*sin_p*sin_r + cos_y*cos_r, sin_y*sin_p*cos_r - cos_y*sin_r],
            [-sin_p, cos_p*sin_r, cos_p*cos_r]
        ])
        
        joint_angles = {}
        
        # 对每条腿计算逆运动学
        for leg_name in ['RF', 'RB', 'LB', 'LF']:
            # 获取腿的髋关节位置（身体坐标系）
            hip_pos_body = np.array(self.leg_positions[leg_name])
            
            # 获取足端世界坐标
            foot_pos_world = np.array(foot_positions_world.get(leg_name, [0, 0, -self.default_stand_height]))
            
            # 将足端位置从世界坐标系转换到身体坐标系
            foot_pos_body = R.T @ (foot_pos_world - body_pos)
            
            # 计算足端相对于髋关节的位置（身体坐标系）
            foot_pos_hip_body = foot_pos_body - hip_pos_body
            
            # 计算关节角度（注意：这里需要根据实际的关节配置调整坐标系）
            # 简化：直接使用身体坐标系中的相对位置
            angles = self.compute_leg_ik(leg_name, foot_pos_hip_body)
            
            # 存储关节角度
            for i, joint_name in enumerate(self.joint_names[leg_name]):
                joint_angles[joint_name] = angles[i]
        
        return joint_angles
    
    def get_default_foot_positions(self, body_height=None):
        """
        获取默认的足端位置（站立姿态）
        注意：SPOTMICRO的站立姿态不是完全直立的，而是半蹲状态
        
        Args:
            body_height: 身体高度，如果为 None 则使用 default_stand_height
        
        Returns:
            字典，键为腿名称，值为足端位置（身体坐标系）
        """
        if body_height is None:
            body_height = self.default_stand_height
        
        foot_positions = {}
        for leg_name, hip_pos in self.leg_positions.items():
            # 默认足端位置：在髋关节正下方，高度为 body_height
            # 注意：这里需要根据实际的站立姿态调整
            # 对于半蹲姿态，足端应该在髋关节下方，但稍微向前/后偏移
            foot_x_offset = 0.0  # 可以根据需要调整前后偏移
            foot_y_offset = 0.0  # 可以根据需要调整左右偏移
            
            foot_positions[leg_name] = [
                hip_pos[0] + foot_x_offset,
                hip_pos[1] + foot_y_offset,
                -body_height
            ]
        
        return foot_positions
    
    def compute_stand_pose_angles(self, body_height=None):
        """
        计算站立姿态的关节角度（半蹲状态）
        这是通过计算默认足端位置的逆运动学得到的
        
        Returns:
            字典，键为关节名称，值为角度（弧度）
        """
        if body_height is None:
            body_height = self.default_stand_height
        
        # 获取默认足端位置
        foot_positions_body = self.get_default_foot_positions(body_height)
        
        # 计算关节角度（身体位置和姿态都为0）
        body_pos = np.array([0, 0, body_height])
        body_orn = np.array([0, 0, 0])
        
        # 转换到世界坐标系
        foot_positions_world = {}
        for leg_name, foot_pos_body in foot_positions_body.items():
            foot_positions_world[leg_name] = foot_pos_body
        
        # 计算关节角度
        joint_angles = self.compute_all_joint_angles(body_pos, body_orn, foot_positions_world)
        
        return joint_angles

