#!/usr/bin/env python3
"""
SPOTMICRO 运动学计算类（修复版）
基于标准的3连杆逆运动学计算
"""
import numpy as np
import math


class SpotMicroKinematicsFixed:
    """
    SPOTMICRO 运动学类（修复版）
    使用标准的3连杆逆运动学
    """
    def __init__(self):
        # 从 spot_micro_motion_cmd.yaml 中的参数
        self.hip_link_length = 0.055      # 髋关节连杆长度（水平）
        self.upper_leg_link_length = 0.1075  # 上腿长度
        self.lower_leg_link_length = 0.130   # 下腿长度
        self.body_width = 0.078            # 身体宽度（髋关节间距）
        self.body_length = 0.186           # 身体长度（肩关节间距）
        self.default_stand_height = 0.155  # 默认站立高度
        
        # 存储上一次的关节角度，用于平滑
        self.last_joint_angles = {}
        
        # 腿的位置（相对于身体中心，单位：米）
        self.leg_positions = {
            'RF': [self.body_length/2, -self.body_width/2, 0],   # Right Front
            'RB': [-self.body_length/2, -self.body_width/2, 0],  # Right Back
            'LB': [-self.body_length/2, self.body_width/2, 0],    # Left Back
            'LF': [self.body_length/2, self.body_width/2, 0]       # Left Front
        }
        
        # 关节名称映射
        self.joint_names = {
            'RF': ['front_right_shoulder', 'front_right_leg', 'front_right_foot'],
            'RB': ['rear_right_shoulder', 'rear_right_leg', 'rear_right_foot'],
            'LB': ['rear_left_shoulder', 'rear_left_leg', 'rear_left_foot'],
            'LF': ['front_left_shoulder', 'front_left_leg', 'front_left_foot']
        }
    
    def compute_leg_ik(self, leg_name, foot_pos_hip_frame):
        """
        计算单腿的逆运动学（相对于髋关节坐标系）
        使用 SPOTMICRO 的 C++ 实现中的公式
        
        Args:
            leg_name: 'RF', 'RB', 'LB', 'LF'
            foot_pos_hip_frame: 足端在髋关节坐标系中的位置 [x, y, z]
        
        Returns:
            [shoulder_angle, leg_angle, foot_angle] (弧度)
        """
        x, y, z = foot_pos_hip_frame
        
        # 使用 SPOTMICRO 的逆运动学公式
        # l1 = hip_link_length, l2 = upper_leg, l3 = lower_leg
        l1 = self.hip_link_length
        l2 = self.upper_leg_link_length
        l3 = self.lower_leg_link_length
        
        # 计算最大可达距离
        max_reach = l1 + l2 + l3
        current_distance = math.sqrt(x*x + y*y + z*z)
        
        # 如果足端位置超出可达范围，将其限制在可达范围内
        if current_distance > max_reach:
            scale = max_reach / current_distance
            x = x * scale
            y = y * scale
            z = z * scale
        
        # 判断是前腿还是后腿（用于选择公式）
        # RF, RB 是 is_leg_12 = True
        # LF, LB 是 is_leg_12 = False
        is_leg_12 = leg_name in ['RF', 'RB']
        
        # 计算 D 值
        D = (x*x + y*y + z*z - l1*l1 - l2*l2 - l3*l3) / (2*l2*l3)
        
        # 限制 D 在有效范围内
        if D > 1.0:
            D = 1.0
        elif D < -1.0:
            D = -1.0
        
        # 计算 ang3 (foot_angle)
        if is_leg_12:
            foot_angle = math.atan2(math.sqrt(1 - D*D), D)
        else:
            foot_angle = math.atan2(-math.sqrt(1 - D*D), D)
        
        # 计算 ang2 (leg_angle)
        protected_sqrt_val = x*x + y*y - l1*l1
        
        # 如果 protected_sqrt_val 为负数或很小，说明足端位置太接近髋关节
        # 这种情况下，我们需要使用更合理的处理方式
        # 当 y=0 且 x 很小时，protected_sqrt_val 会很小或为负，导致计算出极端角度
        if protected_sqrt_val < l1*l1 * 0.1:  # 如果 protected_sqrt_val 小于 l1^2 的 10%
            # 使用一个基于实际距离的合理值
            # 计算实际距离
            dist_xy = math.sqrt(x*x + y*y)
            if dist_xy < l1 * 0.5:  # 如果距离太近，使用一个最小值
                protected_sqrt_val = l1*l1 * 0.25  # 使用 l1^2 的 25% 作为最小值
            else:
                # 使用实际距离的平方减去 l1^2，但确保不为负
                protected_sqrt_val = max(dist_xy*dist_xy - l1*l1, l1*l1 * 0.1)
        
        sqrt_val = math.sqrt(protected_sqrt_val)
        
        # 计算 leg_angle
        leg_angle = math.atan2(z, sqrt_val) - \
                   math.atan2(l3 * math.sin(foot_angle), l2 + l3 * math.cos(foot_angle))
        
        # 计算 ang1 (shoulder_angle)
        # 使用 SPOTMICRO 的原始公式（所有腿都相同）
        # 从 C++ 实现看：ang1 = atan2(y4, x4) + atan2(sqrt(protected_sqrt_val), -l1)
        # 当 y=0 时，atan2(0, x) 会根据 x 的正负返回 0 或 π，这会导致极端角度
        # 我们需要处理这种情况
        if abs(y) < 1e-6:  # y 接近 0
            if abs(x) < 1e-6:  # x 也接近 0，使用默认角度
                shoulder_angle = 0.0
            else:
                # y=0 但 x 不为 0，根据 x 的正负确定角度
                # 但需要限制在合理范围内（-90° 到 90°）
                base_angle = 0.0 if x > 0 else math.pi
                # 添加 atan2(sqrt_val, -l1) 的贡献，但限制总角度在合理范围内
                atan2_contrib = math.atan2(sqrt_val, -l1)
                shoulder_angle = base_angle + atan2_contrib
                # 将角度归一化到 -π 到 π 范围
                if shoulder_angle > math.pi:
                    shoulder_angle -= 2 * math.pi
                elif shoulder_angle < -math.pi:
                    shoulder_angle += 2 * math.pi
        else:
            # y 不为 0，正常计算
            shoulder_angle = math.atan2(y, x) + math.atan2(sqrt_val, -l1)
        
        # 调试输出（仅对右侧腿）
        if leg_name in ['RF', 'RB'] and hasattr(self, '_debug_ik') and self._debug_ik:
            print(f"      IK计算过程:")
            print(f"        x={x:.4f}, y={y:.4f}, z={z:.4f}")
            print(f"        D={D:.4f}, protected_sqrt={protected_sqrt_val:.4f}")
            print(f"        computed: shoulder={math.degrees(shoulder_angle):.1f}°, leg={math.degrees(leg_angle):.1f}°, foot={math.degrees(foot_angle):.1f}°")
        
        # 检查角度是否合理（防止极端值）
        if math.isnan(shoulder_angle) or math.isnan(leg_angle) or math.isnan(foot_angle):
            if leg_name in ['RF', 'RB'] and hasattr(self, '_debug_ik') and self._debug_ik:
                print(f"        ⚠️ 角度无效 (NaN)")
            return [0.0, -0.75, 1.5]
        
        if math.isinf(shoulder_angle) or math.isinf(leg_angle) or math.isinf(foot_angle):
            if leg_name in ['RF', 'RB'] and hasattr(self, '_debug_ik') and self._debug_ik:
                print(f"        ⚠️ 角度无效 (Inf)")
            return [0.0, -0.75, 1.5]
        
        # 检查角度范围（允许更大的范围，因为步态可能需要更大的角度）
        # 只检查极端值，不要过于严格
        # 注意：leg_angle 可能是负数（-π 到 0），foot_angle 可能是正数（0 到 π）
        # shoulder_angle 应该在 -π/2 到 π/2 之间（约 -90° 到 90°）
        # leg_angle 应该在 -π 到 0 之间（约 -180° 到 0°）
        # foot_angle 应该在 0 到 π 之间（约 0° 到 180°）
        
        # 检查 shoulder_angle 是否在合理范围内
        if abs(shoulder_angle) > math.pi / 2 + 0.5:  # 允许稍微超出 ±90°
            if leg_name in ['RF', 'RB'] and hasattr(self, '_debug_ik') and self._debug_ik:
                print(f"        ⚠️ shoulder角度超出范围: {math.degrees(shoulder_angle):.1f}°")
            # 如果超出范围，使用上一次的角度或默认角度
            last_key = f"{leg_name}_shoulder"
            if last_key in self.last_joint_angles:
                shoulder_angle = self.last_joint_angles[last_key]
            else:
                shoulder_angle = 0.0
        
        # 检查 leg_angle 是否在合理范围内（应该在 -π 到 0 之间）
        # 注意：leg_angle 通常是负数，范围大约在 -π 到 0 之间（约 -180° 到 0°）
        # 但实际使用中，leg_angle 可能在 -2.5 到 0 之间（约 -143° 到 0°）
        # 如果超出这个范围，可能是计算错误
        # 放宽范围检查，允许更大的角度变化（步态可能需要更大的角度）
        if leg_angle > 0.5 or leg_angle < -2.9:  # 允许稍微超出 -π（约 -3.14）
            if leg_name in ['RF', 'RB'] and hasattr(self, '_debug_ik') and self._debug_ik:
                print(f"        ⚠️ leg角度超出范围: {math.degrees(leg_angle):.1f}°")
            # 如果角度超出范围，尝试使用上一次的角度，但如果上一次的角度也不合理，使用默认角度
            last_key = f"{leg_name}_leg"
            if last_key in self.last_joint_angles:
                last_angle = self.last_joint_angles[last_key]
                # 如果上一次的角度在合理范围内，使用它；否则使用默认角度
                if -2.9 <= last_angle <= 0.5:
                    leg_angle = last_angle
                else:
                    leg_angle = -0.75
            else:
                leg_angle = -0.75
        
        # 检查 foot_angle 是否在合理范围内（应该在 0 到 π 之间）
        if foot_angle < -0.5 or foot_angle > math.pi + 0.5:
            if leg_name in ['RF', 'RB'] and hasattr(self, '_debug_ik') and self._debug_ik:
                print(f"        ⚠️ foot角度超出范围: {math.degrees(foot_angle):.1f}°")
            last_key = f"{leg_name}_foot"
            if last_key in self.last_joint_angles:
                foot_angle = self.last_joint_angles[last_key]
            else:
                foot_angle = 1.5
        
        # 存储当前角度
        self.last_joint_angles[f"{leg_name}_shoulder"] = shoulder_angle
        self.last_joint_angles[f"{leg_name}_leg"] = leg_angle
        self.last_joint_angles[f"{leg_name}_foot"] = foot_angle
        
        return [shoulder_angle, leg_angle, foot_angle]
    
    def compute_all_joint_angles(self, body_pos, body_orn, foot_positions_world):
        """
        计算所有关节角度
        
        Args:
            body_pos: 身体位置 [x, y, z] (世界坐标系)
            body_orn: 身体姿态 [roll, pitch, yaw] (弧度)
            foot_positions_world: 字典，键为腿名称，值为足端世界坐标位置 [x, y, z]
        
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
            
            # 调试输出（仅对右侧腿）
            if leg_name in ['RF', 'RB'] and hasattr(self, '_debug_ik'):
                if self._debug_ik:
                    print(f"    {leg_name} IK调试:")
                    print(f"      foot_pos_world: {foot_pos_world}")
                    print(f"      foot_pos_body: {foot_pos_body}")
                    print(f"      hip_pos_body: {hip_pos_body}")
                    print(f"      foot_pos_hip_body: {foot_pos_hip_body}")
            
            # 计算关节角度
            angles = self.compute_leg_ik(leg_name, foot_pos_hip_body)
            
            # 调试输出（仅对右侧腿）
            if leg_name in ['RF', 'RB'] and hasattr(self, '_debug_ik'):
                if self._debug_ik:
                    print(f"      computed angles: {[math.degrees(a) for a in angles]}")
            
            # 存储关节角度
            for i, joint_name in enumerate(self.joint_names[leg_name]):
                joint_angles[joint_name] = angles[i]
        
        return joint_angles
    
    def get_default_foot_positions(self, body_height=None):
        """
        获取默认的足端位置（站立姿态）
        根据配置文件，前腿有 stand_front_x_offset，后腿有 stand_back_x_offset
        
        注意：返回的是身体坐标系中的位置（相对于身体中心）
        足端应该在地面上，所以 z 坐标应该是 -body_height（相对于身体中心）
        """
        if body_height is None:
            body_height = self.default_stand_height
        
        # 从配置文件读取的偏移值
        stand_front_x_offset = 0.015   # 前腿的前后偏移（米）
        stand_back_x_offset = -0.000    # 后腿的前后偏移（米）
        
        foot_positions = {}
        for leg_name, hip_pos in self.leg_positions.items():
            # 根据前腿/后腿选择不同的 x 偏移
            if leg_name in ['RF', 'LF']:  # 前腿
                x_offset = stand_front_x_offset
            else:  # 后腿
                x_offset = stand_back_x_offset
            
            # 足端位置（身体坐标系，相对于身体中心）
            # x: 髋关节 x 位置 + 前后偏移
            # y: 髋关节 y 位置（左右位置不变）
            # z: -body_height（足端在地面上，身体中心在 body_height 高度）
            foot_positions[leg_name] = [
                hip_pos[0] + x_offset,  # 添加 x 偏移
                hip_pos[1],              # y 位置不变
                -body_height             # z 位置：足端在地面（z=0），身体中心在 body_height
            ]
        
        return foot_positions
    
    def compute_stand_pose_angles(self, body_height=None):
        """
        计算站立姿态的关节角度（半蹲状态）
        """
        if body_height is None:
            body_height = self.default_stand_height
        
        # 获取默认足端位置（身体坐标系）
        foot_positions_body = self.get_default_foot_positions(body_height)
        
        # 计算关节角度（身体位置和姿态都为0）
        body_pos = np.array([0, 0, body_height])
        body_orn = np.array([0, 0, 0])
        
        # 转换到世界坐标系
        # foot_pos_body是身体坐标系中的位置（相对于身体中心）
        # body_pos是世界坐标系中的身体位置
        # 当身体姿态为0时，foot_pos_world = body_pos + foot_pos_body
        foot_positions_world = {}
        for leg_name, foot_pos_body in foot_positions_body.items():
            foot_pos_world = body_pos + np.array(foot_pos_body)
            foot_positions_world[leg_name] = foot_pos_world.tolist()
        
        # 启用调试输出（仅对初始姿态计算）
        old_debug = getattr(self, '_debug_ik', False)
        self._debug_ik = True
        
        # 计算关节角度
        joint_angles = self.compute_all_joint_angles(body_pos, body_orn, foot_positions_world)
        
        # 恢复调试状态
        self._debug_ik = old_debug
        
        return joint_angles

