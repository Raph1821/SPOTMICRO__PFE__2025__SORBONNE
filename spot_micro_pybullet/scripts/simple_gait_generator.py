#!/usr/bin/env python3
"""
简单的步态生成器
根据步态参数计算足端位置增量（相对于默认位置）
参考 spot_mini_mini 的 Bezier 步态实现
"""
import numpy as np
import math


class SimpleGaitGenerator:
    """
    简单的步态生成器
    根据 StepLength, StepVelocity, LateralFraction, YawRate 等参数
    计算足端位置增量（相对于默认位置）
    """
    def __init__(self, dt=1.0/240.0):
        self.dt = dt
        self.time = 0.0
        
        # 8 相步态参数（来自 yaml 配置）
        self.swing_time = 0.36  # 摆动时间
        self.stance_time = 0.36 * 3  # 支撑时间（3倍摆动时间，因为8相步态）
        self.phase_time = self.swing_time + self.stance_time  # 总相位时间
        
        # 相位偏移（8相步态：每次只有一条腿在摆动）
        # 顺序：RB(0) -> RF(0.25) -> LF(0.5) -> LB(0.75)
        self.phase_offsets = {
            'RB': 0.0,      # 相位 0
            'RF': 0.25,     # 相位 2
            'LF': 0.5,      # 相位 4
            'LB': 0.75      # 相位 6
        }
        
        # 默认足端位置（身体坐标系，相对于身体中心）
        self.default_foot_positions = {}
    
    def set_default_foot_positions(self, foot_positions):
        """设置默认足端位置（站立姿态）"""
        self.default_foot_positions = foot_positions.copy()
    
    def update_time(self, dt=None):
        """更新时间"""
        if dt is None:
            dt = self.dt
        self.time += dt
    
    def reset_time(self):
        """重置时间"""
        self.time = 0.0
    
    def get_foot_step_delta(self, phase, step_length, lateral_fraction, 
                            clearance_height, penetration_depth, is_swing):
        """
        计算单腿的步态增量（相对于默认位置）
        
        Args:
            phase: 当前相位（0-1）
            step_length: 步长（前后方向，注意：这是半步长）
            lateral_fraction: 横向分数（左右方向）
            clearance_height: 抬腿高度
            penetration_depth: 穿透深度
            is_swing: 是否在摆动相
        
        Returns:
            [delta_x, delta_y, delta_z] 相对于默认位置的增量
        """
        # 计算极坐标分量（用于处理横向运动）
        X_POLAR = np.cos(lateral_fraction) if abs(lateral_fraction) > 0.001 else 1.0
        Y_POLAR = np.sin(lateral_fraction) if abs(lateral_fraction) > 0.001 else 0.0
        
        if is_swing:
            # 摆动相：使用简化的抛物线轨迹
            # X方向：从 -step_length 到 +step_length
            x_delta = step_length * (2.0 * phase - 1.0)  # -step_length 到 +step_length
            
            # Y方向：横向分量
            y_delta = step_length * (2.0 * phase - 1.0) * Y_POLAR
            
            # Z方向：抛物线抬腿轨迹（最高点在中间）
            z_delta = clearance_height * 4.0 * phase * (1.0 - phase) - penetration_depth
            
            # 应用极坐标变换
            step_x = x_delta * X_POLAR
            step_y = y_delta if abs(Y_POLAR) > 0.001 else 0.0
            
        else:
            # 支撑相：使用线性轨迹
            # X方向：从 +step_length 到 -step_length（身体向前移动，足端相对向后）
            x_delta = step_length * (1.0 - 2.0 * phase)  # +step_length 到 -step_length
            
            # Y方向：横向分量
            y_delta = step_length * (1.0 - 2.0 * phase) * Y_POLAR
            
            # Z方向：在地面上（保持穿透深度）
            z_delta = -penetration_depth
            
            # 应用极坐标变换
            step_x = x_delta * X_POLAR
            step_y = y_delta if abs(Y_POLAR) > 0.001 else 0.0
        
        return np.array([step_x, step_y, z_delta])
    
    def get_foot_positions(self, step_length, lateral_fraction, yaw_rate, 
                          step_velocity, clearance_height, penetration_depth,
                          body_height):
        """
        根据步态参数计算足端位置（身体坐标系）
        
        Args:
            step_length: 步长（前后方向，这是半步长）
            lateral_fraction: 横向分数（左右方向）
            yaw_rate: 偏航速率
            step_velocity: 步速
            clearance_height: 抬腿高度
            penetration_depth: 穿透深度
            body_height: 身体高度
        
        Returns:
            字典，键为腿名称，值为足端位置（身体坐标系）
        """
        foot_positions = {}
        
        # 如果没有运动命令，直接返回默认位置
        if abs(step_length) < 0.001 and abs(lateral_fraction) < 0.001 and abs(yaw_rate) < 0.001:
            return self.default_foot_positions.copy()
        
        # 计算步态周期
        if step_velocity > 0.001:
            Tstance = 2.0 * abs(step_length) / abs(step_velocity) if abs(step_length) > 0.001 else 0.0
            Tstride = Tstance + self.swing_time
        else:
            Tstance = 0.0
            Tstride = float('inf')
        
        # 限制 Tstance 在合理范围内
        if Tstance > 1.3 * self.swing_time:
            Tstance = 1.3 * self.swing_time
        
        # 平滑过渡因子
        smooth_factor = min(1.0, self.time * 2.0)  # 0.5秒内逐渐增加到1.0
        effective_step_length = step_length * smooth_factor
        
        for leg_name in ['RF', 'RB', 'LB', 'LF']:
            # 获取默认位置（相对于身体中心）
            default_pos = np.array(self.default_foot_positions.get(leg_name, [0, 0, -body_height]))
            
            # 计算当前相位（0-1）
            phase_offset = self.phase_offsets[leg_name]
            
            if Tstride < float('inf') and Tstride > 0.001:
                # 使用简单的时间模运算计算相位
                phase = ((self.time / Tstride) + phase_offset) % 1.0
                
                # 判断是摆动相还是支撑相
                swing_phase_ratio = self.swing_time / Tstride
                
                if phase < swing_phase_ratio:
                    # 摆动相
                    is_swing = True
                    swing_phase = phase / swing_phase_ratio  # 归一化到 0-1
                else:
                    # 支撑相
                    is_swing = False
                    stance_phase = (phase - swing_phase_ratio) / (1.0 - swing_phase_ratio)  # 归一化到 0-1
                    swing_phase = stance_phase  # 复用变量名
            else:
                # 没有运动，使用默认位置
                is_swing = False
                swing_phase = 0.0
            
            # 计算步态增量
            step_delta = self.get_foot_step_delta(
                swing_phase, effective_step_length, lateral_fraction,
                clearance_height, penetration_depth, is_swing
            )
            
            # 应用到默认位置
            foot_pos = default_pos + step_delta
            
            # 限制足端位置，防止极端值
            max_x_offset = 0.05
            max_y_offset = 0.03
            max_z_offset = clearance_height + 0.02
            
            foot_pos[0] = np.clip(foot_pos[0], default_pos[0] - max_x_offset, default_pos[0] + max_x_offset)
            foot_pos[1] = np.clip(foot_pos[1], default_pos[1] - max_y_offset, default_pos[1] + max_y_offset)
            foot_pos[2] = np.clip(foot_pos[2], -body_height - 0.05, max_z_offset)
            
            # 偏航旋转（原地旋转）
            if abs(yaw_rate) > 0.001:
                yaw_angle = yaw_rate * self.time * smooth_factor
                # 获取腿相对于身体中心的位置
                leg_offset = foot_pos[:2] - default_pos[:2]
                # 旋转增量
                cos_yaw = math.cos(yaw_angle)
                sin_yaw = math.sin(yaw_angle)
                rotated_offset = np.array([
                    cos_yaw * leg_offset[0] - sin_yaw * leg_offset[1],
                    sin_yaw * leg_offset[0] + cos_yaw * leg_offset[1]
                ])
                foot_pos[0] = default_pos[0] + rotated_offset[0]
                foot_pos[1] = default_pos[1] + rotated_offset[1]
            
            foot_positions[leg_name] = foot_pos.tolist()
        
        return foot_positions
