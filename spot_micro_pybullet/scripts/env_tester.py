#!/usr/bin/env python3
"""
SpotMicro PyBullet Environment Tester
类似 spot_mini_mini 的 env_tester.py，但适配 SPOTMICRO 项目
提供 GUI 参数滑块来实时调整机器人姿态和步态参数
独立运行，不依赖 ROS
"""
import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import sys
import argparse
import math
import re

# 导入 SPOTMICRO 运动学类
try:
    from spot_micro_kinematics_fixed import SpotMicroKinematicsFixed as SpotMicroKinematics
    print("使用修复版运动学类")
except ImportError:
    from spot_micro_kinematics import SpotMicroKinematics
    print("使用原版运动学类")
from simple_gait_generator import SimpleGaitGenerator

# ARGUMENTS
descr = "SpotMicro PyBullet Environment Tester (类似 spot_mini_mini 的 env_tester.py)"
parser = argparse.ArgumentParser(description=descr)
parser.add_argument("-hf", "--HeightField", help="Use HeightField", action='store_true')
parser.add_argument("-r", "--DebugRack", help="Put Robot on an Elevated Rack", action='store_true')
parser.add_argument("-p", "--DebugPath", help="Draw Robot's Foot Path", action='store_true')
parser.add_argument("--urdf", help="Path to URDF file", type=str, default="")
parser.add_argument("--test-joints", help="Test mode: set all joints to 0 to verify control works", action='store_true')
ARGS = parser.parse_args()


def resolve_urdf_paths(urdf_path):
    """
    将 URDF 文件中的 package:// 路径转换为绝对路径
    """
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)  # spot_micro_pybullet 目录
    
    # 读取 URDF 文件
    with open(urdf_path, 'r', encoding='utf-8') as f:
        urdf_content = f.read()
    
    # 替换 package:// 路径
    def replace_package(match):
        package_name = match.group(1)
        relative_path = match.group(2)
        
        if package_name == 'spot_micro_pybullet':
            # 转换为绝对路径
            abs_path = os.path.join(package_dir, relative_path)
            # 标准化路径（处理 .. 和 .）
            abs_path = os.path.normpath(abs_path)
            # 检查文件是否存在
            if not os.path.exists(abs_path):
                print(f"警告: 文件不存在: {abs_path}")
            # 只返回路径部分，不包含引号（因为引号已经在 XML 中了）
            return abs_path
        else:
            # 其他包暂时保持原样
            return match.group(0)
    
    # 匹配 package://package_name/path/to/file（在引号内）
    # 注意：需要匹配 filename="package://..." 中的 package:// 部分
    pattern = r'package://([^/]+)/([^"]+)'
    urdf_content = re.sub(pattern, replace_package, urdf_content)
    
    # 调试：检查替换后的前几行
    lines = urdf_content.split('\n')
    print(f"URDF 解析: 检查前 5 行替换结果...")
    for i, line in enumerate(lines[:5]):
        if 'filename=' in line:
            print(f"  行 {i+1}: {line.strip()[:100]}")
    
    # 保存临时 URDF 文件
    temp_urdf = urdf_path + '.temp'
    with open(temp_urdf, 'w', encoding='utf-8') as f:
        f.write(urdf_content)
    
    return temp_urdf


class SpotMicroGUI:
    """GUI 类，提供参数滑块控制"""
    def __init__(self, robot_id):
        time.sleep(0.5)
        
        self.robot_id = robot_id
        self.cyaw = 0
        self.cpitch = -7
        self.cdist = 0.66
        
        # 身体位置和姿态参数
        self.xId = p.addUserDebugParameter("x", -0.10, 0.10, 0.0)
        self.yId = p.addUserDebugParameter("y", -0.10, 0.10, 0.0)
        self.zId = p.addUserDebugParameter("z", -0.10, 0.20, 0.155)  # 默认站立高度
        self.rollId = p.addUserDebugParameter("roll", -np.pi/4, np.pi/4, 0.0)
        self.pitchId = p.addUserDebugParameter("pitch", -np.pi/4, np.pi/4, 0.0)
        self.yawId = p.addUserDebugParameter("yaw", -np.pi/4, np.pi/4, 0.0)
        
        # 步态参数（适配 SPOTMICRO 的 8 相步态）
        self.StepLengthID = p.addUserDebugParameter("Step Length", -0.1, 0.1, 0.0)
        self.YawRateId = p.addUserDebugParameter("Yaw Rate", -1.0, 1.0, 0.0)
        self.LateralFractionId = p.addUserDebugParameter("Lateral Fraction", -0.5, 0.5, 0.0)
        self.StepVelocityId = p.addUserDebugParameter("Step Velocity", 0.001, 3.0, 0.1)
        
        # 足端参数
        self.ClearanceHeightId = p.addUserDebugParameter("Clearance Height", 0.0, 0.1, 0.05)
        self.PenetrationDepthId = p.addUserDebugParameter("Penetration Depth", 0.0, 0.05, 0.005)
    
    def UserInput(self):
        """读取 GUI 参数"""
        robot_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        p.resetDebugVisualizerCamera(
            cameraDistance=self.cdist,
            cameraYaw=self.cyaw,
            cameraPitch=self.cpitch,
            cameraTargetPosition=robot_pos
        )
        
        # 键盘控制相机
        keys = p.getKeyboardEvents()
        if keys.get(100):  # D
            self.cyaw += 1
        if keys.get(97):   # A
            self.cyaw -= 1
        if keys.get(99):   # C
            self.cpitch += 1
        if keys.get(102):  # F
            self.cpitch -= 1
        if keys.get(122):  # Z
            self.cdist += 0.01
        if keys.get(120):  # X
            self.cdist -= 0.01
        if keys.get(27):   # ESC
            p.disconnect()
            sys.exit()
        
        # 读取参数
        pos = np.array([
            p.readUserDebugParameter(self.xId),
            p.readUserDebugParameter(self.yId),
            p.readUserDebugParameter(self.zId)
        ])
        orn = np.array([
            p.readUserDebugParameter(self.rollId),
            p.readUserDebugParameter(self.pitchId),
            p.readUserDebugParameter(self.yawId)
        ])
        StepLength = p.readUserDebugParameter(self.StepLengthID)
        YawRate = p.readUserDebugParameter(self.YawRateId)
        LateralFraction = p.readUserDebugParameter(self.LateralFractionId)
        StepVelocity = p.readUserDebugParameter(self.StepVelocityId)
        ClearanceHeight = p.readUserDebugParameter(self.ClearanceHeightId)
        PenetrationDepth = p.readUserDebugParameter(self.PenetrationDepthId)
        
        return pos, orn, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, PenetrationDepth


def get_urdf_path():
    """获取 URDF 文件路径"""
    if ARGS.urdf:
        if os.path.exists(ARGS.urdf):
            return ARGS.urdf
        else:
            print(f"错误: 指定的 URDF 文件不存在: {ARGS.urdf}")
            sys.exit(1)
    
    # 默认路径（相对于脚本位置）
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, '..', 'urdf', 'spot_micro_pybullet_gen_ros.urdf')
    
    if os.path.exists(urdf_path):
        return urdf_path
    
    print("错误: 无法找到 URDF 文件")
    print(f"请检查路径: {urdf_path}")
    print("或使用 --urdf 参数指定 URDF 文件路径")
    sys.exit(1)


def main():
    """主函数"""
    print("=" * 60)
    print("启动 SpotMicro PyBullet 测试环境")
    print("=" * 60)
    
    # 连接 PyBullet
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0/240.0)
    
    # 加载地面
    if ARGS.HeightField:
        # 可以加载高度场地形
        plane_id = p.loadURDF("plane.urdf")
        print("使用高度场地形")
    else:
        plane_id = p.loadURDF("plane.urdf")
    
    # 获取并处理 URDF 文件
    urdf_path = get_urdf_path()
    print(f"找到 URDF 文件: {urdf_path}")
    
    # 解析 package:// 路径
    try:
        resolved_urdf = resolve_urdf_paths(urdf_path)
        print(f"已解析 URDF 路径，使用临时文件: {resolved_urdf}")
    except Exception as e:
        print(f"警告: 无法解析 URDF 路径: {e}")
        print("尝试直接加载原始 URDF（可能失败）")
        resolved_urdf = urdf_path
    
    # 加载机器人 URDF
    if ARGS.DebugRack:
        start_pos = [0, 0, 1.0]  # 放在架子上
        print("调试模式: 机器人放在架子上")
    else:
        start_pos = [0, 0, 0.3]  # 正常位置
    
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    
    try:
        robot_id = p.loadURDF(resolved_urdf, start_pos, start_orientation, useFixedBase=False)
        print(f"✓ 机器人已加载，ID: {robot_id}")
        
        # 禁用所有关节的默认控制，允许我们手动控制
        # 同时设置关节阻尼来减少摆动
        num_joints = p.getNumJoints(robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(robot_id, i)
            joint_type = joint_info[2]
            if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                # 禁用默认的关节控制
                p.setJointMotorControl2(
                    robot_id,
                    i,
                    p.VELOCITY_CONTROL,
                    targetVelocity=0,
                    force=0
                )
                # 设置关节阻尼来减少摆动
                p.changeDynamics(robot_id, i, linearDamping=0.0, angularDamping=0.5)
        print("✓ 已禁用默认关节控制并设置阻尼")
    except Exception as e:
        print(f"✗ 加载机器人失败: {e}")
        print("\n可能的原因:")
        print("1. URDF 文件中的 STL 文件路径不正确")
        print("2. STL 文件不存在")
        print("3. package:// 路径解析失败")
        print("\n建议:")
        print("- 检查 URDF 文件中的 mesh 路径")
        print("- 确保所有 STL 文件存在于 urdf/stl/ 目录下")
        p.disconnect()
        sys.exit(1)
    
    # 创建 GUI
    gui = SpotMicroGUI(robot_id)
    
    # 创建运动学对象（使用 SPOTMICRO 的实际参数）
    kinematics = SpotMicroKinematics()
    print("✓ 已初始化 SPOTMICRO 运动学模型")
    print(f"  身体尺寸: {kinematics.body_length}x{kinematics.body_width}m")
    print(f"  腿长: 髋={kinematics.hip_link_length}m, 上腿={kinematics.upper_leg_link_length}m, 下腿={kinematics.lower_leg_link_length}m")
    
    # 创建步态生成器
    gait_generator = SimpleGaitGenerator(dt=1.0/240.0)
    print("✓ 已初始化步态生成器")
    
    # 获取关节信息（重新获取，因为上面已经获取过了）
    num_joints = p.getNumJoints(robot_id)
    print(f"✓ 机器人有 {num_joints} 个关节")
    
    # 创建关节映射（使用 SPOTMICRO 的关节命名）
    joint_name_to_index = {}
    joint_names = []
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        joint_name_to_index[joint_name] = i
        if joint_name and joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:  # 只显示可动关节
            joint_names.append(joint_name)
            print(f"  关节 {i}: {joint_name} (类型: {joint_type})")
    
    # 验证关节映射
    print("\n验证关节映射:")
    all_joints_found = True
    for leg_name in ['RF', 'RB', 'LB', 'LF']:
        for joint_name in kinematics.joint_names[leg_name]:
            if joint_name in joint_name_to_index:
                print(f"  ✓ {leg_name} - {joint_name}: 关节 {joint_name_to_index[joint_name]}")
            else:
                print(f"  ✗ {leg_name} - {joint_name}: 未找到!")
                all_joints_found = False
    
    if not all_joints_found:
        print("\n警告: 部分关节未找到，可能影响控制")
    
    # 主循环
    print("\n" + "=" * 60)
    print("开始仿真循环...")
    print("=" * 60)
    print("使用右侧滑块调整参数")
    print("键盘控制: A/D-左右旋转, C/F-上下俯仰, Z/X-缩放, ESC-退出")
    print("=" * 60 + "\n")
    
    t = 0
    # 直接使用参考角度作为初始姿态（确保初始化正确）
    print("\n使用参考初始角度（半蹲状态，基于spot_mini_mini）")
    # 使用参考角度（基于spot_mini_mini的stand姿态）
    # 注意：这些角度是经过验证的，能确保机器人正确初始化
    initial_stand_angles = {}
    # 前右腿 (RF) - 参考: 0.15192765, -0.7552236, 1.5104472
    initial_stand_angles['front_right_shoulder'] = 0.15   # 约8.6°
    initial_stand_angles['front_right_leg'] = -0.75       # 约-43°
    initial_stand_angles['front_right_foot'] = 1.5         # 约86°
    # 后右腿 (RB) - 参考: 0.15192765, -0.7552236, 1.5104472
    initial_stand_angles['rear_right_shoulder'] = 0.15
    initial_stand_angles['rear_right_leg'] = -0.75
    initial_stand_angles['rear_right_foot'] = 1.5
    # 后左腿 (LB) - 参考: -0.15192765, -0.7552236, 1.5104472
    initial_stand_angles['rear_left_shoulder'] = -0.15
    initial_stand_angles['rear_left_leg'] = -0.75
    initial_stand_angles['rear_left_foot'] = 1.5
    # 前左腿 (LF) - 参考: -0.15192765, -0.7552236, 1.5104472
    initial_stand_angles['front_left_shoulder'] = -0.15
    initial_stand_angles['front_left_leg'] = -0.75
    initial_stand_angles['front_left_foot'] = 1.5
    
    print("参考角度:")
    for leg_name in ['RF', 'RB', 'LB', 'LF']:
        for joint_name in kinematics.joint_names[leg_name]:
            if joint_name in initial_stand_angles:
                angle = initial_stand_angles[joint_name]
                print(f"  {joint_name}: {math.degrees(angle):.2f}°")
    
    # 初始化默认足端位置
    default_foot_pos = kinematics.get_default_foot_positions(body_height=0.155)
    gait_generator.set_default_foot_positions(default_foot_pos)
    
    # 确保步态生成器时间从0开始
    gait_generator.reset_time()
    
    # 跟踪上一次的运动状态，用于检测步态开始/停止
    last_has_motion = False
    
    try:
        while True:
            # 读取 GUI 输入
            pos, orn, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, PenetrationDepth = gui.UserInput()
            
            # 检测是否有运动命令
            has_motion = abs(StepLength) > 0.001 or abs(LateralFraction) > 0.001 or abs(YawRate) > 0.001
            
            # 如果从静止状态变为运动状态，重置步态生成器时间，确保平滑开始
            if has_motion and not last_has_motion:
                gait_generator.reset_time()
                print("步态开始，重置时间")
            
            # 如果从运动状态变为静止状态，也重置时间
            if not has_motion and last_has_motion:
                gait_generator.reset_time()
                print("步态停止，回到站立姿态")
            
            last_has_motion = has_motion
            
            # 更新步态生成器时间（只在有运动时更新）
            if has_motion:
                gait_generator.update_time()
            
            # 根据步态参数计算足端位置
            if has_motion:
                # 有运动命令，使用步态生成器
                foot_positions_body = gait_generator.get_foot_positions(
                    step_length=StepLength,
                    lateral_fraction=LateralFraction,
                    yaw_rate=YawRate,
                    step_velocity=max(StepVelocity, 0.01),  # 确保不为0
                    clearance_height=ClearanceHeight,
                    penetration_depth=PenetrationDepth,
                    body_height=pos[2]
                )
            else:
                # 没有运动命令，使用默认站立姿态
                # 平滑过渡：如果之前有运动，逐渐回到默认位置
                default_foot_pos = kinematics.get_default_foot_positions(body_height=pos[2])
                foot_positions_body = default_foot_pos
            
            # 将足端位置从身体坐标系转换到世界坐标系
            foot_positions_world = {}
            roll, pitch, yaw = orn
            cos_r, sin_r = math.cos(roll), math.sin(roll)
            cos_p, sin_p = math.cos(pitch), math.sin(pitch)
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            
            # ZYX 欧拉角旋转矩阵
            R = np.array([
                [cos_y*cos_p, cos_y*sin_p*sin_r - sin_y*cos_r, cos_y*sin_p*cos_r + sin_y*sin_r],
                [sin_y*cos_p, sin_y*sin_p*sin_r + cos_y*cos_r, sin_y*sin_p*cos_r - cos_y*sin_r],
                [-sin_p, cos_p*sin_r, cos_p*cos_r]
            ])
            
            for leg_name in ['RF', 'RB', 'LB', 'LF']:
                foot_pos_body = np.array(foot_positions_body[leg_name])
                foot_pos_world = pos + R @ foot_pos_body
                foot_positions_world[leg_name] = foot_pos_world.tolist()
            
            # 重要：不要每帧都重置身体位置，这会阻止物理仿真
            # 只在初始化时设置一次，之后让物理引擎自然工作
            # 身体位置应该通过足端位置和关节控制来自然维持
            # 如果需要调整身体位置，应该通过调整足端位置来实现
            if t == 0:
                # 只在第一帧设置初始位置
                quat = p.getQuaternionFromEuler([0, 0, 0])  # 初始姿态为0
                initial_pos = [0, 0, 0.3]  # 初始位置
                p.resetBasePositionAndOrientation(robot_id, initial_pos, quat)
                print("✓ 已设置初始位置（之后不再重置，让物理引擎工作）")
            
            # 测试模式：直接设置所有关节为 0，验证控制是否工作
            if ARGS.test_joints:
                if t == 0:
                    print("\n测试模式: 设置所有关节角度为 0")
                # 直接设置所有关节为 0
                for joint_name in joint_name_to_index.keys():
                    if 'shoulder' in joint_name or 'leg' in joint_name or 'foot' in joint_name:
                        joint_idx = joint_name_to_index[joint_name]
                        p.setJointMotorControl2(
                            robot_id,
                            joint_idx,
                            p.POSITION_CONTROL,
                            targetPosition=0.0,
                            force=1000,
                            maxVelocity=10.0
                        )
            else:
                # 正常模式：计算并应用关节角度
                try:
                    # 如果没有运动命令，直接使用初始站立姿态角度（确保初始化正确）
                    if not has_motion:
                        # 使用初始站立姿态角度（半蹲状态）
                        joint_angles = initial_stand_angles.copy()
                    else:
                        # 有运动命令，使用计算出的足端位置来计算关节角度
                        try:
                            # 调试：打印足端位置（特别是右侧腿）
                            if t % 120 == 0:  # 每0.5秒打印一次
                                print(f"\n=== 调试信息 (t={t/240:.2f}s) ===")
                                print(f"步态参数: StepLength={StepLength:.3f}, StepVelocity={StepVelocity:.3f}")
                                print("足端位置（世界坐标系）:")
                                for leg_name in ['RF', 'RB']:  # 只打印右侧腿
                                    if leg_name in foot_positions_world:
                                        foot_pos = foot_positions_world[leg_name]
                                        print(f"  {leg_name}: [{foot_pos[0]:.4f}, {foot_pos[1]:.4f}, {foot_pos[2]:.4f}]")
                            
                            # 启用逆运动学调试
                            kinematics._debug_ik = (t % 120 == 0)
                            joint_angles = kinematics.compute_all_joint_angles(pos, orn, foot_positions_world)
                            kinematics._debug_ik = False
                            
                            # 调试：检查计算出的角度是否合理（特别是右侧腿）
                            if t % 120 == 0:  # 每0.5秒打印一次
                                print("右侧腿关节角度:")
                                for leg_name in ['RF', 'RB']:
                                    print(f"  {leg_name}:")
                                    for joint_name in kinematics.joint_names[leg_name]:
                                        if joint_name in joint_angles:
                                            angle = joint_angles[joint_name]
                                            deg = math.degrees(angle)
                                            status = "⚠️" if abs(angle) > 2.0 or math.isnan(angle) or math.isinf(angle) else "✓"
                                            print(f"    {status} {joint_name}: {deg:.1f}° ({angle:.4f} rad)")
                                        else:
                                            print(f"    ✗ {joint_name}: 未找到!")
                                print("=" * 50)
                        except Exception as e:
                            if t % 240 == 0:
                                print(f"运动学计算错误: {e}")
                            # 出错时使用初始姿态
                            joint_angles = initial_stand_angles.copy()
                    
                    # 调试：检查关节角度是否有效
                    if t == 0:  # 只在第一帧打印
                        print("\n初始关节角度:")
                        for leg_name in ['RF', 'RB', 'LB', 'LF']:
                            for joint_name in kinematics.joint_names[leg_name]:
                                if joint_name in joint_angles:
                                    angle = joint_angles[joint_name]
                                    print(f"  {joint_name}: {math.degrees(angle):.2f}° ({angle:.4f} rad)")
                                    if math.isnan(angle) or math.isinf(angle):
                                        print(f"    ⚠️ 警告: 角度无效!")
                    
                    # 应用关节角度到 PyBullet
                    for joint_name, angle in joint_angles.items():
                        if joint_name in joint_name_to_index:
                            joint_idx = joint_name_to_index[joint_name]
                            
                            # 检查角度是否有效
                            if math.isnan(angle) or math.isinf(angle):
                                if t % 240 == 0:
                                    print(f"警告: {joint_name} 角度无效: {angle}")
                                continue  # 跳过无效角度
                            
                        # 使用位置控制，设置目标角度
                        # 添加位置增益和速度增益来减少摆动
                        p.setJointMotorControl2(
                            robot_id,
                            joint_idx,
                            p.POSITION_CONTROL,
                            targetPosition=angle,
                            force=1000,  # 最大力
                            maxVelocity=10.0,  # 最大速度
                            positionGain=2.0,  # 位置增益，减少摆动
                            velocityGain=0.1   # 速度增益，增加阻尼
                        )
                except Exception as e:
                    if t % 240 == 0:  # 每秒打印一次错误
                        print(f"运动学计算错误: {e}")
                        import traceback
                        traceback.print_exc()
            
            # 步进仿真
            p.stepSimulation()
            
            # 控制循环频率
            time.sleep(1.0/240.0)
            t += 1
            
            if t % 240 == 0:
                print(f"运行中... t={t/240:.1f}s, StepLength={StepLength:.3f}, StepVelocity={StepVelocity:.3f}, 身体高度={pos[2]:.3f}m")
    
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        # 清理临时文件
        if 'resolved_urdf' in locals() and resolved_urdf != urdf_path:
            try:
                os.remove(resolved_urdf)
                print(f"已清理临时文件: {resolved_urdf}")
            except:
                pass
        p.disconnect()
        print("仿真已关闭")


if __name__ == '__main__':
    main()

