#!/usr/bin/env python3
"""
SpotMicro PyBullet Simulation Node
Subscribes to servo commands from spot_micro_motion_cmd
Simulates physics in PyBullet (headless for speed)
Publishes joint states for RViz visualization
"""
import rospy
import pybullet as p
import pybullet_data
import numpy as np
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Header
from i2cpwm_board.msg import ServoArray, Servo
import math
import os
import tf2_ros
from geometry_msgs.msg import TransformStamped

class SpotMicroPyBulletSim:
    def __init__(self):
        rospy.init_node('spot_micro_pybullet_sim', anonymous=False)
        rospy.loginfo("Starting SpotMicro PyBullet Simulation (Headless Mode)")
        
        # Add servo center angle offsets (from config in degrees, convert to radians)
        self.servo_center_offsets = {
            1:  math.radians(88.2),   # RF_3
            2:  math.radians(-27.6),  # RF_2
            3:  math.radians(-5.4),   # RF_1
            4:  math.radians(85.8),   # RB_3
            5:  math.radians(-35.4),  # RB_2
            6:  math.radians(-4.4),   # RB_1
            7:  math.radians(-73.9),  # LB_3
            8:  math.radians(38.7),   # LB_2
            9:  math.radians(-0.4),   # LB_1
            10: math.radians(-82.8),  # LF_3
            11: math.radians(38.6),   # LF_2
            12: math.radians(-7.6),   # LF_1
        }
        
        rospy.loginfo(f"Servo center offsets loaded: {len(self.servo_center_offsets)} servos")


        # Get parameters
        self.use_gui = rospy.get_param('~use_gui', True)
        self.urdf_path = rospy.get_param('~urdf_path', '')
        self.update_rate = rospy.get_param('~rate', 50)  # Hz
        
        # Lidar parameters (from motion_cmd config or defaults)
        # RPLidar A1M8 specifications: 360 samples, 0.15-12m range
        self.lidar_enabled = rospy.get_param('~lidar_enabled', True)
        self.lidar_update_rate = rospy.get_param('~lidar_rate', 10)  # Hz (typical for RPLidar A1)
        self.lidar_samples = rospy.get_param('~lidar_samples', 360)  # 360 degrees
        self.lidar_min_range = rospy.get_param('~lidar_min_range', 0.15)  # meters
        self.lidar_max_range = rospy.get_param('~lidar_max_range', 12.0)  # meters
        self.lidar_angle_min = rospy.get_param('~lidar_angle_min', -math.pi)  # -180 degrees
        self.lidar_angle_max = rospy.get_param('~lidar_angle_max', math.pi)  # +180 degrees
        
        # Lidar position relative to base_link (from config or motion_cmd node)
        # Try to get from motion_cmd node config first, then use defaults
        self.lidar_x_pos = rospy.get_param('/spot_micro_motion_cmd/lidar_x_pos', 0.045)
        self.lidar_y_pos = rospy.get_param('/spot_micro_motion_cmd/lidar_y_pos', 0.0)
        self.lidar_z_pos = rospy.get_param('/spot_micro_motion_cmd/lidar_z_pos', 0.085)
        self.lidar_yaw_angle = rospy.get_param('/spot_micro_motion_cmd/lidar_yaw_angle', 180)
        
        rospy.loginfo(f"Lidar parameters: enabled={self.lidar_enabled}, rate={self.lidar_update_rate}Hz, "
                     f"samples={self.lidar_samples}, range=[{self.lidar_min_range}, {self.lidar_max_range}]m")
        
        # Connect to PyBullet
        if self.use_gui:
            p.connect(p.GUI)
            # Optimize GUI performance
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
            rospy.loginfo("PyBullet GUI mode enabled (slower)")
        else:
            p.connect(p.DIRECT)
            rospy.loginfo("PyBullet headless mode (fast)")
        
        # Setup simulation
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1./240.)
        
        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf")
        
        # Create test environment with obstacles
        enable_obstacles = rospy.get_param('~enable_obstacles', True)
        if enable_obstacles:
            self._create_test_environment()
        
        # Load robot URDF
        if not self.urdf_path:
            # Default to package URDF
            import rospkg
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('spot_micro_pybullet')
            self.urdf_path = os.path.join(pkg_path, 'urdf', 'spot_micro_pybullet_gen_ros.urdf')
        
        rospy.loginfo(f"Loading URDF: {self.urdf_path}")
        
        # Load robot at appropriate height
        # Start a bit away from origin to avoid immediate collisions
        start_pos = [-2.0, -2.0, 0.3]
        # Rotate robot 180 degrees clockwise (around Z-axis)
        start_orientation = p.getQuaternionFromEuler([0, 0, math.pi])
        self.robot_id = p.loadURDF(self.urdf_path, start_pos, start_orientation,
                                     useFixedBase=False)
        
        rospy.loginfo(f"Robot loaded with ID: {self.robot_id}")
        
        # Map servo numbers to joint indices and inversions
        self.servo_to_joint_map, self.servo_inversions = self._create_servo_joint_map()
        
        # Servo angle storage (for commanded positions)
        self.servo_angles = {}
        
        # Set initial pose to lie_down (squatting) pose immediately after loading
        # This prevents the robot from starting upright and then transitioning to squat
        self._set_initial_lie_down_pose()
        
        # Publishers
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # Lidar publisher
        if self.lidar_enabled:
            self.lidar_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
            self.lidar_link_index = self._get_lidar_link_index()
            self.lidar_last_update = rospy.Time.now()
            self.lidar_update_period = rospy.Duration(1.0 / self.lidar_update_rate)
            rospy.loginfo("Lidar sensor enabled and publisher created")
        
        # TF broadcaster for lidar_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Subscribers
        rospy.Subscriber('/servos_proportional', ServoArray, self.servo_callback)
        
        rospy.loginfo("SpotMicro PyBullet Simulation Ready!")
        rospy.loginfo(f"Servo to joint map: {self.servo_to_joint_map}")
        rospy.loginfo(f"Servo inversions: {self.servo_inversions}")
    
    def _create_servo_joint_map(self):
        """
        Map servo numbers (from config) to PyBullet joint indices
        Based on SpotMicro standard numbering:
        Link 1 = hip joint (abduction/adduction)
        Link 2 = upper leg joint (shoulder/thigh)
        Link 3 = lower leg joint (knee)
        
        RF_3, RF_2, RF_1 = Right Front (link3=knee, link2=shoulder, link1=hip) = servos 1, 2, 3
        RB_3, RB_2, RB_1 = Right Back (link3=knee, link2=shoulder, link1=hip) = servos 4, 5, 6
        LB_3, LB_2, LB_1 = Left Back (link3=knee, link2=shoulder, link1=hip) = servos 7, 8, 9
        LF_3, LF_2, LF_1 = Left Front (link3=knee, link2=shoulder, link1=hip) = servos 10, 11, 12
        
        Direction inversions from config:
        - RF_1 (servo 3): direction = -1
        - LB_1 (servo 9): direction = -1
        """
        servo_map = {}
        servo_inversions = {}
        
        # Get all joint info from PyBullet
        num_joints = p.getNumJoints(self.robot_id)
        rospy.loginfo(f"Robot has {num_joints} joints")
        
        # Joint name mapping from URDF with inversion flags
        # NOTE: In SpotMicro convention:
        # _1 suffix in URDF = hip (link 1)
        # _2 suffix in URDF = shoulder/upper leg (link 2)  
        # _3 suffix in URDF = knee/lower leg (link 3)
        # But in the config naming:
        # RF_1 = link 1 (hip), RF_2 = link 2 (shoulder), RF_3 = link 3 (knee)
        
        joint_configs = {
            # Right Front (RF) - servos 1, 2, 3
            'front_right_foot': (1, 1),      # RF_3 (knee/link3), direction: 1
            'front_right_leg': (2, 1),       # RF_2 (shoulder/link2), direction: 1
            'front_right_shoulder': (3, 1), # RF_1 (hip/link1), direction: -1 (INVERTED)
            
            # Right Back (RB) - servos 4, 5, 6
            'rear_right_foot': (4, 1),       # RB_3 (knee/link3), direction: 1
            'rear_right_leg': (5, 1),        # RB_2 (shoulder/link2), direction: 1
            'rear_right_shoulder': (6, 1),   # RB_1 (hip/link1), direction: 1
            
            # Left Back (LB) - servos 7, 8, 9
            'rear_left_foot': (7, -1),        # LB_3 (knee/link3), direction: 1
            'rear_left_leg': (8, -1),         # LB_2 (shoulder/link2), direction: 1
            'rear_left_shoulder': (9, 1),   # LB_1 (hip/link1), direction: -1 (INVERTED)
            
            # Left Front (LF) - servos 10, 11, 12
            'front_left_foot': (10, -1),      # LF_3 (knee/link3), direction: 1
            'front_left_leg': (11, -1),       # LF_2 (shoulder/link2), direction: 1
            'front_left_shoulder': (12, 1),  # LF_1 (hip/link1), direction: 1
        }
        
        # Build maps
        for joint_idx in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, joint_idx)
            joint_name = joint_info[1].decode('utf-8')
            
            if joint_name in joint_configs:
                servo_num, inversion = joint_configs[joint_name]
                servo_map[servo_num] = joint_idx
                servo_inversions[servo_num] = inversion
                inv_str = "INVERTED" if inversion == -1 else "normal"
                
                # Map servo number back to config name for logging
                servo_name_map = {
                    1: 'RF_3', 2: 'RF_2', 3: 'RF_1',
                    4: 'RB_3', 5: 'RB_2', 6: 'RB_1',
                    7: 'LB_3', 8: 'LB_2', 9: 'LB_1',
                    10: 'LF_3', 11: 'LF_2', 12: 'LF_1'
                }
                config_name = servo_name_map.get(servo_num, f"servo{servo_num}")
                
                # Map joint type
                joint_type_map = {
                    'shoulder': 'hip(link1)',
                    'leg': 'shoulder(link2)',
                    'foot': 'knee(link3)'
                }
                joint_type = 'unknown'
                for key, val in joint_type_map.items():
                    if key in joint_name:
                        joint_type = val
                        break
                
                rospy.loginfo(f"  {config_name} (servo {servo_num:2d}) -> Joint {joint_idx} ({joint_name:25s}) = {joint_type:15s} [{inv_str}]")
        
        return servo_map, servo_inversions
    
    def _set_initial_lie_down_pose(self):
        """
        Set robot to lie_down (squatting) pose immediately after loading URDF.
        This prevents the robot from starting upright and then transitioning to squat,
        which can cause dangerous forward tilting.
        
        The lie_down pose corresponds to proportional servo commands of 0,
        which means the joints should be at their center angles.
        """
        rospy.loginfo("Setting initial lie_down (squatting) pose...")
        
        # Disable default joint motors temporarily to allow direct position setting
        num_joints = p.getNumJoints(self.robot_id)
        for joint_idx in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, joint_idx)
            joint_type = joint_info[2]
            
            # Only set revolute joints
            if joint_type == p.JOINT_REVOLUTE:
                # Disable default motor to allow direct position control
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.VELOCITY_CONTROL,
                    force=0
                )
        
        # Set all servos to center angles (lie_down pose has proportional command = 0)
        for servo_num in range(1, 13):  # Servos 1-12
            if servo_num in self.servo_to_joint_map:
                joint_idx = self.servo_to_joint_map[servo_num]
                
                # Get center offset for this servo
                center_offset = self.servo_center_offsets.get(servo_num, 0.0)
                
                # Apply inversion if needed
                inversion = self.servo_inversions.get(servo_num, 1)
                adjusted_angle = center_offset * inversion
                
                # Reset joint state directly to the target position
                p.resetJointState(
                    bodyUniqueId=self.robot_id,
                    jointIndex=joint_idx,
                    targetValue=adjusted_angle,
                    targetVelocity=0.0
                )
        
        # Run a few simulation steps to let the robot settle into the pose
        # This ensures the robot is stable before starting the main loop
        for _ in range(100):  # Run 100 steps (~0.4 seconds at 240Hz)
            p.stepSimulation()
        
        # Re-enable position control for all joints so servo_callback can control them
        for servo_num in range(1, 13):  # Servos 1-12
            if servo_num in self.servo_to_joint_map:
                joint_idx = self.servo_to_joint_map[servo_num]
                
                # Get center offset for this servo
                center_offset = self.servo_center_offsets.get(servo_num, 0.0)
                
                # Apply inversion if needed
                inversion = self.servo_inversions.get(servo_num, 1)
                adjusted_angle = center_offset * inversion
                
                # Set joint to position control mode (same as in servo_callback)
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=adjusted_angle,
                    force=500,
                    maxVelocity=5.0
                )
        
        rospy.loginfo("Initial lie_down pose set successfully")
    
    def _create_test_environment(self):
        """
        Create a room-like environment with various obstacles for testing
        path planning and obstacle avoidance
        """
        rospy.loginfo("Creating test environment with obstacles...")
        
        # Room parameters
        room_size = 5.0  # 5x5 meters room
        wall_height = 1.0
        wall_thickness = 0.1
        
        # Colors (RGBA)
        wall_color = [0.8, 0.8, 0.8, 1.0]
        obstacle_color = [0.6, 0.3, 0.1, 1.0]
        
        # Create walls (4 walls forming a square room)
        # North wall
        wall_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[room_size/2, wall_thickness/2, wall_height/2]
        )
        wall_visual = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[room_size/2, wall_thickness/2, wall_height/2],
            rgbaColor=wall_color
        )
        p.createMultiBody(
            baseMass=0,  # Static object
            baseCollisionShapeIndex=wall_shape,
            baseVisualShapeIndex=wall_visual,
            basePosition=[0, room_size/2, wall_height/2]
        )
        
        # South wall
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=wall_shape,
            baseVisualShapeIndex=wall_visual,
            basePosition=[0, -room_size/2, wall_height/2]
        )
        
        # East wall
        wall_shape_ew = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[wall_thickness/2, room_size/2, wall_height/2]
        )
        wall_visual_ew = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[wall_thickness/2, room_size/2, wall_height/2],
            rgbaColor=wall_color
        )
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=wall_shape_ew,
            baseVisualShapeIndex=wall_visual_ew,
            basePosition=[room_size/2, 0, wall_height/2]
        )
        
        # West wall
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=wall_shape_ew,
            baseVisualShapeIndex=wall_visual_ew,
            basePosition=[-room_size/2, 0, wall_height/2]
        )
        
        # # Add various obstacles
        # # Minimum obstacle height: 0.4m (above lidar height ~0.24m when robot is standing)
        # # Lidar is at base_link (0.155m) + lidar_z_pos (0.085m) = ~0.24m
        # # Use 0.4m to ensure all obstacles are clearly detectable by lidar
        # min_obstacle_height = 0.4  # Ensure all obstacles are detectable by lidar
        
        # # Robot dimensions for spacing calculations
        # robot_width = 0.15  # Robot width (from footprint: 0.15m total)
        # min_passage_width = robot_width * 2.0 # Minimum passage width: 0.225m
        # box_half_size = 0.3  # Box half-extent (total width: 0.6m)
        # cylinder_radius = 0.15  # Cylinder radius (total width: 0.3m)
        
        # # Minimum distance between obstacle centers = obstacle_size + min_passage_width
        # min_box_spacing = box_half_size * 2 + min_passage_width  # 0.6 + 0.225 = 0.825m
        # min_cylinder_spacing = cylinder_radius * 2 + min_passage_width  # 0.3 + 0.225 = 0.525m
        
        # # 1. Box obstacles (like furniture)
        # # Positions spaced to ensure at least min_passage_width between obstacles
        # box_positions = [
        #     [1.2, 1.2, min_obstacle_height],   # Corner obstacle (top-right, orange box)
        #     [-1.5, 0.8, min_obstacle_height],  # Medium box (left side) - moved further
        #     [0.8, -1.5, min_obstacle_height + 0.1],  # Tall box (bottom-right) - moved further
        # ]
        
        # for i, pos in enumerate(box_positions):
        #     height = pos[2]
        #     box_shape = p.createCollisionShape(
        #         p.GEOM_BOX,
        #         halfExtents=[box_half_size, box_half_size, height]
        #     )
        #     box_visual = p.createVisualShape(
        #         p.GEOM_BOX,
        #         halfExtents=[box_half_size, box_half_size, height],
        #         rgbaColor=[0.7, 0.4, 0.2, 1.0]
        #     )
        #     p.createMultiBody(
        #         baseMass=0,
        #         baseCollisionShapeIndex=box_shape,
        #         baseVisualShapeIndex=box_visual,
        #         basePosition=[pos[0], pos[1], height]
        #     )
        
        # # 2. Cylindrical obstacles (like pillars or trash cans)
        # # Positions spaced to ensure at least min_passage_width between obstacles
        # cylinder_positions = [
        #     [-1.0, -1.2, max(0.4, min_obstacle_height)],  # Moved further
        #     [1.5, -0.8, max(0.3, min_obstacle_height)],   # Moved further
        # ]
        
        # for pos in cylinder_positions:
        #     height = pos[2]
        #     cyl_shape = p.createCollisionShape(
        #         p.GEOM_CYLINDER,
        #         radius=cylinder_radius,
        #         height=height*2
        #     )
        #     cyl_visual = p.createVisualShape(
        #         p.GEOM_CYLINDER,
        #         radius=cylinder_radius,
        #         length=height*2,
        #         rgbaColor=[0.5, 0.5, 0.5, 1.0]
        #     )
        #     p.createMultiBody(
        #         baseMass=0,
        #         baseCollisionShapeIndex=cyl_shape,
        #         baseVisualShapeIndex=cyl_visual,
        #         basePosition=[pos[0], pos[1], height]
        #     )
        
        # # 3. Sphere obstacles (like balls or rounded objects) - REMOVED
        # # Spheres were causing confusion, replaced with taller box obstacles
        # # If you want to add sphere obstacles back, uncomment below and ensure radius >= min_obstacle_height
        # # sphere_positions = [
        # #     [0.0, 1.5, min_obstacle_height],
        # #     [-0.5, -0.5, min_obstacle_height],
        # # ]
        # # 
        # # for pos in sphere_positions:
        # #     radius = min_obstacle_height  # Ensure sphere extends above lidar height
        # #     sphere_shape = p.createCollisionShape(
        # #         p.GEOM_SPHERE,
        # #         radius=radius
        # #     )
        # #     sphere_visual = p.createVisualShape(
        # #         p.GEOM_SPHERE,
        # #         radius=radius,
        # #         rgbaColor=[0.8, 0.2, 0.2, 1.0]
        # #     )
        # #     p.createMultiBody(
        # #         baseMass=0,
        # #         baseCollisionShapeIndex=sphere_shape,
        # #         baseVisualShapeIndex=sphere_visual,
        # #         basePosition=[pos[0], pos[1], radius]  # Position at radius height
        # #     )
        
        # # 4. Create a passage (two boxes forming a corridor)
        # # Ensure passage width is at least 1.5x robot width
        # passage_box_half_x = 0.2  # Box half-extent in X direction
        # passage_box_half_y = 0.8  # Box half-extent in Y direction
        # passage_height = min_obstacle_height
        
        # # Calculate passage width: distance between boxes - box widths
        # # passage_width = (right_pos - left_pos) - (2 * passage_box_half_x)
        # # We want: passage_width >= min_passage_width
        # # So: (right_pos - left_pos) >= min_passage_width + 2 * passage_box_half_x
        # #     = 0.225 + 0.4 = 0.625m
        # passage_center_gap = min_passage_width + 2 * passage_box_half_x  # 0.225 + 0.4 = 0.625m
        # passage_left_x = -0.5
        # passage_right_x = passage_left_x + passage_center_gap  # -0.5 + 0.625 = 0.125
        
        # passage_box_shape = p.createCollisionShape(
        #     p.GEOM_BOX,
        #     halfExtents=[passage_box_half_x, passage_box_half_y, passage_height]
        # )
        # passage_box_visual = p.createVisualShape(
        #     p.GEOM_BOX,
        #     halfExtents=[passage_box_half_x, passage_box_half_y, passage_height],
        #     rgbaColor=[0.4, 0.4, 0.6, 1.0]
        # )
        
        # # Left side of passage
        # p.createMultiBody(
        #     baseMass=0,
        #     baseCollisionShapeIndex=passage_box_shape,
        #     baseVisualShapeIndex=passage_box_visual,
        #     basePosition=[passage_left_x, 0.0, passage_height]
        # )
        
        # # Right side of passage
        # p.createMultiBody(
        #     baseMass=0,
        #     baseCollisionShapeIndex=passage_box_shape,
        #     baseVisualShapeIndex=passage_box_visual,
        #     basePosition=[passage_right_x, 0.0, passage_height]
        # )
        
        # actual_passage_width = passage_center_gap - 2 * passage_box_half_x
        # rospy.loginfo(f"Passage created: width={actual_passage_width:.3f}m (min required: {min_passage_width:.3f}m)")
        
        # # 5. Optional: Add a ramp for testing climbing
        # ramp_length = 1.0
        # ramp_width = 0.6
        # ramp_angle = 15 * (3.14159 / 180)  # 15 degrees
        
        # ramp_shape = p.createCollisionShape(
        #     p.GEOM_BOX,
        #     halfExtents=[ramp_length/2, ramp_width/2, 0.05]
        # )
        # ramp_visual = p.createVisualShape(
        #     p.GEOM_BOX,
        #     halfExtents=[ramp_length/2, ramp_width/2, 0.05],
        #     rgbaColor=[0.6, 0.6, 0.3, 1.0]
        # )
        
        # ramp_orientation = p.getQuaternionFromEuler([0, ramp_angle, 0])
        # p.createMultiBody(
        #     baseMass=0,
        #     baseCollisionShapeIndex=ramp_shape,
        #     baseVisualShapeIndex=ramp_visual,
        #     basePosition=[2.0, 1.5, 0.2],
        #     baseOrientation=ramp_orientation
        # )

        # Simple corner obstacles
        min_obstacle_height = 0.5  # Tall enough to be detected by lidar
        
        # BIG BOX - Top Right corner
        box_size = 0.5  # Large box (1.0m x 1.0m)
        box_shape_tr = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[box_size, box_size, min_obstacle_height]
        )
        box_visual_tr = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[box_size, box_size, min_obstacle_height],
            rgbaColor=[0.9, 0.5, 0.2, 1.0]  # Orange
        )
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=box_shape_tr,
            baseVisualShapeIndex=box_visual_tr,
            basePosition=[1.0, 1.0, min_obstacle_height]  # Top right
        )
        
        # BIG BOX - Bottom Left corner
        box_shape_bl = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[box_size, box_size, min_obstacle_height]
        )
        box_visual_bl = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[box_size, box_size, min_obstacle_height],
            rgbaColor=[0.9, 0.5, 0.2, 1.0]  # Orange
        )
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=box_shape_bl,
            baseVisualShapeIndex=box_visual_bl,
            basePosition=[-1.0, -1.0, min_obstacle_height]  # Bottom left
        )
        
        # BIG CYLINDER - Top Left corner
        cylinder_radius = 0.4  # Large cylinder (0.8m diameter)
        cyl_shape_tl = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=cylinder_radius,
            height=min_obstacle_height * 2
        )
        cyl_visual_tl = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=cylinder_radius,
            length=min_obstacle_height * 2,
            rgbaColor=[0.3, 0.6, 0.9, 1.0]  # Blue
        )
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=cyl_shape_tl,
            baseVisualShapeIndex=cyl_visual_tl,
            basePosition=[-1.0, 1.0, min_obstacle_height]  # Top left
        )
        
        # BIG CYLINDER - Bottom Right corner
        cyl_shape_br = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=cylinder_radius,
            height=min_obstacle_height * 2
        )
        cyl_visual_br = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=cylinder_radius,
            length=min_obstacle_height * 2,
            rgbaColor=[0.3, 0.6, 0.9, 1.0]  # Blue
        )
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=cyl_shape_br,
            baseVisualShapeIndex=cyl_visual_br,
            basePosition=[1.0, -1.0, min_obstacle_height]  # Bottom right
        )



        
        rospy.loginfo("Test environment created successfully!")
        rospy.loginfo(f"Room size: {room_size}x{room_size}m")
        # rospy.loginfo(f"Obstacles added: {len(box_positions)} boxes, {len(cylinder_positions)} cylinders")
        rospy.loginfo(f"Minimum obstacle height: {min_obstacle_height}m (above lidar height ~0.24m)")
    
    def _get_lidar_link_index(self):
        """Get the link index for lidar_link in PyBullet"""
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(-1, num_joints):  # -1 is base_link
            if i == -1:
                link_name = p.getBodyInfo(self.robot_id)[1].decode('utf-8')
            else:
                link_info = p.getJointInfo(self.robot_id, i)
                link_name = link_info[12].decode('utf-8')  # child frame name
            
            if link_name == 'lidar_link':
                return i
        rospy.logwarn("lidar_link not found in URDF, using base_link for lidar")
        return -1  # Fallback to base_link
    
    def _publish_lidar_scan(self):
        """Perform lidar scan and publish LaserScan message"""
        if not self.lidar_enabled:
            return
        
        # Check if it's time to update lidar
        now = rospy.Time.now()
        if (now - self.lidar_last_update) < self.lidar_update_period:
            return
        self.lidar_last_update = now
        
        # Get lidar position and orientation
        if self.lidar_link_index == -1:
            # Use base_link
            base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        else:
            # Get lidar_link pose
            link_state = p.getLinkState(self.robot_id, self.lidar_link_index)
            base_pos = link_state[0]  # world position
            base_orn = link_state[1]   # world orientation
        
        # Convert quaternion to rotation matrix for yaw
        orn_matrix = p.getMatrixFromQuaternion(base_orn)
        yaw = math.atan2(orn_matrix[3], orn_matrix[0])
        
        # Add lidar yaw offset
        lidar_yaw_rad = math.radians(self.lidar_yaw_angle)
        total_yaw = yaw + lidar_yaw_rad
        
        # Calculate lidar position in world frame
        # Apply lidar offset relative to base_link
        lidar_pos = [
            base_pos[0] + self.lidar_x_pos * math.cos(yaw) - self.lidar_y_pos * math.sin(yaw),
            base_pos[1] + self.lidar_x_pos * math.sin(yaw) + self.lidar_y_pos * math.cos(yaw),
            base_pos[2] + self.lidar_z_pos
        ]
        
        # Generate ray directions (360 degrees)
        angles = np.linspace(self.lidar_angle_min, self.lidar_angle_max, self.lidar_samples)
        
        # Prepare ray start and end positions
        ray_from = []
        ray_to = []
        
        for angle in angles:
            # Ray direction in lidar frame (horizontal plane)
            ray_dir_x = math.cos(total_yaw + angle)
            ray_dir_y = math.sin(total_yaw + angle)
            ray_dir_z = 0.0  # Horizontal scan
            
            # Ray start (lidar position)
            ray_from.append(lidar_pos)
            
            # Ray end (lidar position + direction * max_range)
            ray_to.append([
                lidar_pos[0] + ray_dir_x * self.lidar_max_range,
                lidar_pos[1] + ray_dir_y * self.lidar_max_range,
                lidar_pos[2] + ray_dir_z
            ])
        
        # Perform batch raycast
        ray_results = p.rayTestBatch(ray_from, ray_to)
        
        # Create LaserScan message
        scan_msg = LaserScan()
        scan_msg.header = Header()
        scan_msg.header.stamp = now
        scan_msg.header.frame_id = "lidar_link"
        
        scan_msg.angle_min = self.lidar_angle_min
        scan_msg.angle_max = self.lidar_angle_max
        scan_msg.angle_increment = (self.lidar_angle_max - self.lidar_angle_min) / self.lidar_samples
        scan_msg.time_increment = 1.0 / (self.lidar_update_rate * self.lidar_samples)
        scan_msg.scan_time = 1.0 / self.lidar_update_rate
        scan_msg.range_min = self.lidar_min_range
        scan_msg.range_max = self.lidar_max_range
        
        # Fill ranges
        scan_msg.ranges = []
        for i, result in enumerate(ray_results):
            hit_fraction = result[2]  # Fraction of ray length where hit occurred
            if hit_fraction > 0:
                range_val = hit_fraction * self.lidar_max_range
                # Clamp to valid range
                if range_val < self.lidar_min_range:
                    range_val = float('inf')  # Too close
                elif range_val > self.lidar_max_range:
                    range_val = float('inf')  # Too far
                scan_msg.ranges.append(range_val)
            else:
                scan_msg.ranges.append(float('inf'))  # No hit
        
        # Publish scan
        self.lidar_pub.publish(scan_msg)
    
    def _publish_lidar_tf(self):
        """Publish TF transform from base_link to lidar_link"""
        try:
            # Get base_link transform
            base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
            
            # Create transform from base_link to lidar_link
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "base_link"
            t.child_frame_id = "lidar_link"
            
            # Set translation (lidar offset from base_link)
            t.transform.translation.x = self.lidar_x_pos
            t.transform.translation.y = self.lidar_y_pos
            t.transform.translation.z = self.lidar_z_pos
            
            # Set rotation (lidar yaw angle)
            lidar_yaw_rad = math.radians(self.lidar_yaw_angle)
            quat = p.getQuaternionFromEuler([0, 0, lidar_yaw_rad])
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            rospy.logwarn(f"Failed to publish lidar TF: {e}")
    
    def servo_callback(self, msg):
        """
        Receive servo commands from spot_micro_motion_cmd
        Commands are RELATIVE to servo center angles, need to add offsets
        """
        for servo in msg.servos:
            servo_num = servo.servo
            angle_rad = servo.value  # This is relative to center
            
            # Store commanded angle
            self.servo_angles[servo_num] = angle_rad
            
            # Apply to PyBullet if we have this joint mapped
            if servo_num in self.servo_to_joint_map:
                joint_idx = self.servo_to_joint_map[servo_num]
                
                # Get center offset for this servo
                center_offset = self.servo_center_offsets.get(servo_num, 0.0)
                
                # Add offset: commanded angle is relative, PyBullet needs absolute
                absolute_angle = angle_rad + center_offset
                
                # Apply inversion if needed
                inversion = self.servo_inversions.get(servo_num, 1)
                adjusted_angle = absolute_angle * inversion
                
                # Set joint position with position control
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=adjusted_angle,
                    force=500,
                    maxVelocity=5.0
                )
    
    def publish_joint_states(self):
        """Publish current joint positions to /joint_states for RViz"""
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        
        num_joints = p.getNumJoints(self.robot_id)
        
        for joint_idx in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, joint_idx)
            joint_name = joint_info[1].decode('utf-8')
            joint_type = joint_info[2]
            
            # Only publish revolute/prismatic joints
            if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
                joint_state.name.append(joint_name)
                
                # Get actual joint state from simulation
                joint_state_info = p.getJointState(self.robot_id, joint_idx)
                position = joint_state_info[0]
                velocity = joint_state_info[1]
                
                joint_state.position.append(position)
                joint_state.velocity.append(velocity)
        
        self.joint_state_pub.publish(joint_state)
    
    def run(self):
        """Main simulation loop"""
        rate = rospy.Rate(self.update_rate)
        
        rospy.loginfo(f"Simulation running at {self.update_rate} Hz")
        
        while not rospy.is_shutdown():
            # Step physics simulation
            p.stepSimulation()
            
            # Publish joint states for RViz
            self.publish_joint_states()
            
            # Publish lidar scan
            if self.lidar_enabled:
                self._publish_lidar_scan()
                self._publish_lidar_tf()
            
            rate.sleep()
    
    def shutdown(self):
        """Clean shutdown"""
        rospy.loginfo("Shutting down PyBullet simulation")
        p.disconnect()

if __name__ == '__main__':
    try:
        sim = SpotMicroPyBulletSim()
        sim.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'sim' in locals():
            sim.shutdown()