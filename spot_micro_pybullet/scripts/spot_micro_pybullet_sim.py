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
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from i2cpwm_board.msg import ServoArray, Servo
import math
import os

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
        
        # Load robot URDF
        if not self.urdf_path:
            # Default to package URDF
            import rospkg
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path('spot_micro_pybullet')
            self.urdf_path = os.path.join(pkg_path, 'urdf', 'spot_micro_pybullet_gen_ros.urdf')
        
        rospy.loginfo(f"Loading URDF: {self.urdf_path}")
        
        # Load robot at appropriate height
        start_pos = [0, 0, 0.3]
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robot_id = p.loadURDF(self.urdf_path, start_pos, start_orientation,
                                     useFixedBase=False)
        
        rospy.loginfo(f"Robot loaded with ID: {self.robot_id}")
        
        # Map servo numbers to joint indices and inversions
        self.servo_to_joint_map, self.servo_inversions = self._create_servo_joint_map()
        
        # Servo angle storage (for commanded positions)
        self.servo_angles = {}
        
        # Publishers
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
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