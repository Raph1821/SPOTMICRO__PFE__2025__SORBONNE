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
        
        # Get parameters
        self.use_gui = rospy.get_param('~use_gui', False)
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
        
        # Map servo numbers to joint indices
        self.servo_to_joint_map = self._create_servo_joint_map()
        
        # Servo angle storage (for commanded positions)
        self.servo_angles = {}
        
        # Publishers
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/servos_proportional', ServoArray, self.servo_callback)
        
        rospy.loginfo("SpotMicro PyBullet Simulation Ready!")
        rospy.loginfo(f"Servo to joint map: {self.servo_to_joint_map}")
    
    def _create_servo_joint_map(self):
        """
        Map servo numbers (from config) to PyBullet joint indices
        Based on spot_micro_motion_cmd.yaml:
        RF = Right Front, RB = Right Back, LF = Left Front, LB = Left Back
        _1 = shoulder, _2 = leg, _3 = foot
        """
        servo_map = {}
        
        # Get all joint info from PyBullet
        num_joints = p.getNumJoints(self.robot_id)
        rospy.loginfo(f"Robot has {num_joints} joints")
        
        # Joint name mapping from URDF
        joint_name_to_servo = {
            'front_right_shoulder': 1,  # RF_3
            'front_right_leg': 2,       # RF_2
            'front_right_foot': 3,      # RF_1
            'rear_right_shoulder': 4,   # RB_3
            'rear_right_leg': 5,        # RB_2
            'rear_right_foot': 6,       # RB_1
            'rear_left_shoulder': 7,    # LB_3
            'rear_left_leg': 8,         # LB_2
            'rear_left_foot': 9,        # LB_1
            'front_left_shoulder': 10,  # LF_3
            'front_left_leg': 11,       # LF_2
            'front_left_foot': 12,      # LF_1
        }
        
        # Build reverse map (servo number -> joint index)
        for joint_idx in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, joint_idx)
            joint_name = joint_info[1].decode('utf-8')
            
            if joint_name in joint_name_to_servo:
                servo_num = joint_name_to_servo[joint_name]
                servo_map[servo_num] = joint_idx
                rospy.loginfo(f"  Servo {servo_num} -> Joint {joint_idx} ({joint_name})")
        
        return servo_map
    
    def servo_callback(self, msg):
        """
        Receive servo commands from spot_micro_motion_cmd
        msg.servos contains list of Servo with .servo (number) and .value (angle in radians)
        """
        for servo in msg.servos:
            servo_num = servo.servo
            angle_rad = servo.value
            
            # Store commanded angle
            self.servo_angles[servo_num] = angle_rad
            
            # Apply to PyBullet if we have this joint mapped
            if servo_num in self.servo_to_joint_map:
                joint_idx = self.servo_to_joint_map[servo_num]
                
                # Set joint position with position control
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=angle_rad,
                    force=500,  # Max force
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
