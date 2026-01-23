#!/usr/bin/env python3
"""
Spot Micro RL Deployment Script
Deploy trained policy to real robot via ROS
"""

import rospy
import argparse
import numpy as np
import pickle
from collections import OrderedDict

from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

from spot_micro_rl.ars import ARSAgent
from spot_micro_rl.spot_kinematics import SpotModel
from spot_micro_rl.bezier_gait import BezierGait
from spot_micro_rl.lie_algebra import RPY, RpToTrans


class SpotRLController:
    """
    RL Controller for real Spot Micro robot
    Deploys trained policy via ROS topics
    """
    
    def __init__(self, model_path, control_freq=50):
        """
        Initialize RL controller
        
        Args:
            model_path: Path to trained model
            control_freq: Control frequency (Hz)
        """
        # Load model
        print(f"Loading model from: {model_path}")
        
        # Load config
        config_path = model_path.replace('.pkl', '_config.pkl')
        try:
            with open(config_path, 'rb') as f:
                self.config = pickle.load(f)
        except FileNotFoundError:
            print(f"Warning: Config file not found at {config_path}")
            print("Using default configuration")
            self.config = {
                'state_dim': 33,
                'action_dim': 14
            }
        
        # Create agent
        self.agent = ARSAgent(
            state_dim=self.config['state_dim'],
            action_dim=self.config['action_dim'],
            num_deltas=self.config.get('num_deltas', 16),
            step_size=self.config.get('step_size', 0.02),
            delta_std=self.config.get('delta_std', 0.03),
            num_best_deltas=self.config.get('num_best_deltas', 16),
            num_workers=1,
            rollout_length=1000,
            shift=self.config.get('shift', 0.0),
            seed=self.config.get('seed', 42)
        )
        
        # Load trained policy
        self.agent.load(model_path)
        print("✓ Model loaded successfully")
        
        # Initialize kinematics (from RL_app config)
        self.spot = SpotModel(
            shoulder_length=0.055,
            elbow_length=0.1075,
            wrist_length=0.130,
            hip_x=0.186,
            hip_y=0.078,
            foot_x=0.186,
            foot_y=0.17,
            height=0.155
        )
        
        # Initialize Bezier gait
        self.gait = BezierGait(
            dSref=[0.0, 0.5, 0.5, 0.0],  # Trot gait
            dt=1.0/control_freq,
            Tswing=0.15
        )
        
        # Control parameters
        self.control_freq = control_freq
        self.dt = 1.0 / control_freq
        
        # State variables
        self.current_state = np.zeros(self.config['state_dim'])
        self.desired_velocity = 0.0
        self.desired_yaw_rate = 0.0
        self.is_active = False
        
        # ROS initialization
        rospy.init_node('spot_rl_controller', anonymous=True)
        
        # Publishers
        self.joint_pub = rospy.Publisher(
            '/spot_rl/joint_commands',
            Float64MultiArray,
            queue_size=1
        )
        
        # Subscribers
        self.joint_state_sub = rospy.Subscriber(
            '/joint_states',
            JointState,
            self._joint_state_callback
        )
        
        self.imu_sub = rospy.Subscriber(
            '/imu/data',
            Imu,
            self._imu_callback
        )
        
        self.cmd_vel_sub = rospy.Subscriber(
            '/cmd_vel',
            Twist,
            self._cmd_vel_callback
        )
        
        # Control timer
        self.control_timer = rospy.Timer(
            rospy.Duration(self.dt),
            self._control_callback
        )
        
        print(f"✓ ROS node initialized")
        print(f"  Control frequency: {control_freq} Hz")
        print(f"  Publishing to: /spot_rl/joint_commands")
        print(f"  Subscribing to: /joint_states, /imu/data, /cmd_vel")
    
    def _joint_state_callback(self, msg):
        """Handle joint state updates"""
        # Update current state with joint positions and velocities
        # TODO: Map joint names to indices
        pass
    
    def _imu_callback(self, msg):
        """Handle IMU updates"""
        # Update current state with orientation and angular velocities
        # TODO: Extract roll, pitch, yaw from quaternion
        pass
    
    def _cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        self.desired_velocity = msg.linear.x
        self.desired_yaw_rate = msg.angular.z
        
        # Activate controller when velocity command received
        if abs(self.desired_velocity) > 0.01 or abs(self.desired_yaw_rate) > 0.01:
            self.is_active = True
    
    def _control_callback(self, event):
        """Main control loop"""
        if not self.is_active:
            return
        
        # Get action from policy
        action = self.agent.policy.evaluate(
            self.current_state,
            delta=None,
            direction=None
        )
        
        # Parse action
        joint_residuals = action[:12]
        clearance_height = 0.04 + action[12] * 0.02
        penetration_depth = max(0.0, action[13] * 0.01) if len(action) > 13 else 0.005
        
        # Generate nominal gait
        L = min(0.08, abs(self.desired_velocity) * 0.1)  # Step length
        LateralFraction = 0.0
        YawRate = self.desired_yaw_rate
        
        # Get default foot positions
        T_bf = OrderedDict()
        Rwb = np.eye(3)
        for key in ['FL', 'FR', 'BL', 'BR']:
            if key == 'FL':
                pos = [0.093, 0.085, -0.155]
            elif key == 'FR':
                pos = [0.093, -0.085, -0.155]
            elif key == 'BL':
                pos = [-0.093, 0.085, -0.155]
            else:  # BR
                pos = [-0.093, -0.085, -0.155]
            T_bf[key] = RpToTrans(Rwb, np.array(pos))
        
        # Generate trajectory
        T_bf_new = self.gait.GenerateTrajectory(
            L=L,
            LateralFraction=LateralFraction,
            YawRate=YawRate,
            vel=abs(self.desired_velocity),
            T_bf_=T_bf,
            T_bf_curr=T_bf,
            clearance_height=clearance_height,
            penetration_depth=penetration_depth,
            contacts=[1, 1, 1, 1]
        )
        
        # Solve inverse kinematics
        orn = [0, 0, 0]  # Get from IMU
        pos = [0, 0, 0.155]  # Body position
        joint_angles = self.spot.IK(orn, pos, T_bf_new)
        
        # Add learned residuals
        joint_angles_flat = joint_angles.flatten()
        joint_angles_final = joint_angles_flat + joint_residuals * 0.1
        
        # Publish joint commands
        msg = Float64MultiArray()
        msg.data = joint_angles_final.tolist()
        self.joint_pub.publish(msg)
    
    def run(self):
        """Run controller"""
        print("\n" + "=" * 60)
        print("Spot RL Controller Running")
        print("=" * 60)
        print("Send velocity commands to /cmd_vel to control the robot")
        print("Press Ctrl+C to stop")
        print("=" * 60 + "\n")
        
        rospy.spin()


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description='Deploy trained Spot Micro RL policy to real robot'
    )
    
    # Model arguments
    parser.add_argument('model_path', type=str,
                       help='Path to trained model (.pkl file)')
    
    # Control arguments
    parser.add_argument('--control_freq', type=int, default=50,
                       help='Control frequency in Hz (default: 50)')
    
    args = parser.parse_args()
    
    try:
        # Create and run controller
        controller = SpotRLController(
            model_path=args.model_path,
            control_freq=args.control_freq
        )
        controller.run()
        
    except rospy.ROSInterruptException:
        print("\nController stopped by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
