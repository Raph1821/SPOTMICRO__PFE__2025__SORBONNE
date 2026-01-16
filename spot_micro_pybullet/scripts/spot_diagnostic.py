#!/usr/bin/env python3
"""
SpotMicro Joint Diagnostic Tool
Run this to test each joint individually and identify direction/offset issues
"""
import rospy
import pybullet as p
import pybullet_data
import time

class JointDiagnostic:
    def __init__(self):
        rospy.init_node('joint_diagnostic', anonymous=False)
        
        # Connect to PyBullet GUI
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1./240.)
        
        # Load ground and robot
        self.plane_id = p.loadURDF("plane.urdf")
        
        import rospkg
        import os
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('spot_micro_pybullet')
        urdf_path = os.path.join(pkg_path, 'urdf', 'spot_micro_pybullet_gen_ros.urdf')
        
        start_pos = [0, 0, 0.3]
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robot_id = p.loadURDF(urdf_path, start_pos, start_orientation, useFixedBase=False)
        
        print("\n" + "="*60)
        print("SPOTMICRO JOINT DIAGNOSTIC TOOL")
        print("="*60)
        
        # Get all joints
        num_joints = p.getNumJoints(self.robot_id)
        print(f"\nRobot has {num_joints} joints\n")
        
        self.joints = {}
        for i in range(num_joints):
            info = p.getJointInfo(self.robot_id, i)
            joint_name = info[1].decode('utf-8')
            joint_type = info[2]
            
            if joint_type == p.JOINT_REVOLUTE:
                lower_limit = info[8]
                upper_limit = info[9]
                self.joints[i] = {
                    'name': joint_name,
                    'lower': lower_limit,
                    'upper': upper_limit
                }
                print(f"Joint {i}: {joint_name}")
                print(f"  Range: {lower_limit:.2f} to {upper_limit:.2f} rad")
                print(f"  Range: {lower_limit*57.3:.1f}° to {upper_limit*57.3:.1f}°\n")
    
    def test_joint(self, joint_idx):
        """Test a single joint through its range"""
        if joint_idx not in self.joints:
            print(f"Invalid joint index: {joint_idx}")
            return
        
        joint = self.joints[joint_idx]
        print(f"\n{'='*60}")
        print(f"Testing Joint {joint_idx}: {joint['name']}")
        print(f"{'='*60}")
        
        # Move through range
        steps = 20
        for i in range(steps + 1):
            t = i / steps
            angle = joint['lower'] + t * (joint['upper'] - joint['lower'])
            
            p.setJointMotorControl2(
                self.robot_id, joint_idx,
                p.POSITION_CONTROL,
                targetPosition=angle,
                force=500
            )
            
            for _ in range(10):
                p.stepSimulation()
                time.sleep(1./240.)
            
            print(f"  Angle: {angle:.3f} rad ({angle*57.3:.1f}°)")
        
        # Return to center
        center = (joint['lower'] + joint['upper']) / 2
        p.setJointMotorControl2(
            self.robot_id, joint_idx,
            p.POSITION_CONTROL,
            targetPosition=center,
            force=500
        )
        for _ in range(50):
            p.stepSimulation()
            time.sleep(1./240.)
    
    def run_interactive(self):
        """Interactive testing mode"""
        print("\nCommands:")
        print("  test <joint_num> - Test a specific joint")
        print("  all - Test all joints sequentially")
        print("  list - List all joints")
        print("  quit - Exit")
        
        while True:
            try:
                cmd = input("\n> ").strip().lower()
                
                if cmd == 'quit':
                    break
                elif cmd == 'list':
                    for idx, joint in self.joints.items():
                        print(f"Joint {idx}: {joint['name']}")
                elif cmd == 'all':
                    for idx in sorted(self.joints.keys()):
                        self.test_joint(idx)
                        time.sleep(1)
                elif cmd.startswith('test '):
                    try:
                        joint_idx = int(cmd.split()[1])
                        self.test_joint(joint_idx)
                    except (ValueError, IndexError):
                        print("Usage: test <joint_number>")
                else:
                    print("Unknown command")
            except KeyboardInterrupt:
                break
        
        p.disconnect()

if __name__ == '__main__':
    diag = JointDiagnostic()
    diag.run_interactive()
