#!/usr/bin/env python3
"""
Bridge node: Converts servo commands from spot_micro_motion_cmd 
to Gazebo joint position commands
"""
import rospy
from i2cpwm_board.msg import ServoArray
from std_msgs.msg import Float64

class ServoToGazeboBridge:
    def __init__(self):
        rospy.init_node('servo_to_gazebo_bridge')
        
        # Mapping: servo number -> controller topic
        self.servo_to_controller = {
            1: '/front_right_shoulder_position_controller/command',
            2: '/front_right_leg_position_controller/command',
            3: '/front_right_foot_position_controller/command',
            4: '/rear_right_shoulder_position_controller/command',
            5: '/rear_right_leg_position_controller/command',
            6: '/rear_right_foot_position_controller/command',
            7: '/rear_left_shoulder_position_controller/command',
            8: '/rear_left_leg_position_controller/command',
            9: '/rear_left_foot_position_controller/command',
            10: '/front_left_shoulder_position_controller/command',
            11: '/front_left_leg_position_controller/command',
            12: '/front_left_foot_position_controller/command',
        }
        
        # Create publishers for each controller
        self.publishers = {}
        for servo_num, topic in self.servo_to_controller.items():
            self.publishers[servo_num] = rospy.Publisher(topic, Float64, queue_size=1)
        
        # Subscribe to servo commands
        rospy.Subscriber('/servos_proportional', ServoArray, self.servo_callback)
        
        rospy.loginfo("Servo to Gazebo bridge ready")
    
    def servo_callback(self, msg):
        """Convert servo commands to Gazebo joint commands"""
        for servo in msg.servos:
            servo_num = servo.servo
            angle = servo.value  # Already in radians
            
            if servo_num in self.publishers:
                cmd = Float64()
                cmd.data = angle
                self.publishers[servo_num].publish(cmd)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    bridge = ServoToGazeboBridge()
    bridge.run()
