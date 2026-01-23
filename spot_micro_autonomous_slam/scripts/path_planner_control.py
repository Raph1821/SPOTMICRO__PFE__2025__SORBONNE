#!/usr/bin/env python3
"""
Control script for Path Planning Mode
Provides utilities to:
1. Programmatically set goals
2. Return to exploration mode
3. Monitor planner status
"""

import rospy
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
import sys

class PathPlannerControl:
    def __init__(self):
        rospy.init_node('path_planner_control', anonymous=True)
        
        self.goal_pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)
        self.return_pub = rospy.Publisher('/return_to_exploration', Bool, queue_size=1)
        self.mode_sub = rospy.Subscriber('planner_mode', String, self.mode_callback)
        self.current_goal_sub = rospy.Subscriber('current_goal', PointStamped, self.goal_callback)
        
        self.current_mode = "unknown"
        self.current_goal = None
    
    def mode_callback(self, msg):
        self.current_mode = msg.data
    
    def goal_callback(self, msg):
        self.current_goal = msg.point
    
    def set_goal(self, x, y, frame_id='map'):
        """Set a goal point"""
        msg = PointStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = rospy.Time.now()
        msg.point = Point(x=x, y=y, z=0.0)
        
        rospy.loginfo(f"Setting goal: ({x:.2f}, {y:.2f})")
        self.goal_pub.publish(msg)
    
    def return_to_exploration(self):
        """Return to exploration mode"""
        rospy.loginfo("Returning to exploration mode")
        self.return_pub.publish(Bool(True))
    
    def get_status(self):
        """Get planner status"""
        return {
            'mode': self.current_mode,
            'goal': self.current_goal
        }
    
    def print_status(self):
        """Print current status"""
        status = self.get_status()
        print(f"Mode: {status['mode']}")
        if status['goal']:
            print(f"Goal: ({status['goal'].x:.2f}, {status['goal'].y:.2f})")
        else:
            print("Goal: None")

def main():
    control = PathPlannerControl()
    rospy.sleep(1.0)  # Wait for connections
    
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 path_planner_control.py goal <x> <y>  - Set goal")
        print("  python3 path_planner_control.py return        - Return to exploration")
        print("  python3 path_planner_control.py status        - Print status")
        print("\nExample:")
        print("  python3 path_planner_control.py goal 1.5 2.0")
        return
    
    command = sys.argv[1]
    
    if command == "goal" and len(sys.argv) >= 4:
        try:
            x = float(sys.argv[2])
            y = float(sys.argv[3])
            control.set_goal(x, y)
        except ValueError:
            print("Error: x and y must be floats")
    elif command == "return":
        control.return_to_exploration()
    elif command == "status":
        control.print_status()
    else:
        print("Unknown command or invalid arguments")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
