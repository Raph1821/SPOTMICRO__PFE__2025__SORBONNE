#!/usr/bin/env python3
"""
Simple Autonomous Explorer Node
A simple exploration node that:
1. Uses laser scan to detect obstacles
2. Moves forward when path is clear
3. Turns when obstacle detected
4. Explores unknown areas in the map
"""

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import tf2_ros
import tf2_geometry_msgs

class SimpleExplorer:
    def __init__(self):
        rospy.init_node('simple_explorer', anonymous=False)
        
        # Parameters
        self.enabled = rospy.get_param('~enabled', False)  # Start disabled by default
        self.max_forward_speed = rospy.get_param('~max_forward_speed', 0.15)  # m/s
        self.max_turn_speed = rospy.get_param('~max_turn_speed', 0.3)  # rad/s
        self.obstacle_distance = rospy.get_param('~obstacle_distance', 0.4)  # meters
        self.safe_distance = rospy.get_param('~safe_distance', 0.5)  # meters
        self.turn_angle = rospy.get_param('~turn_angle', math.pi / 2)  # 90 degrees
        
        # State
        self.laser_scan = None
        self.occupancy_map = None
        self.current_pose = None
        self.is_turning = False
        self.turn_start_time = None
        self.turn_duration = 0.0
        self.last_obstacle_time = rospy.Time.now()
        self.stuck_counter = 0
        self.last_position = None
        self.last_turn_direction = 1.0  # Remember last turn direction to avoid oscillation
        self.consecutive_turns = 0  # Count consecutive turns
        self.post_turn_forward_time = None  # Time to move forward after turning
        self.post_turn_forward_duration = 1.0  # Move forward for 1 second after turning
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.stand_cmd_pub = rospy.Publisher('stand_cmd', Bool, queue_size=1, latch=True)
        self.walk_cmd_pub = rospy.Publisher('walk_cmd', Bool, queue_size=1, latch=True)
        
        # Subscribers
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.enable_sub = rospy.Subscriber('/enable_exploration', Bool, self.enable_callback)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Robot ready flag
        self.robot_ready = False
        
        rospy.loginfo("Simple Explorer initialized")
        rospy.loginfo(f"Enabled: {self.enabled}")
        rospy.loginfo(f"Max forward speed: {self.max_forward_speed} m/s")
        rospy.loginfo(f"Max turn speed: {self.max_turn_speed} rad/s")
        rospy.loginfo(f"Obstacle distance: {self.obstacle_distance} m")
        
        if self.enabled:
            self.prepare_robot()
    
    def enable_callback(self, msg):
        """Enable/disable exploration"""
        if msg.data and not self.enabled:
            self.enabled = True
            rospy.loginfo("Exploration enabled")
            self.prepare_robot()
        elif not msg.data and self.enabled:
            self.enabled = False
            rospy.loginfo("Exploration disabled")
            self.stop_robot()
    
    def prepare_robot(self):
        """Prepare robot for exploration: stand and walk"""
        rospy.loginfo("Preparing robot for exploration...")
        stand_msg = Bool()
        stand_msg.data = True
        self.stand_cmd_pub.publish(stand_msg)
        rospy.sleep(3.0)  # Wait for robot to stand
        
        walk_msg = Bool()
        walk_msg.data = True
        self.walk_cmd_pub.publish(walk_msg)
        rospy.sleep(1.0)  # Wait for state transition
        
        self.robot_ready = True
        rospy.loginfo("Robot ready for exploration!")
    
    def stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
    
    def scan_callback(self, msg):
        """Process laser scan data"""
        self.laser_scan = msg
    
    def map_callback(self, msg):
        """Process occupancy grid map"""
        self.occupancy_map = msg
    
    def get_current_pose(self):
        """Get current robot pose from TF"""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
            self.current_pose = transform.transform.translation
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False
    
    def check_obstacle_ahead(self):
        """Check if there's an obstacle ahead using laser scan"""
        if self.laser_scan is None:
            return True  # Assume obstacle if no scan data
        
        # Get ranges
        ranges = np.array(self.laser_scan.ranges)
        
        # Filter out invalid readings
        valid_ranges = ranges[np.isfinite(ranges)]
        if len(valid_ranges) == 0:
            return True  # Assume obstacle if no valid readings
        
        # Check front sector (30 degrees on each side of front)
        num_readings = len(ranges)
        angle_increment = self.laser_scan.angle_increment
        angle_min = self.laser_scan.angle_min
        
        # Front sector: -30 to +30 degrees
        front_start_angle = -math.pi / 6  # -30 degrees
        front_end_angle = math.pi / 6     # +30 degrees
        
        front_start_idx = int((front_start_angle - angle_min) / angle_increment)
        front_end_idx = int((front_end_angle - angle_min) / angle_increment)
        
        # Ensure indices are valid
        front_start_idx = max(0, min(front_start_idx, num_readings - 1))
        front_end_idx = max(0, min(front_end_idx, num_readings - 1))
        
        if front_start_idx > front_end_idx:
            front_start_idx, front_end_idx = front_end_idx, front_start_idx
        
        front_ranges = ranges[front_start_idx:front_end_idx+1]
        front_ranges = front_ranges[np.isfinite(front_ranges)]
        
        if len(front_ranges) == 0:
            return True  # Assume obstacle if no valid front readings
        
        min_front_distance = np.min(front_ranges)
        
        # Check if obstacle is too close
        if min_front_distance < self.obstacle_distance:
            return True
        
        return False
    
    def find_best_direction(self):
        """Find the best direction to turn based on laser scan"""
        if self.laser_scan is None:
            return self.last_turn_direction  # Use last direction if no scan
        
        ranges = np.array(self.laser_scan.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, self.laser_scan.range_max)
        
        num_readings = len(ranges)
        angle_increment = self.laser_scan.angle_increment
        angle_min = self.laser_scan.angle_min
        
        # Check left and right sides (avoid front)
        # Left side: 90 to 180 degrees (or equivalent)
        # Right side: -90 to -180 degrees (or equivalent)
        
        # Find indices for left and right sectors
        left_start_angle = math.pi / 2  # 90 degrees
        left_end_angle = math.pi  # 180 degrees
        right_start_angle = -math.pi  # -180 degrees
        right_end_angle = -math.pi / 2  # -90 degrees
        
        left_start_idx = int((left_start_angle - angle_min) / angle_increment)
        left_end_idx = int((left_end_angle - angle_min) / angle_increment)
        right_start_idx = int((right_start_angle - angle_min) / angle_increment)
        right_end_idx = int((right_end_angle - angle_min) / angle_increment)
        
        # Ensure indices are valid
        left_start_idx = max(0, min(left_start_idx, num_readings - 1))
        left_end_idx = max(0, min(left_end_idx, num_readings - 1))
        right_start_idx = max(0, min(right_start_idx, num_readings - 1))
        right_end_idx = max(0, min(right_end_idx, num_readings - 1))
        
        # Get left and right ranges
        if left_start_idx < left_end_idx:
            left_ranges = ranges[left_start_idx:left_end_idx+1]
        else:
            left_ranges = np.concatenate([ranges[left_start_idx:], ranges[:left_end_idx+1]])
        
        if right_start_idx < right_end_idx:
            right_ranges = ranges[right_start_idx:right_end_idx+1]
        else:
            right_ranges = np.concatenate([ranges[right_start_idx:], ranges[:right_end_idx+1]])
        
        left_ranges = left_ranges[np.isfinite(left_ranges)]
        right_ranges = right_ranges[np.isfinite(right_ranges)]
        
        # Calculate average distance for left and right
        left_avg = np.mean(left_ranges) if len(left_ranges) > 0 else 0.0
        right_avg = np.mean(right_ranges) if len(right_ranges) > 0 else 0.0
        
        # If consecutive turns, prefer same direction to avoid oscillation
        if self.consecutive_turns > 2:
            rospy.loginfo(f"Consecutive turns detected ({self.consecutive_turns}), using last direction to avoid oscillation")
            return self.last_turn_direction
        
        # Choose direction with more open space
        if left_avg > right_avg * 1.1:  # Left is significantly more open
            return -1.0  # Turn left
        elif right_avg > left_avg * 1.1:  # Right is significantly more open
            return 1.0  # Turn right
        else:
            # Similar, use last direction to avoid oscillation
            return self.last_turn_direction
    
    def run(self):
        """Main exploration loop"""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            if not self.enabled:
                rate.sleep()
                continue
            
            if not self.robot_ready:
                rate.sleep()
                continue
            
            # Get current pose
            if not self.get_current_pose():
                rospy.logwarn_throttle(5.0, "Waiting for TF transform (map -> base_footprint)...")
                rate.sleep()
                continue
            
            # Check if stuck (not moving)
            if self.last_position is not None:
                dist_moved = math.sqrt(
                    (self.current_pose.x - self.last_position.x)**2 +
                    (self.current_pose.y - self.last_position.y)**2
                )
                if dist_moved < 0.05:  # Moved less than 5cm
                    self.stuck_counter += 1
                else:
                    self.stuck_counter = 0
                    self.last_position = self.current_pose
            
            if self.last_position is None:
                self.last_position = self.current_pose
            
            # If stuck for too long, turn
            if self.stuck_counter > 50:  # 5 seconds at 10Hz
                rospy.logwarn("Robot appears stuck, turning...")
                self.is_turning = True
                self.turn_start_time = rospy.Time.now()
                self.turn_duration = 2.0  # Turn for 2 seconds
                self.stuck_counter = 0
            
            # Create velocity command
            cmd = Twist()
            
            # If we just finished turning, move forward for a bit before checking obstacles again
            if self.post_turn_forward_time is not None:
                elapsed = (rospy.Time.now() - self.post_turn_forward_time).to_sec()
                if elapsed < self.post_turn_forward_duration:
                    # Continue moving forward after turn
                    cmd.linear.x = self.max_forward_speed
                    cmd.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd)
                    rate.sleep()
                    continue
                else:
                    # Forward period finished, reset
                    self.post_turn_forward_time = None
                    self.consecutive_turns = 0  # Reset counter after successful forward movement
            
            # Check for obstacles
            obstacle_ahead = self.check_obstacle_ahead()
            
            if obstacle_ahead or self.is_turning:
                # Turn
                if not self.is_turning:
                    # Start turning
                    self.is_turning = True
                    self.turn_start_time = rospy.Time.now()
                    turn_direction = self.find_best_direction()
                    self.turn_direction = turn_direction
                    self.last_turn_direction = turn_direction
                    self.consecutive_turns += 1
                    
                    # Increase turn angle if consecutive turns (stuck in corner)
                    turn_angle = self.turn_angle
                    if self.consecutive_turns > 2:
                        turn_angle = self.turn_angle * 1.5  # Turn 135 degrees instead of 90
                        rospy.logwarn(f"Consecutive turns: {self.consecutive_turns}, increasing turn angle to {math.degrees(turn_angle)} degrees")
                    
                    self.turn_duration = abs(turn_angle / self.max_turn_speed)
                    rospy.loginfo(f"Obstacle detected, turning {'right' if turn_direction > 0 else 'left'} ({math.degrees(turn_angle):.1f} degrees)")
                
                # Continue turning
                elapsed = (rospy.Time.now() - self.turn_start_time).to_sec()
                if elapsed < self.turn_duration:
                    cmd.angular.z = self.max_turn_speed * self.turn_direction
                    cmd.linear.x = 0.0
                else:
                    # Finished turning, start forward movement period
                    self.is_turning = False
                    self.post_turn_forward_time = rospy.Time.now()
                    rospy.loginfo("Turn completed, moving forward before next obstacle check")
                    cmd.angular.z = 0.0
                    cmd.linear.x = self.max_forward_speed  # Start moving forward immediately
            else:
                # Move forward (no obstacle)
                cmd.linear.x = self.max_forward_speed
                cmd.angular.z = 0.0
                self.is_turning = False
                self.consecutive_turns = 0  # Reset counter when moving forward successfully
            
            # Publish command
            self.cmd_vel_pub.publish(cmd)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        explorer = SimpleExplorer()
        explorer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Simple Explorer shutting down")

