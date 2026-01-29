#!/usr/bin/env python3
"""
Path Planner Override with Obstacle Avoidance
- Listens to RViz clicked goals
- Overrides simple_explorer
- Drives robot to clicked goal with reactive obstacle avoidance
- Visualizes the objective goal as a red sphere in RViz
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

class PathPlanner:
    def __init__(self):
        rospy.init_node("path_planner")

        # Parameters
        self.max_forward_speed = rospy.get_param("~max_forward_speed", 0.35)
        self.max_turn_speed = rospy.get_param("~max_turn_speed", 0.45)
        self.goal_threshold = rospy.get_param("~goal_threshold", 0.3)
        self.obstacle_distance = rospy.get_param("~obstacle_distance", 0.4)
        self.turn_angle = rospy.get_param("~turn_angle", math.pi / 2)
        
        # Goal-directed navigation parameters
        self.angle_tolerance = 0.3  # ~17 degrees - acceptable heading error
        self.goal_weight = 0.65  # How much to weight goal direction vs obstacle avoidance

        # State
        self.current_goal = None
        self.laser_scan = None
        
        # Reactive navigation state (from simple_explorer)
        self.is_turning = False
        self.turn_start_time = None
        self.turn_duration = 0.0
        self.turn_direction = 1.0
        self.last_turn_direction = 1.0
        self.consecutive_turns = 0
        self.post_turn_forward_time = None
        self.post_turn_forward_duration = 1.0
        
        # Wall following state
        self.wall_following_mode = False
        self.wall_follow_start_time = None
        self.wall_follow_max_duration = 10.0  # Follow wall for max 10 seconds
        self.stuck_counter = 0

        # Publishers
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.exploration_enable_pub = rospy.Publisher(
            "/enable_exploration", Bool, queue_size=1, latch=True
        )
        self.goal_marker_pub = rospy.Publisher(
            "/goal_marker", MarkerArray, queue_size=1
        )

        # Subscribers
        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal",
            PoseStamped,
            self.goal_callback
        )
        self.scan_sub = rospy.Subscriber(
            "/scan",
            LaserScan,
            self.scan_callback
        )

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("Path Planner ready — click a goal in RViz")

    # --------------------------------------------------

    def scan_callback(self, msg):
        """Store laser scan data"""
        self.laser_scan = msg

    # --------------------------------------------------

    def goal_callback(self, msg):
        """Handle new goal from RViz"""
        self.current_goal = (
            msg.pose.position.x,
            msg.pose.position.y
        )

        rospy.loginfo(
            f"New RViz goal: ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})"
        )

        # Reset turning state when new goal is set
        self.is_turning = False
        self.consecutive_turns = 0
        self.post_turn_forward_time = None

        # Publish goal marker
        self.publish_goal_marker()

        # Disable exploration
        self.exploration_enable_pub.publish(Bool(data=False))

    # --------------------------------------------------

    def get_robot_pose(self):
        """Get robot position and orientation"""
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rospy.Time(0)
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y

            q = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )

            return x, y, yaw
        except:
            return None

    # --------------------------------------------------

    def goal_reached(self, rx, ry):
        """Check if goal is reached"""
        gx, gy = self.current_goal
        return math.hypot(gx - rx, gy - ry) < self.goal_threshold

    # --------------------------------------------------

    def check_obstacle_ahead(self):
        """Check for obstacles in front of robot"""
        if self.laser_scan is None:
            return True
        
        ranges = np.array(self.laser_scan.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]
        if len(valid_ranges) == 0:
            return True
        
        num_readings = len(ranges)
        angle_increment = self.laser_scan.angle_increment
        angle_min = self.laser_scan.angle_min
        
        # Check front 60-degree cone (-30° to +30°)
        front_start_angle = -math.pi / 6
        front_end_angle = math.pi / 6
        
        front_start_idx = int((front_start_angle - angle_min) / angle_increment)
        front_end_idx = int((front_end_angle - angle_min) / angle_increment)
        
        front_start_idx = max(0, min(front_start_idx, num_readings - 1))
        front_end_idx = max(0, min(front_end_idx, num_readings - 1))
        
        if front_start_idx > front_end_idx:
            front_start_idx, front_end_idx = front_end_idx, front_start_idx
        
        front_ranges = ranges[front_start_idx:front_end_idx+1]
        front_ranges = front_ranges[np.isfinite(front_ranges)]
        
        if len(front_ranges) == 0:
            return True
        
        return np.min(front_ranges) < self.obstacle_distance

    # --------------------------------------------------

    def get_angle_to_goal(self):
        """Calculate angle error to current goal"""
        if self.current_goal is None:
            return None
        
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return None
        
        robot_x, robot_y, robot_yaw = robot_pose
        goal_x, goal_y = self.current_goal
        
        # Calculate angle to goal
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        angle_to_goal = math.atan2(dy, dx)
        
        # Calculate angle error (normalize to [-pi, pi])
        angle_error = angle_to_goal - robot_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        return angle_error

    # --------------------------------------------------

    def find_best_direction_to_goal(self):
        """
        Find best turn direction considering BOTH goal and obstacles
        """
        angle_to_goal = self.get_angle_to_goal()
        
        # If no goal, fall back to obstacle-based direction
        if angle_to_goal is None:
            return self.find_best_direction_obstacles()
        
        # Prefer turning toward goal
        if abs(angle_to_goal) < 0.1:  # Almost aligned
            return 0.0  # No turn needed
        elif angle_to_goal > 0:
            preferred_direction = 1.0  # Turn left to goal
        else:
            preferred_direction = -1.0   # Turn right to goal
        
        # Check if turning toward goal is safe
        obstacle_direction = self.find_best_direction_obstacles()
        
        # If obstacle avoidance agrees with goal direction, use it
        if preferred_direction == obstacle_direction:
            rospy.loginfo_throttle(2.0, f"Turning toward goal (angle_err={math.degrees(angle_to_goal):.1f}°)")
            return preferred_direction
        
        # Conflict: goal wants one direction, obstacles want another
        if abs(angle_to_goal) > math.pi / 3:  # >60 degrees off
            # Goal is far off to the side, prioritize it more
            rospy.loginfo_throttle(2.0, f"Large angle to goal ({math.degrees(angle_to_goal):.1f}°), prioritizing goal direction")
            return preferred_direction
        else:
            # Goal is somewhat aligned, consider obstacles more
            rospy.loginfo_throttle(2.0, f"Obstacle conflict, using obstacle direction")
            return obstacle_direction

    # --------------------------------------------------

    def find_best_direction_obstacles(self):
        """Find best turn direction based only on obstacles"""
        if self.laser_scan is None:
            return self.last_turn_direction
        
        ranges = np.array(self.laser_scan.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, self.laser_scan.range_max)
        
        num_readings = len(ranges)
        angle_increment = self.laser_scan.angle_increment
        angle_min = self.laser_scan.angle_min
        
        # Left side: 90° to 180°
        left_start_idx = int((math.pi/2 - angle_min) / angle_increment)
        left_end_idx = int((math.pi - angle_min) / angle_increment)
        # Right side: -180° to -90°
        right_start_idx = int((-math.pi - angle_min) / angle_increment)
        right_end_idx = int((-math.pi/2 - angle_min) / angle_increment)
        
        left_start_idx = max(0, min(left_start_idx, num_readings - 1))
        left_end_idx = max(0, min(left_end_idx, num_readings - 1))
        right_start_idx = max(0, min(right_start_idx, num_readings - 1))
        right_end_idx = max(0, min(right_end_idx, num_readings - 1))
        
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
        
        left_avg = np.mean(left_ranges) if len(left_ranges) > 0 else 0.0
        right_avg = np.mean(right_ranges) if len(right_ranges) > 0 else 0.0
        
        # After multiple turns, stick with same direction
        if self.consecutive_turns > 2:
            return self.last_turn_direction
        
        # Turn toward more open space
        if left_avg > right_avg * 1.1:
            return -1.0  # Turn left
        elif right_avg > left_avg * 1.1:
            return 1.0  # Turn right
        else:
            return self.last_turn_direction

    # --------------------------------------------------

    def navigate_with_goal(self):
        """
        Goal-directed reactive navigation
        Combines goal seeking with obstacle avoidance
        """
        cmd = Twist()
        
        # Get angle to goal
        angle_to_goal = self.get_angle_to_goal()
        
        if angle_to_goal is not None:
            # Adjust angular velocity to point toward goal
            angular_cmd = max(-self.max_turn_speed, 
                            min(self.max_turn_speed, 
                                angle_to_goal * 1.5))  # Proportional control
            
            # Move forward while turning (unless angle is very large)
            if abs(angle_to_goal) > math.pi / 2:  # >90 degrees
                # Need to turn a lot, slow down forward motion
                linear_cmd = self.max_forward_speed * 0.3
            elif abs(angle_to_goal) > math.pi / 4:  # >45 degrees
                # Moderate turn, half speed
                linear_cmd = self.max_forward_speed * 0.5
            else:
                # Mostly aligned, full speed
                linear_cmd = self.max_forward_speed
            
            cmd.linear.x = linear_cmd
            cmd.angular.z = angular_cmd
            
            rospy.loginfo_throttle(2.0, 
                f"→ Goal nav: angle_err={math.degrees(angle_to_goal):.1f}°, "
                f"cmd=({linear_cmd:.2f}, {angular_cmd:.2f})")
        else:
            # No goal, just move forward
            cmd.linear.x = self.max_forward_speed
            cmd.angular.z = 0.0
        
        return cmd

    # --------------------------------------------------

    def check_if_stuck(self, obstacle_ahead):
        """Detect if robot is stuck in a loop"""
        if obstacle_ahead:
            self.stuck_counter += 1
            rospy.loginfo_throttle(1.0, f"Stuck counter: {self.stuck_counter}")
        else:
            self.stuck_counter = 0

        if self.stuck_counter > 12:
            if not self.wall_following_mode:
                rospy.logwarn("STUCK! Entering wall-following mode")
                self.wall_following_mode = True
                self.wall_follow_start_time = rospy.Time.now()
            self.stuck_counter = 0

    # --------------------------------------------------

    def wall_follow_control(self):
        """
        Follow the wall on the right side to navigate around obstacles
        Uses averaged sensor readings for robustness
        """
        if self.laser_scan is None:
            return Twist()
        
        ranges = np.array(self.laser_scan.ranges)
        num_readings = len(ranges)
        angle_min = self.laser_scan.angle_min
        angle_increment = self.laser_scan.angle_increment
        
        # Get AVERAGE distances from multiple readings for robustness
        
        # Right side: -60° to -120° (wider arc)
        right_start_angle = -math.pi / 3  # -60 degrees
        right_end_angle = -2 * math.pi / 3  # -120 degrees
        right_start_idx = int((right_start_angle - angle_min) / angle_increment)
        right_end_idx = int((right_end_angle - angle_min) / angle_increment)
        right_start_idx = max(0, min(right_start_idx, num_readings - 1))
        right_end_idx = max(0, min(right_end_idx, num_readings - 1))
        
        if right_start_idx > right_end_idx:
            right_ranges = ranges[right_end_idx:right_start_idx+1]
        else:
            right_ranges = ranges[right_start_idx:right_end_idx+1]
        
        # Filter out invalid readings and only keep those < 2m (actual obstacles)
        right_ranges = right_ranges[np.isfinite(right_ranges) & (right_ranges < 2.0)]
        
        # Front: -30° to +30°
        front_start_angle = -math.pi / 6
        front_end_angle = math.pi / 6
        front_start_idx = int((front_start_angle - angle_min) / angle_increment)
        front_end_idx = int((front_end_angle - angle_min) / angle_increment)
        front_start_idx = max(0, min(front_start_idx, num_readings - 1))
        front_end_idx = max(0, min(front_end_idx, num_readings - 1))
        front_ranges = ranges[front_start_idx:front_end_idx+1]
        front_ranges = front_ranges[np.isfinite(front_ranges)]
        
        # Calculate distances
        if len(right_ranges) > 0:
            right_distance = np.min(right_ranges)  # Use minimum for safety
            right_avg = np.mean(right_ranges)
        else:
            right_distance = float('inf')
            right_avg = float('inf')
        
        if len(front_ranges) > 0:
            front_distance = np.min(front_ranges)
        else:
            front_distance = float('inf')
        
        # Wall following parameters
        desired_wall_distance = 0.35  # Stay 35cm from wall (closer is more reliable)
        
        cmd = Twist()
        
        # Priority 1: Obstacle directly ahead - turn left away from wall
        if front_distance < self.obstacle_distance:
            cmd.angular.z = self.max_turn_speed
            cmd.linear.x = 0.0
            rospy.loginfo_throttle(1.0, f"Wall-follow: Obstacle ahead ({front_distance:.2f}m), turning left")
        
        # Priority 2: No wall detected on right - turn right to find it
        elif right_distance > 1.5 or len(right_ranges) == 0:
            cmd.angular.z = -self.max_turn_speed * 0.4
            cmd.linear.x = self.max_forward_speed * 0.6
            rospy.loginfo_throttle(1.0, f"Wall-follow: No wall detected, turning right")
        
        # Priority 3: Too close to wall - turn left
        elif right_distance < desired_wall_distance * 0.8:
            cmd.angular.z = self.max_turn_speed * 0.6
            cmd.linear.x = self.max_forward_speed * 0.4
            rospy.loginfo_throttle(1.0, f"Wall-follow: Too close ({right_distance:.2f}m < {desired_wall_distance*0.8:.2f}m)")
        
        # Priority 4: Too far from wall - turn right
        elif right_distance > desired_wall_distance * 1.5:
            cmd.angular.z = -self.max_turn_speed * 0.4
            cmd.linear.x = self.max_forward_speed * 0.6
            rospy.loginfo_throttle(1.0, f"Wall-follow: Too far ({right_distance:.2f}m > {desired_wall_distance*1.5:.2f}m)")
        
        # Priority 5: Good distance - go straight
        else:
            cmd.angular.z = 0.0
            cmd.linear.x = self.max_forward_speed
            rospy.loginfo_throttle(1.0, f"Wall-follow: Good distance ({right_distance:.2f}m)")
        
        return cmd

    # --------------------------------------------------

    def publish_goal_marker(self):
        """Publish current goal as a red sphere marker in RViz"""
        if self.current_goal is None:
            return

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_planner_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.current_goal[0]
        marker.pose.position.y = self.current_goal[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.goal_marker_pub.publish(marker_array)

    # --------------------------------------------------

    def clear_goal_marker(self):
        """Clear the goal marker from RViz"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_planner_goal"
        marker.id = 0
        marker.action = Marker.DELETE
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.goal_marker_pub.publish(marker_array)

    # --------------------------------------------------

    def run(self):
        """Main loop with obstacle avoidance and wall following"""
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.current_goal is None:
                rate.sleep()
                continue

            pose = self.get_robot_pose()
            if pose is None:
                rate.sleep()
                continue

            rx, ry, yaw = pose
            
            cmd = Twist()

            # Goal reached
            if self.goal_reached(rx, ry):
                rospy.loginfo("Goal reached — resuming exploration")
                self.current_goal = None
                self.clear_goal_marker()
                self.exploration_enable_pub.publish(Bool(data=True))
                self.cmd_vel_pub.publish(Twist())
                rate.sleep()
                continue

            # Post-turn forward period
            if self.post_turn_forward_time is not None:
                elapsed = (rospy.Time.now() - self.post_turn_forward_time).to_sec()
                if elapsed < self.post_turn_forward_duration:
                    cmd.linear.x = self.max_forward_speed
                    cmd.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd)
                    rate.sleep()
                    continue
                else:
                    self.post_turn_forward_time = None
                    self.consecutive_turns = 0

            # Check for obstacles
            obstacle_ahead = self.check_obstacle_ahead()
            self.check_if_stuck(obstacle_ahead)

            # Wall following mode
            if self.wall_following_mode:
                # Check if wall-follow duration exceeded
                if (rospy.Time.now() - self.wall_follow_start_time).to_sec() > self.wall_follow_max_duration:
                    rospy.loginfo("Wall-following timeout, returning to normal navigation")
                    self.wall_following_mode = False
                else:
                    cmd = self.wall_follow_control()
                    self.cmd_vel_pub.publish(cmd)
                    rate.sleep()
                    continue

            if obstacle_ahead or self.is_turning:
                # OBSTACLE AVOIDANCE MODE (with goal-awareness)
                if not self.is_turning:
                    self.is_turning = True
                    self.turn_start_time = rospy.Time.now()
                    
                    # Use goal-aware turn direction
                    turn_direction = self.find_best_direction_to_goal()
                    self.turn_direction = turn_direction
                    self.last_turn_direction = turn_direction
                    self.consecutive_turns += 1
                    
                    turn_angle = self.turn_angle
                    if self.consecutive_turns > 2:
                        turn_angle = self.turn_angle * 1.5
                    
                    self.turn_duration = abs(turn_angle / self.max_turn_speed)
                    rospy.loginfo(f"Obstacle - turning {'right' if turn_direction > 0 else 'left'}")
                
                elapsed = (rospy.Time.now() - self.turn_start_time).to_sec()
                if elapsed < self.turn_duration:
                    cmd.angular.z = self.max_turn_speed * self.turn_direction
                    cmd.linear.x = 0.0
                else:
                    self.is_turning = False
                    self.post_turn_forward_time = rospy.Time.now()
                    cmd.angular.z = 0.0
                    cmd.linear.x = self.max_forward_speed
            else:
                # NO OBSTACLE - Navigate toward goal
                cmd = self.navigate_with_goal()
                self.is_turning = False
                self.consecutive_turns = 0

            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

# ------------------------------------------------------

if __name__ == "__main__":
    try:
        planner = PathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
