#!/usr/bin/env python3
"""
Hybrid Explorer: Frontier goals + Reactive navigation
Uses frontier detection to find goals, but reactive control like simple_explorer
"""

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from scipy.ndimage import binary_dilation, label

class HybridExplorer:
    def __init__(self):
        rospy.init_node('hybrid_explorer', anonymous=False)
        
        # Parameters
        self.enabled = rospy.get_param('~enabled', True)
        self.max_forward_speed = rospy.get_param('~max_forward_speed', 0.15)
        self.max_turn_speed = rospy.get_param('~max_turn_speed', 0.3)
        self.obstacle_distance = rospy.get_param('~obstacle_distance', 0.4)
        self.turn_angle = rospy.get_param('~turn_angle', math.pi / 2)
        
        # Frontier parameters
        self.frontier_min_size = 5
        self.max_exploration_distance = 4.0
        self.goal_reached_threshold = 0.2  # Consider goal reached when within 30cm
        
        # Goal-directed navigation parameters
        self.angle_tolerance = 0.3  # ~17 degrees - acceptable heading error
        self.goal_weight = 0.65  # How much to weight goal direction vs obstacle avoidance
        
        # State
        self.laser_scan = None
        self.map = None
        self.current_goal = None
        self.robot_ready = False
        
        # Reactive navigation state (from simple_explorer)
        self.is_turning = False
        self.turn_start_time = None
        self.turn_duration = 0.0
        self.last_turn_direction = 1.0
        self.consecutive_turns = 0
        self.post_turn_forward_time = None
        self.post_turn_forward_duration = 1.0
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.stand_cmd_pub = rospy.Publisher('stand_cmd', Bool, queue_size=1, latch=True)
        self.walk_cmd_pub = rospy.Publisher('walk_cmd', Bool, queue_size=1, latch=True)
        self.frontier_markers_pub = rospy.Publisher('/frontiers', MarkerArray, queue_size=1)
        
        # Subscribers
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.enable_sub = rospy.Subscriber('/enable_exploration', Bool, self.enable_callback)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Wall following state
        self.wall_following_mode = False
        self.wall_follow_start_time = None
        self.wall_follow_max_duration = 10.0  # Follow wall for max 10 seconds
        self.stuck_counter = 0
        
        rospy.loginfo("Hybrid Explorer initialized")
        
        if self.enabled:
            self.prepare_robot()
    
    def prepare_robot(self):
        """Prepare robot for exploration"""
        rospy.loginfo("Preparing robot for exploration...")
        stand_msg = Bool()
        stand_msg.data = True
        self.stand_cmd_pub.publish(stand_msg)
        rospy.sleep(3.0)
        
        walk_msg = Bool()
        walk_msg.data = True
        self.walk_cmd_pub.publish(walk_msg)
        rospy.sleep(1.0)
        
        self.robot_ready = True
        rospy.loginfo("Robot ready!")
    
    def enable_callback(self, msg):
        if msg.data and not self.enabled:
            self.enabled = True
            rospy.loginfo("Exploration enabled")
            if not self.robot_ready:
                self.prepare_robot()
        elif not msg.data and self.enabled:
            self.enabled = False
            rospy.loginfo("Exploration disabled")
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
    
    def scan_callback(self, msg):
        self.laser_scan = msg
    
    def map_callback(self, msg):
        self.map = msg
    
    def get_robot_pose(self):
        """Get robot position and orientation"""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            # Get yaw from quaternion
            q = transform.transform.rotation
            # Simple yaw extraction: atan2(2(w*z + x*y), 1 - 2(y^2 + z^2))
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            return (x, y, yaw)
        except:
            return None
    
    def world_to_map(self, wx, wy):
        if self.map is None:
            return None, None
        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)
        return mx, my
    
    def map_to_world(self, mx, my):
        if self.map is None:
            return None, None
        wx = mx * self.map.info.resolution + self.map.info.origin.position.x
        wy = my * self.map.info.resolution + self.map.info.origin.position.y
        return wx, wy
    
    def is_frontier_cell(self, grid, x, y, width, height):
        if x < 0 or x >= width or y < 0 or y >= height:
            return False
        if grid[y, x] != 0:
            return False
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    if grid[ny, nx] == -1:
                        return True
        return False
    
    def find_frontiers(self):
        """Find frontiers"""
        if self.map is None:
            return []
        
        width = self.map.info.width
        height = self.map.info.height
        grid = np.array(self.map.data, dtype=np.int8).reshape((height, width))
        
        frontier_mask = np.zeros((height, width), dtype=bool)
        for y in range(0, height, 3):
            for x in range(0, width, 3):
                if self.is_frontier_cell(grid, x, y, width, height):
                    frontier_mask[y, x] = True
        
        frontier_mask = binary_dilation(frontier_mask, iterations=2)
        labeled_frontiers, num_regions = label(frontier_mask)
        
        if num_regions == 0:
            return []
        
        frontiers = []
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return []
        
        robot_x, robot_y, _ = robot_pose
        
        for region_id in range(1, num_regions + 1):
            region_cells = np.argwhere(labeled_frontiers == region_id)
            if len(region_cells) < self.frontier_min_size:
                continue
            
            centroid_y = int(np.mean(region_cells[:, 0]))
            centroid_x = int(np.mean(region_cells[:, 1]))
            wx, wy = self.map_to_world(centroid_x, centroid_y)
            
            if wx is None or wy is None:
                continue
            
            distance = math.sqrt((wx - robot_x)**2 + (wy - robot_y)**2)
            
            if distance > self.max_exploration_distance or distance < 0.3:
                continue
            
            if grid[centroid_y, centroid_x] == 100:
                continue
            
            score = (
                0.6 * (1.0 / distance) +
                0.3 * (len(region_cells) / 50.0)
            )

            frontiers.append({
                'position': (wx, wy),
                'size': len(region_cells),
                'distance': distance,
                'score': score
            })
        
        frontiers.sort(key=lambda f: f['score'], reverse=True)
        return frontiers
    
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
    
    def is_goal_reached(self):
        """Check if current goal is reached"""
        if self.current_goal is None:
            return False
        
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return False
        
        robot_x, robot_y, _ = robot_pose
        goal_x, goal_y = self.current_goal
        
        distance = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
        return distance < self.goal_reached_threshold
    
    def check_obstacle_ahead(self):
        """Check for obstacles"""
        if self.laser_scan is None:
            return True
        
        ranges = np.array(self.laser_scan.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]
        if len(valid_ranges) == 0:
            return True
        
        num_readings = len(ranges)
        angle_increment = self.laser_scan.angle_increment
        angle_min = self.laser_scan.angle_min
        
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
    
    def find_best_direction_to_goal(self):
        """
        Find best turn direction considering BOTH goal and obstacles
        This is the KEY improvement over simple_explorer
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
        # Weight them: 70% goal, 30% obstacles
        if abs(angle_to_goal) > math.pi / 3:  # >60 degrees off
            # Goal is far off to the side, prioritize it more
            rospy.loginfo_throttle(2.0, f"Large angle to goal ({math.degrees(angle_to_goal):.1f}°), prioritizing goal direction")
            return preferred_direction
        else:
            # Goal is somewhat aligned, consider obstacles more
            rospy.loginfo_throttle(2.0, f"Obstacle conflict, using obstacle direction")
            return obstacle_direction
    
    def find_best_direction_obstacles(self):
        """Find best turn direction based only on obstacles (original logic)"""
        if self.laser_scan is None:
            return self.last_turn_direction
        
        ranges = np.array(self.laser_scan.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, self.laser_scan.range_max)
        
        num_readings = len(ranges)
        angle_increment = self.laser_scan.angle_increment
        angle_min = self.laser_scan.angle_min
        
        left_start_idx = int((math.pi/2 - angle_min) / angle_increment)
        left_end_idx = int((math.pi - angle_min) / angle_increment)
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
        
        if self.consecutive_turns > 2:
            return self.last_turn_direction
        
        if left_avg > right_avg * 1.1:
            return -1.0
        elif right_avg > left_avg * 1.1:
            return 1.0
        else:
            return self.last_turn_direction
    
    def publish_goal_marker(self):
        """Publish current goal as a green sphere marker"""
        if self.current_goal is None:
            return
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "frontier_goal"
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
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.frontier_markers_pub.publish(marker_array)
    
    def navigate_with_goal(self):
        """
        Goal-directed reactive navigation
        Combines goal seeking with obstacle avoidance
        """
        cmd = Twist()
        
        # Check if goal is reached
        if self.is_goal_reached():
            rospy.loginfo("Goal reached! Finding new frontier...")
            self.current_goal = None
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return cmd
        
        # Get angle to goal
        angle_to_goal = self.get_angle_to_goal()
        
        if angle_to_goal is not None:
            # Adjust angular velocity to point toward goal
            # But limit it to avoid spinning in place
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



    def run(self):
        """Main loop - GOAL-DIRECTED reactive navigation"""
        rate = rospy.Rate(10)
        last_frontier_check = rospy.Time.now()
        
        rospy.loginfo("=== Hybrid Explorer Started ===")
        rospy.loginfo("Mode: Goal-directed reactive navigation")
        
        while not rospy.is_shutdown():
            if not self.enabled or not self.robot_ready:
                rate.sleep()
                continue
            
            # Update frontier goal every 3 seconds or when goal reached
            if self.current_goal is None or \
               (rospy.Time.now() - last_frontier_check).to_sec() > 3.0:
                frontiers = self.find_frontiers()
                if frontiers:
                    self.current_goal = frontiers[0]['position']
                    rospy.loginfo(f"New frontier goal: ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f}), dist={frontiers[0]['distance']:.2f}m")
                else:
                    rospy.loginfo("No frontiers found, continuing exploration")
                last_frontier_check = rospy.Time.now()
            
            # Publish goal marker
            self.publish_goal_marker()
            
            cmd = Twist()
            
            # Post-turn forward period (from simple_explorer)
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
                    
                    # KEY CHANGE: Use goal-aware turn direction
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

if __name__ == '__main__':
    try:
        explorer = HybridExplorer()
        explorer.run()
    except rospy.ROSInterruptException:
        pass