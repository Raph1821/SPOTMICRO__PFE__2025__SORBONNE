#!/usr/bin/env python3
"""
Path Planning Module for Spot Micro
Allows user to:
1. Click on map in RVIZ to set destination
2. Navigate to that destination autonomously
3. Return to exploration mode when goal reached or button pressed
"""

import rospy
import math
import numpy as np
import actionlib
from geometry_msgs.msg import PointStamped, Twist, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path, OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, String
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner', anonymous=False)
        
        # Parameters from YAML config
        self.goal_tolerance = rospy.get_param('path_planning/goal_tolerance', 0.3)  # meters
        self.max_forward_speed = rospy.get_param('robot/max_forward_speed', 0.15)  # m/s
        self.max_turn_speed = rospy.get_param('robot/max_turn_speed', 0.3)  # rad/s
        self.pid_angle_gain = rospy.get_param('path_planning/pid_angle_gain', 0.5)
        
        # State
        self.current_goal = None
        self.robot_pose = None
        self.in_path_planning_mode = False
        self.goal_reached = False
        self.move_base_client = None
        self.current_move_base_goal = None
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('current_goal', PointStamped, queue_size=1)
        self.mode_pub = rospy.Publisher('planner_mode', String, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher('computed_path', Path, queue_size=1)
        self.enable_exploration_pub = rospy.Publisher('/enable_exploration', Bool, queue_size=1, latch=True)
        self.path_marker_pub = rospy.Publisher('path_visualization', Marker, queue_size=1)
        
        # Subscribers
        self.clicked_point_sub = rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback)
        self.return_to_exploration_sub = rospy.Subscriber('/return_to_exploration', Bool, self.return_to_exploration_callback)
        self.goal_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.goal_status_callback)
        self.plan_sub = rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.plan_callback)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Connect to move_base action server
        self.connect_to_move_base()
        
        rospy.loginfo("Path Planner initialized")
        rospy.loginfo("Exploration mode: ENABLED")
        rospy.loginfo("Click on map in RVIZ to set destination")
        rospy.loginfo("Publish Bool(True) to /return_to_exploration to return to exploration mode")
    
    def connect_to_move_base(self):
        """Connect to move_base action server with retries"""
        rospy.loginfo("Attempting to connect to move_base action server...")
        
        max_attempts = 5
        attempt = 0
        
        while attempt < max_attempts and not rospy.is_shutdown():
            try:
                self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                
                if self.move_base_client.wait_for_server(timeout=rospy.Duration(3)):
                    rospy.loginfo("✓ Successfully connected to move_base!")
                    return True
                else:
                    attempt += 1
                    rospy.logwarn(f"move_base not ready (attempt {attempt}/{max_attempts}), retrying in 2s...")
                    rospy.sleep(2)
            except Exception as e:
                attempt += 1
                rospy.logwarn(f"Connection attempt {attempt}/{max_attempts} failed: {e}")
                rospy.sleep(2)
        
        rospy.logwarn("✗ Could not connect to move_base - will use manual navigation")
        self.move_base_client = None
        return False
    
    def clicked_point_callback(self, msg):
        """Handle clicked points from RVIZ"""
        rospy.loginfo(f"Received goal: ({msg.point.x:.2f}, {msg.point.y:.2f})")
        
        if self.in_path_planning_mode:
            rospy.loginfo("Already navigating. Canceling previous goal and setting new one...")
            # Cancel existing goal
            if self.move_base_client:
                self.move_base_client.cancel_goal()
            rospy.sleep(0.2)
        
        self.current_goal = msg.point
        self.goal_reached = False
        
        # Enter path planning mode (or update if already in it)
        self.enter_path_planning_mode()
    
    def return_to_exploration_callback(self, msg):
        """Handle return to exploration command"""
        if msg.data:
            rospy.loginfo("Returning to exploration mode")
            self.return_to_exploration_mode()
    
    def goal_status_callback(self, msg):
        """Monitor move_base goal status"""
        if len(msg.status_list) == 0:
            return
        
        # Get latest goal status
        latest_status = msg.status_list[-1]
        
        if self.in_path_planning_mode:
            if latest_status.status == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached!")
                self.goal_reached = True
            elif latest_status.status == GoalStatus.ABORTED:
                rospy.logwarn("Goal aborted")
            elif latest_status.status == GoalStatus.PREEMPTED:
                rospy.loginfo("Goal preempted")
    
    def plan_callback(self, msg):
        """Receive and visualize planned path from move_base"""
        if len(msg.poses) > 0:
            # Create line strip marker for path visualization
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = rospy.Time.now()
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Red line for path
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.scale.x = 0.05  # Line width
            
            # Extract points from path
            for pose in msg.poses:
                p = Point()
                p.x = pose.pose.position.x
                p.y = pose.pose.position.y
                p.z = 0.0
                marker.points.append(p)
            
            # Publish marker
            self.path_marker_pub.publish(marker)
    
    def get_current_pose(self):
        """Get current robot pose from TF"""
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
            self.robot_pose = transform.transform.translation
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"Waiting for TF transform: {e}")
            return False
    
    def enter_path_planning_mode(self):
        """Enter path planning mode and disable exploration"""
        if not self.in_path_planning_mode:
            rospy.loginfo("=== Entering Path Planning Mode ===")
            self.in_path_planning_mode = True
            
            # Disable exploration with latched publisher (only if not already disabled)
            rospy.loginfo("Disabling exploration...")
            self.enable_exploration_pub.publish(Bool(False))
            rospy.sleep(1.0)  # Give explorer time to stop
        else:
            rospy.loginfo("Updating goal (already in path planning mode)")
        
        # Publish mode
        self.mode_pub.publish(String("path_planning"))
        
        # Publish goal
        self.goal_pub.publish(PointStamped(
            header={'frame_id': 'map'},
            point=self.current_goal
        ))
        
        # Send goal to move_base if available
        if self.move_base_client:
            self.send_goal_to_move_base()
        else:
            rospy.logwarn("move_base not available, using manual navigation")
    
    def send_goal_to_move_base(self):
        """Send goal to move_base action server"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Set goal position
        goal.target_pose.pose.position.x = self.current_goal.x
        goal.target_pose.pose.position.y = self.current_goal.y
        goal.target_pose.pose.position.z = 0.0
        
        # Set goal orientation to identity (let move_base handle orientation naturally)
        # This prevents DWA from trying to force a specific end orientation
        # which can cause backward motion when conflicting with forward movement
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0  # Identity quaternion
        
        # Calculate distance from robot to goal
        if self.robot_pose:
            dx = self.current_goal.x - self.robot_pose.x
            dy = self.current_goal.y - self.robot_pose.y
            distance = math.sqrt(dx**2 + dy**2)
            rospy.loginfo(f"[MOVE_BASE] Goal at ({self.current_goal.x:.2f}, {self.current_goal.y:.2f}), distance={distance:.2f}m")
            
            # Warn if goal is too close (within goal tolerance)
            goal_tolerance = rospy.get_param('move_base/TrajectoryPlannerROS/xy_goal_tolerance', 0.15)
            if distance < goal_tolerance:
                rospy.logwarn(f"⚠️  Goal is too close! Distance {distance:.2f}m < tolerance {goal_tolerance:.2f}m")
                rospy.logwarn("   Try clicking further away from robot")
        else:
            rospy.logwarn("[MOVE_BASE] Could not determine robot pose")
        
        self.move_base_client.send_goal(goal)
        self.current_move_base_goal = goal
    
    def manual_navigation(self):
        """Manual navigation using simple PID control"""
        if not self.get_current_pose():
            rospy.logwarn_throttle(5.0, "Cannot get robot pose")
            return
        
        # Calculate direction to goal
        dx = self.current_goal.x - self.robot_pose.x
        dy = self.current_goal.y - self.robot_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Check if goal reached
        if distance < self.goal_tolerance:
            rospy.loginfo("Goal reached (manual navigation)!")
            self.goal_reached = True
            # Stop robot
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            return
        
        # Target angle
        target_angle = math.atan2(dy, dx)
        
        # Get current robot angle from TF
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
            # Extract yaw from quaternion
            from tf.transformations import euler_from_quaternion
            q = transform.transform.rotation
            _, _, current_angle = euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            # Calculate angle error
            angle_error = target_angle - current_angle
            # Normalize to [-pi, pi] - robust method
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            
            # Simple PID for rotation - more forgiving threshold
            cmd = Twist()
            angle_threshold = 0.2  # ~11 degrees - more forgiving
            
            if abs(angle_error) > angle_threshold:
                # Still rotating
                cmd.angular.z = max(-self.max_turn_speed, min(self.max_turn_speed, angle_error * self.pid_angle_gain))
                cmd.linear.x = self.max_forward_speed * 0.5  # Move at 50% while turning
                rospy.loginfo_throttle(1.0, f"[MANUAL] Rotating: err={angle_error:.3f}rad, dist={distance:.2f}m, cmd=({cmd.linear.x:.3f}, {cmd.angular.z:.3f})")
            else:
                # Aligned, move forward
                cmd.angular.z = 0.0
                cmd.linear.x = self.max_forward_speed  # Full speed forward
                rospy.loginfo_throttle(1.0, f"[MANUAL] Moving fwd: err={angle_error:.3f}rad, dist={distance:.2f}m, cmd_x={cmd.linear.x:.3f}")
            
            self.cmd_vel_pub.publish(cmd)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, f"Cannot get robot angle: {e}")
    
    def return_to_exploration_mode(self):
        """Return to exploration mode"""
        rospy.loginfo("=== Returning to Exploration Mode ===")
        self.in_path_planning_mode = False
        self.current_goal = None
        self.goal_reached = False
        
        # Cancel move_base goal if active
        if self.move_base_client:
            self.move_base_client.cancel_goal()
        
        # Stop robot
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        rospy.sleep(0.5)
        
        # Re-enable exploration with latched publisher
        rospy.loginfo("Re-enabling exploration...")
        self.enable_exploration_pub.publish(Bool(True))
        rospy.sleep(1.0)
        
        # Publish mode
        self.mode_pub.publish(String("exploration"))
        
        rospy.loginfo("Back to exploration mode")
    
    def run(self):
        """Main loop"""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            if self.in_path_planning_mode:
                # Use manual navigation if move_base not available
                if not self.move_base_client:
                    rospy.loginfo_throttle(5.0, "Using manual navigation mode")
                    self.manual_navigation()
                else:
                    rospy.loginfo_throttle(5.0, "[MOVE_BASE] Navigating - check move_base output for cmds")
                    # When using move_base, don't send manual velocity commands
                    # move_base publishes directly to /cmd_vel
                
                # Check if goal reached
                if self.goal_reached:
                    rospy.loginfo("Path planning complete. Waiting for return_to_exploration command...")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        planner = PathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Path Planner shutting down")