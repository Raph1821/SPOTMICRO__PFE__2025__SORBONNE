#!/usr/bin/env python3
"""
Autonomous Explorer Node for Spot Micro Robot

This node implements frontier-based exploration using the move_base navigation stack.
It continuously finds unexplored frontiers in the map and sends navigation goals to
explore the environment autonomously while SLAM generates a complete map.

The exploration algorithm:
1. Monitor the occupancy grid published by hector_slam
2. Detect frontiers (boundaries between known and unknown space)
3. Select the best frontier to explore
4. Send navigation goal to move_base
5. Repeat until exploration complete
"""

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback, MoveBaseActionResult
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Bool, String
import math
import threading
from collections import deque
import tf2_ros
import tf2_geometry_msgs
import time

class AutonomousExplorer:
    def __init__(self):
        """Initialize the autonomous explorer node"""
        rospy.init_node('autonomous_explorer', anonymous=False)
        
        # Initialize TF buffer for getting robot position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Parameters from launch file
        self.exploration_mode = rospy.get_param('~exploration_mode', 'frontier')
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.5)  # meters
        self.frontier_threshold = rospy.get_param('~frontier_threshold', 20)  # unknown cell percentage
        self.min_frontier_distance = rospy.get_param('~min_frontier_distance', 0.3)  # meters
        self.max_frontier_distance = rospy.get_param('~max_frontier_distance', 5.0)  # meters
        self.exploration_timeout = rospy.get_param('~exploration_timeout', 300.0)  # seconds (5 min)
        self.enable_exploration = rospy.get_param('~enable_exploration', True)
        
        # State variables
        self.occupancy_grid = None
        self.current_pose = None
        # Auto-start exploration if enable_exploration is true
        self.is_exploring = self.enable_exploration
        self.exploration_start_time = None if not self.is_exploring else rospy.Time.now()
        self.frontiers = []
        self.current_goal = None
        self.goal_reached = False
        self.stuck_counter = 0
        self.stuck_threshold = 5  # Number of consecutive failed attempts before retry
        
        # Lock for thread safety
        self.lock = threading.Lock()
        
        # Subscribe to map
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        # Note: We get position from TF instead of /odom for better compatibility
        # with both simulation and real robot setups
        
        # Create action client for move_base
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")
        
        # Subscribe to move_base feedback to get more information
        rospy.Subscriber('/move_base/feedback', MoveBaseActionFeedback, self.move_base_feedback_callback)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_base_result_callback)
        
        # Publisher for exploration status
        self.status_pub = rospy.Publisher('exploration_status', String, queue_size=1)
        self.frontier_pub = rospy.Publisher('frontiers', Point, queue_size=1)
        self.goal_pub = rospy.Publisher('exploration_goal', PoseStamped, queue_size=1)
        
        # Publishers for robot state commands (stand and walk)
        self.stand_cmd_pub = rospy.Publisher('stand_cmd', Bool, queue_size=1, latch=True)
        self.walk_cmd_pub = rospy.Publisher('walk_cmd', Bool, queue_size=1, latch=True)
        self.robot_ready = False
        
        # Service for starting/stopping exploration
        rospy.Subscriber('start_exploration', Bool, self.start_exploration_callback)
        rospy.Subscriber('stop_exploration', Bool, self.stop_exploration_callback)
        
        if self.is_exploring:
            rospy.loginfo("Autonomous exploration enabled and will auto-start")
            # Automatically trigger stand and walk states
            self.prepare_robot_for_exploration()
        else:
            rospy.loginfo("Autonomous exploration disabled - waiting for /start_exploration signal")
        
        rospy.loginfo("Autonomous Explorer initialized successfully")
    
    def move_base_feedback_callback(self, feedback_msg):
        """Callback for move_base feedback"""
        rospy.loginfo_throttle(2.0, f"move_base feedback: base_position=({feedback_msg.feedback.base_position.pose.position.x:.2f}, {feedback_msg.feedback.base_position.pose.position.y:.2f})")
    
    def move_base_result_callback(self, result_msg):
        """Callback for move_base result"""
        status = result_msg.status.status
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("move_base goal SUCCEEDED!")
        elif status == GoalStatus.ABORTED:
            rospy.logwarn(f"move_base goal ABORTED: {result_msg.status.text}")
        elif status == GoalStatus.REJECTED:
            rospy.logerr(f"move_base goal REJECTED: {result_msg.status.text}")
        else:
            rospy.loginfo(f"move_base goal status: {status}")
        
    def map_callback(self, msg):
        """Callback for occupancy grid updates"""
        with self.lock:
            self.occupancy_grid = msg
            
    def get_current_pose(self):
        """Get current robot pose from TF"""
        try:
            # Try base_footprint first (used by move_base)
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
            except:
                # Fallback to base_link if base_footprint doesn't exist
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
            
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = transform.header.stamp
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose.pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, f"Failed to get current pose: {e}")
            return None
            
    def prepare_robot_for_exploration(self):
        """Send stand and walk commands to prepare robot for exploration"""
        rospy.loginfo("Preparing robot for exploration: sending stand command...")
        # Send stand command
        stand_msg = Bool()
        stand_msg.data = True
        self.stand_cmd_pub.publish(stand_msg)
        
        # Wait for robot to stand up (typically 2-3 seconds)
        rospy.sleep(3.0)
        
        rospy.loginfo("Sending walk command to enable movement...")
        # Send walk command
        walk_msg = Bool()
        walk_msg.data = True
        self.walk_cmd_pub.publish(walk_msg)
        
        # Wait a bit for state transition
        rospy.sleep(1.0)
        
        self.robot_ready = True
        rospy.loginfo("Robot is ready for exploration!")
        
    def start_exploration_callback(self, msg):
        """Start autonomous exploration"""
        if msg.data:
            if not self.robot_ready:
                self.prepare_robot_for_exploration()
            self.is_exploring = True
            self.exploration_start_time = rospy.Time.now()
            self.stuck_counter = 0
            rospy.loginfo("Exploration started")
            self.status_pub.publish("EXPLORATION_STARTED")
        
    def stop_exploration_callback(self, msg):
        """Stop autonomous exploration"""
        if msg.data:
            self.is_exploring = False
            self.move_base_client.cancel_goal()
            rospy.loginfo("Exploration stopped")
            self.status_pub.publish("EXPLORATION_STOPPED")
    
    def find_frontiers(self):
        """
        Find frontier cells in the occupancy grid.
        
        Frontiers are the boundaries between known (occupied/free) and unknown cells.
        This is the key to exploration - the robot navigates to frontiers to expand
        the known map area.
        
        Returns:
            list: List of frontier points (x, y in world coordinates)
        """
        if self.occupancy_grid is None:
            return []
        
        data = np.array(self.occupancy_grid.data, dtype=np.int8).reshape(
            self.occupancy_grid.info.height, self.occupancy_grid.info.width
        )
        
        frontiers = []
        height, width = data.shape
        resolution = self.occupancy_grid.info.resolution
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        
        # Limit search to a reasonable area around robot (e.g., 20m x 20m)
        # This speeds up frontier detection significantly
        search_radius = 10.0  # meters
        if self.current_pose is not None:
            robot_x = self.current_pose.position.x
            robot_y = self.current_pose.position.y
            
            # Convert to grid coordinates
            grid_x = int((robot_x - origin_x) / resolution)
            grid_y = int((robot_y - origin_y) / resolution)
            search_pixels = int(search_radius / resolution)
            
            # Limit search range
            min_x = max(1, grid_x - search_pixels)
            max_x = min(width - 1, grid_x + search_pixels)
            min_y = max(1, grid_y - search_pixels)
            max_y = min(height - 1, grid_y + search_pixels)
        else:
            # If no robot pose, search entire map (slower)
            min_x, max_x = 1, width - 1
            min_y, max_y = 1, height - 1
        
        rospy.loginfo_throttle(10.0, f"Searching for frontiers in grid region: x=[{min_x}, {max_x}], y=[{min_y}, {max_y}]")
        
        # Find all frontier cells (unknown cells adjacent to known free space)
        # Use step size to speed up (check every 2nd pixel)
        step = 2
        for y in range(min_y, max_y, step):
            for x in range(min_x, max_x, step):
                # Unknown cell (value -1)
                if data[y, x] == -1:
                    # Check if adjacent to free space (only check 4 neighbors for speed)
                    neighbors = [
                        data[y-1, x], data[y+1, x],
                        data[y, x-1], data[y, x+1]
                    ]
                    
                    # If has free neighbors (0), it's a frontier
                    if 0 in neighbors:
                        # Convert from grid coordinates to world coordinates
                        world_x = origin_x + x * resolution
                        world_y = origin_y + y * resolution
                        frontiers.append((world_x, world_y))
        
        rospy.loginfo_throttle(10.0, f"Found {len(frontiers)} frontier cells (before clustering)")
        
        # Cluster frontiers to find frontier centers
        if len(frontiers) == 0:
            return []
        
        frontier_centers = self.cluster_frontiers(frontiers)
        rospy.loginfo_throttle(10.0, f"Clustered into {len(frontier_centers)} frontier groups")
        return frontier_centers
    
    def cluster_frontiers(self, frontiers, cluster_distance=0.5):
        """
        Cluster frontier cells into groups and compute centers.
        
        Args:
            frontiers: List of frontier points
            cluster_distance: Maximum distance for clustering
            
        Returns:
            list: List of frontier cluster centers
        """
        if len(frontiers) == 0:
            return []
        
        frontiers = np.array(frontiers)
        clusters = []
        visited = set()
        
        for i, frontier in enumerate(frontiers):
            if i in visited:
                continue
                
            # Start a new cluster
            cluster = [frontier]
            queue = deque([i])
            visited.add(i)
            
            while queue:
                current_idx = queue.popleft()
                current = frontiers[current_idx]
                
                # Find nearby frontiers
                for j, other in enumerate(frontiers):
                    if j not in visited:
                        dist = np.linalg.norm(current - other)
                        if dist < cluster_distance:
                            cluster.append(other)
                            queue.append(j)
                            visited.add(j)
            
            # Compute cluster center
            if len(cluster) > 0:
                center = np.mean(cluster, axis=0)
                clusters.append(tuple(center))
        
        return clusters
    
    def is_goal_in_free_space(self, goal_x, goal_y):
        """Check if goal position is in free space (not in obstacle)"""
        if self.occupancy_grid is None:
            return False  # Don't allow goals if no map
        
        data = np.array(self.occupancy_grid.data, dtype=np.int8).reshape(
            self.occupancy_grid.info.height, self.occupancy_grid.info.width
        )
        
        resolution = self.occupancy_grid.info.resolution
        origin_x = self.occupancy_grid.info.origin.position.x
        origin_y = self.occupancy_grid.info.origin.position.y
        
        # Convert world coordinates to grid coordinates
        grid_x = int((goal_x - origin_x) / resolution)
        grid_y = int((goal_y - origin_y) / resolution)
        
        # Check bounds
        if grid_x < 0 or grid_x >= self.occupancy_grid.info.width or \
           grid_y < 0 or grid_y >= self.occupancy_grid.info.height:
            return False
        
        # Check if cell is free (value 0) - prefer known free space over unknown
        # We prefer goals in KNOWN free space, but allow unknown if necessary
        cell_value = data[grid_y, grid_x]
        if cell_value > 50:  # Occupied - definitely not OK
            return False
        # Allow unknown (-1) for now, but prefer known free (0)
        # This is because early in exploration, most of the map is unknown
        
        # Also check surrounding cells to ensure goal is in a safe area
        safe_radius = 3  # Check 3 cells around goal
        for dy in range(-safe_radius, safe_radius + 1):
            for dx in range(-safe_radius, safe_radius + 1):
                check_x = grid_x + dx
                check_y = grid_y + dy
                if 0 <= check_x < self.occupancy_grid.info.width and \
                   0 <= check_y < self.occupancy_grid.info.height:
                    check_value = data[check_y, check_x]
                    # If any nearby cell is occupied (value > 50), goal is not safe
                    if check_value > 50:
                        return False
                    # Prefer known free space over unknown space
                    # If too many nearby cells are unknown, goal may not be reachable
                    if check_value == -1:
                        # Count unknown cells, but allow some
                        pass
        
        # Goal is acceptable (free or unknown, but not occupied)
        return True
    
    def select_frontier(self, frontier_centers):
        """
        Select the best frontier to explore next.
        
        Selection criteria:
        1. Distance from current pose (prefer closer, but not too close)
        2. Must be in free space (not in obstacle)
        3. Distance from already visited frontiers (avoid revisiting)
        
        Args:
            frontier_centers: List of frontier centers
            
        Returns:
            tuple: Selected frontier (x, y)
        """
        if len(frontier_centers) == 0:
            return None
        
        if self.current_pose is None:
            # No pose info, select first valid frontier
            for frontier in frontier_centers:
                if self.is_goal_in_free_space(frontier[0], frontier[1]):
                    return frontier
            return None
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        best_frontier = None
        best_score = float('-inf')
        
        for frontier in frontier_centers:
            # Check if goal is in free space
            if not self.is_goal_in_free_space(frontier[0], frontier[1]):
                rospy.logdebug(f"Frontier ({frontier[0]:.2f}, {frontier[1]:.2f}) is in obstacle, skipping")
                continue
            
            # Calculate distance from current position
            dist = math.sqrt(
                (frontier[0] - current_x)**2 + 
                (frontier[1] - current_y)**2
            )
            
            # Distance constraints - prefer goals that are not too close (avoid oscillation)
            if dist < self.min_frontier_distance:
                continue
            if dist > self.max_frontier_distance:
                continue
            
            # Score: prefer medium distance (not too close, not too far)
            # Closer is better, but add penalty for very close goals
            if dist < 0.5:
                score = -dist - 2.0  # Penalty for very close goals
            else:
                score = -dist  # Closer is better
            
            # Prefer new frontiers
            if best_frontier is None or score > best_score:
                best_frontier = frontier
                best_score = score
        
        if best_frontier:
            rospy.loginfo(f"Selected frontier: ({best_frontier[0]:.2f}, {best_frontier[1]:.2f}), distance: {math.sqrt((best_frontier[0] - current_x)**2 + (best_frontier[1] - current_y)**2):.2f}m")
        
        return best_frontier
    
    def send_goal(self, frontier):
        """
        Send navigation goal to move_base.
        
        Args:
            frontier: Frontier point (x, y)
            
        Returns:
            bool: True if goal sent successfully
        """
        if frontier is None:
            return False
        
        # Create goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = frontier[0]
        goal.target_pose.pose.position.y = frontier[1]
        
        # Set orientation to face the frontier
        goal.target_pose.pose.orientation.w = 1.0
        
        try:
            self.move_base_client.send_goal(goal)
            self.current_goal = frontier
            
            # Publish goal for visualization
            self.goal_pub.publish(goal.target_pose)
            rospy.loginfo(f"Sent goal to frontier: ({frontier[0]:.2f}, {frontier[1]:.2f})")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to send goal: {e}")
            return False
    
    def is_goal_reached(self):
        """Check if current goal has been reached"""
        state = self.move_base_client.get_state()
        return state == 3  # SUCCEEDED
    
    def run(self):
        """Main exploration loop"""
        rospy.loginfo("Starting autonomous exploration loop")
        rate = rospy.Rate(1.0)  # Run at 1 Hz
        
        while not rospy.is_shutdown():
            # Wait for robot to be ready (stand and walk states activated)
            if not self.robot_ready and self.is_exploring:
                rospy.loginfo("Waiting for robot to be ready (stand and walk states)...")
                rate.sleep()
                continue
                
            if not self.enable_exploration or not self.is_exploring:
                rate.sleep()
                continue
            
            # Check exploration timeout
            if self.exploration_timeout > 0:
                elapsed = (rospy.Time.now() - self.exploration_start_time).to_sec()
                if elapsed > self.exploration_timeout:
                    rospy.logwarn("Exploration timeout reached")
                    self.is_exploring = False
                    self.status_pub.publish("EXPLORATION_TIMEOUT")
                    rate.sleep()
                    continue
            
            with self.lock:
                if self.occupancy_grid is None:
                    rospy.logwarn_throttle(5.0, "Waiting for map data...")
                    rate.sleep()
                    continue
                
                rospy.loginfo_throttle(5.0, "Map data available, checking TF...")
                
                # Get current pose from TF
                current_pose = self.get_current_pose()
                if current_pose is None:
                    rospy.logwarn_throttle(5.0, "Waiting for TF data (map -> base_footprint)...")
                    rate.sleep()
                    continue
                
                rospy.loginfo_throttle(5.0, f"Current pose: x={current_pose.position.x:.2f}, y={current_pose.position.y:.2f}")
                
                # Store for use in frontier selection
                self.current_pose = current_pose
                
                # Find frontiers and select goal
                rospy.loginfo_throttle(5.0, "Finding frontiers...")
                frontiers = self.find_frontiers()
                
                if len(frontiers) == 0:
                    rospy.logwarn_throttle(5.0, "No frontiers found - waiting for map to expand...")
                    rate.sleep()
                    continue
                
                rospy.loginfo(f"Found {len(frontiers)} frontier clusters")
                
                # Publish frontiers for visualization
                for frontier in frontiers[:10]:  # Limit to first 10
                    point = Point()
                    point.x = frontier[0]
                    point.y = frontier[1]
                    self.frontier_pub.publish(point)
                
                # Check if current goal is reached or needs to be updated
                if self.current_goal is not None:
                    state = self.move_base_client.get_state()
                    
                    # Check if goal is actually making progress (even if state is PENDING)
                    # Sometimes move_base accepts goal but state stays PENDING while planning
                    if state == 1:  # PENDING
                        # Check if robot is actually moving by comparing positions
                        current_pos = self.get_current_pose()
                        if current_pos and hasattr(self, 'last_check_pos'):
                            dist_moved = math.sqrt(
                                (current_pos.position.x - self.last_check_pos.position.x)**2 +
                                (current_pos.position.y - self.last_check_pos.position.y)**2
                            )
                            if dist_moved > 0.01:  # Moved more than 1cm
                                rospy.loginfo_throttle(2.0, f"Robot is moving! Distance moved: {dist_moved:.3f}m (state: {state})")
                                # Treat as active even if state is PENDING
                                state = 2  # Treat as ACTIVE
                        self.last_check_pos = current_pos
                    
                    rospy.loginfo_throttle(2.0, f"move_base state: {state} (1=PENDING, 2=ACTIVE, 3=SUCCEEDED, 4=ABORTED)")
                    
                    if state == 3:  # SUCCEEDED
                        rospy.loginfo("Goal reached, selecting next frontier")
                        self.goal_reached = True
                        self.stuck_counter = 0
                        self.current_goal = None
                    elif state == 4:  # ABORTED
                        self.stuck_counter += 1
                        rospy.logwarn(f"Goal aborted (stuck counter: {self.stuck_counter})")
                        if self.stuck_counter >= self.stuck_threshold:
                            rospy.logwarn("Goal aborted multiple times, selecting new goal")
                            self.goal_reached = True
                            self.stuck_counter = 0
                            self.current_goal = None
                    elif state == 8:  # REJECTED
                        rospy.logwarn("Goal was rejected by move_base, selecting new goal")
                        self.current_goal = None
                    elif state == 2:  # ACTIVE (or treated as active)
                        # Goal is active, move_base should be publishing cmd_vel
                        rospy.loginfo_throttle(2.0, "Goal is active, move_base should be moving robot")
                    else:
                        # PENDING or other state
                        rospy.loginfo_throttle(2.0, f"Waiting for move_base to process goal (state: {state})")
                    rate.sleep()
                    continue
                
                # Select and send new goal
                frontier = self.select_frontier(frontiers)
                if frontier is not None:
                    # Calculate distance to frontier
                    dist_to_frontier = math.sqrt(
                        (frontier[0] - current_pose.position.x)**2 + 
                        (frontier[1] - current_pose.position.y)**2
                    )
                    rospy.loginfo(f"Selected frontier: ({frontier[0]:.2f}, {frontier[1]:.2f}), distance: {dist_to_frontier:.2f}m")
                    
                    # Verify goal is in free space before sending
                    if not self.is_goal_in_free_space(frontier[0], frontier[1]):
                        rospy.logwarn(f"Frontier ({frontier[0]:.2f}, {frontier[1]:.2f}) is in obstacle, skipping...")
                        rate.sleep()
                        continue
                    
                    if self.send_goal(frontier):
                        rospy.loginfo("Goal sent successfully to move_base")
                        # Wait longer to check if path planning succeeds
                        rospy.sleep(3.0)
                        state = self.move_base_client.get_state()
                        rospy.loginfo(f"move_base state after 3s: {state} (1=PENDING, 2=ACTIVE, 3=SUCCEEDED, 4=ABORTED, 8=REJECTED)")
                        if state == 8:  # REJECTED
                            rospy.logwarn("Goal was rejected - may be unreachable or in obstacle")
                            rospy.logwarn("This usually means global path planning failed")
                        elif state == 4:  # ABORTED
                            rospy.logwarn("Goal was aborted - path planning may have failed")
                        elif state == 1:  # PENDING
                            rospy.logwarn("Goal still PENDING after 3s - path planning may be taking too long")
                            rospy.logwarn("Check move_base logs for path planning errors")
                    else:
                        rospy.logwarn("Failed to send goal to move_base")
                else:
                    rospy.logwarn("Could not select valid frontier")
                    if len(frontiers) > 0:
                        rospy.logwarn(f"Found {len(frontiers)} frontiers but none were selected")
                        rospy.logwarn("This may indicate all frontiers are too close, too far, or in obstacles")
                    rate.sleep()
                    continue
            
            rate.sleep()

if __name__ == '__main__':
    try:
        explorer = AutonomousExplorer()
        explorer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Autonomous explorer shutting down")
