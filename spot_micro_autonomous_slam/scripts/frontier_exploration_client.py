#!/usr/bin/env python3
"""
Frontier Exploration Client Node

This node provides an alternative exploration interface using the frontier_exploration
ROS package. It can be used alongside or instead of the autonomous_explorer node
for more advanced frontier selection algorithms.

This implementation provides integration with the frontier_exploration server,
which offers sophisticated frontier detection and selection.
"""

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from actionlib import SimpleActionClient
import actionlib_msgs
import math

# Try to import frontier_exploration (optional dependency)
try:
    from frontier_exploration.msg import ExploreTaskAction, ExploreTaskGoal
    FRONTIER_EXPLORATION_AVAILABLE = True
except ImportError:
    FRONTIER_EXPLORATION_AVAILABLE = False
    rospy.logwarn("frontier_exploration package not available. frontier_exploration_client will be disabled.")

class FrontierExplorationClient:
    """Client for frontier_exploration package integration"""
    
    def __init__(self):
        """Initialize frontier exploration client"""
        if not FRONTIER_EXPLORATION_AVAILABLE:
            rospy.logerr("frontier_exploration package not available. Cannot initialize client.")
            return
            
        rospy.init_node('frontier_exploration_client', anonymous=False)
        
        # Parameters
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.explore_boundary = rospy.get_param('~explore_boundary', 1.0)  # meters
        self.enable_frontier_server = rospy.get_param('~enable_frontier_server', False)
        
        self.is_exploring = False
        self.current_goal = None
        
        # Only initialize if frontier_exploration server is available and enabled
        if self.enable_frontier_server:
            try:
                self.explore_client = SimpleActionClient('explore_server', ExploreTaskAction)
                rospy.loginfo("Waiting for explore_server...")
                self.explore_client.wait_for_server(rospy.Duration(5.0))
                rospy.loginfo("Connected to explore_server")
                self.frontier_server_available = True
            except Exception as e:
                rospy.logwarn(f"Frontier exploration server not available: {e}")
                self.frontier_server_available = False
        else:
            self.frontier_server_available = False
        
        # Subscribers
        rospy.Subscriber('start_frontier_exploration', Bool, self.start_callback)
        rospy.Subscriber('stop_frontier_exploration', Bool, self.stop_callback)
        
        # Publishers
        self.status_pub = rospy.Publisher('frontier_exploration_status', String, queue_size=1)
        
        rospy.loginfo("Frontier Exploration Client initialized")
    
    def start_callback(self, msg):
        """Start frontier exploration"""
        if msg.data and not self.is_exploring:
            self.start_exploration()
    
    def stop_callback(self, msg):
        """Stop frontier exploration"""
        if msg.data and self.is_exploring:
            self.stop_exploration()
    
    def start_exploration(self):
        """Start autonomous frontier exploration"""
        if not FRONTIER_EXPLORATION_AVAILABLE:
            rospy.logerr("frontier_exploration package not available")
            return
            
        if not self.frontier_server_available:
            rospy.logwarn("Frontier exploration server not available")
            self.status_pub.publish("FRONTIER_SERVER_UNAVAILABLE")
            return
        
        try:
            # Create explore task
            goal = ExploreTaskGoal()
            
            # Set explore boundary relative to current robot position
            # Using a rectangular boundary around the robot
            goal.explore_boundary = self.explore_boundary
            
            self.explore_client.send_goal(goal)
            self.is_exploring = True
            rospy.loginfo("Frontier exploration started")
            self.status_pub.publish("FRONTIER_EXPLORATION_STARTED")
        except Exception as e:
            rospy.logerr(f"Failed to start frontier exploration: {e}")
            self.status_pub.publish("FRONTIER_EXPLORATION_FAILED")
    
    def stop_exploration(self):
        """Stop frontier exploration"""
        try:
            self.explore_client.cancel_goal()
            self.is_exploring = False
            rospy.loginfo("Frontier exploration stopped")
            self.status_pub.publish("FRONTIER_EXPLORATION_STOPPED")
        except Exception as e:
            rospy.logerr(f"Failed to stop frontier exploration: {e}")
    
    def run(self):
        """Main loop"""
        rate = rospy.Rate(1.0)
        
        while not rospy.is_shutdown():
            if self.is_exploring and self.frontier_server_available:
                state = self.explore_client.get_state()
                
                if state == actionlib_msgs.msg.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Frontier exploration completed")
                    self.is_exploring = False
                    self.status_pub.publish("FRONTIER_EXPLORATION_COMPLETE")
                elif state == actionlib_msgs.msg.GoalStatus.ABORTED:
                    rospy.logerr("Frontier exploration aborted")
                    self.is_exploring = False
                    self.status_pub.publish("FRONTIER_EXPLORATION_ABORTED")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        if not FRONTIER_EXPLORATION_AVAILABLE:
            rospy.logerr("frontier_exploration package not available. Please install it or use autonomous_explorer.py instead.")
            exit(1)
            
        client = FrontierExplorationClient()
        if hasattr(client, 'frontier_server_available') and client.frontier_server_available:
            client.run()
        else:
            rospy.loginfo("Running without frontier_exploration server. Use autonomous_explorer instead.")
            rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Frontier exploration client shutting down")
