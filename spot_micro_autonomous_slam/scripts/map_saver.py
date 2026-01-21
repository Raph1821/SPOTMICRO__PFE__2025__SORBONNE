#!/usr/bin/env python3
"""
Map Saver Utility

Automatically saves maps at specified intervals or when exploration completes.
Stores multiple map snapshots and the final map with metadata.

Usage:
  rosrun spot_micro_autonomous_slam map_saver.py
  
The saved maps will be in the directory specified by the map_save_dir parameter.
"""

import rospy
import os
import subprocess
from datetime import datetime
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
import signal
import sys

class MapSaver:
    """Automatically saves maps from SLAM"""
    
    def __init__(self):
        """Initialize map saver"""
        rospy.init_node('map_saver', anonymous=False)
        
        # Parameters
        self.map_save_dir = rospy.get_param('~map_save_dir', os.path.expanduser('~/spot_micro_maps'))
        self.save_interval = rospy.get_param('~save_interval', 60.0)  # seconds
        self.auto_save = rospy.get_param('~auto_save', True)
        
        # Create directory if it doesn't exist
        os.makedirs(self.map_save_dir, exist_ok=True)
        
        # State variables
        self.last_save_time = rospy.Time.now()
        self.map_received = False
        self.exploration_complete = False
        
        # Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('exploration_status', String, self.exploration_status_callback)
        rospy.Subscriber('save_map_now', Bool, self.save_map_callback)
        
        # Publishers
        self.save_status_pub = rospy.Publisher('map_save_status', String, queue_size=1)
        
        rospy.loginfo(f"Map saver initialized. Maps will be saved to: {self.map_save_dir}")
        rospy.loginfo(f"Auto-save interval: {self.save_interval} seconds")
        
        # Register signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        """Handle shutdown signal"""
        rospy.loginfo("Saving final map before shutdown...")
        self.save_map("final_map")
        rospy.signal_shutdown("Map saver shutting down")
    
    def map_callback(self, msg):
        """Callback for map updates"""
        self.map_received = True
        
        # Auto-save at intervals
        if self.auto_save:
            elapsed = (rospy.Time.now() - self.last_save_time).to_sec()
            if elapsed >= self.save_interval:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.save_map(f"map_{timestamp}")
                self.last_save_time = rospy.Time.now()
    
    def exploration_status_callback(self, msg):
        """Callback for exploration status updates"""
        status = msg.data
        
        if status == "EXPLORATION_COMPLETE":
            rospy.loginfo("Exploration complete! Saving final map...")
            self.exploration_complete = True
            self.save_map("final_exploration_map")
        elif status == "EXPLORATION_TIMEOUT":
            rospy.loginfo("Exploration timeout! Saving map...")
            self.save_map("timeout_map")
        elif status == "EXPLORATION_FAILED":
            rospy.loginfo("Exploration failed! Saving map...")
            self.save_map("failed_exploration_map")
    
    def save_map_callback(self, msg):
        """Save map on demand"""
        if msg.data:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.save_map(f"manual_save_{timestamp}")
    
    def save_map(self, map_name):
        """
        Save the current map using map_server.
        
        Args:
            map_name: Name for the saved map (without extension)
        """
        if not self.map_received:
            rospy.logwarn("No map received yet, cannot save")
            self.save_status_pub.publish("NO_MAP_RECEIVED")
            return
        
        try:
            map_path = os.path.join(self.map_save_dir, map_name)
            
            # Call map_saver service
            command = ['rosrun', 'map_server', 'map_saver', '-f', map_path]
            rospy.loginfo(f"Saving map: {map_path}")
            
            # Run map_saver command
            result = subprocess.run(command, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                rospy.loginfo(f"Map saved successfully: {map_path}")
                self.save_status_pub.publish(f"MAP_SAVED: {map_name}")
                
                # Log map info
                yaml_file = map_path + ".yaml"
                pgm_file = map_path + ".pgm"
                if os.path.exists(yaml_file) and os.path.exists(pgm_file):
                    yaml_size = os.path.getsize(yaml_file)
                    pgm_size = os.path.getsize(pgm_file)
                    rospy.loginfo(f"  YAML: {yaml_size} bytes")
                    rospy.loginfo(f"  PGM: {pgm_size} bytes")
            else:
                rospy.logerr(f"Failed to save map: {result.stderr}")
                self.save_status_pub.publish(f"MAP_SAVE_FAILED: {map_name}")
        
        except subprocess.TimeoutExpired:
            rospy.logerr("Map saver command timed out")
            self.save_status_pub.publish("MAP_SAVE_TIMEOUT")
        except Exception as e:
            rospy.logerr(f"Error saving map: {e}")
            self.save_status_pub.publish(f"MAP_SAVE_ERROR: {str(e)}")
    
    def run(self):
        """Main loop"""
        rospy.loginfo("Map saver running...")
        
        # Save metadata file
        try:
            metadata_file = os.path.join(self.map_save_dir, "metadata.txt")
            with open(metadata_file, 'w') as f:
                f.write(f"Spot Micro Autonomous SLAM Map Archive\n")
                f.write(f"Created: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"Save Directory: {self.map_save_dir}\n")
                f.write(f"Auto-save Interval: {self.save_interval} seconds\n")
        except Exception as e:
            rospy.logwarn(f"Could not write metadata: {e}")
        
        rospy.spin()

if __name__ == '__main__':
    try:
        saver = MapSaver()
        saver.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Map saver shutting down")
