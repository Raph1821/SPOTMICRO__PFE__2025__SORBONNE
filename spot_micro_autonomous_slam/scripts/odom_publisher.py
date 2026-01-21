#!/usr/bin/env python3
"""
Odometry Publisher Node

Publishes /odom topic (nav_msgs/Odometry) from TF transform between odom and base_footprint.
This is required for move_base to work properly.
"""

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import math

class OdometryPublisher:
    def __init__(self):
        rospy.init_node('odom_publisher', anonymous=False)
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publisher for odometry
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # Rate for publishing (20 Hz)
        self.rate = rospy.Rate(20.0)
        
        rospy.loginfo("Odometry publisher initialized")
        rospy.loginfo("Waiting for TF transform (odom -> base_footprint)...")
        
    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
        
    def run(self):
        """Main loop"""
        last_time = rospy.Time.now()
        last_x = 0.0
        last_y = 0.0
        last_yaw = 0.0
        
        while not rospy.is_shutdown():
            try:
                # Get transform from odom to base_footprint
                transform = self.tf_buffer.lookup_transform('odom', 'base_footprint', rospy.Time(0))
                
                current_time = rospy.Time.now()
                dt = (current_time - last_time).to_sec()
                
                # Log first successful transform
                if last_time.to_sec() == 0:
                    rospy.loginfo("Successfully received TF transform, starting to publish odom")
                
                if dt > 0:
                    # Extract position
                    x = transform.transform.translation.x
                    y = transform.transform.translation.y
                    yaw = self.quaternion_to_yaw(transform.transform.rotation)
                    
                    # Calculate velocities (simple finite difference)
                    vx = (x - last_x) / dt if dt > 0 else 0.0
                    vy = (y - last_y) / dt if dt > 0 else 0.0
                    vyaw = (yaw - last_yaw) / dt if dt > 0 else 0.0
                    
                    # Create odometry message
                    odom = Odometry()
                    odom.header.stamp = current_time
                    odom.header.frame_id = "odom"
                    odom.child_frame_id = "base_footprint"
                    
                    # Position
                    odom.pose.pose.position.x = x
                    odom.pose.pose.position.y = y
                    odom.pose.pose.position.z = 0.0
                    odom.pose.pose.orientation = transform.transform.rotation
                    
                    # Velocity
                    odom.twist.twist.linear.x = vx
                    odom.twist.twist.linear.y = vy
                    odom.twist.twist.linear.z = 0.0
                    odom.twist.twist.angular.x = 0.0
                    odom.twist.twist.angular.y = 0.0
                    odom.twist.twist.angular.z = vyaw
                    
                    # Covariance (set to reasonable values)
                    odom.pose.covariance[0] = 0.01  # x
                    odom.pose.covariance[7] = 0.01  # y
                    odom.pose.covariance[35] = 0.01  # yaw
                    odom.twist.covariance[0] = 0.01  # vx
                    odom.twist.covariance[7] = 0.01  # vy
                    odom.twist.covariance[35] = 0.01  # vyaw
                    
                    # Publish
                    self.odom_pub.publish(odom)
                    
                    # Log periodically for debugging
                    rospy.loginfo_throttle(5.0, f"Publishing odom: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}, vx={vx:.2f}, vy={vy:.2f}, vyaw={vyaw:.2f}")
                    
                    # Update last values
                    last_x = x
                    last_y = y
                    last_yaw = yaw
                    last_time = current_time
                    
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn_throttle(1.0, f"TF lookup failed: {e}")
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = OdometryPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Odometry publisher shutting down")

