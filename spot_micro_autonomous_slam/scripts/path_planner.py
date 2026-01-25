#!/usr/bin/env python3
"""
Path Planner Override
- Listens to RViz clicked goals
- Overrides simple_explorer
- Drives robot to clicked goal
"""

import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
import tf2_ros

class PathPlanner:
    def __init__(self):
        rospy.init_node("path_planner")

        # Parameters
        self.max_forward_speed = rospy.get_param("~max_forward_speed", 0.25)
        self.max_turn_speed = rospy.get_param("~max_turn_speed", 0.4)
        self.goal_threshold = rospy.get_param("~goal_threshold", 0.3)

        # State
        self.current_goal = None

        # Publishers
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.exploration_enable_pub = rospy.Publisher(
            "/enable_exploration", Bool, queue_size=1, latch=True
        )

        # Subscribers
        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal",
            PoseStamped,
            self.goal_callback
        )

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("Path Planner ready — click a goal in RViz")

    # --------------------------------------------------

    def goal_callback(self, msg):
        self.current_goal = (
            msg.pose.position.x,
            msg.pose.position.y
        )

        rospy.loginfo(
            f"New RViz goal: ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f})"
        )

        # Disable exploration
        self.exploration_enable_pub.publish(Bool(data=False))

    # --------------------------------------------------

    def get_robot_pose(self):
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
        gx, gy = self.current_goal
        return math.hypot(gx - rx, gy - ry) < self.goal_threshold

    # --------------------------------------------------

    def run(self):
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
            gx, gy = self.current_goal

            dx = gx - rx
            dy = gy - ry

            angle_to_goal = math.atan2(dy, dx)
            angle_error = math.atan2(
                math.sin(angle_to_goal - yaw),
                math.cos(angle_to_goal - yaw)
            )

            cmd = Twist()

            # Goal reached
            if self.goal_reached(rx, ry):
                rospy.loginfo("Goal reached — resuming exploration")
                self.current_goal = None
                self.exploration_enable_pub.publish(Bool(data=True))
                self.cmd_vel_pub.publish(Twist())
                rate.sleep()
                continue

            # Angular control
            cmd.angular.z = max(
                -self.max_turn_speed,
                min(self.max_turn_speed, angle_error * 2.0)
            )

            # Linear control
            if abs(angle_error) < math.pi / 4:
                cmd.linear.x = self.max_forward_speed
            else:
                cmd.linear.x = self.max_forward_speed * 0.3

            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

# ------------------------------------------------------

if __name__ == "__main__":
    try:
        planner = PathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
