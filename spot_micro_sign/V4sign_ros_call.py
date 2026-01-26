#!/usr/bin/env python3
import rospy
import socket
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from math import radians


VEL_VAL = 0.20 #for velocity value change incrementally
ANGLE_VAL = 3 # for angular value change incrementally
ANGLE_RAD = radians(ANGLE_VAL)


def main():
    rospy.init_node("sign_publisher_node")

    pub_stand = rospy.Publisher("/stand_cmd", Bool, queue_size=1)
    pub_idle  = rospy.Publisher("/idle_cmd",  Bool, queue_size=1)
    pub_walk  = rospy.Publisher("/walk_cmd",  Bool, queue_size=1)
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rospy.loginfo("SignPublisherNode started. Recieving info")

    # socket Receiver
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 5005)) # Listens at port 5005 here


    # Keeping track of current velocity state
    current_twist = Twist()

    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024)
        msg = data.decode().strip()

        rospy.loginfo(f"Received: {msg}")

        if msg == "WALK":
            pub_walk.publish(True)

        elif msg == "STAND":
            pub_stand.publish(True)
            # Reset velocities when standing or it will resume last values and problems
            current_twist = Twist()
            pub_vel.publish(current_twist)

        elif msg == "IDLE":
            pub_idle.publish(True)

        elif msg == "FORWARD":
            current_twist.linear.x += VEL_VAL
            pub_vel.publish(current_twist)
            rospy.loginfo(f"Received: {msg, current_twist}")
            print(msg, current_twist) # debug

        elif msg == "BACKWARD":
            current_twist.linear.x -= VEL_VAL
            pub_vel.publish(current_twist)
            rospy.loginfo(f"Received: {msg, current_twist}")
            print(msg, current_twist)  # debug

        elif msg == "ROTATE_LEFT":
            current_twist.angular.z += ANGLE_RAD
            pub_vel.publish(current_twist)
            rospy.loginfo(f"Received: {msg, current_twist}")
            print(msg, current_twist)  # debug

        elif msg == "ROTATE_RIGHT":
            current_twist.angular.z -= ANGLE_RAD
            pub_vel.publish(current_twist)
            rospy.loginfo(f"Received: {msg, current_twist}")
            print(msg, current_twist)  # debug

        elif msg.startswith("VEL"):
            parts = msg.split()
            if len(parts) == 4:
                current_twist.linear.x = float(parts[1])
                current_twist.linear.y = float(parts[2])
                current_twist.angular.z = float(parts[3])
                pub_vel.publish(current_twist)
                rospy.loginfo(f"Received: {msg, current_twist}")
                print(msg, current_twist)  # debug



if __name__ == "__main__":
    main()
