#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

# Import the V2 sampler version of sign_pipe
from V2sign_pipe import fire_sign_command, cap


def main():
    rospy.init_node("sign_publisher_node")

    pub_stand = rospy.Publisher("/stand_cmd", Bool, queue_size=1)
    pub_idle  = rospy.Publisher("/idle_cmd",  Bool, queue_size=1)
    pub_walk  = rospy.Publisher("/walk_cmd",  Bool, queue_size=1)

    rospy.loginfo("SignPublisherNode started.")

    #rate = rospy.Rate(30) # already time break 2sec in V2sign_pipe

    while not rospy.is_shutdown() and cap.isOpened():
        command, frame = fire_sign_command()

        if command == "stand":
            rospy.loginfo("PUBLISH → STAND")
            pub_stand.publish(True)

        elif command == "idle":
            rospy.loginfo("PUBLISH → IDLE")
            pub_idle.publish(True)

        elif command == "walk":
            rospy.loginfo("PUBLISH → WALK")
            pub_walk.publish(True)

        #rate.sleep()

    cap.release()


if __name__ == "__main__":
    main()
