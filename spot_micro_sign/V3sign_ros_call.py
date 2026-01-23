#!/usr/bin/env python3
import rospy
import socket
from std_msgs.msg import Bool

def main():
    rospy.init_node("sign_publisher_node")

    pub_stand = rospy.Publisher("/stand_cmd", Bool, queue_size=1)
    pub_idle  = rospy.Publisher("/idle_cmd",  Bool, queue_size=1)
    pub_walk  = rospy.Publisher("/walk_cmd",  Bool, queue_size=1)

    rospy.loginfo("SignPublisherNode started. Recieving info")

    # --- UDP Receiver ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 5005)) # Listens at port 5005 here

    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1024)
        msg = data.decode().strip()

        rospy.loginfo(f"Received: {msg}")

        if msg == "WALK":
            pub_walk.publish(True)

        elif msg == "STAND":
            pub_stand.publish(True)

        elif msg == "IDLE":
            pub_idle.publish(True)

if __name__ == "__main__":
    main()
