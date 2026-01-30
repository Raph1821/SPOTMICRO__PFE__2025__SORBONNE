import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Vector3
from math import pi

msg = "awaiting SIGN command"

valid_cmds = ('quit','Quit','walk','stand','idle','angle_cmd')


class SpotMicroKeyboardControl():
    def __init__(self):
        rospy.init_node("spot_micro_sign_control")

        # Event messages
        self._walk_event_cmd_msg  = Bool(data=True)
        self._stand_event_cmd_msg = Bool(data=True)
        self._idle_event_cmd_msg  = Bool(data=True)

        # Publishers (same topics as C++ controller)
        self._ros_pub_walk_cmd  = rospy.Publisher('/walk_cmd',  Bool, queue_size=1)
        self._ros_pub_stand_cmd = rospy.Publisher('/stand_cmd', Bool, queue_size=1)
        self._ros_pub_idle_cmd  = rospy.Publisher('/idle_cmd',  Bool, queue_size=1)

    def reset_all_motion_commands_to_zero(self):
        zero_vel = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        pub.publish(zero_vel)

    def run(self):
        self.reset_all_motion_commands_to_zero()
        rospy.loginfo('SIGN control loop started.')

        while not rospy.is_shutdown():
            print(msg)
            userInput = input("Command?: ")

            if userInput == 'quit':
                rospy.loginfo("Exiting SIGN control node...")
                break

            elif userInput == 'stand':
                self._ros_pub_stand_cmd.publish(self._stand_event_cmd_msg)
                rospy.loginfo('Stand command issued from SIGN.')

            elif userInput == 'idle':
                self._ros_pub_idle_cmd.publish(self._idle_event_cmd_msg)
                rospy.loginfo('Idle command issued from SIGN.')

            elif userInput == 'walk':
                self._ros_pub_walk_cmd.publish(self._walk_event_cmd_msg)
                rospy.loginfo('Walk command issued from SIGN.')

            else:
                rospy.logwarn("Invalid SIGN command: %s", userInput)


if __name__ == "__main__":
    node = SpotMicroKeyboardControl()
    node.run()
