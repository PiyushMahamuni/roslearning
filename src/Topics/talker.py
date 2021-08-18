#! /usr/bin/env python

import rospy
from std_msgs.msg import String     # msg type

# CONSTANTS
NODE_NAME = "talker_py"
TOPIC_NAME = "chat"
PUB_FREQ = 1

# GLOBALS
pub = None  # will be publisher for /chat topic

# function that gets called peridodically with PUB_FREQ fequency
def talk(event):
    hello_str = f"{talk.count} Hello World"
    pub.publish(String(hello_str))
    rospy.loginfo(f"[{NODE_NAME}] I published {hello_str}")
    talk.count += 1


def setup():
    global pub
    rospy.init_node(NODE_NAME)
    pub = rospy.Publisher(TOPIC_NAME, String, queue_size=10)
    talk.count = 0
    dur = rospy.Duration(1 / PUB_FREQ)
    rospy.Timer(dur, talk)


if __name__ == "__main__":
    try:
        setup()
        rospy.spin()    # keep program from exiting
    except rospy.ROSInterruptException:
        pass