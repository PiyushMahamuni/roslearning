#! /usr/bin/env python

import rospy
from std_msgs.msg import String

# CONSTANTS
NODE_NAME = "listener_py"
TOPIC_NAME = "chat"

# GLOBALS
sub = None      # Will be subscriber object for topic name

# callback function for /chat topic
def callback(msg):
    rospy.loginfo(f"I heard [{msg.data}]")

# setup node
def setup():
    global sub
    rospy.init_node(NODE_NAME)
    sub = rospy.Subscriber(TOPIC_NAME, String, callback, queue_size=10)


if __name__ == "__main__":
    try:
        setup()
        rospy.spin()    # keep node from exiting and allow ros to process incoming msgs
    except rospy.ROSInterruptException:
        pass