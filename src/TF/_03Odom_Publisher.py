#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

# CONSTANTS
NODE_NAME = "odom_publisher"
ODOM_TOPIC = "/odom"
odom_msg = Odometry()
odom_msg.pose.pose.orientation = Quaternion()
odom_msg.pose.pose.orientation.x = -0.3
odom_msg.pose.pose.orientation.y = 0.45
odom_msg.pose.pose.orientation.z = 0.87
odom_msg.pose.pose.orientation.w = 0.21

# GLOBALS
odom_pub = None


def setup():
    rospy.init_node(NODE_NAME)
    global odom_pub
    odom_pub = rospy.Publisher(ODOM_TOPIC, Odometry, queue_size=1)


def main():
    setup()

    loop_rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        odom_pub.publish(odom_msg)
        loop_rate.sleep()


if __name__ == "__main__":
    main()