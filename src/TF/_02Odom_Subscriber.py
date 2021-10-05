#! /usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry

# CONSTANTS
NODE_NAME = "odom_subscriber"
ODOM_TOPIC = "/odom"


# GLOBALS
odom_sub = None


def odom_callback(msg):
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    
    # needs an iterable with values of quternions as sequenced list - x, y, z, w
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
    print(f"{'-' * 50}")
    print(f"roll: {roll}, pitch: {pitch}, yaw: {yaw}")
    print(f"{'-' * 50}")


def setup():
    rospy.init_node(NODE_NAME)
    global odom_sub
    odom_sub = rospy.Subscriber(
        ODOM_TOPIC, Odometry, odom_callback, queue_size=1)


def main():
    setup()
    rospy.spin()


if __name__ == "__main__":
    main()
