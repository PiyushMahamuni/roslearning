#! /usr/bin/env python

import rospy, tf, geometry_msgs.msg, turtlesim.srv

# CONSTANTS
NODE_NAME = "frame_a_to_frame_b_listener"
PARENT_FRAME = "/frame_a"
CHILD_FRAME = "/frame_b"

# GLOBALS
listener = None # listener to the transformation


def setup():
    rospy.init_node(NODE_NAME)
    global listener
    listener = tf.TransformListener()


def main():
    setup()
    loop_rate = rospy.Rate(2.0)
    listener.waitForTransform(PARENT_FRAME, CHILD_FRAME, rospy.Time(), rospy.Duration(4))
    # if we try to listen to non existing transformation, it will then raise a programming exception
    # the last arg is timeout duration

    while not rospy.is_shutdown():
        try:
            (translation, rotation) = listener.lookupTransform(PARENT_FRAME, CHILD_FRAME, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rpy = tf.transformations.euler_from_quaternion(rotation)
        print(f"Transformation between {PARENT_FRAME} and {CHILD_FRAME} detected!")
        print(f"Translation vector: {translation}")
        print(f"Roll = {rpy[0]}, Pitch = {rpy[1]}, Yaw = {rpy[2]}")

        loop_rate.sleep()
    

if __name__ == "__main__":
    main()