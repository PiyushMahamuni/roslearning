#!/usr/bin/env python

import rospy
import tf


# CONSTANTS
NODE_NAME = "frame_a_frame_b_broadcaster"
PARENT_FRAME = "frame_a"
CHILD_FRAME = "frame_b"
transform_broadcaster = None


# SETTING UP THE NODE
def setup():
    rospy.init_node(NODE_NAME)
    global transform_broadcaster
    transform_broadcaster = tf.TransformBroadcaster()


def main():
    setup()
    loop_rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        # create a quaternion
        rotation_quaternion = tf.transformations.quaternion_from_euler(
            0.2, 0.3, 0.1)
        # create translation vector
        translation_vector = (1.0, 2.0, 3.0)

        # current time
        ctime = rospy.Time.now()
        # each transformation needs to be stamped with appropriate time, here the time
        # just before broadcasting it is used.
        transform_broadcaster.sendTransform(
            translation_vector, rotation_quaternion, ctime, CHILD_FRAME, PARENT_FRAME)
        # the last two arguments are strings - frame_id(s) of parent and then child frame
        loop_rate.sleep()


if __name__ == "__main__":
    main()
    # now after running this script, open a new terminal and run following command
    # rosrun tf2_tools echo.py frame_a frame_b
    # rosrun tf2_tools view_frames.py
    # evince frames.py