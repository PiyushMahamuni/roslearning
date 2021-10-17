#! /usr/bin/env python

import rospy, tf
from turtlesim.msg import Pose


# CONSTANTS
NODE_NAME = "turtle_tf_broadcaster"

# GLOBALS
pose_sub = None
transform_broadcaster = None

# you can pass extra arguments to a callback functions by mentioning the arguements in the
# rospy.Subscirber for which you pass the function address as the callback
def pose_callback(msg, turtlename):
    # convert degrees to quaternions
    rotation_quternion = tf.transformations.quaternion_from_euler(0, 0, msg.theta)
    # arg list - roll, pitch, yaw. roll and pitch are always zero for turtlesim_node

    # translation vector
    translation_vector = (msg.x, msg.y, 0)

    current_time = rospy.Time.now()

    transform_broadcaster.sendTransform(translation_vector, rotation_quternion, current_time, turtlename+"_frame", "world")
    # first - child id - name of the frame whose origin's cordinates and rotation in the parent frame you're brodcasting
    

def setup():
    rospy.init_node(NODE_NAME)
    
    # Create a transform broadcaster
    global transform_broadcaster
    transform_broadcaster = tf.TransformBroadcaster()

    # we are going to use launch file to launch the turltesim node, this tf broadcaster nodes
    # with a launch file. This node will be started 2 times for 2 different turtlebots, whose names
    # are provided as parameters to the launch file, that name of bot will be provided back to
    # the nodes with following line.
    # see ./follower.launch file    
    turtlename = rospy.get_param('~turtle')

    # subscribe to the pose topic
    global pose_sub
    pose_sub = rospy.Subscriber(f'/{turtlename}/pose', Pose, pose_callback, turtlename, queue_size=1)

def main():
    setup()
    rospy.spin()


if __name__ == "__main__":
    main()