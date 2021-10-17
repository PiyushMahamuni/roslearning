#! /usr/bin/env python

import rospy, tf, turtlesim.srv, math
from geometry_msgs.msg import Twist

# THIS NODE WILL PUBLISH VELOCITY COMMANDS FOR turtle2 AND WILL MAKE IT FOLLOW THE turtle1
# THE FRAMES OF turtle1 AND turtle2 IN world FRAME OF REFERENCE MUST HAVE BEEN PUBLISHED
# THE TF PACKAGE HELP US NOW TO FIND THE TRANSFOMRATION BETWEEN turtle1_frame AND turtle2_frame AND
# LOCATE THE turtle1 robot in turtle2_frame - the info that we will use to make the robot move
# towards the turtl1 robot

# CONSTANTS
NODE_NAME = "turtle_tf_listener"

# GLOBALS
transform_listener = None
vel_pub = None
turtle1 = turtle2 = None
vel_cmd = None

def setup():
    # initialize the node
    rospy.init_node(NODE_NAME)
    
    # create transform listener
    global transform_listener
    transform_listener = tf.TransformListener()

    # create a second turtle by calling the service
    rospy.wait_for_service("spawn")
    spawner = rospy.ServiceProxy("spawn", turtlesim.srv.Spawn)
    global turtle2
    turtle2 = rospy.get_param("~turtle2")
    spawner(4, 2, 0, turtle2)
    # spawn the turtle with given name at 4, 2, 0

    # create a velocity publisher for this robot
    global vel_pub
    vel_pub = rospy.Publisher(turtle2+"/cmd_vel", Twist, queue_size=1)

    # get the frame id of turtle1
    global turtle1
    turtle1 = rospy.get_param("~turtle1")

    # create the velocity command message
    global vel_cmd
    vel_cmd = Twist()

def main():
    setup()
    
    loop_rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        try:
            (translation, rotation) = transform_listener.lookupTransform(f"/{turtle2}_frame", f"/{turtle1}_frame", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        turtle1_x = translation[0]
        turtle1_y = translation[1]
        # the position of turlte1 in turtle2_frame
        angular = 4 * math.atan2(turtle1_y, turtle1_x)
        linear = 0.5 * math.sqrt(turtle1_x ** 2 + turtle1_y ** 2)

        vel_cmd.linear.x = linear
        vel_cmd.angular.z = angular

        vel_pub.publish(vel_cmd)

        loop_rate.sleep()



if __name__ == "__main__":
    main()