#! /usr/bin/env python

import rospy, sys, math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

# CONSTANTS
NODE_NAME = "turtle_control"
VEL_TOPIC = "turtle1/cmd_vel"
POSE_TOPIC = "turtle1/pose"
LT = 0.3                # Linear threshold

# GLOBALS
tpos = cpos = Pose()    # will store target position and current position
pose_sub = None         # will be subscriber object for POSE_TOPIC
vel_pub = None          # wiill be publisher object for VEL_TOPIC
vel_pub_timer = None    # timer to publishe velocity periodically
loop_rate = None

# callback for POSE_TOPIC
def pose_callback(msg):
    global cpos
    cpos = msg


# callback for vel_pub_timer
def pub_vel(msg):
    vel_pub.publish(msg)

# setup node
def setup():
    global pose_sub, vel_pub, vel_pub_timer, loop_rate
    # initialize node
    rospy.init_node(NODE_NAME)
    # initialize subscriber
    pose_sub = rospy.Subscriber(POSE_TOPIC, Pose, pose_callback, queue_size=10)
    # initialize publisher
    vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=10)
    # initialize velocity publisher timer
    vel_pub_timer = rospy.Timer(rospy.Duration(0.25), pub_vel)
    vel_pub_timer.shutdown()
    # initialize loop rate
    loop_rate = rospy.Rate(100)

# main driver
def move(distance, speed, log = False):
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0
    # avoid conflicting speed values
    speed = abs(speed) if distance > 0 else -abs(speed)
    # update current position
    rospy.wait_for_message(POSE_TOPIC, Pose)
    # cpos must be updated now

    # calculate target position
    tpos.x = cpos.x + distance * math.cos(cpos.theta)
    tpos.y = cpos.y + distance * math.sin(cpos.theta)

    if log:
        rospy.loginfo(f"Command recieved to make the robot move {distance} distance with {speed} speed")
        print(f"Current pos: x = {cpos.x}, y = {cpos.y}")
        print(f"Target pos:  x = {tpos.x}, y = {tpos.y}")
        choice = input("Enter to continue, abort to abort")
        if choice == "abort":
            rospy.loginof("Operation aborted! Terminating...")
            sys.exit(0)
    
    # choose whethet to use x or y cordinate to stop the robot at right time
    if abs(cpos.theta) < (math.pi * 3 / 4) and abs(cpos.theta) > (math.pi / 4):
        # use y cord
        if log:
            print("Using y cord")
        if tpos.y < cpos.y:
            i = -1
        else:
            i = 1
        # start the robot
        msg.linear.x = speed
        pub_vel(msg)
        while tpos.y * i > cpos.y * i:
            loop_rate.sleep()
    else:
        # use x cord
        if log:
            print("Using x cord")
        if tpos.x < cpos.x:
            i = -1
        else:
            i = 1
        # start the robot
        msg.linear.x = speed
        pub_vel(msg)
        while tpos.x * i > cpos.x * i:
            loop_rate.sleep()
    # stop robot
    msg.linear.x = 0
    pub_vel(msg)
    print(f"Position reached: x = {cpos.x}, y = {cpos.y}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} {{distance m}} {{speed m/s}}")
        sys.exit(1)
    
    setup()
    move(float(sys.argv[1]), float(sys.argv[2]), True)
