#! /usr/bin/env python

import sys, math, rospy, time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# CONSTANTS
NODE_NAME = "turnN"
VEL_TOPIC = "turtle1/cmd_vel"
POSE_TOPIC = "turtle1/pose"
DTRF = math.pi / 180  # degree to radian factor

# GLOBALS
vel_msg = Twist()
vel_pub = None
pose_sub = None
cpos = None  # to save current position


# callback for POSE_TOPIC
def pose_callback(msg):
    global cpos
    cpos = msg


def setup():
    f"""Setup this node with `{NODE_NAME}` name and initialize velocity publisher, position subscriber"""
    rospy.init_node(NODE_NAME)
    global vel_pub, pose_sub
    vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)
    pose_sub = rospy.Subscriber(POSE_TOPIC, Pose, pose_callback, queue_size=1)


def stop_robot():
    vel_msg = Twist()
    vel_msg.linear.x = vel_msg.linear.y = vel_msg.linear.z = 0
    vel_msg.angular.x = vel_msg.angular.y = vel_msg.angular.z = 0
    vel_pub.publish(vel_msg)


def turn(radians: float, speed: float, log: bool = False) -> bool:
    stop_robot()
    rospy.wait_for_message(POSE_TOPIC, Pose)
    # calculate target theta
    loop_freq = 20
    loop_rate = rospy.Rate(loop_freq)
    loop_dur = loop_freq ** -1
    cclk = True if radians > 0 else False
    radians = abs(radians)
    speed = abs(speed)
    total_time = radians / speed
    loop_count = 0
    while loop_count * loop_dur < total_time:
        loop_count += 1
    loop_count -= 1
    rem_time = total_time - loop_count * loop_dur
    speed = speed if cclk else -speed
    if log:
        rospy.loginfo(f"[{NODE_NAME}] Command recieved to make the robot turn {radians} radians\n with {speed} radians/second speed")
        choice = input("Enter any key to continue, enter abort to abort command: ")
        if choice == "abort":
            rospy.loginfo(f"[{NODE_NAME}] Command aborted by user!")
            return None
    vel_msg = Twist()
    vel_msg.angular.z = speed
    while loop_count:
        vel_pub.publish(vel_msg)
        loop_count -= 1
        loop_rate.sleep()
    vel_pub.publish(vel_msg)
    time.sleep(rem_time)
    stop_robot()
    if log:
        rospy.loginfo(f"[{NODE_NAME}] Turn command completed!")


if __name__ == "__main__":
    usage = f"{sys.argv[0]} degrees speed"
    if len(sys.argv) != 3:
        print(usage)
        sys.exit(1)
    setup()
    turn(float(sys.argv[1]) * DTRF, float(sys.argv[2]) * DTRF, True)
