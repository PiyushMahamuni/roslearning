#! /usr/bin/env python

import rospy, sys, math, time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# CONSTANTS
NODE_NAME = "turntoN"
VEL_TOPIC = "turtle1/cmd_vel"
POSE_TOPIC = "turtle1/pose"
DTRF = math.pi / 180  # Degrees to Radians Conversion Factor

# GLOBALS
vel_pub = None
pose_sub = None
cpos = None


# callback for POSE_TOPIC
def pose_callback(msg):
    global cpos
    cpos = msg


# setup this node
def setup():
    rospy.init_node(NODE_NAME)
    global vel_pub, pose_sub
    vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)
    pose_sub = rospy.Subscriber(POSE_TOPIC, Pose, pose_callback, queue_size=1)


def stop_robot():
    vel_msg = Twist()
    vel_pub.publish(vel_msg)


#turn by a specific angle radians with given speed
def turn(radians: float, speed: float) -> None:
    stop_robot()
    rospy.wait_for_message(POSE_TOPIC, Pose)
    loop_freq = 20
    loop_rate = rospy.Rate(loop_freq)
    speed = abs(speed)
    cclk = True if radians > 0 else False
    radians = abs(radians)
    loop_dur = loop_freq**-1
    total_time = radians / speed
    loop_count = 0
    while loop_count * loop_dur < total_time:
        loop_count += 1
    loop_count -= 1
    rem_time = total_time - loop_count * loop_dur
    vel_msg = Twist()
    vel_msg.angular.z = speed if cclk else -speed
    while loop_count:
        vel_pub.publish(vel_msg)
        loop_count -= 1
        loop_rate.sleep()
    vel_pub.publish(vel_msg)
    time.sleep(rem_time)
    vel_msg.angular.z = 0
    vel_pub.publish(vel_msg)


#turnto specific angle in radians
def turntor(radians: float, speed: float, log: bool = False) -> None:
    stop_robot()
    rospy.wait_for_message(POSE_TOPIC, Pose)
    angle_to_traverse = radians - cpos.theta
    while angle_to_traverse > math.pi:
        angle_to_traverse -= 2 * math.pi
    while angle_to_traverse < -math.pi:
        angle_to_traverse += 2 * math.pi
    if(log):
        rospy.loginfo(f"[{NODE_NAME}] Command Recieved to turn the robot to {radians} radians with {speed} speed")
        choice = input("Enter any key to continue, abort to abort: ")
        if choice == "abort":
            rospy.loginfo(f"[{NODE_NAME}] Command aborted by user!")
            return None
    turn(angle_to_traverse, speed)
    if log:
        rospy.loginfo(f"[{NODE_NAME}] turntor command completed!")
    

if __name__ == "__main__":
    print("Debugging")
    usage = f"{sys.argv[0]} degree speed"
    if len(sys.argv) != 3:
        print(usage)
        sys.exit(1)
    setup()
    turntor(DTRF * float(sys.argv[1]), DTRF * float(sys.argv[2]), True)
