#! /usr/bin/env python
import sys, math, rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

# CONSTANTS
NODE_NAME = "00moveN"
VEL_TOPIC = "turtle1/cmd_vel"
POSE_TOPIC = "turtle1/pose"
LIMITS = [0.5, 11-0.5]

# GLOBALS
cpos = None
vel_pub = None
pose_sub = None

# POSE_TOPIC callback function
def pose_callback(msg):
    global cpos
    cpos = msg

# setup this node
def setup():
    rospy.init_node(NODE_NAME)
    global vel_pub, pose_sub
    vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size = 1)
    pose_sub = rospy.Subscriber(POSE_TOPIC, Pose, pose_callback, queue_size = 1)


def stop_robot():
    """stop the turtle1 robot"""
    vel_msg = Twist()
    vel_msg.linear.x = vel_msg.linear.y = vel_msg.linear.z = 0
    vel_msg.angular.x = vel_msg.angular.y = vel_msg.angular.z = 0
    vel_pub.publish(vel_msg)


def moveto(distance: float, speed: float, log: bool = False) -> bool:
    """move certain distance forward or backward with given speed, returns true if operation was
    successful"""
    stop_robot()
    rospy.wait_for_message(POSE_TOPIC, Pose)
    # calculate target destination
    tx = cpos.x + math.cos(cpos.theta) * distance
    ty = cpos.y + math.sin(cpos.theta) * distance
    if tx < LIMITS[0] or tx > LIMITS[1] or ty < LIMITS[0] or ty > LIMITS[1]:
        rospy.logerr(f"[{NODE_NAME}] Can't complete requested command, robot will crash")
        return False
    speed = abs(speed) if distance > 0 else -abs(speed)
    vel_msg = Twist()
    vel_msg.linear.x = speed
    if log:
        rospy.loginfo(f"""[{NODE_NAME}] Command requested to move the robot {distance} distance
        with {speed} speed""")
        choice = input("Enter any key to continue, abort to abort: ")
        if choice == "abort":
            rospy.loginfo(f"[{NODE_NAME}] Command aborted by user!")
            return False

    loop_rate = rospy.Rate(50)
    if math.pi/4 < abs(cpos.theta) < 3 * math.pi / 4:
        # use y cordinate
        if cpos.y > ty:
            while cpos.y > ty:
                loop_rate.sleep()
                vel_pub.publish(vel_msg)
        else:
            while cpos.y < ty:
                loop_rate.sleep()
                vel_pub.publish(vel_msg)
    else:
        # use x cordinate
        if cpos.x > tx:
            while cpos.x > tx:
                loop_rate.sleep()
                vel_pub.publish(vel_msg)
        else:
            while cpos.x < tx:
                loop_rate.sleep()
                vel_pub.publish(vel_msg)
    stop_robot()
    if log:
        rospy.loginfo(f"[{NODE_NAME}] moveto command completed!")
    return True


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} distance speed")
        sys.exit(1)
    setup()
    moveto(float(sys.argv[1]), float(sys.argv[2]), True)
