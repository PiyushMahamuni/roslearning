#! /usr/bin/env python
import rospy, sys, math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# CONSTANTS
NODE_NAME = "goto"
POSE_TOPIC = "turtle1/pose"
VEL_TOPIC = "turtle1/cmd_vel"

# GLOBALS
pose_sub = None
vel_pub = None
cpos = None

# callback function for POSE_TOPIC
def pose_callback(msg):
    global cpos
    cpos = msg

def setup():
    global pose_sub, vel_pub
    # initialize node, setup velocity publisher, position subscriber
    rospy.init_node(NODE_NAME)
    vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)
    pose_sub = rospy.Subscriber(POSE_TOPIC, Pose, pose_callback, queue_size=1)

def goto(x, y, kpd=0.5, kpa=2):
    rospy.wait_for_message(POSE_TOPIC, Pose)
    vel_msg = Twist()
    loop_rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        xE = x - cpos.x
        yE = y - cpos.y
        distE = math.sqrt(xE**2 + yE**2)
        if(distE < 0.01):
            break
        vel_msg.linear.x = kpd * distE
        angleE = math.asin(yE/distE)
        if x < cpos.x:
            if angleE > 0:
                angleE = math.pi - angleE
            else:
                angleE = -math.pi - angleE
        angleE -= cpos.theta
        if angleE > math.pi:
            angleE = angleE - 2 * math.pi
        elif angleE < -math.pi:
            angleE = angleE + 2 * math.pi

        vel_msg.angular.z = kpa * angleE
        loop_rate.sleep()
        vel_pub.publish(vel_msg)


if __name__ == "__main__":
    usage = sys.argv[0] + " x y kpd kpa\n"
    if len(sys.argv) < 3:
        print(usage)
        sys.exit(1)
    setup()
    if len(sys.argv) == 5:
        goto(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
    else:
        goto(float(sys.argv[1]), float(sys.argv[2]))