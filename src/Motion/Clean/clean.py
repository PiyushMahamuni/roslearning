#! /usr/bin/env python

import rospy, sys, math, time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn

# CONSTANTS
NODE_NAME = "turntoN"
VEL_TOPIC = "turtle1/cmd_vel"
POSE_TOPIC = "turtle1/pose"
DTRF = math.pi / 180  # Degrees to Radians Conversion Factor
LIMITS = [0.5, 11-0.5]


# GLOBALS
vel_pub: rospy.Publisher = None
pose_sub: rospy.Subscriber = None
cpos: Pose = None

# callback for POSE_TOPIC
def pose_callback(msg):
    """updates global `cpos: Pose` variable"""
    global cpos
    cpos = msg


# setup this node
def setup():
    """Setup the velocity publisher and position subscriber"""
    rospy.init_node(NODE_NAME)
    global vel_pub, pose_sub
    vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)
    pose_sub = rospy.Subscriber(POSE_TOPIC, Pose, pose_callback, queue_size=1)


def stop_robot():
    """Stops the robot"""
    vel_cmd = Twist()
    vel_pub.publish(vel_cmd)


def move(distance: float, speed: float, log: bool = False) -> bool:
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
    vel_cmd = Twist()
    vel_cmd.linear.x = speed
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
                vel_pub.publish(vel_cmd)
        else:
            while cpos.y < ty:
                loop_rate.sleep()
                vel_pub.publish(vel_cmd)
    else:
        # use x cordinate
        if cpos.x > tx:
            while cpos.x > tx:
                loop_rate.sleep()
                vel_pub.publish(vel_cmd)
        else:
            while cpos.x < tx:
                loop_rate.sleep()
                vel_pub.publish(vel_cmd)
    stop_robot()
    if log:
        rospy.loginfo(f"[{NODE_NAME}] moveto command completed!")
    return True


#turn by a specific angle radians with given speed
def turn(radians: float, speed: float) -> None:
    """Turns robot through given radians with given speed"""
    stop_robot()
    rospy.wait_for_message(POSE_TOPIC, Pose)
    loop_freq = 20
    loop_rate = rospy.Rate(loop_freq)
    speed = abs(speed)
    radians = abs(radians)
    loop_dur = loop_freq**-1
    total_time = radians / speed
    loop_count = 0
    while loop_count * loop_dur < total_time:
        loop_count += 1
    loop_count -= 1
    rem_time = total_time - loop_count * loop_dur
    vel_cmd = Twist()
    vel_cmd.angular.z = speed if radians > 0 else -speed
    while loop_count:
        vel_pub.publish(vel_cmd)
        loop_count -= 1
        loop_rate.sleep()
    vel_pub.publish(vel_cmd)
    time.sleep(rem_time)
    vel_cmd.angular.z = 0
    vel_pub.publish(vel_cmd)


#turnto specific angle in radians
def turntor(radians: float, speed: float, log: bool = False) -> None:
    """Turns the robot facing towards a specific angle"""
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


# goto specific location x, y
def goto(x, y, kpd=0.5, kpa=2) -> bool:
    """Makes the robot move to a specific location, returns true or false based on wheter
    the operation was successful or not"""
    rospy.wait_for_message(POSE_TOPIC, Pose)
    vel_cmd = Twist()
    loop_rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        xE = x - cpos.x
        yE = y - cpos.y
        distE = math.sqrt(xE**2 + yE**2)
        if(distE < 0.01):
            return True
        vel_cmd.linear.x = kpd * distE
        angleE = math.asin(yE/distE)
        if x < cpos.x:
            angleE = math.pi * (1 if yE > 0 else -1) - angleE
        angleE -= cpos.theta
        angleE -= 2 * math.pi * (1 if angleE > math.pi else (-1 if angleE < -math.pi else angleE))

        vel_cmd.angular.z = kpa * angleE
        loop_rate.sleep()
        vel_pub.publish(vel_cmd)
    return False


# move in a spiral tragectory
def spiral_clean(rk: float = 0.001) -> None:
    """moves the robot in a spiral trajectory until it gets too close to the walls"""
    stop_robot()
    loop_freq = 20
    loop_rate = rospy.Rate(loop_freq)
    vel_cmd = Twist()
    vel_cmd.angular.z = math.pi / 4  # 45 degrees per second
    while cpos.x < LIMITS[1] and cpos.x > LIMITS[0] and cpos.y < LIMITS[1] and cpos.y > LIMITS[0]:
        vel_cmd.linear.x += rk
        vel_pub.publish(vel_cmd)
        loop_rate.sleep()
    stop_robot()


# move in a grid pattern
def grid_clean() -> None:
    """Moves the robot in a grid-like pattern"""
    goto(1, 1)
    speed = math.pi / 4
    turntor(0, speed)
    alt_angle = math.pi / 2  # alternating angle : 90 and -90
    while move(1, 1):
        turntor(alt_angle, speed)
        move(9, 1)
        turntor(0, speed)
        alt_angle *= -1


if __name__ == "__main__":
    setup()
    # MENU
    print("Cleaning Application---")
    choice = input("Enter `grid` for grid cleaning, `spiral` for spiral cleaning: ")
    if choice == "grid":
        grid_clean()
    elif choice == "spiral":
        spiral_clean()
    else:
        print("Sorry, Invalide option")
        sys.exit(1)
