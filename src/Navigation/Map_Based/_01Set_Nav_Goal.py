#! /usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from math import radians, degrees
import actionlib_msgs.msg
from geometry_msgs.msg import Point


# CONSTANTS
NODE_NAME = "set_nav_goal"

# GLOBALS
ac = None

# THIS FUNCTION SETUPS THE GLOBALS AND OTHER NODE PARAMETERS


def setup():
    # define a client to send a goal request to the move_base server through a simple action client
    rospy.init_node(NODE_NAME)
    global ac
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)


# THIS FUNCTION WILL MAKE THE ROBOT MOVE
def move_to_goal(xGoal, yGoal):
    while ac.wait_for_server(rospy.Duration.from_sec(0.5)):
        rospy.loginfo("Waiting for the move_base action server to come up")
    goal = MoveBaseGoal()

    # SET UP THE FRAME PARAMETERS
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # MOVING TOWARDS THE GOAL
    goal.target_pose.pose.position = Point(xGoal, yGoal, 0)
    goal.target_pose.pose.orientation.x = goal.target_pose.pose.orientation.y = goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Sending Goal Location...")

    ac.send_goal(goal)
    ac.wait_for_result(rospy.Duration(60))
    
    if ac.get_state() == actionlib_msgs.msg.GoalStatus.SUCCEEDED:
        rospy.loginfo("Robot reached the destination")
        return True
    else:
        rospy.loginfo("Robot failed to reach destination")
        return False


def main():
    setup()
    x_goal = -2.0043
    y_goal = 4.1093
    print("Calling move_to_goal()")
    move_to_goal(x_goal, y_goal)
    rospy.spin()


if __name__ == "__main__":
    main()