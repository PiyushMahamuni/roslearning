#! /usr/bin/env python
import rospy, actionlib, sys
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import actionlib_msgs.msg
from geometry_msgs.msg import Point


# CONSTANTS
NODE_NAME = "set_nav_goal"

# GLOBALS
ac = None

# THIS FUNCTION SETUPS THE GLOBALS AND OTHER NODE PARAMETERS


def setup():
    rospy.init_node(NODE_NAME)
    global ac
    # define a client to send a goal request to the move_base server through a simple action client
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # ACTIONLIB IS SUITABLE FOR ROBOT NAVIGATION BECAUSE IT IS ASYNCHRONOUS AND ALLOWS ROBOT
    # TO PERFORM OTHER TASKS WHILE NAVIGATING.
    # USING ROS SERVICE FOR NAVIGATION WOULDN'T BE APPROPRIATE BECAUSE IT WILL BLOCK THE ROBOT
    # FROM PERFORMING ANYTHING ELSE.
    # THAT'S WHY move_base IS IMPLEMENTED AS AN ACTIONLIB.

    # the move_baser action_lib server has global path planner and local path planner
    # the second parameter on line 23 to the constructor is the type of the message exchanged
    # between action lib client and server.


# THIS FUNCTION WILL MAKE THE ROBOT MOVE
def move_to_goal(xGoal, yGoal):
    while not ac.wait_for_server(rospy.Duration.from_sec(0.5)):
        rospy.loginfo("Waiting for the move_base action server to come up")
        if rospy.is_shutdown():
            rospy.loginfo(f"Ternimating the {NODE_NAME} node")
            sys.exit()
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